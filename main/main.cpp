#include "board.hpp"
#include "imu.hpp"
#include "motors.hpp"
#include "tilt_estimator.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <string_view>
#include <fcntl.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_stdio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr ComplementaryFilterConfig TILT_FILTER_CONFIG{
    .correction_time_constant_s = 1.0f,
    .gyro_bias_rad_s = 0.0f, // Replace with a stationary calibration of gyro_y.
};
constexpr TickType_t SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(10);
static_assert(SAMPLE_PERIOD_TICKS > 0, "Sampling period must be at least one tick");

constexpr float MOTOR_TEST_DUTY = 0.50f;
constexpr TickType_t MOTOR_TEST_DURATION = pdMS_TO_TICKS(300);
constexpr motors::Config MOTOR_CONFIG{
    .pwm_frequency_hz = 5'000,
    .duty_limit = MOTOR_TEST_DUTY,
};

void motor_test_task(void*)
{
    // Only this task calls the motor driver after initialization.
    char command[16]{};
    std::size_t length = 0;
    bool overflow = false;
    ESP_LOGI("Motor", "Send 'ping' followed by Enter to check the console");
    ESP_LOGI("Motor", "Send 'test left' or 'test right' followed by Enter: 300 ms at 50%%, one motor only");
    ESP_LOGI("Motor", "'test' is an alias for 'test left'");

    while (true) {
        // Bound console work so a stream of input cannot starve the IMU task.
        vTaskDelay(pdMS_TO_TICKS(10));
        char c;
        const ssize_t received = read(STDIN_FILENO, &c, 1);
        if (received <= 0) {
            if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                errno != EINTR && errno != EIO) {
                ESP_LOGE("Motor", "Console read failed: errno=%d", errno);
                break;
            }
            if (received < 0 && errno == EIO) {
                // Discard partial commands across USB disconnects.
                length = 0;
                overflow = false;
            }
            continue;
        }
        if (c != '\r' && c != '\n') {
            if (length < sizeof(command) - 1) {
                command[length++] = c;
            } else {
                overflow = true;
            }
            continue;
        }

        const std::string_view input{command, length};
        const bool test_left = !overflow && (input == "test" || input == "test left");
        const bool test_right = !overflow && input == "test right";
        const bool ping = !overflow && input == "ping";
        const bool has_input = length != 0 || overflow;
        length = 0;
        overflow = false;
        if (ping) {
            ESP_LOGI("Motor", "pong");
            continue;
        }
        if (!test_left && !test_right) {
            if (has_input) {
                ESP_LOGW("Motor", "Unknown command; use 'ping', 'test left' or 'test right'");
            }
            continue;
        }

        ESP_LOGI("Motor", "Testing %s motor", test_left ? "left" : "right");
        // Do not log or read the console while the motors are powered.
        const esp_err_t drive_err = motors::set_duty(
            test_left ? MOTOR_TEST_DUTY : 0.0f,
            test_right ? MOTOR_TEST_DUTY : 0.0f);
        if (drive_err == ESP_OK) {
            vTaskDelay(MOTOR_TEST_DURATION);
        }
        const esp_err_t stop_err = motors::stop();
        if (drive_err != ESP_OK || stop_err != ESP_OK) {
            ESP_LOGE("Motor", "Test disabled until reboot: drive=%s, stop=%s",
                     esp_err_to_name(drive_err), esp_err_to_name(stop_err));
            break;
        }
        ESP_LOGI("Motor", "Test finished; both outputs disabled");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    (void)motors::stop();
    vTaskDelete(nullptr);
}

void start_motor_test_console()
{
    // ESP-IDF 6.1 nonblocking stdin reads need the USB driver's RX buffer.
    const esp_err_t err = esp_stdio_install_io_driver();
    if (err != ESP_OK) {
        ESP_LOGE("Motor", "Cannot initialize console: %s; motor test disabled",
                 esp_err_to_name(err));
        return;
    }
    const int flags = fcntl(STDIN_FILENO, F_GETFL);
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE("Motor", "Cannot configure console; motor test disabled");
        return;
    }
    if (xTaskCreate(motor_test_task, "motor_test", 4096, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE("Motor", "Cannot create task; motor test disabled");
    }
}
}  // namespace

// Main application entry point for ESP32-C6.
extern "C" void app_main(void) 
{
    // initialize the SPI bus with the specified configuration and no DMA channel, with error checking.
    ESP_ERROR_CHECK(board::init());

    const esp_err_t motor_init_err = motors::init(MOTOR_CONFIG);
    if (motor_init_err != ESP_OK) {
        ESP_LOGE("Motor", "Initialization failed: %s", esp_err_to_name(motor_init_err));
    }

    Imu imu;
    ESP_ERROR_CHECK(
        imu.init(
            board::SPI_HOST, 
            board::IMU_CS_PIN,
            1'000'000
        )
    );

    if (motor_init_err == ESP_OK) {
        start_motor_test_console();
    }

    TiltEstimate tilt_estimate{};
    std::int64_t previous_sample_time_us = esp_timer_get_time();
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        ImuSample sample{};
        esp_err_t err = imu.read_and_write_to_sample(sample);

        if (err == ESP_OK) {
            const std::int64_t sample_time_us = esp_timer_get_time();
            const float dt_s = static_cast<float>(
                sample_time_us - previous_sample_time_us) * 1e-6f;
            previous_sample_time_us = sample_time_us;

            // Assumed mounting: sensor X forward, Y along the axle (left), Z up.
            // Upright at rest gives accel_z ~= +9.81; positive gyro_y is nose down.
            const TiltMeasurement measurement{
                .accel_forward_mps2 = sample.accel_x,
                .accel_up_mps2 = sample.accel_z,
                .gyro_rate_rad_s = sample.gyro_y,
            };
            if (update_complementary_filter(
                    tilt_estimate, measurement, dt_s, TILT_FILTER_CONFIG)) {
                // Teleplot reads one >name:value measurement per line.
                // std::printf(">tilt_rad:%.5f\n", tilt_estimate.tilt_rad);
            } else {
                ESP_LOGW("Tilt", "Rejected sample, timing, or filter configuration");
            }
        } else {
            ESP_LOGE("IMU", "Read failed: %s", esp_err_to_name(err));
        }

        // Nominally 100 Hz with the current 100 Hz RTOS tick; dt above is measured.
        xTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD_TICKS);
    }
}
