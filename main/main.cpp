#include "board.hpp"
#include "control.hpp"
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

constexpr float MOTOR_TEST_DUTY = 1.0f;
constexpr std::int64_t MOTOR_TEST_DURATION_US = 300'000;
constexpr motors::Config MOTOR_CONFIG{
    .pwm_frequency_hz = 5'000,
    .duty_limit = MOTOR_TEST_DUTY,
    .left_mapping = {.deadband = 0.0f, .min_duty = 0.0f, .max_duty = 1.0f},
    .right_mapping = {.deadband = 0.0f, .min_duty = 0.0f, .max_duty = 1.0f},
};
constexpr ControlConfig CONTROL_CONFIG{
    .kp = 0.0f,
    .kd = 0.0f,
    .target_tilt_rad = 0.0f,
    .command_limit = 1.0f,
    .max_tilt_rad = 0.5f,
};
constexpr float MAX_CONTROL_INTERVAL_S = 0.05f;

enum class Command { none, ping, test_left, test_right, balance, stop, console_error };
enum class MotorMode { idle, test_left, test_right, balance };

Command read_command()
{
    static char command[16]{};
    static std::size_t length = 0;
    static bool overflow = false;

    // Bound console work so input cannot starve the sensor/control loop.
    for (unsigned count = 0; count < sizeof(command); ++count) {
        char c;
        const ssize_t received = read(STDIN_FILENO, &c, 1);
        if (received <= 0) {
            if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                errno != EINTR && errno != EIO) {
                ESP_LOGE("Motor", "Console read failed: errno=%d", errno);
                return Command::console_error;
            }
            if (received < 0 && errno == EIO) {
                // Discard partial commands across USB disconnects.
                length = 0;
                overflow = false;
                return Command::stop;
            }
            return Command::none;
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
        const bool too_long = overflow;
        length = 0;
        overflow = false;
        if (input.empty() && !too_long) {
            continue;
        }
        if (!too_long) {
            if (input == "ping") return Command::ping;
            if (input == "test" || input == "test left") return Command::test_left;
            if (input == "test right") return Command::test_right;
            if (input == "balance") return Command::balance;
            if (input == "stop") return Command::stop;
        }
        ESP_LOGW("Motor", "Use 'ping', 'test left', 'test right', 'balance' or 'stop'");
        return Command::none;
    }
    return Command::none;
}

bool init_console()
{
    // ESP-IDF 6.1 nonblocking stdin reads need the USB driver's RX buffer.
    const esp_err_t err = esp_stdio_install_io_driver();
    if (err != ESP_OK) {
        ESP_LOGE("Motor", "Cannot initialize console: %s; motors remain disabled",
                 esp_err_to_name(err));
        return false;
    }
    const int flags = fcntl(STDIN_FILENO, F_GETFL);
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE("Motor", "Cannot configure console; motors remain disabled");
        return false;
    }
    ESP_LOGI("Motor", "Commands: ping, test left, test right, balance, stop (then Enter)");
    ESP_LOGI("Motor", "Tests: 300 ms at 100%%; 'test' aliases 'test left'. Balancing starts OFF");
    return true;
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

    bool motors_ready = motor_init_err == ESP_OK;
    bool console_ready = init_console();
    MotorMode mode = MotorMode::idle;
    std::int64_t test_deadline_us = 0;
    std::int64_t next_test_time_us = 0;

    TiltEstimate tilt_estimate{};
    std::int64_t previous_sample_time_us = esp_timer_get_time();
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        const std::int64_t now_us = esp_timer_get_time();
        const Command command = console_ready ? read_command() : Command::none;
        if (command == Command::stop || command == Command::console_error) {
            mode = MotorMode::idle;
            if (command == Command::console_error) console_ready = false;
        } else if (command == Command::ping) {
            ESP_LOGI("Motor", "pong");
        } else if (command != Command::none) {
            if (!motors_ready) {
                ESP_LOGW("Motor", "Motor driver unavailable until reboot");
            } else if (mode != MotorMode::idle || now_us < next_test_time_us) {
                ESP_LOGW("Motor", "Send 'stop' and wait for any test cooldown first");
            } else if (command == Command::balance) {
                mode = MotorMode::balance;
                ESP_LOGI("Control", "Balancing enabled; kp=%.3f kd=%.3f",
                         CONTROL_CONFIG.kp, CONTROL_CONFIG.kd);
            } else {
                mode = command == Command::test_left ? MotorMode::test_left : MotorMode::test_right;
                test_deadline_us = now_us + MOTOR_TEST_DURATION_US;
                next_test_time_us = test_deadline_us + 1'000'000;
                ESP_LOGI("Motor", "Testing %s motor", mode == MotorMode::test_left ? "left" : "right");
            }
        }

        ImuSample sample{};
        esp_err_t err = imu.read_and_write_to_sample(sample);
        bool sample_valid = false;
        float dt_s = 0.0f;

        if (err == ESP_OK) {
            const std::int64_t sample_time_us = esp_timer_get_time();
            dt_s = static_cast<float>(
                sample_time_us - previous_sample_time_us) * 1e-6f;
            previous_sample_time_us = sample_time_us;

            // Assumed mounting: sensor X forward, Y along the axle (left), Z up.
            // Upright at rest gives accel_z ~= +9.81; positive gyro_y is nose down.
            const TiltMeasurement measurement{
                .accel_forward_mps2 = sample.accel_x,
                .accel_up_mps2 = sample.accel_z,
                .gyro_rate_rad_s = sample.gyro_y,
            };
            sample_valid = update_complementary_filter(
                tilt_estimate, measurement, dt_s, TILT_FILTER_CONFIG);
            if (sample_valid) {
                // Teleplot reads one >name:value measurement per line.
                // std::printf(">tilt_rad:%.5f\n", tilt_estimate.tilt_rad);
            } else {
                ESP_LOGW("Tilt", "Rejected sample, timing, or filter configuration");
            }
        } else {
            ESP_LOGE("IMU", "Read failed: %s", esp_err_to_name(err));
        }

        const bool test_finished =
            (mode == MotorMode::test_left || mode == MotorMode::test_right) &&
            esp_timer_get_time() >= test_deadline_us;
        if (test_finished) mode = MotorMode::idle;

        float u = 0.0f;
        const bool control_rejected = mode == MotorMode::balance &&
            (!sample_valid || dt_s > MAX_CONTROL_INTERVAL_S ||
             !compute_control_command(tilt_estimate.tilt_rad,
                 sample.gyro_y - TILT_FILTER_CONFIG.gyro_bias_rad_s, u, CONTROL_CONFIG));
        if (control_rejected) mode = MotorMode::idle;

        // This loop is the sole motor owner. Manual tests bypass f(u).
        if (motors_ready) {
            switch (mode) {
            case MotorMode::idle: err = motors::stop(); break;
            case MotorMode::test_left: err = motors::set_duty(MOTOR_TEST_DUTY, 0.0f); break;
            case MotorMode::test_right: err = motors::set_duty(0.0f, MOTOR_TEST_DUTY); break;
            case MotorMode::balance: err = motors::set_command(u, u); break;
            }
            if (err != ESP_OK) {
                (void)motors::stop();
                mode = MotorMode::idle;
                motors_ready = false;
                ESP_LOGE("Motor", "Disabled until reboot: %s", esp_err_to_name(err));
            }
        }
        if (test_finished && motors_ready) {
            ESP_LOGI("Motor", "Test finished; both outputs disabled");
        }
        if (control_rejected) {
            ESP_LOGW("Control", "Balancing disabled: invalid sample/timing/config or tilt limit; send 'balance' to rearm");
        }

        // Nominally 100 Hz with the current 100 Hz RTOS tick; dt above is measured.
        xTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD_TICKS);
    }
}
