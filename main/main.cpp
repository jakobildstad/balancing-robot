#include "imu.hpp"

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Replace these with your actual ESP32-C6 GPIO numbers.
constexpr int MOSI_PIN = -1;
constexpr int MISO_PIN = -1;
constexpr int SCLK_PIN = -1;
constexpr int CS_PIN   = -1;

static_assert(MOSI_PIN >= 0 && MISO_PIN >= 0 &&
              SCLK_PIN >= 0 && CS_PIN >= 0,
              "Set the four IMU GPIO pins before building");

extern "C" void app_main(void)
{
    spi_bus_config_t bus{};
    bus.mosi_io_num = MOSI_PIN;
    bus.miso_io_num = MISO_PIN;
    bus.sclk_io_num = SCLK_PIN;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));

    spi_device_interface_config_t device{};
    device.clock_speed_hz = 1'000'000;
    device.mode = 0;
    device.spics_io_num = CS_PIN;
    device.queue_size = 1;

    spi_device_handle_t imu_spi = nullptr;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device, &imu_spi));

    vTaskDelay(pdMS_TO_TICKS(100)); // Allow the IMU to start.
    ESP_ERROR_CHECK(imu_init(imu_spi));

    while (true) {
        ImuSample sample{};
        esp_err_t err = imu_read(imu_spi, sample);

        if (err == ESP_OK) {
            ESP_LOGI("IMU", "accel: %d %d %d | gyro: %d %d %d",
                     sample.accel_x, sample.accel_y, sample.accel_z,
                     sample.gyro_x, sample.gyro_y, sample.gyro_z);
        } else {
            ESP_LOGE("IMU", "Read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}