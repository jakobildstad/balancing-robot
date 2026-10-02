#include "board.hpp"
#include "imu.hpp"

#include <cstdio>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Main application entry point for ESP32-C6.
extern "C" void app_main(void) 
{
    // initialize the SPI bus with the specified configuration and no DMA channel, with error checking.
    ESP_ERROR_CHECK(board::init());

    Imu imu;
    ESP_ERROR_CHECK(
        imu.init(
            board::SPI_HOST, 
            board::IMU_CS_PIN,
            1'000'000
        )
    );

    while (true) {
        ImuSample sample{};
        esp_err_t err = imu.read_and_write_to_sample(sample);

        if (err == ESP_OK) {
            // Teleplot reads one >name:value measurement per line.
            std::printf(">accel_x_mps2:%.5f\n>accel_y_mps2:%.5f\n>accel_z_mps2:%.5f\n"
                        ">gyro_x_rad_s:%.5f\n>gyro_y_rad_s:%.5f\n>gyro_z_rad_s:%.5f\n",
                        sample.accel_x, sample.accel_y, sample.accel_z,
                        sample.gyro_x, sample.gyro_y, sample.gyro_z);
        } else {
            ESP_LOGE("IMU", "Read failed: %s", esp_err_to_name(err));
        }

        // Delay for 100 milliseconds before the next read, resulting in 10 hz sampling rate.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
