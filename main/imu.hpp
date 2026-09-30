#pragma once

#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"

struct ImuSample {
    std::int16_t accel_x;
    std::int16_t accel_y;
    std::int16_t accel_z;
    std::int16_t gyro_x;
    std::int16_t gyro_y;
    std::int16_t gyro_z;
};

esp_err_t imu_init(spi_device_handle_t spi);
esp_err_t imu_read(spi_device_handle_t spi, ImuSample& sample);