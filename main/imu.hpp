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

class Imu {
public:
    Imu() = default;
    ~Imu();

    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

    // The SPI bus must be initialized first and outlive this device.
    esp_err_t init(spi_host_device_t host, int cs_pin);
    esp_err_t read(ImuSample& sample);

private:
    spi_device_handle_t spi_ = nullptr;
    bool initialized_ = false;
};
