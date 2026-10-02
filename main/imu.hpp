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
    // constructor and destructor
    Imu() = default;
    ~Imu(); // defined in imu.cpp

    // Prevent copying and assignment to avoid multiple instances managing the same SPI device.
    Imu(const Imu&) = delete; // delete simply means forbidden operation
    Imu& operator=(const Imu&) = delete;

    // Declare user facing functions
    esp_err_t init(spi_host_device_t host, int cs_pin);
    esp_err_t read_and_write_to_sample(ImuSample& sample);

private:
    spi_device_handle_t spi_ = nullptr;
    bool initialized_ = false;
};
