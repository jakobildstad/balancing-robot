#pragma once

#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"

struct ImuSample {
    // Sensor-frame acceleration in m/s^2, including the response to gravity.
    float accel_x;
    float accel_y;
    float accel_z;
    // Sensor-frame angular velocity in rad/s; gyro bias is not removed yet.
    float gyro_x;
    float gyro_y;
    float gyro_z;
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
    esp_err_t init(spi_host_device_t host, int cs_pin, int clock_speed_hz = 1'000'000);
    esp_err_t read_and_write_to_sample(ImuSample& sample);

private:
    spi_device_handle_t spi_ = nullptr;
    bool initialized_ = false;
};
