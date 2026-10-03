#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

namespace board {
constexpr spi_host_device_t SPI_HOST = SPI2_HOST;
constexpr int MOSI_PIN = 19;
constexpr int MISO_PIN = 20;
constexpr int SCLK_PIN = 18;
constexpr int IMU_CS_PIN = 21;

// L298N wiring using the selected GPIOs on the right-hand header.
constexpr int LEFT_MOTOR_ENABLE_PIN = 0;  // ENA
constexpr int LEFT_MOTOR_IN1_PIN = 1;
constexpr int LEFT_MOTOR_IN2_PIN = 6;
constexpr int RIGHT_MOTOR_ENABLE_PIN = 7;  // ENB
constexpr int RIGHT_MOTOR_IN1_PIN = 10;    // IN3
constexpr int RIGHT_MOTOR_IN2_PIN = 11;   // IN4
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;

esp_err_t init();
}  // namespace board
