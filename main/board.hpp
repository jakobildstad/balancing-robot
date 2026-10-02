#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

namespace board {
constexpr spi_host_device_t SPI_HOST = SPI2_HOST;
constexpr int MOSI_PIN = 19;
constexpr int MISO_PIN = 20;
constexpr int SCLK_PIN = 18;
constexpr int IMU_CS_PIN = 21;

esp_err_t init();
}  // namespace board
