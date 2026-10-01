#include "imu.hpp"

#include <array>
#include <cstdint>

#include "esp_log.h"

namespace {
constexpr std::uint8_t WHO_AM_I = 0x00;
constexpr std::uint8_t USER_CTRL = 0x03;
constexpr std::uint8_t PWR_MGMT_1 = 0x06;
constexpr std::uint8_t PWR_MGMT_2 = 0x07;
constexpr std::uint8_t ACCEL_XOUT_H = 0x2D;
constexpr std::uint8_t REG_BANK_SEL = 0x7F;


esp_err_t write_register(spi_device_handle_t spi,
                         std::uint8_t address,
                         std::uint8_t value) 
{
    std::array<std::uint8_t, 2> tx{address, value};

    spi_transaction_t transaction{};
    transaction.length = tx.size() * 8;
    transaction.tx_buffer = tx.data();

    return spi_device_transmit(spi, &transaction);
}


esp_err_t read_register(spi_device_handle_t spi,
                        std::uint8_t address,
                        std::uint8_t& value)
{
    std::array<std::uint8_t, 2> tx{
        static_cast<std::uint8_t>(address | 0x80), 0
    };
    std::array<std::uint8_t, 2> rx{};

    spi_transaction_t transaction{};
    transaction.length = tx.size() * 8;
    transaction.tx_buffer = tx.data();
    transaction.rx_buffer = rx.data();

    esp_err_t err = spi_device_transmit(spi, &transaction);
    if (err == ESP_OK) {
        value = rx[1];  // rx[0] arrives while sending the address.
    }
    return err;
}


std::int16_t signed_word(std::uint8_t high, std::uint8_t low)
{
    int value = (static_cast<int>(high) << 8) | low;
    if (value >= 0x8000) {
        value -= 0x10000;
    }
    return static_cast<std::int16_t>(value);
}
}  // namespace


esp_err_t imu_init(spi_device_handle_t spi)
{
    // The identity and measurement registers are in bank 0.
    esp_err_t err = write_register(spi, REG_BANK_SEL, 0x00);
    if (err != ESP_OK) return err;

    std::uint8_t identity = 0;
    err = read_register(spi, WHO_AM_I, identity);
    if (err != ESP_OK) return err;

    if (identity != 0xEA) {
        ESP_LOGE("IMU", "WHO_AM_I was 0x%02X, expected 0xEA", identity);
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = write_register(spi, USER_CTRL, 0x10);  // Disable I2C interface.
    if (err != ESP_OK) return err;

    err = write_register(spi, PWR_MGMT_1, 0x01); // Wake; select clock.
    if (err != ESP_OK) return err;

    return write_register(spi, PWR_MGMT_2, 0x00); // Enable all axes.
}


esp_err_t imu_read(spi_device_handle_t spi, ImuSample& sample) 
{
    // One address byte, then 12 consecutive data bytes.
    std::array<std::uint8_t, 13> tx{};
    std::array<std::uint8_t, 13> rx{};
    tx[0] = ACCEL_XOUT_H | 0x80;  // Top bit means SPI read.

    spi_transaction_t transaction{};
    transaction.length = tx.size() * 8;
    transaction.tx_buffer = tx.data();
    transaction.rx_buffer = rx.data();

    esp_err_t err = spi_device_transmit(spi, &transaction);
    if (err != ESP_OK) return err;

    sample.accel_x = signed_word(rx[1],  rx[2]);
    sample.accel_y = signed_word(rx[3],  rx[4]);
    sample.accel_z = signed_word(rx[5],  rx[6]);
    sample.gyro_x  = signed_word(rx[7],  rx[8]);
    sample.gyro_y  = signed_word(rx[9],  rx[10]);
    sample.gyro_z  = signed_word(rx[11], rx[12]);

    return ESP_OK;
}