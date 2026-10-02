#include "imu.hpp"

#include <array>
#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Unnamed namespace for internal helper functions and constants.
namespace {
// Register addresses for the IMU.
constexpr std::uint8_t WHO_AM_I = 0x00;
constexpr std::uint8_t USER_CTRL = 0x03;
constexpr std::uint8_t PWR_MGMT_1 = 0x06;
constexpr std::uint8_t PWR_MGMT_2 = 0x07;
constexpr std::uint8_t ACCEL_XOUT_H = 0x2D;
constexpr std::uint8_t REG_BANK_SEL = 0x7F;


// function to write a value to a register on the IMU over SPI.
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


// function to read a value from a register on the IMU over SPI.
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


// Convert two bytes (high and low) into a signed 16-bit integer.
std::int16_t signed_word(std::uint8_t high, std::uint8_t low)
{
    int value = (static_cast<int>(high) << 8) | low;
    if (value >= 0x8000) {
        value -= 0x10000;
    }
    return static_cast<std::int16_t>(value);
}


// Configure the IMU by writing to its registers over SPI.
esp_err_t configure(spi_device_handle_t spi)
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
}  // namespace


Imu::~Imu()
{
    if (spi_ != nullptr) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_remove_device(spi_));
    }
}

esp_err_t init(
    spi_host_device_t host,
    int cs_pin,
    int clock_speed_hz = 1'000'000
);
{
    if (spi_ != nullptr) return ESP_ERR_INVALID_STATE;

    // Configure the SPI device interface for the IMU.
    spi_device_interface_config_t device{};
    device.clock_speed_hz = clock_speed_hz;
    device.spics_io_num = cs_pin;
    device.mode = 0; // the clock idles low and data is sampled on the rising edge.
    device.queue_size = 1;

    esp_err_t err = spi_bus_add_device(host, &device, &spi_);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(100)); // Allow the IMU to start.
    err = configure(spi_);

    // Error handling
    if (err != ESP_OK) {
        esp_err_t cleanup_err = spi_bus_remove_device(spi_);
        if (cleanup_err == ESP_OK) {
            spi_ = nullptr;
        } else {
            ESP_LOGE("IMU", "Device cleanup failed: %s", esp_err_to_name(cleanup_err));
        }
        return err;
    }

    initialized_ = true;
    return ESP_OK;
}


esp_err_t Imu::read_and_write_to_sample(ImuSample& sample)
{
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // One address byte, then 12 consecutive data bytes.
    std::array<std::uint8_t, 13> tx{};
    std::array<std::uint8_t, 13> rx{};
    tx[0] = ACCEL_XOUT_H | 0x80;  // Top bit means SPI read.

    spi_transaction_t transaction{};
    transaction.length = tx.size() * 8;
    transaction.tx_buffer = tx.data();
    transaction.rx_buffer = rx.data();

    esp_err_t err = spi_device_transmit(spi_, &transaction);
    if (err != ESP_OK) return err;

    sample.accel_x = signed_word(rx[1],  rx[2]);
    sample.accel_y = signed_word(rx[3],  rx[4]);
    sample.accel_z = signed_word(rx[5],  rx[6]);
    sample.gyro_x  = signed_word(rx[7],  rx[8]);
    sample.gyro_y  = signed_word(rx[9],  rx[10]);
    sample.gyro_z  = signed_word(rx[11], rx[12]);

    return ESP_OK;
}
