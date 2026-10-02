#include "board.hpp"

esp_err_t board::init()
{
    // Configure the SPI bus for the IMU.
    spi_bus_config_t bus{};
    bus.mosi_io_num = MOSI_PIN;
    bus.miso_io_num = MISO_PIN;
    bus.sclk_io_num = SCLK_PIN;
    // The following pins are not used in this configuration, so they are set to -1.
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;

    return spi_bus_initialize(SPI_HOST, &bus, SPI_DMA_DISABLED);
}
