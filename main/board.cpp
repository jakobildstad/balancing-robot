#include "board.hpp"

esp_err_t board::init()
{
    spi_bus_config_t bus{};
    bus.mosi_io_num = MOSI_PIN;
    bus.miso_io_num = MISO_PIN;
    bus.sclk_io_num = SCLK_PIN;
    // Unused signals must be disabled explicitly; zero would select GPIO0.
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;

    return spi_bus_initialize(SPI_HOST, &bus, SPI_DMA_DISABLED);
}
