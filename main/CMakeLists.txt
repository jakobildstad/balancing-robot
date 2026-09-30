idf_component_register(
    SRCS "imu.cpp" "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_spi
)