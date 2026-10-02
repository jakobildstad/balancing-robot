idf_component_register(
    SRCS "board.cpp" "imu.cpp" "main.cpp" "tilt_estimator.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_spi
)
