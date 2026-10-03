idf_component_register(
    SRCS "board.cpp" "imu.cpp" "main.cpp" "tilt_estimator.cpp" "motors_l298n.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_spi esp_timer esp_driver_gpio esp_driver_ledc
                  esp_stdio
)
