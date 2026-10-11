idf_component_register(
    SRCS "wireless.cpp" "wireless_protocol.cpp" "console.cpp" "control.cpp" "board.cpp" "imu.cpp" "main.cpp" "tilt_estimator.cpp" "motors_l298n.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_spi esp_timer esp_driver_gpio esp_driver_ledc
                  esp_stdio esp_wifi esp_event esp_netif esp_http_server nvs_flash bootloader_support
    EMBED_TXTFILES "web/index.html"
)
