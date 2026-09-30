idf_component_register(
    SRCS "main.c" "board.c" "bridge.c" "modem.c" "status.c" "sha256.c" "mavframe.c" "tunnel.c" "snapshot.c"
         "mavpos.c" "camera.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_driver_rmt esp_event esp_netif esp_timer esp_hw_support
                  esp_rom efuse lwip nvs_flash log
)
