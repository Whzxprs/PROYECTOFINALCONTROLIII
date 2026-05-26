idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_gpio
        esp_timer
        esp_adc
        freertos
        log
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
)
