idf_component_register(
    SRCS "main.c" "src/rc522_implementation.c" "src/wifi_implementation.c"
    INCLUDE_DIRS "." "include"      # din egen include/ i main
    REQUIRES
        rc522
        4x4_matrix_keypad_driver
        lcd_1602_i2c_driver
        project_config
        led_driver
        AM312_driver
        nvs_flash         # <-- behövs för nvs_flash.h + länkning
        esp_wifi          # <-- om wifi_implementation.c använder Wi-Fi
        esp_event
        esp_netif
)