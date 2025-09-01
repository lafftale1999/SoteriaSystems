idf_component_register(
    SRCS "main.c" "src/rc522_implementation.c" "src/wifi_implementation.c" "src/alarm_system.c" "src/system_formatter.c" "src/hash_sha256.c" "src/time_sync.c"
    INCLUDE_DIRS "." "include"
    REQUIRES
        rc522
        4x4_matrix_keypad_driver
        lcd_1602_i2c_driver
        project_config
        led_driver
        AM312_driver
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        app_events
)