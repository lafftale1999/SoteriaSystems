idf_component_register(
    SRCS "main.c" "src/rc522_implementation.c"
    INCLUDE_DIRS "." "include"      # din egen include/ i main
    REQUIRES rc522 4x4_matrix_keypad_driver lcd_1602_i2c_driver project_config led_driver AM312_driver
)