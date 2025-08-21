#include <stdio.h>
#include "4x4_matrix.h"
#include "lcd_1602.h"
#include "led_driver.h"

#include "include/rc522_implementation.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

const char* TAG = "MAIN";

char buf[5];

int app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    // I2C initialization
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    if (i2c_open(&bus_handle, &dev_handle, DEVICE_ADDRESS) != 0) {
        ESP_LOGE(TAG, "Unable to initialize I2C");
        return 1;
    }

    // LCD initialization
    if (lcd_1602_init(dev_handle) != 0) {
        ESP_LOGE(TAG, "Unable to initialize LCD1602");
        return 1;
    }

    uint8_t row_pins[] = {GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_17, GPIO_NUM_16};
    uint8_t col_pins[] = {GPIO_NUM_39, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_35};
    _4x4_matrix_init(row_pins, col_pins);

    led_handle_t green_led = led_init(GPIO_NUM_27);
    led_handle_t orange_led = led_init(GPIO_NUM_14);
    led_handle_t red_led = led_init(GPIO_NUM_12);

    if(!(green_led && orange_led && red_led)) {
        ESP_LOGE(TAG, "Unable to initialize all leds");
    }

    rc522_init();

    
    while(true) {
        lcd_1602_send_string(dev_handle, "ENTER CODE:");

        led_on(orange_led);
        _4x4_matrix_scan_keys(buf, 5);
        led_off(orange_led);

        if (!strcmp(buf, "0493")) {
            lcd_1602_send_string(dev_handle, "WELCOME!");
            led_on(green_led);
        }
        else {
            lcd_1602_send_string(dev_handle, "Incorrect!");
            led_on(red_led);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));

        led_off(green_led);
        led_off(red_led);
    }
}