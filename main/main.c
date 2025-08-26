#include <stdio.h>
#include "4x4_matrix.h"
#include "lcd_1602.h"
#include "led_driver.h"
#include "am312.h"
#include "include/rc522_implementation.h"
#include "wifi_implementation.h"

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <string.h>

const char* TAG = "MAIN";

char buf[5];

static void motion_cb(void *user) {
    const char *name = (const char*)user;
    ESP_LOGI(TAG, "%s", name);
}

// Själva larmet ska ha en bit som säger ifall det är triggered eller inte som vi inväntar.


int app_main(void)
{

    vTaskDelay(pdMS_TO_TICKS(5000));

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // I2C init
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    if (i2c_open(&bus_handle, &dev_handle, DEVICE_ADDRESS) != 0) {
        ESP_LOGE(TAG, "Unable to initialize I2C");
        return 1;
    }

    // LCD init
    if (lcd_1602_init(dev_handle) != 0) {
        ESP_LOGE(TAG, "Unable to initialize LCD1602");
        return 1;
    }

    // Keypad init
    uint8_t row_pins[] = {GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_17, GPIO_NUM_16};
    uint8_t col_pins[] = {GPIO_NUM_39, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_35};
    _4x4_matrix_init(row_pins, col_pins);

    // LEDs init
    led_handle_t green_led = led_init(GPIO_NUM_27);
    led_handle_t orange_led = led_init(GPIO_NUM_14);
    led_handle_t red_led = led_init(GPIO_NUM_12);

    if(!(green_led && orange_led && red_led)) {
        ESP_LOGE(TAG, "Unable to initialize all leds");
        return 1;
    }

    if(rc522_init() != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize rc522");
        return 1;
    }
    
    am312_handle_t sensor = am312_init(GPIO_NUM_13, motion_cb, "hall", 2500, 250);
    
    if(sensor == NULL) {
        ESP_LOGE(TAG, "Unable to initialize motions sensor");
        return 1;
    }

    wifi_init();
    wait_for_connection();

    while(true) {
        if (wifi_is_connected()) {
            led_blink(green_led);
        }
        
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

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}