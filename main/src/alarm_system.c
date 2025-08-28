#include "alarm_system.h"
#include "4x4_matrix.h"
#include "rc522_implementation.h"
#include "lcd_1602.h"
#include "am312.h"
#include "led_driver.h"
#include "app_events.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>
#include <ctype.h>

#define TAG "ALARM_SYSTEM"

typedef enum {
    ALARM_ACTIVE,
    ALARM_SLEEP,
    ALARM_WAITING,
    ALARM_TRIGGERED
} alarm_state;

typedef struct alarm_system_handle{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t lcd_i2c_handle;
    led_handle_t green_led;
    led_handle_t orange_led;
    led_handle_t red_led;
    am312_handle_t sensor;
    rc522_handle_t rc522;
} alarm_system_handle;

static bool is_armed = false;
static alarm_state state = ALARM_SLEEP;
QueueHandle_t app_queue;
uint8_t wrong_entries = 0; // should be save in NVS to not be able to restart.

static void motion_cb(void) {
    app_handle_t event = {
        .event_type = EV_MOTION,
        .motion = 1
    };

    xQueueSend(app_queue, &event, portMAX_DELAY);
}

static void set_alarm_armed(alarm_system_handle_t handle) {
    xQueueReset(app_queue);
    lcd_1602_send_string(handle->lcd_i2c_handle, "Pin:");
    rc522_start(handle->rc522);

    app_handle_t event;
    uint8_t pin_len = 0;
    uint8_t pin_max_len = 4;
    char pin[5];

    bool is_waiting = true;
    while(is_waiting) {
        if(!xQueueReceive(app_queue, &event, portMAX_DELAY)) continue;

        switch (event.event_type) {
        case EV_RFID:
            // send rfid
            is_waiting = false;
            lcd_1602_send_string(handle->lcd_i2c_handle, "Waiting ...");
            break;

        case EV_CHAR_RECEIVED:
            if(event.key.key_pressed == 'c') {
                is_waiting = false;
            } else {
                if(pin_len < pin_max_len) {
                    pin[pin_len++] = event.key.key_pressed;
                    send_char(handle->lcd_i2c_handle, '*');
                }
                if(pin_len == pin_max_len) {
                    pin[pin_max_len] = '\0';
                    is_waiting = false;
                    // send rfid
                    lcd_1602_send_string(handle->lcd_i2c_handle, "Waiting ...");
                }
            }    
            break;

        default:
            break;
        }
    }

    rc522_pause(handle->rc522);
}

static void alarm_wakeup(alarm_system_handle_t handle, app_handle_t event) {
    
    if (!event.event_type == EV_CHAR_RECEIVED) return;

    char menu_options[33];
    if(!is_armed) {
        snprintf(menu_options, sizeof(menu_options), "A:Arm B:New\nC:Cancel");
    }
    else if (is_armed) {
        snprintf(menu_options, sizeof(menu_options), "A:Disarm B:New\nC:Cancel");
    }

    lcd_1602_send_string(handle->lcd_i2c_handle, menu_options);
    led_on(handle->orange_led);

    while(1) {
        if (event.key.key_pressed == ' a') {
            set_alarm_armed(handle);
        }
        else if (event.key.key_pressed == 'b') {

        }
        else if (event.key.key_pressed == 'c') {
            lcd_1602_send_string(handle->lcd_i2c_handle, " ");
            break;
        }
    }
}

void run_alarm(alarm_system_handle_t handle) {
    char menu_choice;

    xTaskCreate(_4x4_matrix_task, "_4x4_matrix_task", 1048, NULL, 5, NULL);
    
    app_handle_t event;
    
    while(1) {
        if(!xQueueReceive(app_queue, &event, portMAX_DELAY)) continue;
        
        switch(state) {
            case ALARM_SLEEP:
                alarm_wakeup(handle, event);
                break;

            case ALARM_ACTIVE:

                break;

            case ALARM_WAITING:

                break;
        }

    }
}

uint8_t init_alarm(alarm_system_handle_t handle) {
    
    if (handle != NULL) return 1;

    app_queue = xQueueCreate(16, sizeof(app_handle_t));
    if(!app_queue) goto _failed;

    alarm_system_handle_t h_temp = (alarm_system_handle_t)calloc(1, sizeof(alarm_system_handle));
    bool i2c_init = false;
    if (h_temp == NULL) return NULL;

    // I2C init
    if (i2c_open(&h_temp->bus_handle, &h_temp->lcd_i2c_handle, DEVICE_ADDRESS) != 0) {
        ESP_LOGE(TAG, "Unable to initialize I2C");
        goto _failed;
    }
    i2c_init = true;

    // LCD init
    if (lcd_1602_init(h_temp->lcd_i2c_handle) != 0) {
        ESP_LOGE(TAG, "Unable to initialize LCD1602");
        goto _failed;
    }

    // Keypad init
    uint8_t row_pins[] = {GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_17, GPIO_NUM_16};
    uint8_t col_pins[] = {GPIO_NUM_39, GPIO_NUM_36, GPIO_NUM_34, GPIO_NUM_35};
    _4x4_matrix_init(row_pins, col_pins, app_queue);

    // LEDs init
    h_temp->green_led = led_init(GPIO_NUM_27);
    h_temp->orange_led = led_init(GPIO_NUM_14);
    h_temp->red_led = led_init(GPIO_NUM_12);

    if(!(h_temp->green_led && h_temp->orange_led && h_temp->red_led)) {
        ESP_LOGE(TAG, "Unable to initialize all leds");
        goto _failed;
    }

    if(rc522_init(h_temp->rc522, app_queue) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize rc522");
        goto _failed;
    }
    
    h_temp->sensor = am312_init(GPIO_NUM_13, motion_cb, "hall", 2500, 250);
    
    if(h_temp->sensor == NULL) {
        ESP_LOGE(TAG, "Unable to initialize motions sensor");
        goto _failed;
    }

    return 0;

_failed:
    if(i2c_init) {
        i2c_master_bus_rm_device(h_temp->lcd_i2c_handle);
        i2c_del_master_bus(h_temp->bus_handle);
    }
    
    if(h_temp->green_led) free(h_temp->green_led);
    if(h_temp->orange_led) free(h_temp->orange_led);
    if(h_temp->red_led) free(h_temp->red_led);
    if(h_temp->sensor) free(h_temp->sensor);
    if(h_temp) free(h_temp);
    if(app_queue) {
        QueueHandle_t temp = app_queue;
        app_queue = NULL;
        vQueueDelete(temp);
    }

    return 1;
}