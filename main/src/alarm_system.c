#include "alarm_system.h"
#include "4x4_matrix.h"
#include "rc522_implementation.h"
#include "lcd_1602.h"
#include "am312.h"
#include "led_driver.h"
#include "app_events.h"
#include "credentials.h"
#include "wifi_implementation.h"
#include "system_formatter.h"
#include "http_implementaion.h"
#include "hash_sha256.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>
#include <ctype.h>

#define TAG "ALARM_SYSTEM"
#define WAIT_TIME_MS        10000
#define CHECK_IN_PERIOD_MS (3600 * 1000)
#define WIFI_CHECK_PERIOD   5000

typedef enum {
    ALARM_ARMED,
    ALARM_DISARMED,
    ALARM_IS_TRIGGERED,
    ALARM_CHECK_IN,
    ALARM_NEW_USER,
    SYSTEM_ACTION_COUNT
} alarm_system_action;

static const char* alarm_system_event[SYSTEM_ACTION_COUNT] = {
    [ALARM_ARMED]       = "alarm_armed",
    [ALARM_DISARMED]     = "alarm_disarmed",
    [ALARM_IS_TRIGGERED]   = "alarm_triggered",
    [ALARM_CHECK_IN]    = "alarm_check_in",
    [ALARM_NEW_USER]    = "new_user"
};

static const char* json_keys[] = {
    "serial",
    "action",
    "credentials",
    "type"
};

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
    QueueHandle_t app_queue;
} alarm_system_handle;

static bool is_armed = false;
static alarm_state state = ALARM_SLEEP;
uint8_t wrong_entries = 0; // should be save in NVS to not be able to restart.

static TaskHandle_t s_alarm_task = NULL;

bool alarm_send_to_server(alarm_system_handle_t handle, alarm_system_action action, app_handle_t event) {
    http_request_args_t *args = calloc(1, sizeof(*args));

    args->caller = xTaskGetCurrentTaskHandle();
    args->status = ESP_FAIL;

    if (action == ALARM_ARMED || action == ALARM_DISARMED || action == ALARM_NEW_USER) {
        char* action_value = alarm_system_event[action];
        char cred_value_hashed[SHA256_OUT_SIZE];
        char type_value[4];

        if (event.event_type == EV_RFID) {
            hash_sha256(event.rfid.uid, strlen(event.rfid.uid), cred_value_hashed);
            snprintf(type_value, sizeof(type_value), "tag");
        } 

        else if (event.event_type == EV_PIN) {
            hash_sha256(event.pin.pin, strlen(event.pin.pin), cred_value_hashed);
            snprintf(type_value, sizeof(type_value), "pin");
        }

        char *json_values[ALARM_ARMED_DISARMED_JSON_LEN];
        json_values[0] = DEVICE_ID;
        json_values[1] = action_value;
        json_values[2] = cred_value_hashed;
        json_values[3] = type_value;
        
        create_request_body(json_keys, json_values, ALARM_ARMED_DISARMED_JSON_LEN, args->post_data, sizeof(args->post_data));
        snprintf(args->url, sizeof(args->url), "%s/%s", API_URL, API_CHECK_AUTH);
    }

    else if (action == ALARM_IS_TRIGGERED) {
        char *json_values[ALARM_TRIGGERED_JSON_LEN];
        json_values[0] = DEVICE_ID;
        json_values[1] = alarm_system_event[action];
        
        create_request_body(json_keys, json_values, ALARM_TRIGGERED_JSON_LEN, args->post_data, sizeof(args->post_data));
        snprintf(args->url, sizeof(args->url), "%s/%s", API_URL, API_ALARM_TRIGGERED);
    }

    else if (action == ALARM_CHECK_IN) {
        char *json_values[ALARM_CHECK_IN_JSON_LEN];
        json_values[0] = DEVICE_ID;
        json_values[1] = alarm_system_event[action];

        create_request_body(json_keys, json_values, ALARM_CHECK_IN_JSON_LEN, args->post_data, sizeof(args->post_data));
        snprintf(args->url, sizeof(args->url), "%s/%s", API_URL, API_CHECK_IN);
    }

    xTaskCreate(http_post_task, "http_post_task", 4096, &args, 5, NULL);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    return args->status == ESP_OK;
}

static void motion_cb(void *handle) {
    alarm_system_handle_t h = (alarm_system_handle_t)handle;

    if(!h || !h->app_queue) return;

    app_handle_t event = {
        .event_type = EV_MOTION,
        .motion.id = 1
    };

    xQueueSend(h->app_queue, &event, portMAX_DELAY);
}

static void alarm_triggered_routine(void *args) {
    alarm_system_handle_t handle = (alarm_system_handle_t)args;

    if(!handle) vTaskDelete(NULL);
    app_handle_t event = {
        .event_type = EV_TRIGGERED
    };

    alarm_send_to_server(handle, ALARM_IS_TRIGGERED, event);
    
    while(state == ALARM_TRIGGERED) {
        led_blink(handle->red_led);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    led_off(handle->red_led);

    s_alarm_task = NULL;
    vTaskDelete(NULL);
}

static bool check_authorized(alarm_system_handle_t handle) {
    rc522_start(handle->rc522);

    app_handle_t event;
    alarm_system_action action = is_armed ? ALARM_DISARMED : ALARM_ARMED;

    uint8_t pin_len = 0;
    uint8_t pin_max_len = 4;
    char pin[5];

    bool is_waiting = true;

    while(is_waiting) {
        if(!xQueueReceive(handle->app_queue, &event, pdMS_TO_TICKS(WAIT_TIME_MS))) break;

        switch (event.event_type) {
        case EV_RFID:
            is_waiting = false;
            break;

        case EV_CHAR_RECEIVED:
            if(event.key.key_pressed == 'C') {
                rc522_pause(handle->rc522);
                return false;
            } else {
                if(pin_len < pin_max_len) {
                    pin[pin_len++] = event.key.key_pressed;
                    send_char(handle->lcd_i2c_handle, '*');
                }
                if(pin_len == pin_max_len) {
                    pin[pin_max_len] = '\0';
                    is_waiting = false;
                    app_handle_t pin_event = {
                        .event_type = EV_PIN
                    };
                    memcpy(pin_event.pin.pin, pin, sizeof(pin_event.pin.pin));

                    event = pin_event;
                }
            }    
            break;

        default:
            break;
        }
    }

    rc522_pause(handle->rc522);
    lcd_1602_send_string(handle->lcd_i2c_handle, "Waiting ...");

    return alarm_send_to_server(handle, action, event);
}

static void alarm_add_user(alarm_system_handle_t handle) {
    xQueueReset(handle->app_queue);
    lcd_1602_send_string(handle->lcd_i2c_handle, "ADD NEW TAG\nScan new tag...");
    rc522_start(handle->rc522);

    app_handle_t event;

    bool event_received = false;
    bool tag_added = false;

    while(!event_received) {
        if(!xQueueReceive(handle->app_queue, &event, pdMS_TO_TICKS(WAIT_TIME_MS))) break;

        if(event.event_type == EV_RFID) {
            event_received = true;
        }
    }

    led_off(handle->orange_led);
    rc522_pause(handle->rc522);
    
    if(event_received) {
        led_handle_t led;
        tag_added = alarm_send_to_server(handle, ALARM_NEW_USER, event);
        if(tag_added) {
            led = handle->green_led;
            led_on(led);
            lcd_1602_send_string(handle->lcd_i2c_handle, "Tag added!\nCheck homepage!");
        }
        else {
            led = handle->red_led;
            led_on(led);
            lcd_1602_send_string(handle->lcd_i2c_handle, "Tag not added!\nCheck homepage!");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));

        led_off(led);
    }
}

static void set_alarm_armed(alarm_system_handle_t handle) {
    
    xQueueReset(handle->app_queue);
    
    char menu_options[33];
    if(!is_armed) {
        snprintf(menu_options, sizeof(menu_options), "ARM SYSTEM\nENTER PIN: ");
    }
    else if (is_armed) {
        snprintf(menu_options, sizeof(menu_options), "DISARM SYSTEM\nENTER PIN: ");
    }

    lcd_1602_send_string(handle->lcd_i2c_handle, menu_options);

    bool is_auth = check_authorized(handle);

    led_handle_t temp;
    led_off(handle->orange_led);

    if(is_auth) {
        temp = handle->green_led;
        led_on(temp);
        lcd_1602_send_string(handle->lcd_i2c_handle, "Welcome!");
        wrong_entries = 0;
    }
    else {
        temp = handle->red_led;
        led_on(temp);
        lcd_1602_send_string(handle->lcd_i2c_handle, "Not Authorized!");
        wrong_entries++;
    }

    if(wrong_entries >= 3 && state != ALARM_TRIGGERED) {
        state = ALARM_TRIGGERED;
        if(s_alarm_task == NULL) {
            xTaskCreate(alarm_triggered_routine, "alarm_trigger_routine", 2048, handle, 5, &s_alarm_task);
        }
    }

    if(is_auth) {
        is_armed = !is_armed;
        am312_set_armed(handle->sensor, is_armed);
        state = ALARM_SLEEP;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    led_off(temp);

    lcd_1602_send_string(handle->lcd_i2c_handle, " ");
}

static void alarm_wakeup(alarm_system_handle_t handle) {
    char menu_options[33];
    if(!is_armed) {
        snprintf(menu_options, sizeof(menu_options), "A:Arm B:New\nC:Cancel");
    }
    else if (is_armed) {
        snprintf(menu_options, sizeof(menu_options), "A:Disarm B:New\nC:Cancel");
    }

    lcd_1602_send_string(handle->lcd_i2c_handle, menu_options);
    led_on(handle->orange_led);

    app_handle_t event;

    while(1) {
        lcd_1602_send_string(handle->lcd_i2c_handle, " ");

        if(!xQueueReceive(handle->app_queue, &event, pdMS_TO_TICKS(WAIT_TIME_MS))) continue;

        if(event.event_type == EV_CHAR_RECEIVED) {
            if (event.key.key_pressed == 'A') {
                set_alarm_armed(handle);
                break;
            }
            else if (event.key.key_pressed == 'B') {
                alarm_add_user(handle);
                break;
            }
            else if (event.key.key_pressed == 'C') {
                lcd_1602_send_string(handle->lcd_i2c_handle, " ");
                led_off(handle->orange_led);
                break;
            }
        }
    }
}

void alarm_check_in_routine(void *args) {
    alarm_system_handle_t handle = (alarm_system_handle_t) args;
    const TickType_t check_in_period = pdMS_TO_TICKS(CHECK_IN_PERIOD_MS);
    const TickType_t wifi_period    = pdMS_TO_TICKS(WIFI_CHECK_PERIOD);

    TickType_t last_wifi     = xTaskGetTickCount();
    TickType_t last_check_in = last_wifi;

    for (;;) {
        led_handle_t led = wifi_is_connected() ? handle->green_led : handle->red_led;
        led_on(led);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_off(led);

        TickType_t now = xTaskGetTickCount();
        if ((now - last_check_in) >= check_in_period) {
            app_handle_t event = {
                .event_type = EV_CHECK_IN
            };

            alarm_send_to_server(handle, ALARM_CHECK_IN, event);

            last_check_in += check_in_period;
        }

        vTaskDelayUntil(&last_wifi, wifi_period);
    }
}

void run_alarm(alarm_system_handle_t handle) {
    xTaskCreate(_4x4_matrix_task, "_4x4_matrix_task", 1024, NULL, 5, NULL);
    
    app_handle_t event;
    
    while(1) {
        xQueueReset(handle->app_queue);
        if(!xQueueReceive(handle->app_queue, &event, portMAX_DELAY)) continue;
        
        if(event.event_type == EV_CHAR_RECEIVED) {
            alarm_wakeup(handle);
        }
        else if(event.event_type == EV_MOTION) {
            if(state != ALARM_TRIGGERED) {
                state = ALARM_TRIGGERED;
                if(s_alarm_task == NULL) {
                    xTaskCreate(alarm_triggered_routine, "alarm_trigger_routine", 2048, handle, 5, &s_alarm_task);
                }
            }
        }
    }
}

uint8_t init_alarm(alarm_system_handle_t *out_handle) {
    
    if (!out_handle || *out_handle != NULL) return 1;

    alarm_system_handle_t h_temp = (alarm_system_handle_t)calloc(1, sizeof(alarm_system_handle));
    bool i2c_init = false;
    if (h_temp == NULL) goto _failed;

    h_temp->app_queue = xQueueCreate(16, sizeof(app_handle_t));
    if(!h_temp->app_queue) goto _failed;

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
    _4x4_matrix_init(row_pins, col_pins, h_temp->app_queue);

    // LEDs init
    h_temp->green_led = led_init(GPIO_NUM_27);
    h_temp->orange_led = led_init(GPIO_NUM_14);
    h_temp->red_led = led_init(GPIO_NUM_12);

    if(!(h_temp->green_led && h_temp->orange_led && h_temp->red_led)) {
        ESP_LOGE(TAG, "Unable to initialize all leds");
        goto _failed;
    }

    if(rc522_init(&(h_temp->rc522), h_temp->app_queue) != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize rc522");
        goto _failed;
    }

    if(h_temp->rc522 == NULL) ESP_LOGE(TAG, "Unable to initialize rc522");
    
    h_temp->sensor = am312_init(GPIO_NUM_13, motion_cb, h_temp, 2500, 250);
    
    if(h_temp->sensor == NULL) {
        ESP_LOGE(TAG, "Unable to initialize motions sensor");
        goto _failed;
    }

    wifi_init();
    wait_for_connection();

    if(!wifi_is_connected()) goto _failed;

    *out_handle = h_temp;

    xTaskCreate(alarm_check_in_routine, "alarm_check_in", 2048, h_temp, 5, NULL);

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
    if(h_temp->app_queue) {
        QueueHandle_t temp = h_temp->app_queue;
        h_temp->app_queue = NULL;
        vQueueDelete(temp);
    }

    if(h_temp) free(h_temp);

    return 1;
}