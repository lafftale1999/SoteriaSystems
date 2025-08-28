#include "am312.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define TAG "AM312"

#ifndef AM312_SHARED_QUEUE_LEN
#define AM312_SHARED_QUEUE_LEN 16
#endif
#ifndef AM312_TASK_STACK
#define AM312_TASK_STACK 2048
#endif
#ifndef AM312_TASK_PRIO
#define AM312_TASK_PRIO 10
#endif

#ifndef AM312_MIN_SETTLE_MS
#define AM312_MIN_SETTLE_MS 2000
#endif

#ifndef AM312_MIN_DEBOUNCE_MS
#define AM312_MIN_DEBOUNCE_MS 200
#endif

typedef enum {
    EV_PULSE,
    EV_DELETE
} am312_ev_type_t;

typedef struct{
    am312_ev_type_t event_type;
    struct am312_handle *handle;
    int gpio_level;
    TickType_t tick;
} am312_event_t;

struct am312_handle {
    gpio_num_t pin;
    am312_cb_t cb_func;
    void *user;
    volatile bool is_armed;
    volatile bool is_alive;
    TickType_t armed_since;
    TickType_t last_tick;
    TickType_t settle_ticks;
    TickType_t debounce_ticks;
    QueueHandle_t app_queue;
};

static QueueHandle_t am312_queue = NULL;
static TaskHandle_t am312_task_handle = NULL;
static size_t am312_handle_count = 0;

static void IRAM_ATTR am312_isr(void *arg);
static void am312_task(void *arg);

static bool am312_shared_init_once() {
    if(!am312_queue) {
        am312_queue = xQueueCreate(AM312_SHARED_QUEUE_LEN, sizeof(am312_event_t));
        if (!am312_queue) return false;
    }

    if(!am312_task_handle) {
        if(xTaskCreate(am312_task, "am312_task", AM312_TASK_STACK, NULL, AM312_TASK_PRIO, &am312_task_handle) != pdPASS) {
            vQueueDelete(am312_queue);
            am312_queue = NULL;
            return false;
        }
    }

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return false;
    }

    return true;
}

am312_handle_t am312_init(gpio_num_t pin, am312_cb_t cb_func, void *user, uint32_t settle_ms, uint32_t debounce_ms) {
    if(!am312_shared_init_once()) return NULL;

    am312_handle_t handle = calloc(1, sizeof(*handle));
    if(!handle) return NULL;

    handle->pin = pin;
    handle->cb_func = cb_func;
    handle->user = user;
    handle->settle_ticks = pdMS_TO_TICKS(settle_ms > AM312_MIN_SETTLE_MS ? settle_ms : AM312_MIN_SETTLE_MS);
    handle->debounce_ticks = pdMS_TO_TICKS(debounce_ms > AM312_MIN_DEBOUNCE_MS ? debounce_ms : AM312_MIN_DEBOUNCE_MS);
    handle->is_alive = true;
    handle->is_armed = false;

    gpio_config_t cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };

    if(gpio_config(&cfg) != ESP_OK) {
        free(handle);
        return NULL;
    }

    if(gpio_isr_handler_add(pin, am312_isr, handle) != ESP_OK) {
        free(handle);
        return NULL;
    }

    gpio_intr_disable(pin);
    am312_handle_count++;
    return handle;
}

uint8_t am312_set_armed(am312_handle_t handle, bool armed) {
    if(!handle || !handle->is_alive) return 1;

    if(armed) {
        handle->armed_since = xTaskGetTickCount();
        handle->last_tick = 0;
        handle->is_armed = true;
        gpio_intr_enable(handle->pin);
    }
    else {
        gpio_intr_disable(handle->pin);
        handle->is_armed = false;
    }

    return 0;
}

uint8_t am312_destroy(am312_handle_t handle) {
    if(!handle) return 1;
    gpio_intr_disable(handle->pin);
    gpio_isr_handler_remove(handle->pin);

    handle->is_armed = false;
    handle->is_alive = false;

    if(am312_queue) {
        am312_event_t event = {
            .event_type = EV_DELETE,
            .handle = handle
        };

        (void)xQueueSend(am312_queue, &event, pdMS_TO_TICKS(50));
    }
    else {
        free(handle);
    }

    if(am312_handle_count > 0) am312_handle_count--;
    if(am312_handle_count == 0) {
        am312_shared_shutdown();
    }

    return 0;
}

uint8_t am312_shared_shutdown(void) {
    if(am312_task_handle) {
        TaskHandle_t temp = am312_task_handle;
        am312_task_handle = NULL;
        vTaskDelete(temp);
    }

    if(am312_queue) {
        QueueHandle_t temp = am312_queue;
        am312_queue = NULL;
        vQueueDelete(temp);
    }

    return 0;
}

static void IRAM_ATTR am312_isr(void *arg) {
    am312_handle_t handle = (am312_handle_t) arg;

    if(!handle || !handle->is_alive || !handle->is_armed) return;

    am312_event_t event = {
        .event_type = EV_PULSE,
        .handle = handle,
        .gpio_level = gpio_get_level(handle->pin),
        .tick = xTaskGetTickCountFromISR()
    };

    BaseType_t higher_priority = pdFALSE;

    if(am312_queue) xQueueSendFromISR(am312_queue, &event, &higher_priority);
    if(higher_priority) portYIELD_FROM_ISR();
}

static void am312_task(void *arg) {
    am312_event_t event;

    while(1) {
        if(!xQueueReceive(am312_queue, &event, portMAX_DELAY)) continue;

        if(event.event_type == EV_DELETE) {
            free(event.handle);
            continue;
        }

        am312_handle_t handle = event.handle;

        if(!handle || !handle->is_alive || !handle->is_armed) continue;
        if((event.tick - handle->armed_since) < handle->settle_ticks) continue;
        if((event.tick - handle->last_tick) < handle->debounce_ticks) continue;

        handle->last_tick = event.tick;

        if(event.gpio_level == 1 && handle->cb_func) {
            handle->cb_func(handle->user);
        }
    }
}