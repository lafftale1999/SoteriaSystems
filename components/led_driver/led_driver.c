#include "led_driver.h"
#include "driver/gpio.h"

typedef struct led_handle {
    uint8_t is_on;
    uint8_t pin;
};

void led_blink(led_handle_t handle)
{
    handle->is_on ^= 1;
    gpio_set_level(handle->pin, handle->is_on);
}

void led_on(led_handle_t handle) {
    handle->is_on = 1;
    gpio_set_level(handle->pin, handle->is_on);
}

void led_off(led_handle_t handle) {
    handle->is_on = 0;
    gpio_set_level(handle->pin, handle->is_on);
}

led_handle_t led_init(uint8_t pin) {
    
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        return NULL;
    }
    
    led_handle_t handle = (led_handle_t) malloc(sizeof *handle);

    if (handle == NULL) {
        return NULL;
    }

    handle->pin = pin;
    handle->is_on = 0;

    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    if (gpio_config(&config) != ESP_OK) {
        free(handle);
        return NULL;
    }

    return handle;
}
