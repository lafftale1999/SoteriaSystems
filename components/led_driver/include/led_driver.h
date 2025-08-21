#ifndef _LED_DRIVER_H_
#define _LED_DRIVER_H_

#include <stdint.h>

typedef struct led_handle *led_handle_t;

void led_blink(led_handle_t handle);
void led_on(led_handle_t handle);
void led_off(led_handle_t handle);
led_handle_t led_init(uint8_t pin);

#endif