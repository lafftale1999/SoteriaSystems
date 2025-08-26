#ifndef MOTION_SENSOR_DRIVER_H_
#define MOTION_SENSOR_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct am312_handle *am312_handle_t;
typedef void (*am312_cb_t)(void *user);

am312_handle_t am312_init(gpio_num_t pin, am312_cb_t cb_func, void *user, uint32_t settle_ms, uint32_t debounce_ms);
uint8_t am312_set_armed(am312_handle_t handle, bool armed);
uint8_t am312_destroy(am312_handle_t handle);
uint8_t am312_shared_shutdown(void);

#ifdef __cpluscplusd
}
#endif

#endif