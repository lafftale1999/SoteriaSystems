#ifndef _RC522_IMPLEMENTATION_H_
#define _RC522_IMPLEMENTATION_H_

#include <stdint.h>

#include "esp_err.h"
#include "rc522.h"

uint8_t rc522_init(rc522_handle_t *out, QueueHandle_t owner_queue);

#endif