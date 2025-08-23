#include "motion_sensor.h"

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"

// heap allokera en handle för sensorn. SKa egentligen bara trigga ifall någon rör sig framför den.
// en pin som styr ifall den är på eller av - en pin som läser av ifall den triggas eller inte.
//

static void IRAM_ATTR gpio_isr_handler(void *arg) {

}

uint8_t AM312_init(uint8_t pin, void (*intr_func)()) {

}