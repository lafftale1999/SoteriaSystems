#ifndef MOTION_SENSOR_DRIVER_H_
#define MOTION_SENSOR_DRIVER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t AM312_init(uint8_t pin, void (*intr_func)());

#ifdef __cpluscplus
extern "C" }
#endif

#endif