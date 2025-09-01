#ifndef _ALARM_SYSTEM_H_
#define _ALARM_SYSTEM_H_

#define RC522_PICC_UID_STR_BUFFER_SIZE_MAX 30

#include <stdint.h>

#define ALARM_TRIGGERED_JSON_LEN        2
#define ALARM_CHECK_IN_JSON_LEN         2
#define ALARM_ARMED_DISARMED_JSON_LEN   4

typedef struct alarm_system_handle* alarm_system_handle_t;

void run_alarm(alarm_system_handle_t handle);
uint8_t init_alarm(alarm_system_handle_t* out_handle);


#endif