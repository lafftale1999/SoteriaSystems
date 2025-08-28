#ifndef _ALARM_SYSTEM_H_
#define _ALARM_SYSTEM_H_

#define RC522_PICC_UID_STR_BUFFER_SIZE_MAX 30

typedef struct alarm_system_handle* alarm_system_handle_t;

void run_alarm(alarm_system_handle_t handle);
uint8_t init_alarm(alarm_system_handle_t handle);

#endif