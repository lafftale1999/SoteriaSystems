#pragma once

#include <stdint.h>
#include <stdlib.h>

typedef enum app_event_e{
    EV_PIN,
    EV_RFID,
    EV_MOTION,
    EV_TIMEOUT,
    EV_CHAR_RECEIVED
} app_event_e;

typedef struct {
    app_event_e event_type;
    union {
        struct {
            char key_pressed;
        } key;
        
        struct {
            char uid[30];
            size_t len;
        } rfid;

        struct {
            uint8_t id;
        } motion;
    };
} app_handle_t;