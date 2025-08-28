#include "alarm_system.h"
#include "4x4_matrix.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>

typedef enum {
    ALARM_ARMED,
    ALARM_DISARMED,
    ALARM_WAITING
} alarm_state;

typedef enum {
    EV_PIN,
    EV_RFID,
    EV_MOTION,
    EV_TIMEOUT
} app_event_e;

typedef struct {
    app_event_e event_type;
    union {
        struct {
            char key_pressed;
        } key;
        struct {
            char uid[10];
            size_t len;
        } rfid;

        struct {
            uint8_t id;
        } motion;
    };
} app_handle_t;


static alarm_state state = ALARM_DISARMED;
QueueHandle_t app_queue;

// skapa tasks för varje periferi och ta emot callbacks
// starta tasks beroende på vilket state larmet befinner sig i

// passa in buf och char_len (struct) som argument till keypad task och till lcd task
// lcd kontrollerar ifall nu char har kommit och skriver ut den isåfall.

// i main väntar vi att någon sätter antingen rfid bit eller pin bit
void run_menu(void) {
    char menu_options[33];
    if(state == ALARM_DISARMED) {
        snprintf(menu_options, sizeof(menu_options), "A: Arm | B: New\nC: Cancel");
    }
    else if (state == ALARM_ARMED) {
        snprintf(menu_options, sizeof(menu_options), "A Disarm | B: New\nC: Cancel");
    }

    // orange LED tänd
    // starta scanner task
    // starta LED task
    // starta keypad task
}

void run_alarm(void) {
    char menu_choice;

    while(1) {
        menu_choice = _4x4_matrix_get_key_press();

        if(menu_choice == 'a') {
            // run menu
        }
    }
}