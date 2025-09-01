#ifndef HTTP_IMPLEMENTATION_H
#define HTTP_IMPLEMENTATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define HTTP_MAX_URL_SIZE           255
#define MAX_HTTP_OUTPUT_BUFFER      2048
#define HTTP_REQUEST_BODY_SIZE      256
#define HTTP_POST_TASK_STACK_SIZE   4096

typedef struct {
    char url[HTTP_MAX_URL_SIZE];
    TaskHandle_t caller;
    esp_err_t status;
    char response_buffer[MAX_HTTP_OUTPUT_BUFFER];
    char post_data[HTTP_REQUEST_BODY_SIZE];
} http_request_args_t;

void http_post_task(void *pvParameters);
void http_get_task(void *pvParameters);

#endif