#include "http_implementation.h"
#include "credentials.h"

#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "rc522_picc.h"

static const char *HTTP_TAG = "http_request";

esp_err_t _http_event_handler(esp_http_client_event_t *event) {
    
    switch(event->event_id) {
        case HTTP_EVENT_ON_DATA:
            if(!esp_http_client_is_chunked_response(event->client)) {
                strncat((char*) event->user_data, (const char *) event->data, event->data_len);
            }
            break;
        default:
            break;
    }

    return ESP_OK;
}

void http_get_task(void *pvParameters) {
    http_request_args_t *args = (http_request_args_t*) pvParameters;

    memset(args->response_buffer, 0, sizeof(args->response_buffer));

    ESP_LOGI(HTTP_TAG, "Sending GET request to: %s", args->url);

    esp_http_client_config_t config = {
        .url = args->url,
        .event_handler = _http_event_handler,
        .user_data = args->response_buffer
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_err_t ret = esp_http_client_perform(client);

    if(ret == ESP_OK) {
        ESP_LOGI(HTTP_TAG, "HTTP GET Status = %d", esp_http_client_get_status_code(client));
        ESP_LOGI(HTTP_TAG, "Raw response: %s", args->response_buffer);
        // parseJSON(args->response_buffer);

        if(esp_http_client_get_status_code(client) == 200) {
            args->status = ret;
        }

    } else {
        ESP_LOGE(HTTP_TAG, "HTTP GET request failed: %s", esp_err_to_name(ret));
        args->status = ret;
    }

    esp_http_client_cleanup(client);
    xTaskNotifyGive(args->caller);
    vTaskDelete(NULL);
}

void http_post_task(void *pvParameters) {
    http_request_args_t *args = (http_request_args_t*) pvParameters;
    memset(args->response_buffer, 0, sizeof(args->response_buffer));

    esp_http_client_config_t config = {
        .url = args->url,
        .event_handler = _http_event_handler,
        .user_data = args->response_buffer
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, args->post_data, strlen(args->post_data));

    esp_err_t ret = esp_http_client_perform(client);

    if(ret == ESP_OK) {
        ESP_LOGI(HTTP_TAG, "POST Status: %d", esp_http_client_get_status_code(client));
        ESP_LOGI(HTTP_TAG, "Raw response: %s", args->response_buffer);

        if(esp_http_client_get_status_code(client) == 200) {
            args->status = ret;
        }
    } else {
        ESP_LOGE(HTTP_TAG, "POST request failed: %s", esp_err_to_name(ret));
        args->status = ret;
    }

    esp_http_client_cleanup(client);
    xTaskNotifyGive(args->caller);
    vTaskDelete(NULL);
}