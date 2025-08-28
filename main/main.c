#include <stdio.h>
#include "alarm_system.h"

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <string.h>

const char* TAG = "MAIN";

int app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    alarm_system_handle_t alarm_system;
    if (init_alarm(alarm_system) != 0) {
        ESP_LOGE(TAG, "Unable to initialize alarm system");
    }

    wifi_init();
    wait_for_connection();

    while(true) {
        run_alarm(alarm_system);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}