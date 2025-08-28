#include <esp_log.h>
#include "driver/rc522_spi.h"
#include "rc522_picc.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "app_events.h"

static const char *TAG = "rc522";

#define RC522_SPI_BUS_GPIO_MISO    GPIO_NUM_19
#define RC522_SPI_BUS_GPIO_MOSI    GPIO_NUM_23
#define RC522_SPI_BUS_GPIO_SCLK    GPIO_NUM_18
#define RC522_SPI_SCANNER_GPIO_SDA GPIO_NUM_5
#define RC522_SCANNER_GPIO_RST     (-1) // soft-reset

static rc522_spi_config_t driver_config = {
    .host_id = SPI3_HOST,
    .bus_config = &(spi_bus_config_t){
        .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    },
    .dev_config = {
        .spics_io_num = RC522_SPI_SCANNER_GPIO_SDA,
    },
    .rst_io_num = RC522_SCANNER_GPIO_RST,
};

static rc522_driver_handle_t driver;
static QueueHandle_t app_queue;
static bool rc522_is_created = false;

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state == RC522_PICC_STATE_ACTIVE) {
        app_handle_t app_event = {
            .event_type = EV_RFID,
        };

        rc522_picc_uid_to_str(&picc->uid, app_event.rfid.uid, sizeof(app_event.rfid.uid));

        xQueueSend(app_queue, &app_event, portMAX_DELAY);
    }
    else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "Card has been removed");
    }
}



uint8_t rc522_init(rc522_handle_t scanner, QueueHandle_t owner_queue)
{
    if(!rc522_is_created) {
        ESP_ERROR_CHECK(rc522_spi_create(&driver_config, &driver));
        ESP_ERROR_CHECK(rc522_driver_install(driver));

        rc522_config_t scanner_config = {
            .driver = driver,
        };

        ESP_ERROR_CHECK(rc522_create(&scanner_config, &scanner));
        ESP_ERROR_CHECK(rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL));

        rc522_is_created = true;
    }

    if(!app_queue && owner_queue != NULL) {
        app_queue = owner_queue;
    }
    
    return 0;
}
