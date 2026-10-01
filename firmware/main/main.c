/* MavLTE: flight controller UART <-> relay server over the A7670E's mobile data. The ESP32's own Wi-Fi
 * and Bluetooth stay off: nothing starts them, and the build leaves out the libraries that could
 * (CONFIG_APP_NO_BLOBS in sdkconfig.defaults). */
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "battery.h"
#include "board.h"
#include "bridge.h"
#include "modem.h"
#include "status.h"
#include "version.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "MavLTE %s", FIRMWARE_VERSION);
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    board_get();
#if CONFIG_BRIDGE_LOCATOR
    battery_start();
#endif
    bridge_start();
    status_start();
    modem_start();
}
