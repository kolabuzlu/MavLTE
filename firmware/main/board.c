#include "board.h"

#include <stdbool.h>

#include "driver/temperature_sensor.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "board";
static board_t board;
static SemaphoreHandle_t temp_lock; /* the temperature sensor: the bridge and the flight log both read it */

static int board_version(void)
{
#if CONFIG_BRIDGE_BOARD_V1
    return 1;
#elif CONFIG_BRIDGE_BOARD_V2
    return 2;
#else
    uint8_t psram = 0; /* eFuse PSRAM_CAP: 1 = 8 MB, 2 = 2 MB */
    esp_efuse_read_field_blob(ESP_EFUSE_PSRAM_CAP, &psram, esp_efuse_get_field_size(ESP_EFUSE_PSRAM_CAP));
    if (psram == 2) {
        return 1;
    }
    if (psram == 1) {
        return 2;
    }
    ESP_LOGE(TAG, "cannot tell the board version from this chip (PSRAM code %u); assuming V2. "
                  "Set it in menuconfig -> MavLTE -> Board version.", psram);
    return 2;
#endif
}

const board_t *board_get(void)
{
    if (!board.version) {
        temp_lock = xSemaphoreCreateMutex();
        board.version = board_version();
        board.modem_power_gpio = board.version == 1 ? 33 : 21;
        /* pins that are free on the header of each version */
        board.fc_tx_gpio = CONFIG_BRIDGE_FC_TX_GPIO >= 0 ? CONFIG_BRIDGE_FC_TX_GPIO : board.version == 1 ? 41 : 2;
        board.fc_rx_gpio = CONFIG_BRIDGE_FC_RX_GPIO >= 0 ? CONFIG_BRIDGE_FC_RX_GPIO : board.version == 1 ? 42 : 3;
        ESP_LOGI(TAG, "Waveshare ESP32-S3-A7670E-4G, board version V%d", board.version);
    }
    return &board;
}

static int8_t read_chip_temp(void)
{
    static temperature_sensor_handle_t sensor;
    static bool failed;
    if (!sensor && !failed) {
        /* the most exact of its ranges (1 degree); the driver moves to another by itself when the chip
         * gets hotter or colder than that */
        const temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        if (temperature_sensor_install(&config, &sensor) != ESP_OK) {
            sensor = NULL;
            failed = true;
        } else if (temperature_sensor_enable(sensor) != ESP_OK) {
            temperature_sensor_uninstall(sensor);
            sensor = NULL;
            failed = true;
        }
        if (failed) {
            ESP_LOGW(TAG, "the chip's temperature sensor does not start");
        }
    }
    float c;
    if (!sensor || temperature_sensor_get_celsius(sensor, &c) != ESP_OK) {
        return INT8_MIN;
    }
    if (c >= 127.0f) {
        return 127;
    }
    if (c <= -127.0f) {
        return -127;
    }
    return (int8_t)(c < 0.0f ? c - 0.5f : c + 0.5f);
}

int8_t board_chip_temp(void)
{
    if (!temp_lock) {
        return INT8_MIN;
    }
    xSemaphoreTake(temp_lock, portMAX_DELAY);
    int8_t c = read_chip_temp();
    xSemaphoreGive(temp_lock);
    return c;
}
