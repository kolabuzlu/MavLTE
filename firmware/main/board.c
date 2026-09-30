#include "board.h"

#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "board";
static board_t board;

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
        board.version = board_version();
        board.modem_power_gpio = board.version == 1 ? 33 : 21;
        /* pins that are free on the header of each version */
        board.fc_tx_gpio = CONFIG_BRIDGE_FC_TX_GPIO >= 0 ? CONFIG_BRIDGE_FC_TX_GPIO : board.version == 1 ? 41 : 2;
        board.fc_rx_gpio = CONFIG_BRIDGE_FC_RX_GPIO >= 0 ? CONFIG_BRIDGE_FC_RX_GPIO : board.version == 1 ? 42 : 3;
        ESP_LOGI(TAG, "Waveshare ESP32-S3-A7670E-4G, board version V%d", board.version);
    }
    return &board;
}
