/* Waveshare ESP32-S3-A7670E-4G pin map. The board exists in two versions:
 *   V1 (until about 12/2025): ESP32-S3R2, modem power enable on GPIO33
 *   V2 (since about 12/2025): ESP32-S3R8, modem power enable on GPIO21, "VER 2.0" on the back */
#pragma once

#define BOARD_MODEM_TX_GPIO 18 /* ESP32 TX -> A7670E RXD, both versions */
#define BOARD_MODEM_RX_GPIO 17 /* ESP32 RX <- A7670E TXD, both versions */

typedef struct {
    int version;          /* 1 or 2 */
    int modem_power_gpio; /* modem load switch, active high, OR'ed with DIP switch "4G" */
    int fc_tx_gpio;       /* ESP32 TX -> flight controller RX */
    int fc_rx_gpio;       /* ESP32 RX <- flight controller TX */
} board_t;

/* The board version comes from menuconfig or, by default, from the chip: only V1 uses an
 * ESP32-S3R2 (2 MB PSRAM), only V2 an ESP32-S3R8 (8 MB). */
const board_t *board_get(void);
