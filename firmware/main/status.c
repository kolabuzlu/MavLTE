#include "status.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include "bridge.h"

#define LED_GPIO 38
#define LEVEL 16 /* of 255: the bare LED is very bright */

static const char *TAG = "status";
static volatile status_modem_t modem_state = STATUS_MODEM_STARTING;
static led_strip_handle_t led;

void status_set_modem(status_modem_t state)
{
    modem_state = state;
}

static void status_task(void *arg)
{
    /* a lamp test at power-on, while the modem starts up: each colour once, from worst to best, a second each */
    static const uint8_t test[][3] = {{LEVEL, 0, 0}, {LEVEL, LEVEL, 0}, {LEVEL, 0, LEVEL}, {0, 0, LEVEL}};
    for (size_t i = 0; i < sizeof(test) / sizeof(test[0]); i++) {
        led_strip_set_pixel(led, 0, test[i][0], test[i][1], test[i][2]);
        led_strip_refresh(led);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    uint32_t shown = UINT32_MAX;
    for (unsigned tick = 0;; tick++) { /* 100 ms per tick */
        bool blink = tick % 10 < 5;
        bridge_state_t link = bridge_state();
        uint8_t r = 0, g = 0, b = 0;
        /* from worst to best: red, yellow, purple, blue */
        if (modem_state == STATUS_MODEM_ERROR) {
            r = LEVEL; /* red: modem, SIM or network problem */
        } else if (modem_state == STATUS_MODEM_STARTING) {
            r = blink ? LEVEL : 0; /* red, blinking: no mobile data yet */
        } else if (!link.relay) {
            r = g = LEVEL; /* yellow: mobile data up, but the relay does not answer */
        } else if (!link.fc) {
            r = b = LEVEL; /* purple: connected to the relay, but no flight controller talks */
        } else {
            b = LEVEL; /* blue: ready to fly, a GCS connected or not */
        }
        uint32_t color = (uint32_t)r << 16 | (uint32_t)g << 8 | b;
        if (color != shown) {
            shown = color;
            led_strip_set_pixel(led, 0, r, g, b);
            led_strip_refresh(led);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void status_start(void)
{
    const led_strip_config_t config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        /* red first: the board's WS2812B-0807 (V1 and V2) takes its colours in that order, not in a WS2812B's usual
         * green first (seen on a V2 board: purple came out cyan, and green red) */
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
    };
    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    if (led_strip_new_rmt_device(&config, &rmt_config, &led) != ESP_OK) {
        ESP_LOGW(TAG, "status LED not available");
        return;
    }
    xTaskCreate(status_task, "status", 3072, NULL, 2, NULL);
}
