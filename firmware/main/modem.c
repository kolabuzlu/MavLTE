#include "modem.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs.h"

#include "board.h"
#include "bridge.h"
#include "status.h"

#define MODEM_UART UART_NUM_1
#define BOOT_BAUD 115200 /* the A7670E's rate after power-up; AT+IPR changes are not saved */
#define PPP_UP BIT0
#define PPP_DOWN BIT1
#define RELAY_SILENCE_LIMIT_MS (3 * 60 * 1000)
#define NVS_NAMESPACE "modem"
#define NVS_BAD_PIN "bad_pin"     /* the menuconfig SIM PIN that the SIM card rejected */
#define NVS_SLOW_UART "slow_uart" /* 1: the modem did not answer at CONFIG_BRIDGE_MODEM_BAUD */

#if CONFIG_BRIDGE_NETWORK_LTE_ONLY
#define NETWORK_MODE 38 /* AT+CNMP: LTE only */
#else
#define NETWORK_MODE 2 /* AT+CNMP: automatic */
#endif

static const char *TAG = "modem";

static const board_t *board;
static esp_netif_t *ppp_netif;
static esp_modem_dce_t *dce;
static EventGroupHandle_t events;
static bool power_switchable; /* DIP switch "4G" is off, so the firmware controls the modem's power */
static bool pin_rejected;     /* never retry a wrong SIM PIN: three tries lock the SIM */
static bool fast_baud_failed; /* the modem took AT+IPR but did not answer at that rate: stay at BOOT_BAUD */
static int baud = BOOT_BAUD;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_PPP_GOT_IP) {
        xEventGroupClearBits(events, PPP_DOWN);
        xEventGroupSetBits(events, PPP_UP);
    } else if (id == IP_EVENT_PPP_LOST_IP) {
        xEventGroupClearBits(events, PPP_UP);
        xEventGroupSetBits(events, PPP_DOWN);
    }
}

/* PPP errors that end the link without IP_EVENT_PPP_LOST_IP, e.g. the modem stops answering
 * LCP echo requests. NETIF_PPP_ERRORUSER is our own hang-up. */
static void on_ppp_status(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id > NETIF_PPP_ERRORNONE && id < NETIF_PP_PHASE_OFFSET && id != NETIF_PPP_ERRORUSER) {
        ESP_LOGW(TAG, "PPP error %" PRId32, id);
        xEventGroupClearBits(events, PPP_UP);
        xEventGroupSetBits(events, PPP_DOWN);
    }
}

/* ---- power */

static void power_init(void)
{
    const gpio_num_t pin = (gpio_num_t)board->modem_power_gpio;
    const gpio_config_t input = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    /* With DIP switch "4G" on, the switch holds the enable line high; driving it low would fight
     * the switch, so then we leave the modem's power alone. */
    gpio_config(&input);
    esp_rom_delay_us(200);
    if (gpio_get_level(pin)) {
        power_switchable = false;
        ESP_LOGW(TAG, "DIP switch \"4G\" is ON, so the modem cannot be power-cycled if it hangs. "
                      "Turn it OFF to let the firmware control the modem's power.");
        return;
    }
    power_switchable = true;
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "modem power on (GPIO%d)", (int)pin);
}

static void power_cycle(void)
{
    if (!power_switchable) {
        return;
    }
    ESP_LOGW(TAG, "power-cycling the modem");
    gpio_set_level((gpio_num_t)board->modem_power_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));
    gpio_set_level((gpio_num_t)board->modem_power_gpio, 1);
}

/* ---- esp_modem */

static bool create_dce(void)
{
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.port_num = MODEM_UART;
    dte_config.uart_config.baud_rate = BOOT_BAUD;
    dte_config.uart_config.tx_io_num = BOARD_MODEM_TX_GPIO;
    dte_config.uart_config.rx_io_num = BOARD_MODEM_RX_GPIO;
    dte_config.uart_config.rts_io_num = UART_PIN_NO_CHANGE; /* RTS/CTS are not wired on this board */
    dte_config.uart_config.cts_io_num = UART_PIN_NO_CHANGE;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_config.uart_config.rx_buffer_size = 8192;
    dte_config.uart_config.tx_buffer_size = 2048;
    dte_config.uart_config.event_queue_size = 32;
    dte_config.dte_buffer_size = 2048;
    dte_config.task_stack_size = 4096;
    dte_config.task_priority = 13; /* above the bridge tasks: keep the modem's RX drained */
    const esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CONFIG_BRIDGE_APN);
    dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600, &dte_config, &dce_config, ppp_netif);
    baud = BOOT_BAUD;
    if (!dce) {
        ESP_LOGE(TAG, "cannot set up the modem UART");
        return false;
    }
    return true;
}

/* The modem answers about 8 s after power-up, at 115200 baud. If only the ESP32 restarted, the
 * modem may still run at the faster rate, so try both. */
static bool sync_modem(uint32_t timeout_ms)
{
    const int rates[2] = {BOOT_BAUD, CONFIG_BRIDGE_MODEM_BAUD};
    uint32_t start = now_ms();
    for (int i = 0; (uint32_t)(now_ms() - start) < timeout_ms; i++) {
        int rate = rates[i % 2];
        if (rate != baud) {
            uart_set_baudrate(MODEM_UART, rate);
            baud = rate;
        }
        if (esp_modem_sync(dce) == ESP_OK) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

/* PPP on a 115200 baud link tops out below the flight controller's 115200 baud telemetry, so the
 * modem UART runs faster. AT+IPR only lasts until the modem restarts. Returns false if the modem
 * is left at a rate it cannot be reached at, so that only a reset brings it back. */
static bool set_fast_baud(void)
{
    if (baud == CONFIG_BRIDGE_MODEM_BAUD || fast_baud_failed) {
        return true;
    }
    if (esp_modem_set_baud(dce, CONFIG_BRIDGE_MODEM_BAUD) != ESP_OK) {
        /* No OK, so it did not switch. Had it switched anyway, sync_modem() finds it at either rate. */
        ESP_LOGW(TAG, "modem did not switch to %d baud; staying at %d", CONFIG_BRIDGE_MODEM_BAUD, BOOT_BAUD);
        return true;
    }
    /* It said OK and switched: from here on the ESP32 may only change rate together with the modem */
    uart_set_baudrate(MODEM_UART, CONFIG_BRIDGE_MODEM_BAUD);
    baud = CONFIG_BRIDGE_MODEM_BAUD;
    vTaskDelay(pdMS_TO_TICKS(100));
    for (int i = 0; i < 5; i++) {
        if (esp_modem_sync(dce) == ESP_OK) {
            ESP_LOGI(TAG, "modem UART at %d baud", baud);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    /* Remembered in flash: with DIP "4G" on, a modem stuck at a rate the link cannot carry only
     * comes back with a power cycle of the whole board, after which it must not happen again. */
    fast_baud_failed = true;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_SLOW_UART, 1);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGW(TAG, "the modem does not answer at %d baud; using %d from now on (erase the flash to try again)",
             CONFIG_BRIDGE_MODEM_BAUD, BOOT_BAUD);
    esp_modem_set_baud(dce, BOOT_BAUD); /* reaches it if only its answers get lost at the fast rate */
    uart_set_baudrate(MODEM_UART, BOOT_BAUD);
    baud = BOOT_BAUD;
    vTaskDelay(pdMS_TO_TICKS(100));
    return esp_modem_sync(dce) == ESP_OK;
}

/* A wrong PIN counts on the SIM card across restarts, and three lock it: once the SIM rejects the
 * PIN from menuconfig, it is remembered in flash and not sent again, not even after a restart. */
static bool pin_known_bad(void)
{
    char stored[32];
    size_t len = sizeof(stored);
    nvs_handle_t nvs;
    if (pin_rejected) {
        return true;
    }
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false; /* nothing stored yet */
    }
    bool bad = nvs_get_str(nvs, NVS_BAD_PIN, stored, &len) == ESP_OK && strcmp(stored, CONFIG_BRIDGE_SIM_PIN) == 0;
    nvs_close(nvs);
    return bad;
}

static void remember_bad_pin(bool bad)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    if (bad) {
        nvs_set_str(nvs, NVS_BAD_PIN, CONFIG_BRIDGE_SIM_PIN);
        nvs_commit(nvs);
    } else if (nvs_erase_key(nvs, NVS_BAD_PIN) == ESP_OK) { /* the SIM was unlocked: start afresh */
        nvs_commit(nvs);
    }
    nvs_close(nvs);
}

static bool check_sim(void)
{
    esp_modem_sim_pin_state_t state = ESP_MODEM_SIM_PIN_STATE_UNKNOWN;
    for (int i = 0; i < 10; i++) { /* the SIM needs a few seconds after the modem boots */
        if (esp_modem_read_pin_state(dce, &state) == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    switch (state) {
    case ESP_MODEM_SIM_PIN_STATE_READY:
        remember_bad_pin(false);
        return true;
    case ESP_MODEM_SIM_PIN_STATE_NEED_PIN: {
        if (CONFIG_BRIDGE_SIM_PIN[0] == '\0') {
            ESP_LOGE(TAG, "the SIM card needs a PIN: set it in menuconfig, or remove the PIN with a phone");
            return false;
        }
        if (pin_known_bad()) {
            ESP_LOGE(TAG, "the SIM card rejected the PIN in menuconfig earlier; not trying it again, so that it "
                          "cannot lock itself. Set the right PIN, or remove the PIN with a phone.");
            return false;
        }
        esp_err_t err = esp_modem_set_pin(dce, CONFIG_BRIDGE_SIM_PIN);
        if (err != ESP_OK) {
            pin_rejected = true; /* not again in this start either way */
            if (err == ESP_FAIL) { /* the SIM answered with an error: a wrong PIN, which it counts */
                remember_bad_pin(true);
                ESP_LOGE(TAG, "the SIM card rejected the PIN from menuconfig");
            } else {
                ESP_LOGE(TAG, "no answer to the SIM PIN; not trying it again until the next start");
            }
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
        return true;
    }
    case ESP_MODEM_SIM_PIN_STATE_NEED_PUK:
        ESP_LOGE(TAG, "the SIM card is locked (PUK needed): unlock it in a phone");
        return false;
    default:
        ESP_LOGE(TAG, "no usable SIM card: is it inserted (nano-SIM, contacts down)?");
        return false;
    }
}

static bool configure(void)
{
    char out[ESP_MODEM_C_API_STR_BUF_SIZE];
    esp_modem_set_echo(dce, false);
    esp_modem_at(dce, "AT+CMEE=2", out, 1000); /* readable error messages in the log */
    if (!check_sim()) {
        return false;
    }
    if (esp_modem_at(dce, "AT+CNMP?", out, 1000) == ESP_OK) {
        const char *p = strstr(out, "+CNMP:");
        if (!p || atoi(p + 6) != NETWORK_MODE) {
            esp_modem_set_network_mode(dce, NETWORK_MODE); /* the modem saves it */
        }
    }
    return true;
}

/* 3GPP registration status from +CEREG (LTE) or +CGREG (2G packet data):
 * 1 = home network, 5 = roaming, 2 = searching, 3 = denied, 0 = not searching. */
static int registration(void)
{
    static const char *const query[2][2] = {{"AT+CEREG?", "+CEREG:"}, {"AT+CGREG?", "+CGREG:"}};
    char out[ESP_MODEM_C_API_STR_BUF_SIZE];
    int result = 0;
    for (int i = 0; i < 2; i++) {
        if (esp_modem_at(dce, query[i][0], out, 1000) != ESP_OK) {
            continue;
        }
        const char *p = strstr(out, query[i][1]);
        const char *comma = p ? strchr(p, ',') : NULL;
        if (comma) {
            int stat = atoi(comma + 1);
            if (stat == 1 || stat == 5) {
                return stat;
            }
            if (stat) {
                result = stat;
            }
        }
    }
    return result;
}

static int16_t signal_dbm(void)
{
    int rssi = 99, ber = 99;
    if (esp_modem_get_signal_quality(dce, &rssi, &ber) != ESP_OK || rssi < 0 || rssi > 31) {
        return BRIDGE_RSSI_UNKNOWN;
    }
    return (int16_t)(-113 + 2 * rssi); /* AT+CSQ scale */
}

static bool wait_registration(uint32_t timeout_ms)
{
    uint32_t start = now_ms(), last_log = start;
    bool denied_logged = false;
    while ((uint32_t)(now_ms() - start) < timeout_ms) {
        int stat = registration();
        if (stat == 1 || stat == 5) {
            return true;
        }
        if (stat == 3 && !denied_logged) {
            denied_logged = true;
            ESP_LOGE(TAG, "the network refused registration: is the SIM active and does it have a data plan?");
        }
        if ((uint32_t)(now_ms() - last_log) >= 10000) {
            last_log = now_ms();
            int16_t dbm = signal_dbm();
            if (dbm == BRIDGE_RSSI_UNKNOWN) {
                ESP_LOGI(TAG, "searching for the network (no signal yet; check the LTE antenna)");
            } else {
                ESP_LOGI(TAG, "searching for the network (signal %d dBm)", dbm);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGW(TAG, "not registered with a network after %" PRIu32 " s", timeout_ms / 1000);
    return false;
}

static const char *rat_name(int act)
{
    switch (act) {
    case 0:
    case 1: return "GSM";
    case 2: return "3G";
    case 3: return "EDGE";
    case 4:
    case 5:
    case 6: return "HSPA";
    case 7: return "LTE";
    default: return "?";
    }
}

static void report_radio(void)
{
    char name[ESP_MODEM_C_API_STR_BUF_SIZE] = "";
    int act = -1;
    int16_t dbm = signal_dbm();
    esp_modem_get_operator_name(dce, name, &act);
    if (dbm == BRIDGE_RSSI_UNKNOWN) {
        ESP_LOGI(TAG, "registered with %s, %s", name, rat_name(act));
    } else {
        ESP_LOGI(TAG, "registered with %s, %s, signal %d dBm", name, rat_name(act), dbm);
    }
    bridge_set_radio(dbm, act >= 0 && act < 0xFF ? (uint8_t)act : BRIDGE_RAT_UNKNOWN);
}

static bool dial(void)
{
    xEventGroupClearBits(events, PPP_UP | PPP_DOWN);
    if (esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA) != ESP_OK) {
        ESP_LOGW(TAG, "the modem did not accept the data call (APN \"%s\")", CONFIG_BRIDGE_APN);
        return false;
    }
    EventBits_t bits = xEventGroupWaitBits(events, PPP_UP | PPP_DOWN, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    if (!(bits & PPP_UP)) {
        ESP_LOGW(TAG, "no IP address from the network (APN \"%s\")", CONFIG_BRIDGE_APN);
        return false;
    }
    return true;
}

/* Ends PPP and returns the modem to command mode. If the modem does not confirm, the next
 * esp_modem_sync() finds out whether it still answers. */
static void hang_up(void)
{
    esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND);
}

/* Last resort: new esp_modem instance and, if we control it, a power cycle of the modem. */
static void hard_reset(void)
{
    ESP_LOGW(TAG, "resetting the modem");
    if (dce) {
        hang_up();
        if (power_switchable) {
            esp_modem_power_down(dce); /* AT+CPOF: lets the modem close its files first */
        } else {
            char out[ESP_MODEM_C_API_STR_BUF_SIZE];
            esp_modem_at(dce, "AT+CRESET", out, 2000); /* the only reset we have with DIP "4G" on */
        }
        esp_modem_destroy(dce);
        dce = NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    power_cycle();
}

typedef enum { LINK_PPP_LOST, LINK_RELAY_SILENT } link_end_t;

/* Watches the connection until it ends. */
static link_end_t stay_online(void)
{
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(events, PPP_DOWN, pdFALSE, pdFALSE, pdMS_TO_TICKS(1000));
        if (bits & PPP_DOWN) {
            ESP_LOGW(TAG, "mobile data connection lost");
            return LINK_PPP_LOST;
        }
        /* PPP can stay up while nothing gets through any more; redialling usually cures it */
        if (bridge_relay_silence_ms() > RELAY_SILENCE_LIMIT_MS) {
            ESP_LOGW(TAG, "nothing from the relay for %d minutes; redialling", RELAY_SILENCE_LIMIT_MS / 60000);
            return LINK_RELAY_SILENT;
        }
    }
}

static void modem_task(void *arg)
{
    unsigned failures = 0, silent = 0; /* silent: redials in a row without hearing the relay */
    for (;;) {
        if (failures) {
            unsigned wait_s = failures < 6 ? failures * 5 : 30;
            status_set_modem(STATUS_MODEM_ERROR);
            ESP_LOGI(TAG, "trying again in %u s", wait_s);
            vTaskDelay(pdMS_TO_TICKS(wait_s * 1000));
        }
        status_set_modem(STATUS_MODEM_STARTING);
        if (!dce && !create_dce()) {
            failures++;
            continue;
        }
        if (!sync_modem(30000)) {
            ESP_LOGE(TAG, "the modem does not answer on its UART");
            failures++;
            hard_reset();
            continue;
        }
        if (!set_fast_baud()) {
            failures++;
            hard_reset();
            continue;
        }
        if (!configure() || !wait_registration(180000)) {
            if (++failures % 3 == 0) {
                hard_reset();
            }
            continue;
        }
        report_radio();
        uint32_t relay_packets = bridge_relay_packets();
        if (!dial()) {
            hang_up();
            if (++failures % 3 == 0) {
                hard_reset();
            }
            continue;
        }
        failures = 0;
        status_set_modem(STATUS_MODEM_ONLINE);
        link_end_t end = stay_online();
        hang_up();
        /* whether the relay answered during this connection: count its packets, as the silence clock
         * restarts when mobile data comes up */
        bool heard = bridge_relay_packets() != relay_packets;
        if (heard || end == LINK_PPP_LOST) {
            silent = 0;
        } else if (++silent >= 2) { /* redialling did not help: reset the modem */
            silent = 0;
            hard_reset();
        }
    }
}

void modem_start(void)
{
    board = board_get();
    events = xEventGroupCreate();
    const esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_PPP();
    ppp_netif = esp_netif_new(&netif_config);
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, on_ppp_status, NULL));
    nvs_handle_t nvs;
    uint8_t slow = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_SLOW_UART, &slow);
        nvs_close(nvs);
    }
    if (slow) {
        fast_baud_failed = true;
        ESP_LOGW(TAG, "modem UART stays at %d baud: it did not work at %d before (erase the flash to try again)",
                 BOOT_BAUD, CONFIG_BRIDGE_MODEM_BAUD);
    }
    power_init();
    xTaskCreate(modem_task, "modem", 6144, NULL, 10, NULL);
}
