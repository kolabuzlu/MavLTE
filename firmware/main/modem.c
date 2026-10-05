#include "modem.h"

#include <inttypes.h>
#include <stdio.h>
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
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "board.h"
#include "bridge.h"
#include "alarm.h"
#include "locator.h"
#include "logrow.h"
#include "netmode.h"
#include "sdlog.h"
#include "status.h"

#define MODEM_UART UART_NUM_1
#define BOOT_BAUD 115200 /* the A7670E's rate after power-up; AT+IPR changes are not saved */
#define PPP_UP BIT0
#define PPP_DOWN BIT1
#define PPP_ENDED BIT2 /* a PPP status event since hang_up() ended PPP: it is over (lwIP's link callback) */
#define RELAY_SILENCE_LIMIT_MS (3 * 60 * 1000)
#define RELAY_SILENCE_EARLY_MS (60 * 1000) /* with the network there: one redial this soon */
#define NVS_NAMESPACE "modem"
#define NVS_BAD_PIN "bad_pin"     /* the menuconfig SIM PIN that the SIM card rejected */
#define NVS_SLOW_UART "slow_uart" /* 1: the modem did not answer at CONFIG_BRIDGE_MODEM_BAUD */
#define NVS_NO_CMUX "no_cmux"     /* 1: the modem did not take CMUX: plain data calls, no GNSS during them */
#define RADIO_EVERY_MS 5000       /* signal, network and cell, read again during a CMUX data call */
#define RADIO_FRESH_MS 30000      /* older readings are left out of the flight log */
#define NET_REFUSED_MS 10000      /* the modem refused a network: not again before this */

static const char *TAG = "modem";

static const board_t *board;
static esp_netif_t *ppp_netif;
static esp_modem_dce_t *dce;
static EventGroupHandle_t events;
static bool power_switchable; /* DIP switch "4G" is off, so the firmware controls the modem's power */
static bool pin_rejected;     /* never retry a wrong SIM PIN: three tries lock the SIM */
static bool fast_baud_failed; /* the modem took AT+IPR but did not answer at that rate: stay at BOOT_BAUD */
static bool cmux_off;         /* data calls without the multiplexer (it failed, or no locator) */
static bool in_cmux;          /* this data call runs over CMUX: AT commands still work during it */
static int baud = BOOT_BAUD;
#if CONFIG_BRIDGE_GNSS_GPS_BEIDOU_GALILEO
#define GNSS_MODE 4 /* AT+CGNSSMODE, the A7670E's (a "foreign module"): GPS + BeiDou + Galileo */
#define GNSS_MODE_SET "AT+CGNSSMODE=4\r"
#define GNSS_SYSTEMS "GPS + BeiDou + Galileo"
static bool gnss_mode_set; /* the modem's GNSS uses GNSS_MODE since it last started */
#endif
/* the network as last read, for the flight log (guarded by radio_lock) */
static SemaphoreHandle_t radio_lock;
static int16_t radio_dbm = BRIDGE_RSSI_UNKNOWN;
static char radio_operator[24];
static cell_info_t radio_cell;
static uint32_t radio_ms;
static int radio_act = -1; /* the access technology then (3GPP AcT), -1 none (modem task only) */
/* the network (netmode.h): chosen at the relay, and automatic's fallback to 2G (modem task only) */
static netmode_t netmode;
static int net_set = -1;          /* the modem's AT+CNMP as last read or set, -1 not known */
static bool net_refused;          /* ... it refused the last one, at net_refused_at */
static uint32_t net_refused_at;
static uint32_t relay_count, relay_heard_ms; /* bridge_relay_packets() as last seen, and when it last changed */
#if CONFIG_BRIDGE_LOCATOR_VOICE
static bool voice_ready; /* the modem's audio is set up for the locator voice since it last (re)started */
#endif
static void voice_prepare(void);
static bool apply_network(void);

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_PPP_GOT_IP) {
        xEventGroupClearBits(events, PPP_DOWN);
        xEventGroupSetBits(events, PPP_UP);
    } else if (id == IP_EVENT_PPP_LOST_IP && (xEventGroupGetBits(events) & PPP_UP)) {
        /* (only for the call that is up: ESP-IDF's lost-IP timer can post one about an earlier call, 120 s later,
         * which would end a dial still waiting for its address) */
        xEventGroupClearBits(events, PPP_UP);
        xEventGroupSetBits(events, PPP_DOWN);
    }
}

/* PPP errors that end the link without IP_EVENT_PPP_LOST_IP, e.g. the modem stops answering
 * LCP echo requests. NETIF_PPP_ERRORUSER is our own hang-up. Each of these comes as PPP is over. */
static void on_ppp_status(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id > NETIF_PPP_ERRORNONE && id < NETIF_PP_PHASE_OFFSET) {
        xEventGroupSetBits(events, PPP_ENDED);
    }
    if (id > NETIF_PPP_ERRORNONE && id < NETIF_PP_PHASE_OFFSET && id != NETIF_PPP_ERRORUSER) {
        ESP_LOGW(TAG, "PPP error %" PRId32, id);
        xEventGroupClearBits(events, PPP_UP);
        xEventGroupSetBits(events, PPP_DOWN);
    }
}

/* ---- power */

static void power_init(void)
{
    /* DTR held asserted (low): the modem never ends a data call over it, whatever its AT&D setting */
    const gpio_config_t dtr = {
        .pin_bit_mask = 1ULL << BOARD_MODEM_DTR_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_set_level((gpio_num_t)BOARD_MODEM_DTR_GPIO, 0);
    gpio_config(&dtr);
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
#if CONFIG_BRIDGE_LOCATOR_VOICE
    voice_ready = false; /* perhaps a restarted modem: its audio settings are back to their defaults */
#endif
#ifdef GNSS_MODE
    gnss_mode_set = false; /* and its GNSS on its own satellite systems */
#endif
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
    /* the rate it was last at first: after a hang-up the fast one (the slow one first cost a second each time) */
    const int rates[2] = {baud, baud == BOOT_BAUD ? CONFIG_BRIDGE_MODEM_BAUD : BOOT_BAUD};
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
    /* Until the next power-on; remembered in flash only with DIP "4G" on: then a modem stuck at a rate the link
     * cannot carry comes back only with a power cycle of the whole board, after which it must not happen again.
     * With the switch off the firmware power-cycles the modem itself, and a passing failure (a brownout just
     * after AT+IPR, say) must not slow the link for good. */
    fast_baud_failed = true;
    if (!power_switchable) {
        nvs_handle_t nvs;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_u8(nvs, NVS_SLOW_UART, 1);
            nvs_commit(nvs);
            nvs_close(nvs);
        }
    }
    ESP_LOGW(TAG, "the modem does not answer at %d baud; using %d %s", CONFIG_BRIDGE_MODEM_BAUD, BOOT_BAUD,
             power_switchable ? "until the next power-on" : "from now on (erase the flash to try again)");
    esp_modem_set_baud(dce, BOOT_BAUD); /* reaches it if only its answers get lost at the fast rate */
    uart_set_baudrate(MODEM_UART, BOOT_BAUD);
    baud = BOOT_BAUD;
    vTaskDelay(pdMS_TO_TICKS(100));
    return esp_modem_sync(dce) == ESP_OK;
}

static char answer[320]; /* the modem's answer to the last command() */

static esp_err_t answer_line(uint8_t *data, size_t len)
{
    size_t n = len < sizeof(answer) - 1 ? len : sizeof(answer) - 1;
    memcpy(answer, data, n);
    answer[n] = '\0';
    if (strstr(answer, "\nOK") || strncmp(answer, "OK", 2) == 0) {
        return ESP_OK;
    }
    if (strstr(answer, "ERROR") || strstr(answer, "NO CARRIER")) {
        return ESP_FAIL;
    }
    return ESP_ERR_TIMEOUT; /* more to come */
}

/* Sends an AT command ("AT...\r") and keeps its whole answer in `answer`. */
static esp_err_t command(const char *at, uint32_t timeout_ms)
{
    answer[0] = '\0';
    return esp_modem_command(dce, at, answer_line, timeout_ms);
}

/* The line of `answer` that starts with `prefix`, into out; "" if there is none. */
static const char *answer_field(const char *prefix, char *out, size_t n)
{
    const char *p = strstr(answer, prefix);
    size_t i = 0;
    if (p) {
        for (p += strlen(prefix); *p == ' '; p++) {
        }
        for (; p[i] && p[i] != '\r' && p[i] != '\n' && i < n - 1; i++) {
            out[i] = p[i];
        }
    }
    out[i] = '\0';
    return out;
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

/* PIN tries the SIM card has left (AT+SPIC answers PIN1 first), or -1 if the modem does not say. */
static int pin_tries_left(void)
{
    char field[24];
    if (command("AT+SPIC\r", 2000) != ESP_OK || !answer_field("+SPIC:", field, sizeof(field))[0]) {
        return -1;
    }
    return atoi(field);
}

static bool check_sim(void)
{
    esp_modem_sim_pin_state_t state = ESP_MODEM_SIM_PIN_STATE_UNKNOWN;
    for (int i = 0; i < 20; i++) { /* the SIM needs a few seconds after the modem boots */
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
            sdlog_event("SIM card needs a PIN");
            return false;
        }
        if (pin_known_bad()) {
            ESP_LOGE(TAG, "the SIM card rejected the PIN in menuconfig earlier; not trying it again, so that it "
                          "cannot lock itself. Set the right PIN, or remove the PIN with a phone.");
            return false;
        }
        int left = pin_tries_left();
        if (left >= 0 && left < 2) { /* the last try stays for a phone: a wrong PIN then locks the SIM */
            pin_rejected = true;
            ESP_LOGE(TAG, "the SIM card has %d PIN tr%s left: not trying the PIN from menuconfig, so that it cannot "
                          "lock itself. Unlock the SIM with a phone.", left, left == 1 ? "y" : "ies");
            return false;
        }
        /* The SIM may take up to 9 s to answer (A76XX AT manual): a late "wrong PIN" must not pass for
         * no answer, or the PIN is sent again after each restart until the SIM locks. */
        esp_err_t err = command("AT+CPIN=" CONFIG_BRIDGE_SIM_PIN "\r", 10000);
        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            return true;
        }
        char reason[48];
        answer_field("+CME ERROR:", reason, sizeof(reason));
        if (err == ESP_FAIL && (strstr(reason, "SIM busy") || strcmp(reason, "14") == 0)) {
            ESP_LOGW(TAG, "the SIM card is busy; its PIN goes again in a moment");
            return false;
        }
        pin_rejected = true; /* not again in this start either way */
        if (err == ESP_FAIL) { /* the SIM answered with an error: most likely a wrong PIN, which it counts */
            remember_bad_pin(true);
            ESP_LOGE(TAG, "the SIM card rejected the PIN from menuconfig (%s)", reason[0] ? reason : "ERROR");
        } else {
            ESP_LOGE(TAG, "no answer to the SIM PIN; not trying it again until the next start");
        }
        return false;
    }
    case ESP_MODEM_SIM_PIN_STATE_NEED_PUK:
        ESP_LOGE(TAG, "the SIM card is locked (PUK needed): unlock it in a phone");
        sdlog_event("SIM card locked");
        return false;
    default:
        ESP_LOGE(TAG, "no usable SIM card: is it inserted (nano-SIM, contacts down)?");
        sdlog_event("no SIM card");
        return false;
    }
}

static bool configure(void)
{
    char out[ESP_MODEM_C_API_STR_BUF_SIZE];
#if CONFIG_BRIDGE_LOCATOR_VOICE
    voice_ready = false; /* the modem may have restarted by itself since: its audio is back to its defaults */
#endif
    esp_modem_set_echo(dce, false);
    esp_modem_at(dce, "AT+CMEE=2", out, 1000); /* readable error messages in the log */
    esp_modem_at(dce, "AT+COPS=3,0", out, 1000); /* the operator's name in AT+COPS?, not its number */
    if (!check_sim()) {
        return false;
    }
    net_set = -1; /* read again: the modem may have restarted (it keeps its last setting), or been set by hand */
    net_refused = false;
    apply_network();
    static bool identified;
    if (!identified && command("ATI\r", 2000) == ESP_OK) { /* the GNSS answers differ between them */
        identified = true;
        char model[40], revision[48];
        ESP_LOGI(TAG, "modem %s, firmware %s", answer_field("Model:", model, sizeof(model)),
                 answer_field("Revision:", revision, sizeof(revision)));
    }
#if CONFIG_BRIDGE_LOCATOR
    /* The GNSS runs from now on, during data calls too, so that a fix is at hand whenever it is read
     * (after its "+CGNSSPWR: READY!", 10-30 s later). Started when it is off: this runs before every
     * data call. Its NMEA stays off the UART, which carries PPP. */
    char gnss_power[24];
    if (command("AT+CGNSSPWR?\r", 2000) != ESP_OK ||
        atoi(answer_field("+CGNSSPWR:", gnss_power, sizeof(gnss_power))) != 1) {
        if (command("AT+CGNSSPWR=1\r", 9000) != ESP_OK) { /* up to 9 s (A76XX AT manual) */
            ESP_LOGW(TAG, "the modem's GNSS did not start (%s)", answer);
        }
#ifdef GNSS_MODE
        gnss_mode_set = false; /* started again: on the modem's own satellite systems */
#endif
    }
    command("AT+CGNSSTST=0\r", 2000);
#endif
    voice_prepare(); /* in command mode, before the data call */
    return true;
}

/* 3GPP registration status from +CEREG (LTE) or +CGREG (2G packet data):
 * 1 = home network, 5 = roaming, 2 = searching, 3 = denied, 0 = not searching; -1: no answer to either. */
static int registration(void)
{
    static const char *const query[2][2] = {{"AT+CEREG?", "+CEREG:"}, {"AT+CGREG?", "+CGREG:"}};
    char out[ESP_MODEM_C_API_STR_BUF_SIZE];
    int result = 0;
    bool answered = false;
    for (int i = 0; i < 2; i++) {
        esp_err_t err = esp_modem_at(dce, query[i][0], out, 1000);
        answered |= err != ESP_ERR_TIMEOUT;
        if (err != ESP_OK) {
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
    return answered ? result : -1;
}

static int16_t signal_dbm(void)
{
    int rssi = 99, ber = 99;
    if (esp_modem_get_signal_quality(dce, &rssi, &ber) != ESP_OK || rssi < 0 || rssi > 31) {
        return BRIDGE_RSSI_UNKNOWN;
    }
    return (int16_t)(-113 + 2 * rssi); /* AT+CSQ scale */
}

/* Signal, network and cell, for the relay's link status and the flight log: wherever the modem takes AT commands (in
 * command mode, and on the CMUX command channel during a data call). The operator's name goes into name
 * (ESP_MODEM_C_API_STR_BUF_SIZE bytes), its access technology into *act (-1: none). */
/* The operator's name (n bytes) and access technology (-1: not known) from AT+COPS?: our own command, with a
 * short timeout. esp_modem's esp_modem_get_operator_name() waits 75 s for an answer (what AT+COPS=... may take),
 * which would hold up the whole modem task, every 5 s, with a modem that stopped answering. */
static void operator_name(char *name, size_t n, int *act)
{
    name[0] = '\0';
    *act = -1;
    char line[64];
    if (command("AT+COPS?\r", 2000) != ESP_OK) {
        return;
    }
    const char *open = strchr(answer_field("+COPS:", line, sizeof(line)), '"'); /* 0,0,"Turkcell",7 */
    const char *close = open ? strchr(open + 1, '"') : NULL;
    if (!close) {
        return; /* no operator: not registered */
    }
    size_t len = (size_t)(close - open - 1) < n - 1 ? (size_t)(close - open - 1) : n - 1;
    memcpy(name, open + 1, len);
    name[len] = '\0';
    if (close[1] == ',' && close[2] >= '0' && close[2] <= '9') {
        *act = atoi(close + 2);
    }
}

static int16_t read_radio(char *name, int *act)
{
    int16_t dbm = signal_dbm();
    operator_name(name, ESP_MODEM_C_API_STR_BUF_SIZE, act);
    cell_info_t cell;
    if (command("AT+CPSI?\r", 2000) != ESP_OK || !cell_parse(answer, &cell)) {
        cell_clear(&cell);
    }
    /* the signal's quality too (SINR, LTE only): in the air the signal stays strong (-51 dBm) while the quality
     * falls with the many cells heard at once, and below -12 dB nothing gets through (first flight, 1.8.6) */
    int8_t sinr = BRIDGE_SINR_UNKNOWN;
    if (cell.sinr_db != LOG_I16_UNKNOWN) {
        sinr = (int8_t)(cell.sinr_db < -127 ? -127 : cell.sinr_db > 127 ? 127 : cell.sinr_db);
    }
    bridge_set_radio(dbm, *act >= 0 && *act < 0xFF ? (uint8_t)*act : BRIDGE_RAT_UNKNOWN, sinr);
    xSemaphoreTake(radio_lock, portMAX_DELAY);
    radio_dbm = dbm;
    snprintf(radio_operator, sizeof(radio_operator), "%s", name);
    radio_cell = cell;
    radio_ms = now_ms();
    radio_act = *act;
    xSemaphoreGive(radio_lock);
    return dbm;
}

void modem_log_row(log_row_t *row)
{
    if (!radio_lock) {
        return;
    }
    xSemaphoreTake(radio_lock, portMAX_DELAY);
    if (radio_ms && (uint32_t)(now_ms() - radio_ms) < RADIO_FRESH_MS) {
        row->signal_dbm = radio_dbm == BRIDGE_RSSI_UNKNOWN ? LOG_I16_UNKNOWN : radio_dbm;
        memcpy(row->operator_name, radio_operator, sizeof(row->operator_name));
        row->cell = radio_cell;
    }
    xSemaphoreGive(radio_lock);
}

#if CONFIG_BRIDGE_LOCATOR_VOICE
/* ---- the locator voice: the board's speaker (on the modem's earpiece output), while the relay asks for it */

#define VOICE_FAILED_MS 15000 /* asked for this long and nothing played: the modem does not */
#define VOICE_RETRY_MS 1000   /* after a refusal: the modem still busy with the last one, say */

#if CONFIG_BRIDGE_LOCATOR_SOUND_ALARM
/* The two-tone alarm (alarm.h), a WAV file in the modem's own flash: each command plays it 15 times (30 s) */
#define ALARM_FILE "mavlte_alarm1.wav" /* on its C: drive; another sound would get another name */
#define VOICE_EVERY_MS (15 * ALARM_WAV_MS + 500)
#define VOICE_STOP "AT+CCMXSTOP\r"
#define VOICE_WHAT "the two-tone alarm"

static const char *voice_command(void)
{
    return "AT+CCMXPLAY=\"c:/" ALARM_FILE "\",0,14\r"; /* 14 repeats */
}

/* Reads the modem's UART itself until `pass` or `fail` comes: while none of its own commands waits,
 * esp_modem leaves what arrives unread. */
static bool uart_expect(const char *pass, const char *fail, uint32_t timeout_ms)
{
    char seen[96];
    size_t n = 0;
    for (uint32_t start = now_ms(); (uint32_t)(now_ms() - start) < timeout_ms;) {
        uint8_t c;
        if (uart_read_bytes(MODEM_UART, &c, 1, pdMS_TO_TICKS(20)) != 1) {
            continue;
        }
        if (n == sizeof(seen) - 1) { /* keep the newest half */
            memmove(seen, seen + sizeof(seen) / 2, n - sizeof(seen) / 2);
            n -= sizeof(seen) / 2;
        }
        seen[n++] = (char)c;
        seen[n] = '\0';
        if (strstr(seen, pass)) {
            return true;
        }
        if (strstr(seen, fail)) {
            return false;
        }
    }
    return false;
}

static bool alarm_stored(void)
{
    char size[16];
    return command("AT+FSATTRI=" ALARM_FILE "\r", 2000) == ESP_OK &&
           atoi(answer_field("+FSATTRI:", size, sizeof(size))) == ALARM_WAV_BYTES;
}

/* Puts the alarm's WAV file into the modem's own flash, unless it is there: once per modem. In command
 * mode only, before a data call. The file goes over the UART as raw bytes after the modem's ">" prompt,
 * which esp_modem's commands cannot carry, so this writes and reads the UART itself. */
static void voice_prepare(void)
{
    if (command("AT+FSCD=C:\r", 2000) == ESP_OK && alarm_stored()) {
        return;
    }
    command("AT+FSDEL=" ALARM_FILE "\r", 2000); /* one cut short: AT+CFTRANRX does not overwrite */
    char at[64];
    int n = snprintf(at, sizeof(at), "AT+CFTRANRX=\"c:/%s\",%d\r", ALARM_FILE, ALARM_WAV_BYTES);
    uart_flush_input(MODEM_UART);
    uart_write_bytes(MODEM_UART, at, (size_t)n);
    bool ok = uart_expect(">", "ERROR", 5000);
    if (ok) {
        static uint8_t chunk[256];
        for (size_t done = 0; done < ALARM_WAV_BYTES; done += sizeof(chunk)) {
            size_t len = ALARM_WAV_BYTES - done < sizeof(chunk) ? ALARM_WAV_BYTES - done : sizeof(chunk);
            alarm_wav(chunk, done, len);
            uart_write_bytes(MODEM_UART, chunk, len);
            uart_wait_tx_done(MODEM_UART, pdMS_TO_TICKS(100));
            vTaskDelay(pdMS_TO_TICKS(10)); /* time for the modem to write its flash (SIMCom's advice) */
        }
        ok = uart_expect("OK", "ERROR", 10000);
    }
    if (ok && alarm_stored()) {
        ESP_LOGI(TAG, "locator voice: the alarm is stored on the modem");
    } else {
        ESP_LOGW(TAG, "locator voice: could not store the alarm on the modem; trying again before the next data "
                      "call");
    }
}
#else
/* The spoken phrase: the modem's text-to-speech, about 2 s for the default one */
#define VOICE_EVERY_MS 3000
#define VOICE_STOP "AT+CTTS=0\r"
#define VOICE_WHAT "\"" CONFIG_BRIDGE_LOCATOR_VOICE_TEXT "\""

/* AT+CTTS=2,"<text>": ASCII text; quotes and anything else that would end the command are left out */
static const char *voice_command(void)
{
    static char at[sizeof(CONFIG_BRIDGE_LOCATOR_VOICE_TEXT) + 16];
    if (!at[0]) {
        size_t n = (size_t)snprintf(at, sizeof(at), "AT+CTTS=2,\"");
        for (const char *p = CONFIG_BRIDGE_LOCATOR_VOICE_TEXT; *p && n < 11 + 500; p++) {
            if (*p >= ' ' && *p <= '~' && *p != '"') {
                at[n++] = *p;
            }
        }
        memcpy(at + n, "\"\r", 3);
    }
    return at;
}

static void voice_prepare(void)
{
}
#endif

static bool voice_asked;   /* the relay asks for the voice, as last seen here */
static bool voice_playing; /* the modem took the last command */
static bool voice_spoke;   /* it has taken one since the voice was asked for */
static bool voice_failed;  /* reported as unable to play */
static uint32_t voice_asked_ms, voice_try_ms, voice_ok_ms;

/* About once a second wherever the modem takes AT commands: in command mode, and on the CMUX command
 * channel during a data call (`usable` false where it cannot: a data call without CMUX). A modem still busy
 * with the last command refuses the next one (ERROR), which only means "a second later". */
static void voice_tick(bool usable)
{
    uint32_t now = now_ms();
    if (!bridge_voice_wanted()) {
        if (voice_asked) {
            voice_asked = false;
            if (usable) {
                command(VOICE_STOP, 2000); /* stops it at once */
            }
            ESP_LOGI(TAG, "locator voice off");
            sdlog_event("voice off");
            bridge_set_voice(false, false);
        }
        return;
    }
    if (!voice_asked) {
        voice_asked = true;
        voice_playing = voice_spoke = voice_failed = false;
        voice_ready = false; /* set the audio up again: the modem may have changed it since */
        voice_asked_ms = now;
        voice_try_ms = now - VOICE_EVERY_MS;
        ESP_LOGI(TAG, "locator voice on: %s through the board's speaker", VOICE_WHAT);
        sdlog_event("voice on");
    }
    if (usable && (uint32_t)(now - voice_try_ms) >= (voice_playing ? VOICE_EVERY_MS : VOICE_RETRY_MS)) {
        voice_try_ms = now;
        if (!voice_ready) {
            /* as tried on the A7670E-FASE: the speaker phone path and the highest volumes; tried again at the next
             * sound unless all of them took (else it might sound on the default path, at the default volume) */
            bool ready = command("AT+CSDVC=3\r", 2000) == ESP_OK;
            ready = command("AT+COUTGAIN=7\r", 2000) == ESP_OK && ready;
#if CONFIG_BRIDGE_LOCATOR_SOUND_PHRASE
            ready = command("AT+CTTSPARAM=2,3,0,1,1\r", 2000) == ESP_OK && ready; /* volume, digits, pitch... */
#endif
            voice_ready = ready;
        }
        voice_playing = command(voice_command(), 3000) == ESP_OK;
        if (voice_playing) {
            voice_spoke = true;
            voice_ok_ms = now;
        }
    }
    bool speaking = voice_spoke && (uint32_t)(now - voice_ok_ms) < VOICE_EVERY_MS + VOICE_FAILED_MS;
    bool failed = !speaking && (uint32_t)(now - (voice_spoke ? voice_ok_ms : voice_asked_ms)) >= VOICE_FAILED_MS;
    if (failed && !voice_failed) {
        sdlog_event("voice: the modem does not play it");
        if (usable) {
            ESP_LOGW(TAG, "the locator voice is on, but the modem does not play it (%s)", answer);
        } else {
            ESP_LOGW(TAG, "the locator voice needs the modem's multiplexer (CMUX), which it did not take");
        }
    }
    voice_failed = failed;
    bridge_set_voice(speaking, failed);
}
#else
static void voice_prepare(void)
{
}

static void voice_tick(bool usable)
{
    (void)usable;
}
#endif

/* Waits; meanwhile the locator voice goes on if the modem takes commands (esp_modem calls command mode
 * UNDEF until the first data call, and again after a CMUX one). */
static void pause_ms(uint32_t ms)
{
    for (uint32_t start = now_ms(), waited; (waited = now_ms() - start) < ms;) {
        vTaskDelay(pdMS_TO_TICKS(ms - waited < 1000 ? ms - waited : 1000));
        esp_modem_dce_mode_t mode = esp_modem_get_mode(dce);
        if (dce && (mode == ESP_MODEM_MODE_COMMAND || mode == ESP_MODEM_MODE_UNDEF)) {
            voice_tick(true);
        }
    }
}

/* ---- the network: as chosen at the relay, and automatic's fallback to 2G while LTE fails (netmode.h). The relay keeps
 * the choice, the module only until it restarts: one that finds no 2G or no LTE where it was chosen, and so cannot hear
 * the relay any more, starts on automatic again (as the locator voice: off until the relay says). */

/* What netmode goes by, once a second or so (`online`: in the data call, up for online_ms): the relay's choice,
 * whether the relay answers, the modem's network and LTE's quality as last read, and the aircraft's height. */
static void network_update(bool online, uint32_t online_ms)
{
    uint32_t now = now_ms();
    uint32_t count = bridge_relay_packets();
    if (count != relay_count) {
        relay_count = count;
        relay_heard_ms = now;
    }
    int wanted = bridge_network_wanted();
    if (wanted >= 0 && wanted != netmode.choice) {
        if (!netmode_choose(&netmode, (uint8_t)wanted)) { /* the modem stays as it is: say so here */
            ESP_LOGI(TAG, "network: %s", netmode.why);
            sdlog_event("network: %s", netmode.why);
        }
        bridge_set_net_report(netmode_report(&netmode));
    }
    net_in_t in = {.now_ms = now, .online = online, .online_ms = online_ms, .silence_ms = now - relay_heard_ms,
                   .rat = radio_act >= 0 && radio_act < 0xFF ? (uint8_t)radio_act : 0xFF,
                   .sinr_db = NET_SINR_UNKNOWN};
    if (radio_ms && (uint32_t)(now - radio_ms) < 3 * RADIO_EVERY_MS && radio_cell.sinr_db != LOG_I16_UNKNOWN) {
        in.sinr_db = (int8_t)(radio_cell.sinr_db < -127 ? -127 : radio_cell.sinr_db > 127 ? 127 : radio_cell.sinr_db);
    }
    in.alt_known = bridge_fc_altitude(&in.alt_m);
    if (netmode_tick(&netmode, &in)) {
        bridge_set_net_report(netmode_report(&netmode));
    }
}

/* The modem is to be set to another network (and has not just refused it). */
static bool network_due(void)
{
    return netmode_setting(&netmode) != net_set &&
           (!net_refused || (uint32_t)(now_ms() - net_refused_at) >= NET_REFUSED_MS);
}

/* The modem's AT+CNMP, or -1 if it does not say. */
static int read_network(void)
{
    char field[16];
    return command("AT+CNMP?\r", 2000) == ESP_OK && answer_field("+CNMP:", field, sizeof(field))[0] ? atoi(field) : -1;
}

/* Sets the modem to the network netmode needs (AT+CNMP, which the modem saves), in command mode: a data call would end
 * as the modem leaves the network, and esp_modem take 6 s to hang it up (bench, 1.8.8). The modem then registers
 * again: LTE in about 3 s, 2G in about 8. True if it changed. */
static bool apply_network(void)
{
    if (net_set < 0) {
        net_set = read_network();
    }
    if (!network_due()) {
        return false;
    }
    int want = netmode_setting(&netmode);
    char at[16], said[64];
    snprintf(at, sizeof(at), "AT+CNMP=%d\r", want);
    command(at, 10000); /* up to 10 s */
    snprintf(said, sizeof(said), "%s", answer);
    for (int i = 0; (net_set = read_network()) < 0 && i < 3; i++) { /* still busy with the change: ask again */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (net_set != want) {
        net_refused = true;
        net_refused_at = now_ms();
        for (char *p = said; *p; p++) {
            *p = *p == '\r' || *p == '\n' ? ' ' : *p;
        }
        ESP_LOGW(TAG, "the modem did not take AT+CNMP=%d (%s); it stays on its network for now, and is asked again "
                      "before the next data call", want, said);
        return false;
    }
    net_refused = false;
    if (netmode.fallback) {
        ESP_LOGW(TAG, "network: %s", netmode.why);
    } else {
        ESP_LOGI(TAG, "network: %s", netmode.why);
    }
    sdlog_event("network: %s", netmode.why);
    return true;
}

static bool wait_registration(uint32_t timeout_ms)
{
    uint32_t start = now_ms(), last_log = start;
    bool denied_logged = false;
    unsigned unanswered = 0;
    while ((uint32_t)(now_ms() - start) < timeout_ms) {
        int stat = registration();
        if (stat == 1 || stat == 5) {
            return true;
        }
        if (stat < 0 && ++unanswered >= 3) { /* restarted, perhaps at another rate: modem_task finds it again */
            ESP_LOGW(TAG, "the modem stopped answering while it looked for the network");
            sdlog_event("modem does not answer");
            return false;
        }
        if (stat >= 0) {
            unanswered = 0;
        }
        if (stat == 3 && !denied_logged) {
            denied_logged = true;
            ESP_LOGE(TAG, "the network refused registration: is the SIM active and does it have a data plan?");
            sdlog_event("network refused registration");
        }
        network_update(false, 0);
        if (network_due() && apply_network()) {
            start = now_ms(); /* registering again, on the other network */
        }
        if ((uint32_t)(now_ms() - last_log) >= 10000) {
            last_log = now_ms();
            char name[ESP_MODEM_C_API_STR_BUF_SIZE];
            int act;
            int16_t dbm = read_radio(name, &act);
            if (dbm == BRIDGE_RSSI_UNKNOWN) {
                ESP_LOGI(TAG, "searching for the network (no signal yet; check the LTE antenna)");
            } else {
                ESP_LOGI(TAG, "searching for the network (signal %d dBm)", dbm);
            }
        }
        pause_ms(2000);
    }
    ESP_LOGW(TAG, "not registered with a network after %" PRIu32 " s", timeout_ms / 1000);
    sdlog_event("no network for %" PRIu32 " s", timeout_ms / 1000);
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
    char name[ESP_MODEM_C_API_STR_BUF_SIZE];
    int act;
    int16_t dbm = read_radio(name, &act);
    if (dbm == BRIDGE_RSSI_UNKNOWN) {
        ESP_LOGI(TAG, "registered with %s, %s", name, rat_name(act));
    } else {
        ESP_LOGI(TAG, "registered with %s, %s, signal %d dBm", name, rat_name(act), dbm);
    }
    sdlog_event("registered with %s on %s", name, rat_name(act));
}

#if CONFIG_BRIDGE_LOCATOR
static unsigned cmux_failures;

/* The modem refused the multiplexer: after the second time in a row, data calls go without it (and without GNSS
 * readings or the locator voice during them) until the next power-on. Not remembered in flash: two passing
 * failures (a brownout, say) must not turn the locator off for good. */
static void cmux_failed(void)
{
    if (++cmux_failures < 2) {
        ESP_LOGW(TAG, "the modem did not take CMUX; trying again");
        return;
    }
    cmux_off = true;
    ESP_LOGW(TAG, "the modem does not take CMUX: data calls without it until the next power-on, so no GNSS "
                  "positions during them");
    sdlog_event("no CMUX: no GNSS readings during data calls");
    bridge_set_gnss(NULL);
}
#endif

static bool dial(void)
{
    xEventGroupClearBits(events, PPP_UP | PPP_DOWN);
    in_cmux = false;
#if CONFIG_BRIDGE_LOCATOR
    if (!cmux_off) {
        /* PPP on one CMUX channel, AT commands (GNSS, signal) on another, over the same UART */
        if (esp_modem_set_mode(dce, ESP_MODEM_MODE_CMUX) != ESP_OK) {
            if (esp_modem_get_mode(dce) != ESP_MODEM_MODE_CMUX) { /* the multiplexer did not start */
                cmux_failed();
            } else { /* it did, but the data call did not: hang_up() closes the multiplexer */
                in_cmux = true;
                ESP_LOGW(TAG, "the modem did not accept the data call (APN \"%s\")", CONFIG_BRIDGE_APN);
                sdlog_event("data call refused");
            }
            return false;
        }
        cmux_failures = 0;
        in_cmux = true;
        esp_modem_set_echo(dce, false); /* each CMUX channel has its own echo setting */
    } else
#endif
    if (esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA) != ESP_OK) {
        ESP_LOGW(TAG, "the modem did not accept the data call (APN \"%s\")", CONFIG_BRIDGE_APN);
        sdlog_event("data call refused");
        return false;
    }
    EventBits_t bits = xEventGroupWaitBits(events, PPP_UP | PPP_DOWN, pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    if (!(bits & PPP_UP)) {
        ESP_LOGW(TAG, "no IP address from the network (APN \"%s\")", CONFIG_BRIDGE_APN);
        sdlog_event("no IP address");
        return false;
    }
    return true;
}

/* Ends PPP and returns the modem to command mode. If the modem does not confirm, the next
 * esp_modem_sync() finds out whether it still answers. */
static void hang_up(void)
{
    /* PPP first, waiting until it is over: esp_modem waits for that too, but a PPP event left over from before ends its
     * wait at once, and it then closes the multiplexer while PPP still terminates, so that the next data call cannot
     * start PPP (bench, 1.8.8: 35 s lost). Over at once if it already was (or never ran): lwIP says so straight away. */
    xEventGroupClearBits(events, PPP_ENDED);
    esp_netif_action_stop(ppp_netif, NULL, 0, NULL);
    if (!(xEventGroupWaitBits(events, PPP_ENDED, pdFALSE, pdFALSE, pdMS_TO_TICKS(8000)) & PPP_ENDED)) {
        ESP_LOGW(TAG, "PPP did not end within 8 s");
    }
    esp_err_t err = esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND);
    if (err != ESP_OK && in_cmux) {
        /* The A7670 answers the multiplexer's close-down with a frame esp_modem does not accept: it
         * reports a failure, although both sides have left CMUX. Forget the mode, so that the next
         * data call is allowed to start it again. */
        esp_modem_set_mode(dce, ESP_MODEM_MODE_UNDEF);
    } else if (err == ESP_OK && !in_cmux) {
        command("ATH\r", 5000); /* "+++" leaves the packet data call up (A76XX AT manual): end it */
    }
    in_cmux = false;
}

/* A modem still in CMUX from before the ESP32 restarted does not answer plain AT commands: the
 * multiplexer's close-down frame (DLCI 0) takes it back, at either rate it may run at. */
static void leave_cmux(void)
{
    static const char close_down[] = "\xF9\x03\xEF\x05\xC3\x01\xF2\xF9";
    const int rates[2] = {CONFIG_BRIDGE_MODEM_BAUD, BOOT_BAUD};
    for (int i = 0; i < 2; i++) {
        uart_set_baudrate(MODEM_UART, rates[i]);
        baud = rates[i];
        command(close_down, 300); /* no answer that counts */
    }
}

/* Last resort: a new esp_modem instance and a restart of the modem. One that still answers restarts
 * itself (AT+CRESET): SIMCom warns that cutting the power of a running module may damage its flash.
 * One that does not answer has its power cut, if the firmware controls it (DIP switch "4G" off). */
static void hard_reset(void)
{
    ESP_LOGW(TAG, "resetting the modem");
    sdlog_event("modem reset");
    bool restarting = false;
    if (dce) {
        esp_modem_dce_mode_t mode = esp_modem_get_mode(dce);
        if (in_cmux || mode == ESP_MODEM_MODE_DATA || mode == ESP_MODEM_MODE_CMUX) {
            hang_up(); /* (not in command mode or unknown: esp_modem would spend some 20 s on "+++" for nothing) */
        }
        bool answers = sync_modem(3000);
        if (!answers) {
            leave_cmux(); /* still multiplexed from the data call? */
            answers = sync_modem(3000);
        }
        if (answers) {
            restarting = command("AT+CRESET\r", 9000) == ESP_OK; /* OK, then it restarts: UART back in ~8 s */
        }
        esp_modem_destroy(dce);
        dce = NULL;
    }
    if (!restarting) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        power_cycle();
    }
}

typedef enum { LINK_PPP_LOST, LINK_RELAY_SILENT, LINK_RELAY_QUIET, LINK_NETWORK } link_end_t;

#if CONFIG_BRIDGE_LOCATOR
/* 1e-7 degrees as text, without floating point in printf */
static const char *degrees(char *out, size_t n, int32_t v)
{
    uint32_t a = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    snprintf(out, n, "%s%" PRIu32 ".%06" PRIu32, v < 0 ? "-" : "", a / 10000000, a % 10000000 / 10);
    return out;
}

#ifdef GNSS_MODE
/* The satellite systems asked for in menuconfig, once the GNSS takes the command (after its "+CGNSSPWR:
 * READY!"; until then it answers ERROR, and this tries again at the next reading). */
static void set_gnss_mode(void)
{
    char mode[16];
    if (command("AT+CGNSSMODE?\r", 2000) != ESP_OK) {
        return;
    }
    if (atoi(answer_field("+CGNSSMODE:", mode, sizeof(mode))) != GNSS_MODE &&
        command(GNSS_MODE_SET, 9000) != ESP_OK) {
        return;
    }
    gnss_mode_set = true;
    ESP_LOGI(TAG, "GNSS: " GNSS_SYSTEMS);
}
#endif

/* The GNSS position, over the CMUX command channel while PPP runs on the other. Before the GNSS is
 * ready the modem answers ERROR: then there is simply no reading this time. */
static void read_gnss(void)
{
    static int had_fix = -1;
    static bool shown_raw;
    static uint32_t last_time;
    gnss_fix_t fix;
#ifdef GNSS_MODE
    if (!gnss_mode_set) {
        set_gnss_mode();
    }
#endif
    if (command("AT+CGNSSINFO\r", 2000) != ESP_OK || !gnss_parse(answer, &fix)) {
        return; /* the next reading, in a few seconds */
    }
    uint32_t fix_time = fix.time; /* (before gnss_clear() zeroes it: each repeat of a stale fix must be caught) */
    if (fix.fix >= GNSS_FIX_2D && fix_time && fix_time == last_time) {
        gnss_clear(&fix); /* the same fix again, its time standing still: the GNSS has lost it */
    }
    last_time = fix_time;
    bridge_set_gnss(&fix);
    int has_fix = fix.fix >= GNSS_FIX_2D;
    if (has_fix && !shown_raw) { /* the modem's own words, once: their form differs between firmware versions */
        shown_raw = true;
        char raw[140];
        ESP_LOGI(TAG, "GNSS: %s", answer_field("+CGNSSINFO:", raw, sizeof(raw)));
    }
    if (has_fix != had_fix) {
        if (has_fix) {
            char lat[16], lon[16];
            ESP_LOGI(TAG, "GNSS: position %s, %s from %u satellites", degrees(lat, sizeof(lat), fix.lat),
                     degrees(lon, sizeof(lon), fix.lon), fix.sats);
            sdlog_event("GNSS fix");
        } else {
            if (had_fix > 0) {
                sdlog_event("GNSS fix lost");
            }
            ESP_LOGI(TAG, "GNSS: no position yet (is its antenna on the board's GNSS connector, under open sky?)");
        }
        had_fix = has_fix;
    }
}
#endif

/* The network as last read (within RADIO_EVERY_MS and a little): registered with an operator, and a signal. */
static bool network_there(void)
{
    xSemaphoreTake(radio_lock, portMAX_DELAY);
    bool there = radio_ms && (uint32_t)(now_ms() - radio_ms) < 3 * RADIO_EVERY_MS &&
                 radio_dbm != BRIDGE_RSSI_UNKNOWN && radio_operator[0];
    xSemaphoreGive(radio_lock);
    return there;
}

static bool early_redial_done; /* in this spell of silence from the relay */

/* Watches the connection until it ends; during a CMUX call, reads the GNSS and the signal too. */
static link_end_t stay_online(void)
{
    uint32_t last_radio = now_ms(), packets = bridge_relay_packets(), up_at = now_ms();
#if CONFIG_BRIDGE_LOCATOR
    uint32_t last_gnss = now_ms() - 60000;
#endif
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(events, PPP_DOWN, pdFALSE, pdFALSE, pdMS_TO_TICKS(1000));
        if (bits & PPP_DOWN) {
            ESP_LOGW(TAG, "mobile data connection lost");
            return LINK_PPP_LOST;
        }
        if (bridge_relay_packets() != packets) { /* the relay answers: a later silence gets its early redial */
            packets = bridge_relay_packets();
            early_redial_done = false;
        }
        /* PPP can stay up while nothing gets through any more; redialling usually cures it */
        uint32_t silence = bridge_relay_silence_ms();
        if (silence > RELAY_SILENCE_LIMIT_MS) {
            ESP_LOGW(TAG, "nothing from the relay for %d minutes; redialling", RELAY_SILENCE_LIMIT_MS / 60000);
            sdlog_event("relay silent: redialling");
            return LINK_RELAY_SILENT;
        }
        /* the network there (registered, a signal) and still no relay: PPP may be wedged (a dead bearer after a gap
         * in coverage, say). One early redial; then the 3 minutes above, whose second redial resets the modem. */
        if (!early_redial_done && silence > RELAY_SILENCE_EARLY_MS && in_cmux && network_there()) {
            early_redial_done = true;
            ESP_LOGW(TAG, "nothing from the relay for %" PRIu32 " s although the network is there; redialling",
                     silence / 1000);
            sdlog_event("relay silent with the network there: redialling");
            return LINK_RELAY_QUIET;
        }
        voice_tick(in_cmux);
        /* the network: as chosen at the relay, and automatic's fallback to 2G while LTE fails. The data call ends
         * first, cleanly, and configure() sets the modem in command mode before the next; one the modem refused waits
         * for the next data call. */
        network_update(true, now_ms() - up_at);
        if (netmode_setting(&netmode) != net_set && !net_refused) {
            ESP_LOGI(TAG, "network: changing it, so ending the data call");
            return LINK_NETWORK;
        }
        if (!in_cmux) {
            continue;
        }
#if CONFIG_BRIDGE_LOCATOR
        if ((uint32_t)(now_ms() - last_gnss) >= CONFIG_BRIDGE_LOCATOR_INTERVAL * 1000) {
            last_gnss = now_ms();
            read_gnss();
        }
#endif
        if ((uint32_t)(now_ms() - last_radio) >= RADIO_EVERY_MS) {
            last_radio = now_ms();
            char name[ESP_MODEM_C_API_STR_BUF_SIZE];
            int act;
            read_radio(name, &act);
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
            pause_ms(wait_s * 1000);
        }
        status_set_modem(STATUS_MODEM_STARTING);
        if (!dce && !create_dce()) {
            failures++;
            continue;
        }
        if (!sync_modem(30000)) {
            leave_cmux();
            if (sync_modem(5000)) {
                /* Its data call from then lives on, and it refuses new ones until it restarts (seen on
                 * the A7670E: ATH does not end it). Only with DIP switch "4G" on: otherwise the ESP32's
                 * restart cuts the modem's power too. */
                ESP_LOGI(TAG, "the modem was still multiplexed (CMUX) from before the ESP32 restarted; restarting it");
                hard_reset();
                continue;
            } else {
                ESP_LOGE(TAG, "the modem does not answer on its UART");
                sdlog_event("modem does not answer");
                failures++;
                hard_reset();
                continue;
            }
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
        if (heard || end == LINK_PPP_LOST || end == LINK_NETWORK) {
            silent = 0;
        } else if (end == LINK_RELAY_SILENT && ++silent >= 2) { /* redialling did not help: reset the modem */
            silent = 0;
            hard_reset();
        } /* (an early redial does not count towards a reset) */
    }
}

void modem_start(void)
{
    board = board_get();
    events = xEventGroupCreate();
    radio_lock = xSemaphoreCreateMutex();
    const esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_PPP();
    ppp_netif = esp_netif_new(&netif_config);
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, on_ppp_status, NULL));
    power_init(); /* first: whether the firmware controls the modem's power decides about the slow UART below */
    nvs_handle_t nvs;
    uint8_t slow = 0, no_cmux = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_SLOW_UART, &slow);
        nvs_get_u8(nvs, NVS_NO_CMUX, &no_cmux);
        /* Before 1.8.5 a failure turned CMUX (and with it the GNSS locator) off for good, and the fast UART with
         * the modem's power under the firmware's control: forgotten, tried again */
        if (no_cmux) {
            nvs_erase_key(nvs, NVS_NO_CMUX);
        }
        if (slow && power_switchable) {
            nvs_erase_key(nvs, NVS_SLOW_UART);
            slow = 0;
        }
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (no_cmux) {
        ESP_LOGI(TAG, "an earlier failure had turned CMUX off for good: trying it again");
    }
    netmode_init(&netmode, NET_AUTO); /* until the relay says */
    snprintf(netmode.why, sizeof(netmode.why), "automatic, until the relay says");
    bridge_set_net_report(netmode_report(&netmode));
    if (slow) {
        fast_baud_failed = true;
        ESP_LOGW(TAG, "modem UART stays at %d baud: it did not work at %d before, with DIP switch \"4G\" on (erase the "
                      "flash to try again)", BOOT_BAUD, CONFIG_BRIDGE_MODEM_BAUD);
    }
    xTaskCreate(modem_task, "modem", 6144, NULL, 10, NULL);
}
