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
#include "freertos/task.h"
#include "nvs.h"

#include "board.h"
#include "bridge.h"
#include "locator.h"
#include "status.h"

#define MODEM_UART UART_NUM_1
#define BOOT_BAUD 115200 /* the A7670E's rate after power-up; AT+IPR changes are not saved */
#define PPP_UP BIT0
#define PPP_DOWN BIT1
#define RELAY_SILENCE_LIMIT_MS (3 * 60 * 1000)
#define NVS_NAMESPACE "modem"
#define NVS_BAD_PIN "bad_pin"     /* the menuconfig SIM PIN that the SIM card rejected */
#define NVS_SLOW_UART "slow_uart" /* 1: the modem did not answer at CONFIG_BRIDGE_MODEM_BAUD */
#define NVS_NO_CMUX "no_cmux"     /* 1: the modem did not take CMUX: plain data calls, no GNSS during them */
#define RADIO_EVERY_MS 10000      /* signal and network, read again during a CMUX data call */

#if CONFIG_BRIDGE_NETWORK_LTE_ONLY
#define NETWORK_MODE 38 /* AT+CNMP: LTE only */
#define SET_NETWORK_MODE "AT+CNMP=38\r"
#else
#define NETWORK_MODE 2 /* AT+CNMP: automatic */
#define SET_NETWORK_MODE "AT+CNMP=2\r"
#endif

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
#if CONFIG_BRIDGE_LOCATOR_VOICE
static bool voice_ready; /* the modem's audio is set up for the locator voice since it last (re)started */
#endif

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
    esp_modem_at(dce, "AT+COPS=3,0", out, 1000); /* the operator's name in AT+COPS?, not its number */
    if (!check_sim()) {
        return false;
    }
    if (esp_modem_at(dce, "AT+CNMP?", out, 1000) == ESP_OK) {
        const char *p = strstr(out, "+CNMP:");
        if (!p || atoi(p + 6) != NETWORK_MODE) {
            command(SET_NETWORK_MODE, 10000); /* the modem saves it, which can take up to 10 s */
        }
    }
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
    }
    command("AT+CGNSSTST=0\r", 2000);
#endif
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

#if CONFIG_BRIDGE_LOCATOR_VOICE
/* ---- the locator voice: the modem's text-to-speech on the board's speaker, while the relay asks for it */

#define VOICE_EVERY_MS 3000   /* a phrase at most this often: the default one takes about 2 s */
#define VOICE_FAILED_MS 15000 /* asked for this long and no phrase taken: the modem does not speak */

static bool voice_asked;  /* the relay asks for the voice, as last seen here */
static bool voice_spoke;  /* the modem has taken a phrase since then */
static bool voice_failed; /* reported as unable to speak */
static uint32_t voice_asked_ms, voice_try_ms, voice_ok_ms;

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

/* About once a second wherever the modem takes AT commands: in command mode, and on the CMUX command
 * channel during a data call (`usable` false where it cannot: a data call without CMUX). While the
 * modem is still saying a phrase it refuses the next one (ERROR), which only means "later". */
static void voice_tick(bool usable)
{
    uint32_t now = now_ms();
    if (!bridge_voice_wanted()) {
        if (voice_asked) {
            voice_asked = false;
            if (usable) {
                command("AT+CTTS=0\r", 2000); /* stops it mid-phrase */
            }
            ESP_LOGI(TAG, "locator voice off");
            bridge_set_voice(false, false);
        }
        return;
    }
    if (!voice_asked) {
        voice_asked = true;
        voice_spoke = voice_failed = false;
        voice_ready = false; /* set the audio up again: the modem may have changed it since */
        voice_asked_ms = now;
        voice_try_ms = now - VOICE_EVERY_MS;
        ESP_LOGI(TAG, "locator voice on: \"%s\" through the board's speaker", CONFIG_BRIDGE_LOCATOR_VOICE_TEXT);
    }
    if (usable && (uint32_t)(now - voice_try_ms) >= VOICE_EVERY_MS) {
        voice_try_ms = now;
        if (!voice_ready) {
            /* as tried on the A7670E-FASE: the speaker phone path and the highest volumes */
            command("AT+CSDVC=3\r", 2000);
            command("AT+COUTGAIN=7\r", 2000);
            command("AT+CTTSPARAM=2,3,0,1,1\r", 2000); /* volume, system volume, digits, pitch, speed */
            voice_ready = true;
        }
        if (command(voice_command(), 3000) == ESP_OK) {
            voice_spoke = true;
            voice_ok_ms = now;
        }
    }
    bool speaking = voice_spoke && (uint32_t)(now - voice_ok_ms) < VOICE_FAILED_MS;
    bool failed = !speaking && (uint32_t)(now - (voice_spoke ? voice_ok_ms : voice_asked_ms)) >= VOICE_FAILED_MS;
    if (failed && !voice_failed) {
        if (usable) {
            ESP_LOGW(TAG, "the locator voice is on, but the modem does not speak (it refuses AT+CTTS)");
        } else {
            ESP_LOGW(TAG, "the locator voice needs the modem's multiplexer (CMUX), which it did not take");
        }
    }
    voice_failed = failed;
    bridge_set_voice(speaking, failed);
}
#else
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
        pause_ms(2000);
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

#if CONFIG_BRIDGE_LOCATOR
static unsigned cmux_failures;

/* The modem refused the multiplexer: after the second time in a row, data calls go without it (and
 * without GNSS readings during them), remembered in flash like the UART speed. */
static void cmux_failed(void)
{
    if (++cmux_failures < 2) {
        ESP_LOGW(TAG, "the modem did not take CMUX; trying again");
        return;
    }
    cmux_off = true;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_NO_CMUX, 1);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGW(TAG, "the modem does not take CMUX: data calls without it from now on, so no GNSS positions during "
                  "them (erase the flash to try again)");
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
    bool restarting = false;
    if (dce) {
        hang_up();
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

typedef enum { LINK_PPP_LOST, LINK_RELAY_SILENT } link_end_t;

#if CONFIG_BRIDGE_LOCATOR
/* 1e-7 degrees as text, without floating point in printf */
static const char *degrees(char *out, size_t n, int32_t v)
{
    uint32_t a = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    snprintf(out, n, "%s%" PRIu32 ".%06" PRIu32, v < 0 ? "-" : "", a / 10000000, a % 10000000 / 10);
    return out;
}

/* The GNSS position, over the CMUX command channel while PPP runs on the other. Before the GNSS is
 * ready the modem answers ERROR: then there is simply no reading this time. */
static void read_gnss(void)
{
    static int had_fix = -1;
    static bool shown_raw;
    static uint32_t last_time;
    gnss_fix_t fix;
    if (command("AT+CGNSSINFO\r", 2000) != ESP_OK || !gnss_parse(answer, &fix)) {
        return; /* the next reading, in a few seconds */
    }
    if (fix.fix >= GNSS_FIX_2D && fix.time && fix.time == last_time) {
        gnss_clear(&fix); /* the same fix again, its time standing still: the GNSS has lost it */
    }
    last_time = fix.time;
    bridge_set_gnss(&fix);
    int has_fix = fix.fix >= GNSS_FIX_2D;
    if (has_fix && !shown_raw) { /* the modem's own words, once: their form differs between firmware versions */
        shown_raw = true;
        char raw[140];
        ESP_LOGI(TAG, "GNSS: %s", answer_field("+CGNSSINFO:", raw, sizeof(raw)));
    }
    if (has_fix != had_fix) {
        had_fix = has_fix;
        if (has_fix) {
            char lat[16], lon[16];
            ESP_LOGI(TAG, "GNSS: position %s, %s from %u satellites", degrees(lat, sizeof(lat), fix.lat),
                     degrees(lon, sizeof(lon), fix.lon), fix.sats);
        } else {
            ESP_LOGI(TAG, "GNSS: no position yet (is its antenna on the board's GNSS connector, under open sky?)");
        }
    }
}
#endif

/* Signal and network again, for the link status (only possible during a CMUX data call). */
static void read_radio(void)
{
    char name[ESP_MODEM_C_API_STR_BUF_SIZE] = "";
    int act = -1;
    int16_t dbm = signal_dbm();
    esp_modem_get_operator_name(dce, name, &act);
    bridge_set_radio(dbm, act >= 0 && act < 0xFF ? (uint8_t)act : BRIDGE_RAT_UNKNOWN);
}

/* Watches the connection until it ends; during a CMUX call, reads the GNSS and the signal too. */
static link_end_t stay_online(void)
{
    uint32_t last_radio = now_ms();
#if CONFIG_BRIDGE_LOCATOR
    uint32_t last_gnss = now_ms() - 60000;
#endif
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
        voice_tick(in_cmux);
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
            read_radio();
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
    uint8_t slow = 0, no_cmux = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_SLOW_UART, &slow);
        nvs_get_u8(nvs, NVS_NO_CMUX, &no_cmux);
        nvs_close(nvs);
    }
    if (no_cmux) {
        cmux_off = true;
        bridge_set_gnss(NULL);
        ESP_LOGW(TAG, "data calls without CMUX, so no GNSS positions during them: the modem did not take it before "
                      "(erase the flash to try again)");
    }
    if (slow) {
        fast_baud_failed = true;
        ESP_LOGW(TAG, "modem UART stays at %d baud: it did not work at %d before (erase the flash to try again)",
                 BOOT_BAUD, CONFIG_BRIDGE_MODEM_BAUD);
    }
    power_init();
    xTaskCreate(modem_task, "modem", 6144, NULL, 10, NULL);
}
