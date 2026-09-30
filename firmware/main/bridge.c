#include "bridge.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "board.h"
#include "mavframe.h"
#include "tunnel.h"
#include "version.h"

#define FC_UART UART_NUM_2 /* UART0 is the USB console, UART1 the modem */
#define NET_UP BIT0
#define STATS_INTERVAL_MS 60000

static const char *TAG = "bridge";

static SemaphoreHandle_t lock; /* guards tun, batcher and sock */
static EventGroupHandle_t net_events;
static volatile uint32_t net_generation; /* incremented each time PPP gets an address */
static volatile uint32_t relay_contact_ms; /* last valid packet from the relay, or PPP up */
static tun_client_t tun;
static mav_batcher_t batcher;
static uint8_t batch_buf[TUN_MAX_PAYLOAD];
static uint8_t key[32];
static int sock = -1;
static bool reopen_socket; /* look the relay up again (its address may have changed) */
static struct sockaddr_in server;
static const uint8_t *downlink; /* set by the tunnel's data callback while tun_input runs */
static size_t downlink_len;

static struct {
    uint32_t fc_rx_bytes;
    uint32_t fc_tx_bytes;
    uint32_t paused_bytes; /* telemetry not sent because no GCS was connected */
    uint32_t send_errors;
} stats;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ---- tunnel callbacks (called with lock held) */

static void tun_send_cb(void *ctx, const uint8_t *pkt, size_t len)
{
    if (sock >= 0 && sendto(sock, pkt, len, 0, (const struct sockaddr *)&server, sizeof(server)) < 0) {
        stats.send_errors++;
    }
}

static void tun_data_cb(void *ctx, const uint8_t *data, size_t len)
{
    downlink = data;
    downlink_len = len;
}

static void tun_event_cb(void *ctx, tun_event_t event, uint32_t session)
{
    switch (event) {
    case TUN_EVENT_CONNECTED:
        ESP_LOGI(TAG, "connected to the relay (session %08" PRIx32 ")", session);
        break;
    case TUN_EVENT_TIMEOUT:
        ESP_LOGW(TAG, "no answer from the relay; reconnecting");
        reopen_socket = true;
        break;
    case TUN_EVENT_REJECTED:
        ESP_LOGI(TAG, "the relay does not know our session any more; reconnecting");
        break;
    }
}

static void tun_random_cb(void *ctx, uint8_t *buf, size_t len)
{
    esp_fill_random(buf, len);
}

static void batch_emit(void *ctx, const uint8_t *data, size_t len)
{
#ifndef CONFIG_BRIDGE_ALWAYS_SEND
    if (!tun.gcs_present) { /* nobody is watching: save mobile data */
        stats.paused_bytes += len;
        return;
    }
#endif
    tun_send_data(&tun, data, len);
}

/* ---- flight controller -> relay */

static void uart_task(void *arg)
{
    static uint8_t buf[256];
    for (;;) {
        xSemaphoreTake(lock, portMAX_DELAY);
        uint32_t due = mav_batcher_due_in(&batcher, now_ms(), CONFIG_BRIDGE_BATCH_MS);
        xSemaphoreGive(lock);
        TickType_t wait = pdMS_TO_TICKS(due < 20 ? due : 20);
        int n = uart_read_bytes(FC_UART, buf, 1, wait ? wait : 1);
        if (n > 0) {
            size_t more = 0;
            uart_get_buffered_data_len(FC_UART, &more);
            if (more > sizeof(buf) - 1) {
                more = sizeof(buf) - 1;
            }
            int m = more ? uart_read_bytes(FC_UART, buf + 1, more, 0) : 0;
            if (m > 0) {
                n += m;
            }
        }
        uint32_t now = now_ms();
        xSemaphoreTake(lock, portMAX_DELAY);
        if (n > 0) {
            stats.fc_rx_bytes += (uint32_t)n;
            mav_batcher_feed(&batcher, buf, (size_t)n, now, batch_emit, NULL);
        }
        mav_batcher_poll(&batcher, now, CONFIG_BRIDGE_BATCH_MS, batch_emit, NULL);
        xSemaphoreGive(lock);
    }
}

/* ---- relay -> flight controller, timers, socket life cycle */

static bool open_socket(void)
{
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_DGRAM};
    struct addrinfo *res = NULL;
    char port[8];
    snprintf(port, sizeof(port), "%d", CONFIG_BRIDGE_SERVER_PORT);
    int err = getaddrinfo(CONFIG_BRIDGE_SERVER_HOST, port, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGW(TAG, "cannot resolve %s (error %d)", CONFIG_BRIDGE_SERVER_HOST, err);
        return false;
    }
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) {
        freeaddrinfo(res);
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return false;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    memcpy(&server, res->ai_addr, sizeof(server));
    sock = s;
    xSemaphoreGive(lock);
    freeaddrinfo(res);
    ESP_LOGI(TAG, "relay %s is %s, port %d", CONFIG_BRIDGE_SERVER_HOST, inet_ntoa(server.sin_addr),
             CONFIG_BRIDGE_SERVER_PORT);
    return true;
}

static void close_socket(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    if (sock >= 0) {
        close(sock);
        sock = -1;
    }
    xSemaphoreGive(lock);
}

static void log_stats(void)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool connected = tun_connected(&tun);
    uint16_t rtt = tun.rtt_ms, loss = tun_loss_permille(&tun);
    bool gcs = tun.gcs_present;
    tun_stats_t ts = tun.stats;
    xSemaphoreGive(lock);
    char rtt_text[12] = "?", loss_text[12] = "?";
    if (rtt != TUN_U16_UNKNOWN) {
        snprintf(rtt_text, sizeof(rtt_text), "%u ms", rtt);
    }
    if (loss != TUN_U16_UNKNOWN) {
        snprintf(loss_text, sizeof(loss_text), "%u.%u%%", loss / 10, loss % 10);
    }
    ESP_LOGI(TAG, "relay %s, rtt %s, downlink loss %s, GCS %s | up %" PRIu32 " B in %" PRIu32 " pkts, down %" PRIu32
             " B | FC rx %" PRIu32 " B tx %" PRIu32 " B | held back %" PRIu32 " B, dropped %" PRIu32 ", errors %" PRIu32,
             connected ? "connected" : "not connected", rtt_text, loss_text, gcs ? "connected" : "absent", ts.tx_bytes,
             ts.tx_packets, ts.rx_bytes, stats.fc_rx_bytes, stats.fc_tx_bytes, stats.paused_bytes, ts.dropped,
             stats.send_errors);
}

static void net_task(void *arg)
{
    static uint8_t rx[TUN_MAX_DATAGRAM + 16];
    uint32_t sock_generation = 0;
    uint32_t last_stats = now_ms();
    for (;;) {
        bool up = xEventGroupGetBits(net_events) & NET_UP;
        if (!up || sock_generation != net_generation) {
            close_socket(); /* the old socket belongs to a previous PPP connection */
        }
        if (!up) {
            xEventGroupWaitBits(net_events, NET_UP, pdFALSE, pdFALSE, pdMS_TO_TICKS(100));
        } else if (sock < 0) {
            sock_generation = net_generation;
            if (!open_socket()) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        } else {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(sock, &fds);
            struct timeval tv = {.tv_sec = 0, .tv_usec = 50 * 1000};
            int r = select(sock + 1, &fds, NULL, NULL, &tv);
            if (r > 0) {
                int n = recv(sock, rx, sizeof(rx), 0);
                if (n > 0) {
                    uint32_t now = now_ms();
                    xSemaphoreTake(lock, portMAX_DELAY);
                    downlink_len = 0;
                    tun_input(&tun, rx, (size_t)n, now);
                    if (tun_connected(&tun) && tun.last_rx == now) { /* it was a valid packet */
                        relay_contact_ms = now;
                    }
                    size_t out_len = downlink_len; /* points into rx, which only this task uses */
                    xSemaphoreGive(lock);
                    if (out_len) {
                        uart_write_bytes(FC_UART, downlink, out_len);
                        stats.fc_tx_bytes += out_len;
                    }
                }
            } else if (r < 0) {
                ESP_LOGW(TAG, "select() failed: errno %d", errno);
                close_socket();
            }
        }
        xSemaphoreTake(lock, portMAX_DELAY);
        tun_poll(&tun, now_ms());
        bool reopen = reopen_socket;
        reopen_socket = false;
        xSemaphoreGive(lock);
        if (reopen) {
            close_socket();
        }
        if ((uint32_t)(now_ms() - last_stats) >= STATS_INTERVAL_MS) {
            last_stats = now_ms();
            log_stats();
        }
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_PPP_GOT_IP) {
        const ip_event_got_ip_t *event = data;
        ESP_LOGI(TAG, "mobile data up, address " IPSTR, IP2STR(&event->ip_info.ip));
        relay_contact_ms = now_ms();
        net_generation++;
        xEventGroupSetBits(net_events, NET_UP);
    } else if (id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGW(TAG, "mobile data down");
        xEventGroupClearBits(net_events, NET_UP);
    }
}

uint32_t bridge_relay_silence_ms(void)
{
    return now_ms() - relay_contact_ms;
}

bridge_state_t bridge_state(void)
{
    bridge_state_t state = {false, false};
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        state.relay = tun_connected(&tun);
        state.gcs = state.relay && tun.gcs_present;
        xSemaphoreGive(lock);
    }
    return state;
}

void bridge_set_radio(int16_t rssi_dbm, uint8_t rat)
{
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        tun_set_radio(&tun, rssi_dbm, rat);
        xSemaphoreGive(lock);
    }
}

static bool parse_key(const char *hex, uint8_t *out, size_t len)
{
    if (strlen(hex) != 2 * len) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        unsigned v;
        if (sscanf(hex + 2 * i, "%2x", &v) != 1) {
            return false;
        }
        out[i] = (uint8_t)v;
    }
    return true;
}

void bridge_start(void)
{
    if (!parse_key(CONFIG_BRIDGE_VEHICLE_KEY, key, sizeof(key))) {
        for (;;) {
            ESP_LOGE(TAG, "Vehicle key is not set: menuconfig -> MavLTE -> Vehicle key "
                          "(64 hex characters, the vehicle_key of your relay)");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    lock = xSemaphoreCreateMutex();
    net_events = xEventGroupCreate();

    const uart_config_t uart_config = {
        .baud_rate = CONFIG_BRIDGE_FC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    const board_t *board = board_get();
    ESP_ERROR_CHECK(uart_driver_install(FC_UART, 4096, 4096, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(FC_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(FC_UART, board->fc_tx_gpio, board->fc_rx_gpio, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    gpio_pullup_en((gpio_num_t)board->fc_rx_gpio); /* keep the line idle-high if the FC is unplugged */

    const tun_config_t tun_config = {
        .role = TUN_ROLE_VEHICLE,
        .key = key,
        .key_len = sizeof(key),
        .info = "mavlte-esp32/" FIRMWARE_VERSION,
        .send = tun_send_cb,
        .on_data = tun_data_cb,
        .on_event = tun_event_cb,
        .random = tun_random_cb,
    };
    tun_init(&tun, &tun_config, now_ms());
    mav_batcher_init(&batcher, batch_buf, sizeof(batch_buf), 500);

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, on_ip_event, NULL));
    xTaskCreate(uart_task, "fc_uart", 4096, NULL, 12, NULL);
    xTaskCreate(net_task, "relay", 6144, NULL, 11, NULL);
    ESP_LOGI(TAG, "flight controller: ESP32 TX GPIO%d -> FC RX, ESP32 RX GPIO%d <- FC TX, %d baud; relay %s:%d",
             board->fc_tx_gpio, board->fc_rx_gpio, CONFIG_BRIDGE_FC_BAUD, CONFIG_BRIDGE_SERVER_HOST,
             CONFIG_BRIDGE_SERVER_PORT);
}
