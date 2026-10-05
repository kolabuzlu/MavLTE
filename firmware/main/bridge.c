#include "bridge.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "battery.h"
#include "board.h"
#include "camera.h"
#include "fileout.h"
#include "mavframe.h"
#include "locator.h"
#include "logrow.h"
#include "mavpos.h"
#include "sdlog.h"
#include "snapshot.h"
#include "tunnel.h"
#include "version.h"

#define FC_UART UART_NUM_2 /* UART0 is the USB console, UART1 the modem */
#define NET_UP BIT0
#define STATS_INTERVAL_MS 60000
#define RAT_LTE 7
/* the most of the uplink a photo may take (bytes/s): the rest stays for the telemetry, and less when
 * the round trip shows the link filling up */
#define PHOTO_CAP_LTE 32768.0f
#define PHOTO_CAP_2G 2048.0f
#define POSITION_FRESH_MS 5000
#define IP_UDP_BYTES 28        /* each datagram's IPv4 and UDP headers, which the mobile data plan counts too */
#define CLOCK_STEP_MS 2000     /* a time source this far from the clock moves it */
#define FILE_PACKETS 8
#if CONFIG_BRIDGE_LOCATOR
#define GNSS_FRESH_MS (3 * CONFIG_BRIDGE_LOCATOR_INTERVAL * 1000) /* as for the locator's reports */
#else
#define GNSS_FRESH_MS 15000
#endif

static const char *TAG = "bridge";

static SemaphoreHandle_t lock; /* guards tun, batcher and sock */
static EventGroupHandle_t net_events;
static volatile uint32_t net_generation; /* incremented each time PPP gets an address */
static volatile uint32_t relay_contact_ms; /* last valid packet from the relay, or PPP up */
static volatile uint32_t relay_packets;    /* valid packets from the relay since boot */
static tun_client_t tun;
static mav_batcher_t batcher;
static uint8_t batch_buf[TUN_MAX_PAYLOAD];
static uint8_t key[32];
static int sock = -1;
static bool reopen_socket; /* look the relay up again (its address may have changed) */
static struct sockaddr_in server;
static const uint8_t *downlink; /* set by the tunnel's data callback while tun_input runs */
static size_t downlink_len;
static snap_outbox_t snap;       /* snapshots: photos from the camera on request (guarded by lock) */
static mav_position_t position;  /* where the flight controller says the aircraft is, and its last HEARTBEAT (guarded by lock) */
/* the locator (guarded by lock): the modem's GNSS, and when the flight controller was last heard */
static gnss_fix_t gnss;
static uint32_t gnss_ms;       /* when gnss was read */
static int gnss_state;         /* 0: not read yet, 1: readable, -1: cannot be read (no CMUX) */
static uint16_t battery_mv = LOCATOR_U16_UNKNOWN; /* the board's cell, from its fuel gauge */
static uint8_t battery_pct = LOCATOR_BATTERY_UNKNOWN;
static uint64_t data_bytes;          /* mobile data both ways since power-on (guarded by lock) */
static bool clock_set, clock_gnss;   /* the system clock has the time, from the GNSS (guarded by lock) */
/* logs over 4G (fileout.h): the outbox runs in a task of its own, which may wait for the card; the tunnel's
 * callback hands it the relay's packets through a queue */
static file_outbox_t files;
static sdlog_file_t file_4g;
static QueueHandle_t file_packets;
typedef struct {
    uint8_t type, len;
    uint8_t body[FILE_REQ_LEN];
} file_packet_t;
#if CONFIG_BRIDGE_CAMERA
static QueueHandle_t camera_jobs; /* for the camera task, which may take a second or two per photo */
typedef struct {
    bool release; /* else take */
    uint32_t photo_id;
    uint16_t width, height;
} camera_job_t;
#endif

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
    if (sock < 0) {
        return;
    }
    if (sendto(sock, pkt, len, 0, (const struct sockaddr *)&server, sizeof(server)) < 0) {
        stats.send_errors++;
    } else {
        data_bytes += len + IP_UDP_BYTES;
    }
}

static void tun_data_cb(void *ctx, const uint8_t *data, size_t len)
{
    downlink = data;
    downlink_len = len;
}

/* The system clock, for the flight log's times (called with lock held): the relay's at each connection (its WELCOME
 * carries it since 1.8.0) until the module's GNSS has a fix, then the GNSS's, which needs no network. */
static void set_clock(uint64_t unix_ms, bool from_gnss)
{
    if (unix_ms < 1577836800000ULL || (clock_gnss && !from_gnss)) { /* before 2020: not a time */
        return;
    }
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t off = (int64_t)unix_ms - ((int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000);
    if (clock_set && llabs(off) < CLOCK_STEP_MS) {
        clock_gnss |= from_gnss;
        return;
    }
    tv.tv_sec = (time_t)(unix_ms / 1000);
    tv.tv_usec = (suseconds_t)(unix_ms % 1000 * 1000);
    settimeofday(&tv, NULL);
    char text[24];
    log_time_text(text, (uint32_t)tv.tv_sec);
    const char *source = from_gnss ? "GNSS" : "relay";
    if (clock_set) {
        ESP_LOGI(TAG, "clock moved %lld ms, to the %s's time: %s", (long long)off, source, text);
    } else {
        ESP_LOGI(TAG, "clock set from the %s: %s", source, text);
    }
    sdlog_event("clock set from the %s", source);
    clock_set = true;
    clock_gnss |= from_gnss;
}

static void tun_event_cb(void *ctx, tun_event_t event, uint32_t session)
{
    switch (event) {
    case TUN_EVENT_CONNECTED:
        ESP_LOGI(TAG, "connected to the relay (session %08" PRIx32 ")", session);
        sdlog_event("relay connected");
        snap_restart(&snap, now_ms()); /* perhaps a new relay, that knows nothing of a photo on its way */
        if (tun.server_ms) {
            set_clock(tun.server_ms + (uint32_t)(now_ms() - tun.server_ms_at), false);
        }
        break;
    case TUN_EVENT_TIMEOUT:
        ESP_LOGW(TAG, "no answer from the relay; reconnecting");
        sdlog_event("relay not answering");
        reopen_socket = true;
        break;
    case TUN_EVENT_REJECTED:
        ESP_LOGI(TAG, "the relay does not know our session any more; reconnecting");
        sdlog_event("relay forgot the session");
        break;
    }
}

static void tun_random_cb(void *ctx, uint8_t *buf, size_t len)
{
    esp_fill_random(buf, len);
}

static void tun_packet_cb(void *ctx, uint8_t type, const uint8_t *body, size_t len)
{
    if (type == TUN_FILE_REQ || type == TUN_FILE_ACK) { /* for the logs' task, which may be busy with the card */
        file_packet_t p = {.type = type, .len = (uint8_t)(len < sizeof(p.body) ? len : sizeof(p.body))};
        memcpy(p.body, body, p.len);
        xQueueSend(file_packets, &p, 0); /* full: dropped, and the agent asks again */
        return;
    }
    snap_input(&snap, type, body, len, now_ms());
}

/* ---- snapshots (callbacks called with lock held) */

#if CONFIG_BRIDGE_CAMERA
static void snap_take_cb(void *ctx, uint32_t photo_id, uint16_t width, uint16_t height)
{
    ESP_LOGI(TAG, "photo %" PRIu32 " asked for (%ux%u)", photo_id, width, height);
    sdlog_event("photo %" PRIu32 " asked for (%ux%u)", photo_id, width, height);
    const camera_job_t job = {.release = false, .photo_id = photo_id, .width = width, .height = height};
    if (xQueueSend(camera_jobs, &job, 0) != pdTRUE) {
        snap_photo_taken(&snap, photo_id, SNAP_FAILED, NULL, 0, NULL, now_ms());
    }
}
#endif

static void snap_release_cb(void *ctx, uint32_t photo_id, bool sent)
{
    if (sent) {
        ESP_LOGI(TAG, "photo %" PRIu32 " sent in %" PRIu32 ".%" PRIu32 " s", photo_id, snap.last_ms / 1000,
                 snap.last_ms % 1000 / 100);
        sdlog_event("photo %" PRIu32 " sent", photo_id);
    } else if (snap.last_id == photo_id) {
        ESP_LOGW(TAG, "photo %" PRIu32 ": no word from the relay for a minute; given up", photo_id);
        sdlog_event("photo %" PRIu32 " given up", photo_id);
    }
#if CONFIG_BRIDGE_CAMERA
    /* no waiting here, with the lock held: if the queue were full, the next photo frees this one */
    const camera_job_t job = {.release = true, .photo_id = photo_id};
    xQueueSend(camera_jobs, &job, 0);
#endif
}

static float photo_cap(void)
{
    return tun.rat == RAT_LTE ? PHOTO_CAP_LTE : PHOTO_CAP_2G;
}

#if CONFIG_BRIDGE_CAMERA
static void camera_task(void *arg)
{
    camera_job_t job;
    for (;;) {
        xQueueReceive(camera_jobs, &job, portMAX_DELAY);
        if (job.release) {
            camera_release();
            continue;
        }
        const uint8_t *jpeg = NULL;
        size_t len = 0;
        uint8_t status = camera_take(job.width, job.height, &jpeg, &len);
        uint32_t now = now_ms();
        xSemaphoreTake(lock, portMAX_DELAY);
        snap_where_t where = {SNAP_UNKNOWN_I32, SNAP_UNKNOWN_I32, SNAP_UNKNOWN_I32, SNAP_UNKNOWN_HEADING};
        if (position.valid && (uint32_t)(now - position.when_ms) < POSITION_FRESH_MS) {
            where = (snap_where_t){position.lat, position.lon, position.alt_mm, position.heading};
        }
        snap_photo_taken(&snap, job.photo_id, status, status == SNAP_OK ? jpeg : NULL, len, &where, now);
        xSemaphoreGive(lock);
    }
}
#endif

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
            mav_position_feed(&position, buf, (size_t)n, now); /* for the notes on a photo */
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
    char rtt_text[12] = "?", loss_text[12] = "?", chip_text[12] = "?";
    unsigned heap_kb = (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    unsigned block_kb = (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024);
    int8_t chip = board_chip_temp();
    if (chip != INT8_MIN) {
        snprintf(chip_text, sizeof(chip_text), "%d C", chip);
    }
    if (rtt != TUN_U16_UNKNOWN) {
        snprintf(rtt_text, sizeof(rtt_text), "%u ms", rtt);
    }
    if (loss != TUN_U16_UNKNOWN) {
        snprintf(loss_text, sizeof(loss_text), "%u.%u%%", loss / 10, loss % 10);
    }
    ESP_LOGI(TAG, "relay %s, rtt %s, downlink loss %s, GCS %s | up %" PRIu32 " B in %" PRIu32 " pkts, down %" PRIu32
             " B | FC rx %" PRIu32 " B tx %" PRIu32 " B | held back %" PRIu32 " B, dropped %" PRIu32 ", errors %" PRIu32
             " | chip %s | heap %u KB free, %u KB in one piece",
             connected ? "connected" : "not connected", rtt_text, loss_text, gcs ? "connected" : "absent", ts.tx_bytes,
             ts.tx_packets, ts.rx_bytes, stats.fc_rx_bytes, stats.fc_tx_bytes, stats.paused_bytes, ts.dropped,
             stats.send_errors, chip_text, heap_kb, block_kb);
}

#if CONFIG_BRIDGE_LOCATOR
/* The locator's report (called with lock held): the last GNSS fix, if it is recent, and how long the
 * flight controller has been silent; after a crash too, as long as the board has power. */
static void send_position(uint32_t now)
{
    uint8_t body[LOCATOR_BODY_LEN];
    uint8_t flags = 0;
    const gnss_fix_t *fix = NULL;
    gnss_fix_t none;
    if (gnss_state < 0) {
        flags |= LOCATOR_NO_GNSS;
    } else if (gnss_state > 0 && (uint32_t)(now - gnss_ms) < 3 * CONFIG_BRIDGE_LOCATOR_INTERVAL * 1000) {
        fix = &gnss;
    } else {
        gnss_clear(&none); /* not read (lately): no fix */
        fix = &none;
    }
    uint16_t silent = LOCATOR_U16_UNKNOWN;
    if (position.heartbeat) { /* the flight controller's last HEARTBEAT, not any byte: noise is not one */
        uint32_t s = (uint32_t)(now - position.heartbeat_ms) / 1000;
        silent = s < LOCATOR_U16_UNKNOWN ? (uint16_t)s : LOCATOR_U16_UNKNOWN - 1;
        if (silent >= LOCATOR_FC_SILENT_S) {
            flags |= LOCATOR_FC_SILENT;
        }
    }
    locator_pack(body, fix, flags, silent, battery_mv, battery_pct, board_chip_temp());
    tun_send_packet(&tun, TUN_POSITION, body, sizeof(body));
}
#endif

static void net_task(void *arg)
{
    static uint8_t rx[TUN_MAX_DATAGRAM + 16];
    uint32_t sock_generation = 0;
    uint32_t last_stats = now_ms();
#if CONFIG_BRIDGE_LOCATOR
    uint32_t last_position = now_ms();
    uint32_t last_battery = now_ms() - 60000, battery_every = 30000;
    unsigned battery_misses = 0;
#endif
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
            xSemaphoreTake(lock, portMAX_DELAY);
            bool photo = snap_busy(&snap); /* a photo goes out in small steps */
            xSemaphoreGive(lock);
            struct timeval tv = {.tv_sec = 0, .tv_usec = (photo ? 10 : 50) * 1000};
            int r = select(sock + 1, &fds, NULL, NULL, &tv);
            if (r > 0) {
                int n = recv(sock, rx, sizeof(rx), 0);
                if (n > 0) {
                    uint32_t now = now_ms();
                    xSemaphoreTake(lock, portMAX_DELAY);
                    data_bytes += (uint32_t)n + IP_UDP_BYTES;
                    downlink_len = 0;
                    tun_input(&tun, rx, (size_t)n, now);
                    if (tun_connected(&tun) && tun.last_rx == now) { /* it was a valid packet */
                        relay_contact_ms = now;
                        relay_packets++;
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
#if CONFIG_BRIDGE_LOCATOR
        if ((uint32_t)(now_ms() - last_battery) >= battery_every) { /* the cell, for the locator */
            last_battery = now_ms();
            uint16_t mv = LOCATOR_U16_UNKNOWN;
            uint8_t pct = LOCATOR_BATTERY_UNKNOWN;
            bool read = battery_read(&mv, &pct);
            battery_misses = read ? 0 : battery_misses + 1;
            battery_every = battery_misses < 3 ? 30000 : 600000; /* no gauge: seldom try again */
            xSemaphoreTake(lock, portMAX_DELAY);
            battery_mv = read ? mv : LOCATOR_U16_UNKNOWN;
            battery_pct = read ? pct : LOCATOR_BATTERY_UNKNOWN;
            xSemaphoreGive(lock);
        }
#endif
        xSemaphoreTake(lock, portMAX_DELAY);
        tun_poll(&tun, now_ms());
        snap_poll(&snap, now_ms(), photo_cap());
#if CONFIG_BRIDGE_LOCATOR
        if (tun_connected(&tun) && (uint32_t)(now_ms() - last_position) >= CONFIG_BRIDGE_LOCATOR_INTERVAL * 1000) {
            last_position = now_ms();
            send_position(last_position);
        }
#endif
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
        sdlog_event("mobile data up");
        relay_contact_ms = now_ms();
        net_generation++;
        xEventGroupSetBits(net_events, NET_UP);
    } else if (id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGW(TAG, "mobile data down");
        sdlog_event("mobile data down");
        xEventGroupClearBits(net_events, NET_UP);
    }
}

uint32_t bridge_relay_silence_ms(void)
{
    /* the contact first: one the relay task stamps after our clock reading would make the silence wrap around to
     * some 49 days (and the modem redial for nothing) */
    uint32_t contact = relay_contact_ms;
    return now_ms() - contact;
}

uint32_t bridge_relay_packets(void)
{
    return relay_packets;
}

bridge_state_t bridge_state(void)
{
    bridge_state_t state = {false, false, false};
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        uint32_t now = now_ms(); /* (under the lock: a HEARTBEAT stamped after it would look 49 days old) */
        state.relay = tun_connected(&tun);
        state.gcs = state.relay && tun.gcs_present;
        state.fc = position.heartbeat && (uint32_t)(now - position.heartbeat_ms) < LOCATOR_FC_SILENT_S * 1000;
        xSemaphoreGive(lock);
    }
    return state;
}

void bridge_set_gnss(const gnss_fix_t *fix)
{
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        if (fix) {
            gnss = *fix;
            gnss_ms = now_ms();
            gnss_state = 1;
            if (fix->fix >= GNSS_FIX_2D && fix->time) {
                set_clock((uint64_t)fix->time * 1000, true);
            }
        } else {
            gnss_state = -1;
        }
        xSemaphoreGive(lock);
    }
}

void bridge_set_radio(int16_t rssi_dbm, uint8_t rat)
{
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        tun_set_radio(&tun, rssi_dbm, rat);
        xSemaphoreGive(lock);
    }
}

bool bridge_voice_wanted(void)
{
    bool on = false;
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        on = tun.voice_on;
        xSemaphoreGive(lock);
    }
    return on;
}

void bridge_set_voice(bool speaking, bool failed)
{
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        tun_set_ping_flags(&tun, (speaking ? TUN_PING_SPEAKING : 0) | (failed ? TUN_PING_VOICE_FAILED : 0));
        xSemaphoreGive(lock);
    }
}

void bridge_log_row(log_row_t *row)
{
    if (!lock) {
        return;
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    uint32_t now = now_ms(); /* (under the lock: a reading stamped after it would look 49 days old) */
    if (clock_set) {
        row->utc = (uint32_t)time(NULL);
    }
    /* a reading the locator would no longer send (no data call to read it in, say) counts as none */
    if (gnss_state < 0 || (gnss_state > 0 && (uint32_t)(now - gnss_ms) < GNSS_FRESH_MS)) {
        row->gnss_state = gnss_state;
        row->gnss = gnss;
        row->gnss_age_s = (now - gnss_ms) / 1000;
    }
    row->relay = tun_connected(&tun);
    row->rtt_ms = tun.rtt_ms;
    row->loss_permille = tun_loss_permille(&tun);
    row->data_kb = (uint32_t)(data_bytes / 1024);
    row->gcs = tun.gcs_present;
    row->fc = position;
    row->now_ms = now;
    row->rail_mv = battery_mv;
    row->cell_pct = battery_pct;
    row->voice = !tun.voice_on                                ? 0
                 : (tun.ping_flags & TUN_PING_SPEAKING)       ? 2
                 : (tun.ping_flags & TUN_PING_VOICE_FAILED)   ? 3
                                                              : 1;
    xSemaphoreGive(lock);
    row->chip_c = board_chip_temp();
}

/* ---- logs over 4G (fileout.h), from the flight log's card */

static int files_list_cb(void *ctx, unsigned first, file_entry_t *out, unsigned max, unsigned *total)
{
    return sdlog_list(first, out, max, total);
}

static int files_open_cb(void *ctx, const char *name, uint32_t *size)
{
    return sdlog_open(&file_4g, name, size);
}

static bool files_read_cb(void *ctx, uint32_t offset, uint8_t *buf, size_t len)
{
    return sdlog_read(&file_4g, offset, buf, len);
}

static void files_close_cb(void *ctx)
{
    sdlog_close(&file_4g);
}

static bool files_send_cb(void *ctx, uint8_t type, const uint8_t *body, size_t len)
{
    xSemaphoreTake(lock, portMAX_DELAY);
    bool sent = tun_send_packet(&tun, type, body, len);
    xSemaphoreGive(lock);
    return sent;
}

static void files_task(void *arg)
{
    file_packet_t p;
    for (;;) {
        TickType_t wait = fileout_busy(&files) ? pdMS_TO_TICKS(10) : portMAX_DELAY;
        while (xQueueReceive(file_packets, &p, wait) == pdTRUE) {
            fileout_input(&files, p.type, p.body, p.len, now_ms());
            wait = 0;
        }
        xSemaphoreTake(lock, portMAX_DELAY);
        bool photo = snap_busy(&snap); /* a photo goes first */
        uint16_t rtt = tun.rtt_ms;
        float cap = photo_cap();
        xSemaphoreGive(lock);
        if (!photo && fileout_poll(&files, now_ms(), rtt, cap)) {
            const char *how = files.last_complete ? "sent" : "stopped";
            ESP_LOGI(TAG, "log %s %s over 4G (%" PRIu32 " bytes)", files.last_name, how, files.last_bytes);
            sdlog_event("log %s %s over 4G", files.last_name, how);
        }
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
    /* Until the UART took it, our TX pin floated (reset, boot): the flight controller may have taken that noise
     * for the start of a MAVLink frame and now waits for up to MAV_MAX_FRAME bytes of it, which would swallow the
     * first messages from the GCS (seen: up to 10 parameter reads after 3 of 4 resets). Zero bytes complete such a
     * frame, which then fails its CRC, and are ignored between frames. */
    static const uint8_t zeros[MAV_MAX_FRAME] = {0};
    uart_write_bytes(FC_UART, zeros, sizeof(zeros));

    const tun_config_t tun_config = {
        .role = TUN_ROLE_VEHICLE,
        .key = key,
        .key_len = sizeof(key),
        .info = "mavlte-esp32/" FIRMWARE_VERSION,
        .send = tun_send_cb,
        .on_data = tun_data_cb,
        .on_event = tun_event_cb,
        .random = tun_random_cb,
        .on_packet = tun_packet_cb,
    };
    tun_init(&tun, &tun_config, now_ms());
    mav_batcher_init(&batcher, batch_buf, sizeof(batch_buf), 500);
    mav_position_init(&position);
    snap_config_t snap_config = {.release = snap_release_cb}; /* without take(): "no camera" */
#if CONFIG_BRIDGE_CAMERA
    camera_jobs = xQueueCreate(4, sizeof(camera_job_t));
    snap_config.take = snap_take_cb;
    xTaskCreate(camera_task, "camera", 5120, NULL, 5, NULL);
#endif
    snap_init(&snap, &tun, &snap_config);
    file_packets = xQueueCreate(FILE_PACKETS, sizeof(file_packet_t));
    const file_config_t files_config = {
        .list = files_list_cb,
        .open = files_open_cb,
        .read = files_read_cb,
        .close = files_close_cb,
        .send = files_send_cb,
    };
    fileout_init(&files, &files_config);
    xTaskCreate(files_task, "logs_4g", 4096, NULL, 6, NULL);

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, on_ip_event, NULL));
    xTaskCreate(uart_task, "fc_uart", 4096, NULL, 12, NULL);
    xTaskCreate(net_task, "relay", 6144, NULL, 11, NULL);
    ESP_LOGI(TAG, "flight controller: ESP32 TX GPIO%d -> FC RX, ESP32 RX GPIO%d <- FC TX, %d baud; relay %s:%d",
             board->fc_tx_gpio, board->fc_rx_gpio, CONFIG_BRIDGE_FC_BAUD, CONFIG_BRIDGE_SERVER_HOST,
             CONFIG_BRIDGE_SERVER_PORT);
}
