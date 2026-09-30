/* Stand-in for the ESP32: runs the firmware's tunnel client, batcher and snapshot outbox against a
 * real relay.
 *
 *   tunnel_harness HOST PORT KEYHEX SECONDS
 *
 * Sends a MAVLink 2 frame (msgid 0, payload = 32-bit counter + 5 zero bytes) every 5 ms while
 * connected and echoes every frame it receives back to the server. Its camera takes the same
 * photo every time: PHOTO_BYTES bytes, byte i = (i * 13 + 7) & 0xFF, over Istanbul at 120 m, heading
 * 45 degrees. Prints its statistics on exit. Used by relay/tests/test_c_client.py. */
#define _POSIX_C_SOURCE 200809L

#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "mavframe.h"
#include "snapshot.h"
#include "tunnel.h"

#define PHOTO_BYTES 30000

static int sock = -1;
static struct sockaddr_storage server;
static socklen_t server_len;
static tun_client_t tun;
static mav_batcher_t batcher;
static uint8_t batch_buf[TUN_MAX_PAYLOAD];
static uint8_t echo[TUN_MAX_DATAGRAM];
static size_t echo_len;
static snap_outbox_t snap;
static uint8_t photo[PHOTO_BYTES];
static unsigned photos_sent;

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000);
}

static void do_send(void *ctx, const uint8_t *p, size_t n)
{
    (void)ctx;
    sendto(sock, p, n, 0, (const struct sockaddr *)&server, server_len);
}

static void on_data(void *ctx, const uint8_t *p, size_t n)
{
    (void)ctx;
    memcpy(echo, p, n); /* echoed after tun_input returns */
    echo_len = n;
}

static void on_event(void *ctx, tun_event_t ev, uint32_t session)
{
    (void)ctx;
    static const char *names[] = {"connected", "timeout", "rejected"};
    fprintf(stderr, "harness: %s (session %08x)\n", names[ev], (unsigned)session);
    if (ev == TUN_EVENT_CONNECTED) {
        snap_restart(&snap, now_ms());
    }
}

static void on_packet(void *ctx, uint8_t type, const uint8_t *body, size_t len)
{
    (void)ctx;
    snap_input(&snap, type, body, len, now_ms());
}

static void take(void *ctx, uint32_t photo_id, uint16_t width, uint16_t height)
{
    (void)ctx;
    fprintf(stderr, "harness: photo %u asked for (%ux%u)\n", (unsigned)photo_id, width, height);
    const snap_where_t where = {411234567, 289876543, 120000, 4500};
    snap_photo_taken(&snap, photo_id, SNAP_OK, photo, sizeof(photo), &where, now_ms());
}

static void release(void *ctx, uint32_t photo_id, bool sent)
{
    (void)ctx;
    fprintf(stderr, "harness: photo %u %s\n", (unsigned)photo_id, sent ? "sent" : "given up");
    photos_sent += sent;
}

static void rnd(void *ctx, uint8_t *p, size_t n)
{
    (void)ctx;
    for (size_t i = 0; i < n; i++) {
        p[i] = (uint8_t)rand();
    }
}

static void emit(void *ctx, const uint8_t *p, size_t n)
{
    (void)ctx;
    tun_send_data(&tun, p, n);
}

static size_t make_frame(uint8_t *out, uint32_t counter)
{
    static uint8_t seq;
    uint8_t f[21] = {0xFD, 9, 0, 0, seq++, 1, 1, 0, 0, 0};
    memcpy(f + 10, &counter, 4);
    f[19] = 0x12;
    f[20] = 0x34;
    memcpy(out, f, sizeof(f));
    return sizeof(f);
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "usage: %s HOST PORT KEYHEX SECONDS\n", argv[0]);
        return 2;
    }
    uint8_t key[64];
    size_t key_len = strlen(argv[3]) / 2;
    for (size_t i = 0; i < key_len && i < sizeof(key); i++) {
        unsigned v;
        sscanf(argv[3] + 2 * i, "%2x", &v);
        key[i] = (uint8_t)v;
    }
    struct addrinfo hints = {.ai_family = AF_UNSPEC, .ai_socktype = SOCK_DGRAM}, *res;
    if (getaddrinfo(argv[1], argv[2], &hints, &res) != 0) {
        fprintf(stderr, "cannot resolve %s\n", argv[1]);
        return 2;
    }
    memcpy(&server, res->ai_addr, res->ai_addrlen);
    server_len = res->ai_addrlen;
    sock = socket(res->ai_family, SOCK_DGRAM, 0);
    freeaddrinfo(res);
    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    tun_config_t cfg = {
        .role = TUN_ROLE_VEHICLE, .key = key, .key_len = key_len, .info = "tunnel_harness",
        .send = do_send, .on_data = on_data, .on_event = on_event, .random = rnd, .on_packet = on_packet,
    };
    tun_init(&tun, &cfg, now_ms());
    mav_batcher_init(&batcher, batch_buf, sizeof(batch_buf), 500);
    for (size_t i = 0; i < sizeof(photo); i++) {
        photo[i] = (uint8_t)(i * 13 + 7);
    }
    const snap_config_t snap_cfg = {.take = take, .release = release};
    snap_init(&snap, &tun, &snap_cfg);

    uint32_t start = now_ms(), last_frame = start, counter = 0;
    uint32_t duration = (uint32_t)atoi(argv[4]) * 1000;
    uint8_t rx[2048];
    while ((uint32_t)(now_ms() - start) < duration) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        struct timeval tv = {0, 2000};
        if (select(sock + 1, &fds, NULL, NULL, &tv) > 0) {
            ssize_t n = recv(sock, rx, sizeof(rx), 0);
            if (n > 0) {
                tun_input(&tun, rx, (size_t)n, now_ms());
                if (echo_len) {
                    mav_batcher_feed(&batcher, echo, echo_len, now_ms(), emit, NULL);
                    echo_len = 0;
                }
            }
        }
        uint32_t now = now_ms();
        tun_poll(&tun, now);
        if (tun_connected(&tun) && (uint32_t)(now - last_frame) >= 5) {
            uint8_t frame[32];
            last_frame = now;
            mav_batcher_feed(&batcher, frame, make_frame(frame, counter++), now, emit, NULL);
        }
        mav_batcher_poll(&batcher, now, 20, emit, NULL);
        snap_poll(&snap, now, 65536.0f);
    }
    printf("sessions=%u tx_packets=%u tx_bytes=%u rx_packets=%u rx_bytes=%u dropped=%u bad=%u frames=%u "
           "photos=%u\n",
           (unsigned)tun.stats.sessions, (unsigned)tun.stats.tx_packets, (unsigned)tun.stats.tx_bytes,
           (unsigned)tun.stats.rx_packets, (unsigned)tun.stats.rx_bytes, (unsigned)tun.stats.dropped,
           (unsigned)tun.stats.bad, (unsigned)counter, photos_sent);
    return 0;
}
