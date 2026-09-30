/* Stand-in for the ESP32: runs the firmware's tunnel client and batcher against a real relay.
 *
 *   tunnel_harness HOST PORT KEYHEX SECONDS
 *
 * Sends a MAVLink 2 frame (msgid 0, payload = 32-bit counter + 5 zero bytes) every 5 ms while
 * connected and echoes every frame it receives back to the server. Prints its statistics on
 * exit. Used by relay/tests/test_c_client.py. */
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
#include "tunnel.h"

static int sock = -1;
static struct sockaddr_storage server;
static socklen_t server_len;
static tun_client_t tun;
static mav_batcher_t batcher;
static uint8_t batch_buf[TUN_MAX_PAYLOAD];
static uint8_t echo[TUN_MAX_DATAGRAM];
static size_t echo_len;

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
        .send = do_send, .on_data = on_data, .on_event = on_event, .random = rnd,
    };
    tun_init(&tun, &cfg, now_ms());
    mav_batcher_init(&batcher, batch_buf, sizeof(batch_buf), 500);

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
    }
    printf("sessions=%u tx_packets=%u tx_bytes=%u rx_packets=%u rx_bytes=%u dropped=%u bad=%u frames=%u\n",
           (unsigned)tun.stats.sessions, (unsigned)tun.stats.tx_packets, (unsigned)tun.stats.tx_bytes,
           (unsigned)tun.stats.rx_packets, (unsigned)tun.stats.rx_bytes, (unsigned)tun.stats.dropped,
           (unsigned)tun.stats.bad, (unsigned)counter);
    return 0;
}
