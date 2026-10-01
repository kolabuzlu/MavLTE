/* Client side of the relay tunnel protocol (docs/PROTOCOL.md).
 *
 * Platform independent: the caller supplies the clock (milliseconds, may wrap), sends datagrams
 * and feeds received ones in. Not thread safe; the caller serialises all calls. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sha256.h"

#define TUN_MAGIC 0xA5
#define TUN_VERSION 1
#define TUN_HEADER_LEN 12
#define TUN_TAG_LEN 16
#define TUN_NONCE_LEN 8
#define TUN_INFO_MAX 64
#define TUN_MAX_DATAGRAM 1200
#define TUN_MAX_PAYLOAD (TUN_MAX_DATAGRAM - TUN_HEADER_LEN - TUN_TAG_LEN)

enum { TUN_HELLO = 1, TUN_WELCOME, TUN_DATA, TUN_PING, TUN_PONG, TUN_REJECT, TUN_STATUS };
/* snapshots (snapshot.h): photos from the aircraft's camera on request */
enum { TUN_SNAP_REQ = 8, TUN_SNAP_INFO, TUN_SNAP_DATA, TUN_SNAP_ACK, TUN_SNAP_SYNC };
enum { TUN_ROLE_SERVER = 0, TUN_ROLE_VEHICLE = 1, TUN_ROLE_GCS = 2 };

#define TUN_U16_UNKNOWN 0xFFFF
#define TUN_RSSI_UNKNOWN 0x7FFF
#define TUN_RAT_UNKNOWN 0xFF
#define TUN_PONG_GCS_PRESENT 0x01
#define TUN_PONG_VOICE 0x02        /* the locator voice is switched on: speak */
#define TUN_PING_SPEAKING 0x02     /* vehicle: the locator voice speaks */
#define TUN_PING_VOICE_FAILED 0x04 /* vehicle: asked to speak, but the modem does not */

typedef enum {
    TUN_EVENT_CONNECTED, /* session established */
    TUN_EVENT_TIMEOUT,   /* nothing heard from the server for link_timeout_ms, or 30 HELLOs went
                            unanswered; a good moment to look the server's address up again */
    TUN_EVENT_REJECTED,  /* the server no longer knows the session; starting over */
} tun_event_t;

typedef struct {
    uint8_t role;
    const uint8_t *key;
    size_t key_len;
    const char *info;           /* sent in HELLO for the server log, may be NULL */
    uint32_t hello_interval_ms; /* 0: 1000 */
    uint32_t ping_interval_ms;  /* 0: 1000 */
    uint32_t link_timeout_ms;   /* 0: 10000 */
    void (*send)(void *ctx, const uint8_t *pkt, size_t len);
    void (*on_data)(void *ctx, const uint8_t *data, size_t len);
    void (*on_event)(void *ctx, tun_event_t event, uint32_t session); /* may be NULL */
    void (*random)(void *ctx, uint8_t *buf, size_t len);
    /* packets of the snapshot types (TUN_SNAP_REQ and up) from the server; may be NULL */
    void (*on_packet)(void *ctx, uint8_t type, const uint8_t *body, size_t len);
    void *ctx;
} tun_config_t;

typedef struct {
    uint32_t tx_packets; /* DATA sent */
    uint32_t tx_bytes;   /* MAVLink bytes sent */
    uint32_t rx_packets; /* DATA received */
    uint32_t rx_bytes;   /* MAVLink bytes received */
    uint32_t dropped;    /* DATA not sent because there was no session */
    uint32_t bad;        /* datagrams that failed a check */
    uint32_t sessions;   /* sessions established */
} tun_stats_t;

#define TUN_LOSS_SECONDS 10

typedef struct {
    tun_config_t cfg;
    hmac_sha256_key_t key;
    uint32_t session; /* 0: none */
    uint32_t tx_seq;
    uint32_t rx_top; /* anti-replay window */
    uint64_t rx_mask;
    uint32_t loss_top; /* downlink loss over the last TUN_LOSS_SECONDS */
    uint32_t loss_expected[TUN_LOSS_SECONDS + 1];
    uint32_t loss_received[TUN_LOSS_SECONDS + 1];
    uint8_t loss_slot;
    uint8_t nonce[TUN_NONCE_LEN];
    bool have_nonce;
    bool hello_due;
    uint8_t hellos; /* HELLOs sent without an answer */
    bool gcs_present; /* from the server's last PONG; true until known */
    bool voice_on;    /* the server's last PONG asked for the locator voice; kept without a session, so an
                         aircraft keeps speaking where it has no coverage */
    uint8_t ping_flags; /* TUN_PING_*, sent in every PING */
    uint16_t rtt_ms;
    int16_t rssi_dbm;
    uint8_t rat;
    uint32_t last_rx, last_ping, last_hello, last_roll;
    tun_stats_t stats;
    uint8_t pkt[TUN_MAX_DATAGRAM];
} tun_client_t;

void tun_init(tun_client_t *t, const tun_config_t *cfg, uint32_t now_ms);
/* Forget the session and start over with HELLO (e.g. after the network came back). */
void tun_restart(tun_client_t *t);
/* Timers: HELLO retries, PINGs, link timeout. Call at least every 100 ms. */
void tun_poll(tun_client_t *t, uint32_t now_ms);
/* A datagram received from the server. */
void tun_input(tun_client_t *t, const uint8_t *pkt, size_t len, uint32_t now_ms);
/* Send MAVLink bytes (whole frames, at most TUN_MAX_PAYLOAD). False if there is no session. */
bool tun_send_data(tun_client_t *t, const uint8_t *data, size_t len);
/* Send a packet of another type on the session (snapshots), body at most TUN_MAX_PAYLOAD bytes.
 * False if there is no session. */
bool tun_send_packet(tun_client_t *t, uint8_t type, const uint8_t *body, size_t len);
/* Radio state reported to the server in PINGs. */
void tun_set_radio(tun_client_t *t, int16_t rssi_dbm, uint8_t rat);
/* Flags reported to the server in PINGs (TUN_PING_*). */
void tun_set_ping_flags(tun_client_t *t, uint8_t flags);
/* Downlink loss in per mille over the last seconds, or TUN_U16_UNKNOWN. */
uint16_t tun_loss_permille(const tun_client_t *t);

static inline bool tun_connected(const tun_client_t *t)
{
    return t->session != 0;
}

/* Builds a datagram into out (TUN_HEADER_LEN + body_len + TUN_TAG_LEN bytes). body may already
 * sit at out + TUN_HEADER_LEN. Returns the datagram length. */
size_t tun_encode(const hmac_sha256_key_t *key, uint8_t *out, uint8_t type, uint8_t role, uint32_t session,
                  uint32_t seq, const uint8_t *body, size_t body_len);
