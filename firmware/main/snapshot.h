/* Snapshots, the aircraft's side (docs/PROTOCOL.md, "Snapshots"): answers the relay's SNAP_REQ,
 * has the camera take the photo, and sends it (SNAP_INFO, then SNAP_DATA chunks) as fast as the link
 * carries it without delaying the telemetry, and again whatever the relay's SNAP_ACKs do not show.
 * The same logic as PhotoOutbox, PhotoSender and RateControl in relay/mavrelay.py.
 *
 * Platform independent, like tunnel.c: the caller's camera takes the photo in its own time (a
 * second or two) and hands it over with snap_photo_taken(). Not thread safe; the caller serialises
 * all calls, together with the tunnel's. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tunnel.h"

#define SNAP_CHUNK 1024
#define SNAP_MAX_CHUNKS 256 /* photos up to 256 KB: a 1024x768 JPEG from the OV5640 is 25-80 KB */
#define SNAP_INFO_LEN 31
#define SNAP_SIZES 3 /* 0: 320x240, 1: 640x480, 2: 1024x768 */
#define SNAP_UNKNOWN_I32 INT32_MIN
#define SNAP_UNKNOWN_HEADING 0xFFFF
#define SNAP_ACK_DONE 0x01
#define SNAP_ACK_HAVE_INFO 0x02
#define SNAP_ANSWERS 16 /* requests remembered, so that the relay asking again gets the same answer */

enum { SNAP_OK, SNAP_NO_AIRCRAFT, SNAP_NO_CAMERA, SNAP_FAILED, SNAP_BUSY, SNAP_NO_ANSWER };

extern const uint16_t snap_sizes[SNAP_SIZES][2];

typedef struct {
    int32_t lat, lon; /* 1e-7 degrees */
    int32_t alt_mm;   /* above home */
    uint16_t heading; /* centidegrees */
} snap_where_t;

typedef struct {
    /* Take a photo, width x height, and hand it over with snap_photo_taken(). NULL: no camera. */
    void (*take)(void *ctx, uint32_t photo_id, uint16_t width, uint16_t height);
    /* The photo's bytes are no longer needed: sent (sent true), or given up. */
    void (*release)(void *ctx, uint32_t photo_id, bool sent);
    void *ctx;
} snap_config_t;

/* How fast a photo may go: backs off when the round trip rises above its recent minimum (a queue
 * building up in the network) and creeps back up while it does not, as LEDBAT does (RFC 6817). */
typedef struct {
    float cap, rate, tokens; /* bytes/s, bytes/s, bytes */
    uint32_t last_ms;
    bool have_last;
    uint32_t backed_off_ms;
    bool backed_off;
    uint16_t rtts[60]; /* the round trips (ms) of the last minute, one a second */
    uint32_t rtt_ms_at[60];
    uint8_t rtt_count, rtt_next;
} snap_rate_t;

void snap_rate_init(snap_rate_t *r, float cap, float start);
void snap_rate_set_cap(snap_rate_t *r, float cap);
void snap_rate_sample(snap_rate_t *r, uint16_t rtt_ms, uint32_t now_ms);
/* Bytes that may be sent now. */
float snap_rate_budget(snap_rate_t *r, uint32_t now_ms);

typedef struct {
    snap_config_t cfg;
    tun_client_t *tun;
    uint32_t answer_id[SNAP_ANSWERS];
    uint8_t answer[SNAP_ANSWERS]; /* SNAP_OK (taken, or being taken) or the problem */
    uint8_t answers, answer_next;
    bool taking; /* the camera is at work */
    uint32_t taking_id;
    uint16_t taking_w, taking_h;
    /* the photo on its way */
    bool sending;
    uint8_t info[SNAP_INFO_LEN];
    uint32_t photo_id;
    const uint8_t *data;
    uint32_t size;
    uint16_t chunks;
    uint8_t acked[SNAP_MAX_CHUNKS / 8];
    uint8_t sent[SNAP_MAX_CHUNKS / 8]; /* sent at least once (since the last restart) */
    uint32_t sent_ms[SNAP_MAX_CHUNKS];
    bool info_acked, info_sent, done;
    uint32_t info_ms, last_ack_ms, started_ms;
    snap_rate_t rate;
    uint32_t rtt_ms_at;
    /* the last photo, for the log: id, sent or given up, how long it took */
    uint32_t last_id, last_ms;
    bool last_sent;
    uint8_t pkt[6 + SNAP_CHUNK]; /* a SNAP_DATA body */
} snap_outbox_t;

void snap_init(snap_outbox_t *o, tun_client_t *tun, const snap_config_t *cfg);
/* A snapshot packet from the relay (the tunnel's on_packet). */
void snap_input(snap_outbox_t *o, uint8_t type, const uint8_t *body, size_t len, uint32_t now_ms);
/* The camera's photo for take(): status SNAP_OK and the JPEG, which the caller keeps until release(),
 * or SNAP_NO_CAMERA or SNAP_FAILED. where: the aircraft's position then, or NULL. */
void snap_photo_taken(snap_outbox_t *o, uint32_t photo_id, uint8_t status, const uint8_t *jpeg, size_t len,
                      const snap_where_t *where, uint32_t now_ms);
/* A new session with the relay, which may be a new one that knows nothing of the photo on its way:
 * everything goes again (whatever the relay still has, its first ACK tells). */
void snap_restart(snap_outbox_t *o, uint32_t now_ms);
/* Sends what is due. Call every 10-20 ms while snap_busy(). cap: the most bytes/s a photo may take
 * on the network the modem is on (the rest stays for the telemetry). */
void snap_poll(snap_outbox_t *o, uint32_t now_ms, float cap);

static inline bool snap_busy(const snap_outbox_t *o)
{
    return o->taking || o->sending;
}

/* How long to wait for an ACK before sending a chunk again. */
uint32_t snap_rto_ms(uint16_t rtt_ms);
