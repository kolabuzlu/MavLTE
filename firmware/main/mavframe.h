/* MAVLink frame boundaries and batching.
 *
 * The framer follows only the frame structure (start byte, payload length, signed flag) and never
 * checks CRCs, so it needs no message definitions and passes MAVLink 1, MAVLink 2 and signed
 * MAVLink 2 unchanged. The batcher cuts the byte stream into datagram payloads that end on frame
 * boundaries: losing a datagram then loses whole frames and never corrupts neighbouring ones.
 * Bytes are never changed, reordered or dropped. Same logic as Batcher in relay/mavrelay.py. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAV_STX_V1 0xFE
#define MAV_STX_V2 0xFD
#define MAV_MAX_FRAME 280 /* MAVLink 2, 255 byte payload, signed */

typedef struct {
    uint16_t got;  /* bytes of the frame in progress, 0 between frames */
    uint16_t need; /* total length of the frame in progress, 0 until known */
    uint8_t stx;
    uint8_t plen;
} mav_framer_t;

void mav_framer_reset(mav_framer_t *f);
/* Feed one byte. Returns true if the stream is on a frame boundary after it. */
bool mav_framer_push(mav_framer_t *f, uint8_t b);

static inline bool mav_framer_in_frame(const mav_framer_t *f)
{
    return f->got != 0;
}

typedef void (*mav_emit_fn)(void *ctx, const uint8_t *data, size_t len);

typedef struct {
    uint8_t *buf;
    size_t cap;      /* largest chunk handed out */
    size_t len;      /* bytes buffered */
    size_t ready;    /* length of the prefix of buf that ends on a frame boundary */
    uint32_t t_first; /* arrival time (ms) of buf[0] */
    uint32_t t_frame; /* arrival time (ms) of the first byte of the frame in progress */
    uint32_t frame_timeout_ms;
    mav_framer_t framer;
} mav_batcher_t;

/* buf must hold cap bytes; cap must be at least MAV_MAX_FRAME. A frame start that does not
 * complete within frame_timeout_ms is passed on as plain bytes. */
void mav_batcher_init(mav_batcher_t *b, uint8_t *buf, size_t cap, uint32_t frame_timeout_ms);
void mav_batcher_clear(mav_batcher_t *b);
/* Add bytes; calls emit for chunks that had to be cut because the buffer was full. */
void mav_batcher_feed(mav_batcher_t *b, const uint8_t *data, size_t len, uint32_t now_ms, mav_emit_fn emit,
                      void *ctx);
/* Emits the complete frames once the oldest buffered byte is max_age_ms old (0: right away).
 * Returns true if something was emitted. */
bool mav_batcher_poll(mav_batcher_t *b, uint32_t now_ms, uint32_t max_age_ms, mav_emit_fn emit, void *ctx);
/* Milliseconds until mav_batcher_poll could emit something, or UINT32_MAX if nothing is buffered. */
uint32_t mav_batcher_due_in(const mav_batcher_t *b, uint32_t now_ms, uint32_t max_age_ms);
