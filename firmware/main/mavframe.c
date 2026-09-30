#include "mavframe.h"

#include <string.h>

#define MAV_IFLAG_SIGNED 0x01
#define MAV_SIGNATURE_LEN 13

void mav_framer_reset(mav_framer_t *f)
{
    memset(f, 0, sizeof(*f));
}

bool mav_framer_push(mav_framer_t *f, uint8_t b)
{
    if (f->got == 0) {
        if (b == MAV_STX_V2 || b == MAV_STX_V1) {
            f->stx = b;
            f->got = 1;
            f->need = 0;
            return false;
        }
        return true; /* stray byte between frames */
    }
    f->got++;
    if (f->got == 2) {
        f->plen = b;
        if (f->stx == MAV_STX_V1) {
            f->need = 8 + b;
        }
    } else if (f->got == 3 && f->stx == MAV_STX_V2) {
        if (b & ~MAV_IFLAG_SIGNED) { /* unknown incompatibility flag: this was not a frame start */
            mav_framer_reset(f);
            return true;
        }
        f->need = 12 + f->plen + ((b & MAV_IFLAG_SIGNED) ? MAV_SIGNATURE_LEN : 0);
    }
    if (f->got == f->need) {
        mav_framer_reset(f);
        return true;
    }
    return false;
}

void mav_batcher_init(mav_batcher_t *b, uint8_t *buf, size_t cap, uint32_t frame_timeout_ms)
{
    memset(b, 0, sizeof(*b));
    b->buf = buf;
    b->cap = cap;
    b->frame_timeout_ms = frame_timeout_ms;
}

void mav_batcher_clear(mav_batcher_t *b)
{
    b->len = 0;
    b->ready = 0;
    mav_framer_reset(&b->framer);
}

static void take(mav_batcher_t *b, size_t n, mav_emit_fn emit, void *ctx)
{
    emit(ctx, b->buf, n);
    memmove(b->buf, b->buf + n, b->len - n);
    b->len -= n;
    b->ready = 0;
    b->t_first = b->t_frame; /* anything left is the start of the frame in progress */
}

void mav_batcher_feed(mav_batcher_t *b, const uint8_t *data, size_t len, uint32_t now_ms, mav_emit_fn emit,
                      void *ctx)
{
    for (size_t i = 0; i < len; i++) {
        if (b->len >= b->cap) {
            if (b->ready) {
                take(b, b->ready, emit, ctx);
            } else {
                mav_framer_reset(&b->framer); /* a "frame" longer than a datagram is not MAVLink */
                take(b, b->len, emit, ctx);
            }
        }
        if (b->len == 0) {
            b->t_first = now_ms;
        }
        if (!mav_framer_in_frame(&b->framer)) {
            b->t_frame = now_ms;
        }
        b->buf[b->len++] = data[i];
        if (mav_framer_push(&b->framer, data[i])) {
            b->ready = b->len;
        }
    }
}

bool mav_batcher_poll(mav_batcher_t *b, uint32_t now_ms, uint32_t max_age_ms, mav_emit_fn emit, void *ctx)
{
    if (mav_framer_in_frame(&b->framer) && (uint32_t)(now_ms - b->t_frame) >= b->frame_timeout_ms) {
        mav_framer_reset(&b->framer);
        b->ready = b->len;
    }
    if (b->ready && (uint32_t)(now_ms - b->t_first) >= max_age_ms) {
        take(b, b->ready, emit, ctx);
        return true;
    }
    return false;
}

uint32_t mav_batcher_due_in(const mav_batcher_t *b, uint32_t now_ms, uint32_t max_age_ms)
{
    uint32_t due = UINT32_MAX;
    if (b->ready) {
        uint32_t age = now_ms - b->t_first;
        due = age >= max_age_ms ? 0 : max_age_ms - age;
    }
    if (mav_framer_in_frame(&b->framer)) {
        uint32_t age = now_ms - b->t_frame;
        uint32_t left = age >= b->frame_timeout_ms ? 0 : b->frame_timeout_ms - age;
        if (left < due) {
            due = left;
        }
    }
    return due;
}
