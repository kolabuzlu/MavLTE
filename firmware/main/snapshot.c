#include "snapshot.h"

#include <string.h>

#define GIVE_UP_MS 60000 /* without any ACK */
#define RATE_START 4096.0f
#define RATE_FLOOR 256.0f

const uint16_t snap_sizes[SNAP_SIZES][2] = {{320, 240}, {640, 480}, {1024, 768}};

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static bool bit(const uint8_t *map, unsigned i)
{
    return (map[i / 8] >> (i % 8)) & 1;
}

static void set_bit(uint8_t *map, unsigned i)
{
    map[i / 8] |= (uint8_t)(1u << (i % 8));
}

uint32_t snap_rto_ms(uint16_t rtt_ms)
{
    if (rtt_ms == TUN_U16_UNKNOWN) {
        return 2000;
    }
    uint32_t rto = (uint32_t)rtt_ms * 5 / 2;
    return rto > 1000 ? rto : 1000;
}

/* ---- rate control */

void snap_rate_init(snap_rate_t *r, float cap, float start)
{
    memset(r, 0, sizeof(*r));
    r->cap = cap;
    r->rate = start < cap ? start : cap;
}

void snap_rate_set_cap(snap_rate_t *r, float cap)
{
    r->cap = cap;
    if (r->rate > cap) {
        r->rate = cap;
    }
}

void snap_rate_sample(snap_rate_t *r, uint16_t rtt_ms, uint32_t now_ms)
{
    r->rtts[r->rtt_next] = rtt_ms;
    r->rtt_ms_at[r->rtt_next] = now_ms;
    r->rtt_next = (uint8_t)((r->rtt_next + 1) % 60);
    if (r->rtt_count < 60) {
        r->rtt_count++;
    }
    uint16_t base = rtt_ms; /* the lowest of the last minute: the round trip with no queue in the way */
    for (unsigned i = 0; i < r->rtt_count; i++) {
        if ((uint32_t)(now_ms - r->rtt_ms_at[i]) <= 60000 && r->rtts[i] < base) {
            base = r->rtts[i];
        }
    }
    uint32_t allowance = base / 2 > 150 ? base / 2 : 150;
    if (rtt_ms > base + allowance) {
        if (!r->backed_off || (uint32_t)(now_ms - r->backed_off_ms) >= 1000) {
            r->rate = r->rate / 2 > RATE_FLOOR ? r->rate / 2 : RATE_FLOOR;
            r->backed_off = true;
            r->backed_off_ms = now_ms;
        }
    } else {
        r->rate += r->cap / 10;
        if (r->rate > r->cap) {
            r->rate = r->cap;
        }
    }
}

float snap_rate_budget(snap_rate_t *r, uint32_t now_ms)
{
    if (r->have_last) {
        float most = 0.2f * r->rate > 2 * SNAP_CHUNK ? 0.2f * r->rate : 2 * SNAP_CHUNK; /* no big burst after a pause */
        r->tokens += (float)(uint32_t)(now_ms - r->last_ms) / 1000.0f * r->rate;
        if (r->tokens > most) {
            r->tokens = most;
        }
    }
    r->have_last = true;
    r->last_ms = now_ms;
    return r->tokens;
}

/* ---- the outbox */

void snap_init(snap_outbox_t *o, tun_client_t *tun, const snap_config_t *cfg)
{
    memset(o, 0, sizeof(*o));
    o->cfg = *cfg;
    o->tun = tun;
}

static void pack_info(uint8_t *out, uint32_t photo_id, uint32_t size, uint16_t width, uint16_t height,
                      const snap_where_t *where, uint8_t status)
{
    put_u32(out, photo_id);
    put_u32(out + 4, size);
    put_u16(out + 8, width);
    put_u16(out + 10, height);
    put_u32(out + 12, (uint32_t)(where ? where->lat : SNAP_UNKNOWN_I32));
    put_u32(out + 16, (uint32_t)(where ? where->lon : SNAP_UNKNOWN_I32));
    put_u32(out + 20, (uint32_t)(where ? where->alt_mm : SNAP_UNKNOWN_I32));
    put_u16(out + 24, where ? where->heading : SNAP_UNKNOWN_HEADING);
    out[26] = status;
    put_u32(out + 27, 0); /* time: the relay's clock */
}

static void send_status(snap_outbox_t *o, uint32_t photo_id, uint8_t status)
{
    uint8_t body[SNAP_INFO_LEN];
    pack_info(body, photo_id, 0, 0, 0, NULL, status);
    tun_send_packet(o->tun, TUN_SNAP_INFO, body, sizeof(body));
}

static int find_answer(const snap_outbox_t *o, uint32_t photo_id)
{
    for (int i = 0; i < o->answers; i++) {
        if (o->answer_id[i] == photo_id) {
            return i;
        }
    }
    return -1;
}

static void remember(snap_outbox_t *o, uint32_t photo_id, uint8_t answer)
{
    int i = find_answer(o, photo_id);
    if (i < 0) { /* in place of the oldest */
        i = o->answer_next;
        o->answer_next = (uint8_t)((o->answer_next + 1) % SNAP_ANSWERS);
        if (o->answers < SNAP_ANSWERS) {
            o->answers++;
        }
        o->answer_id[i] = photo_id;
    }
    o->answer[i] = answer;
}

void snap_input(snap_outbox_t *o, uint8_t type, const uint8_t *body, size_t len, uint32_t now_ms)
{
    if (type == TUN_SNAP_REQ && len >= 5) {
        uint32_t photo_id = get_u32(body);
        unsigned size = body[4] < SNAP_SIZES ? body[4] : SNAP_SIZES - 1;
        int i = find_answer(o, photo_id);
        if (i >= 0) { /* asked again: our answer has not reached the relay yet */
            if (o->answer[i] != SNAP_OK) { /* (a photo's SNAP_INFO goes again by itself) */
                send_status(o, photo_id, o->answer[i]);
            }
        } else if (snap_busy(o)) {
            send_status(o, photo_id, SNAP_BUSY);
        } else if (!o->cfg.take) {
            remember(o, photo_id, SNAP_NO_CAMERA);
            send_status(o, photo_id, SNAP_NO_CAMERA);
        } else {
            remember(o, photo_id, SNAP_OK);
            o->taking = true;
            o->taking_id = photo_id;
            o->taking_w = snap_sizes[size][0];
            o->taking_h = snap_sizes[size][1];
            o->cfg.take(o->cfg.ctx, photo_id, o->taking_w, o->taking_h); /* may hand the photo over at once */
        }
    } else if (type == TUN_SNAP_ACK && len >= 5 && o->sending && get_u32(body) == o->photo_id) {
        uint8_t flags = body[4];
        size_t bits = (len - 5) * 8;
        o->last_ack_ms = now_ms;
        o->info_acked = o->info_acked || (flags & SNAP_ACK_HAVE_INFO);
        for (unsigned i = 0; i < o->chunks && i < bits; i++) {
            if (bit(body + 5, i)) {
                set_bit(o->acked, i);
            }
        }
        if (flags & SNAP_ACK_DONE) {
            o->done = true;
        }
    }
}

void snap_photo_taken(snap_outbox_t *o, uint32_t photo_id, uint8_t status, const uint8_t *jpeg, size_t len,
                      const snap_where_t *where, uint32_t now_ms)
{
    if (!o->taking || photo_id != o->taking_id) { /* not the one asked for */
        if (jpeg && o->cfg.release) {
            o->cfg.release(o->cfg.ctx, photo_id, false);
        }
        return;
    }
    o->taking = false;
    if (status == SNAP_OK && (!jpeg || len == 0 || len > (size_t)SNAP_MAX_CHUNKS * SNAP_CHUNK)) {
        status = SNAP_FAILED;
    }
    if (status != SNAP_OK) {
        remember(o, photo_id, status);
        send_status(o, photo_id, status);
        if (jpeg && o->cfg.release) {
            o->cfg.release(o->cfg.ctx, photo_id, false);
        }
        return;
    }
    o->sending = true;
    o->photo_id = photo_id;
    o->data = jpeg;
    o->size = (uint32_t)len;
    o->chunks = (uint16_t)((len + SNAP_CHUNK - 1) / SNAP_CHUNK);
    pack_info(o->info, photo_id, (uint32_t)len, o->taking_w, o->taking_h, where, SNAP_OK);
    memset(o->acked, 0, sizeof(o->acked));
    memset(o->sent, 0, sizeof(o->sent));
    o->info_acked = o->info_sent = o->done = false;
    o->last_ack_ms = o->started_ms = now_ms;
    snap_rate_init(&o->rate, RATE_START, RATE_START); /* the cap comes with each snap_poll() */
    o->rtt_ms_at = now_ms - 1000;
}

void snap_restart(snap_outbox_t *o, uint32_t now_ms)
{
    if (o->sending) {
        memset(o->acked, 0, sizeof(o->acked));
        memset(o->sent, 0, sizeof(o->sent));
        o->info_acked = o->info_sent = false;
        o->last_ack_ms = now_ms;
    }
}

void snap_poll(snap_outbox_t *o, uint32_t now_ms, float cap)
{
    if (!o->sending) {
        return;
    }
    if (o->done || (uint32_t)(now_ms - o->last_ack_ms) > GIVE_UP_MS) {
        o->sending = false;
        o->last_id = o->photo_id;
        o->last_sent = o->done;
        o->last_ms = now_ms - o->started_ms;
        if (o->cfg.release) {
            o->cfg.release(o->cfg.ctx, o->photo_id, o->done);
        }
        return;
    }
    snap_rate_set_cap(&o->rate, cap);
    uint16_t rtt = o->tun->rtt_ms;
    if (rtt != TUN_U16_UNKNOWN && (uint32_t)(now_ms - o->rtt_ms_at) >= 1000) {
        o->rtt_ms_at = now_ms;
        snap_rate_sample(&o->rate, rtt, now_ms);
    }
    uint32_t rto = snap_rto_ms(rtt);
    if (!o->info_acked && (!o->info_sent || (uint32_t)(now_ms - o->info_ms) >= rto)) {
        o->info_sent = true;
        o->info_ms = now_ms;
        tun_send_packet(o->tun, TUN_SNAP_INFO, o->info, SNAP_INFO_LEN);
    }
    float budget = snap_rate_budget(&o->rate, now_ms);
    for (unsigned i = 0; i < o->chunks; i++) {
        if (bit(o->acked, i) || (bit(o->sent, i) && (uint32_t)(now_ms - o->sent_ms[i]) < rto)) {
            continue;
        }
        size_t n = o->size - i * SNAP_CHUNK < SNAP_CHUNK ? o->size - i * SNAP_CHUNK : SNAP_CHUNK;
        if (budget < (float)n) {
            break;
        }
        budget -= (float)n;
        o->rate.tokens -= (float)n;
        set_bit(o->sent, i);
        o->sent_ms[i] = now_ms;
        put_u32(o->pkt, o->photo_id);
        put_u16(o->pkt + 4, (uint16_t)i);
        memcpy(o->pkt + 6, o->data + (size_t)i * SNAP_CHUNK, n);
        tun_send_packet(o->tun, TUN_SNAP_DATA, o->pkt, 6 + n);
    }
}
