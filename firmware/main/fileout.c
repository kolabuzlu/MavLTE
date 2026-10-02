#include "fileout.h"

#include <string.h>

#define RATE_START 4096.0f /* bytes/s, as a photo starts */

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}

void fileout_init(file_outbox_t *o, const file_config_t *cfg)
{
    memset(o, 0, sizeof(*o));
    o->cfg = *cfg;
    snap_rate_init(&o->rate, RATE_START, RATE_START);
}

/* A name from a FILE_REQ: up to 12 bytes, NUL-padded, in capitals as the card has them. */
static void copy_name(char *out, const uint8_t *raw)
{
    size_t i = 0;
    for (; i < FILE_NAME_LEN && raw[i]; i++) {
        char c = (char)raw[i];
        out[i] = c >= 'a' && c <= 'z' ? (char)(c - 'a' + 'A') : c;
    }
    out[i] = '\0';
}

void fileout_input(file_outbox_t *o, uint8_t type, const uint8_t *body, size_t len, uint32_t now_ms)
{
    if (type == TUN_FILE_REQ && len >= FILE_REQ_LEN) {
        uint16_t id = get_u16(body);
        uint32_t offset = get_u32(body + 3);
        if (body[2] == FILE_OP_LIST) {
            o->list_asked = true;
            o->list_id = id;
            o->list_first = offset > 0xFFFF ? 0xFFFF : (uint16_t)offset;
        } else if (body[2] == FILE_OP_GET) {
            o->get_asked = true;
            o->get_id = id;
            o->get_offset = offset;
            copy_name(o->get_name, body + 7);
        } else if (body[2] == FILE_OP_STOP && o->sending && id == o->id) {
            o->stop_asked = true;
        }
    } else if (type == TUN_FILE_ACK && len >= FILE_ACK_LEN && o->sending && get_u16(body) == o->id) {
        uint32_t next = get_u32(body + 2);
        o->last_ack_ms = now_ms; /* a copy of the last ACK still says the receiver is there */
        if (next > o->acked && next <= o->size) {
            o->acked = next;
            o->moved_ms = now_ms;
            o->finished = next >= o->size;
        }
    }
}

static bool send_status(file_outbox_t *o, uint16_t id, uint8_t status, uint32_t offset, uint32_t size)
{
    put_u16(o->pkt, id);
    o->pkt[2] = status;
    put_u32(o->pkt + 3, offset);
    put_u32(o->pkt + 7, size);
    return o->cfg.send(o->cfg.ctx, TUN_FILE_DATA, o->pkt, FILE_DATA_HEAD_LEN);
}

static void end(file_outbox_t *o, bool complete)
{
    o->cfg.close(o->cfg.ctx);
    o->sending = o->finished = o->stop_asked = false;
    memcpy(o->last_name, o->name, sizeof(o->last_name));
    o->last_bytes = o->acked - o->from;
    o->last_complete = complete;
}

static void answer_list(file_outbox_t *o)
{
    unsigned total = 0;
    int n = o->cfg.list(o->cfg.ctx, o->list_first, o->entries, FILE_LIST_MOST, &total);
    uint8_t *p = o->pkt;
    put_u16(p, o->list_id);
    p[2] = n < 0 ? FILE_NO_CARD : FILE_OK;
    put_u16(p + 3, n < 0 ? 0 : (uint16_t)(total > 0xFFFF ? 0xFFFF : total));
    put_u16(p + 5, o->list_first);
    p[7] = (uint8_t)(n < 0 ? 0 : n);
    size_t len = FILE_LIST_HEAD_LEN;
    for (int i = 0; i < n; i++, len += FILE_ENTRY_LEN) {
        uint8_t *e = p + len;
        memset(e, 0, FILE_NAME_LEN);
        for (size_t c = 0; c < FILE_NAME_LEN && o->entries[i].name[c]; c++) {
            e[c] = (uint8_t)o->entries[i].name[c];
        }
        put_u32(e + 12, o->entries[i].size);
        put_u32(e + 16, o->entries[i].start);
        put_u32(e + 20, o->entries[i].end);
    }
    o->cfg.send(o->cfg.ctx, TUN_FILE_LIST, p, len);
}

/* A GET: the receiver asking again from where it stopped (the same request), or a new download. Returns true
 * if a download ended for it. */
static bool start_get(file_outbox_t *o, uint32_t now_ms)
{
    o->get_asked = false;
    bool ended = false;
    if (o->sending && o->id == o->get_id && strcmp(o->name, o->get_name) == 0 && o->get_offset < o->size) {
        o->acked = o->next = o->get_offset; /* the same download, from where the receiver has it */
        o->moved_ms = o->last_ack_ms = now_ms;
        return false;
    }
    if (o->sending) {
        if (o->id != o->get_id) { /* someone else's download: this one ends it */
            send_status(o, o->id, FILE_STOPPED, o->acked, o->size);
        }
        end(o, false);
        ended = true;
    }
    uint32_t size = 0;
    int status = o->cfg.open(o->cfg.ctx, o->get_name, &size);
    if (status != FILE_OK) {
        send_status(o, o->get_id, (uint8_t)status, o->get_offset, 0);
        return ended;
    }
    if (o->get_offset >= size) { /* nothing (more) to send */
        o->cfg.close(o->cfg.ctx);
        send_status(o, o->get_id, FILE_OK, size, size);
        return ended;
    }
    o->sending = true;
    o->finished = o->stop_asked = false;
    o->id = o->get_id;
    memcpy(o->name, o->get_name, sizeof(o->name));
    o->size = size;
    o->acked = o->next = o->from = o->get_offset;
    o->last_ack_ms = o->moved_ms = now_ms;
    snap_rate_init(&o->rate, RATE_START, RATE_START);
    o->rtt_ms_at = now_ms - 1000;
    return ended;
}

bool fileout_poll(file_outbox_t *o, uint32_t now_ms, uint16_t rtt_ms, float cap)
{
    bool ended = false;
    if (o->list_asked) {
        o->list_asked = false;
        answer_list(o);
    }
    if (o->get_asked) {
        ended = start_get(o, now_ms);
    }
    if (!o->sending) {
        o->stop_asked = false;
        return ended;
    }
    if (o->stop_asked || o->finished || (uint32_t)(now_ms - o->last_ack_ms) > FILE_GIVE_UP_MS) {
        end(o, o->finished);
        return true;
    }
    snap_rate_set_cap(&o->rate, cap);
    if (rtt_ms != TUN_U16_UNKNOWN && (uint32_t)(now_ms - o->rtt_ms_at) >= 1000) {
        o->rtt_ms_at = now_ms;
        snap_rate_sample(&o->rate, rtt_ms, now_ms);
    }
    if (o->next > o->acked && (uint32_t)(now_ms - o->moved_ms) >= snap_rto_ms(rtt_ms)) {
        o->next = o->acked; /* no new ACK for a while: from the last one again */
        o->moved_ms = now_ms;
    }
    float budget = snap_rate_budget(&o->rate, now_ms);
    uint32_t window_end = o->size - o->acked > FILE_WINDOW ? o->acked + FILE_WINDOW : o->size;
    while (o->next < window_end) {
        uint32_t n = o->size - o->next < FILE_CHUNK ? o->size - o->next : FILE_CHUNK;
        if (budget < (float)n) {
            break;
        }
        if (!o->cfg.read(o->cfg.ctx, o->next, o->pkt + FILE_DATA_HEAD_LEN, n)) {
            send_status(o, o->id, FILE_CARD_ERROR, o->next, o->size);
            end(o, false);
            return true;
        }
        put_u16(o->pkt, o->id);
        o->pkt[2] = FILE_OK;
        put_u32(o->pkt + 3, o->next);
        put_u32(o->pkt + 7, o->size);
        if (!o->cfg.send(o->cfg.ctx, TUN_FILE_DATA, o->pkt, FILE_DATA_HEAD_LEN + n)) {
            break; /* no session: later */
        }
        budget -= (float)n;
        o->rate.tokens -= (float)n;
        o->next += n;
    }
    return ended;
}
