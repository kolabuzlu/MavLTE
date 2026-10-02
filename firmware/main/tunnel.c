#include "tunnel.h"

#include <string.h>

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

size_t tun_encode(const hmac_sha256_key_t *key, uint8_t *out, uint8_t type, uint8_t role, uint32_t session,
                  uint32_t seq, const uint8_t *body, size_t body_len)
{
    uint8_t mac[SHA256_DIGEST_SIZE];
    out[0] = TUN_MAGIC;
    out[1] = TUN_VERSION;
    out[2] = type;
    out[3] = role;
    put_u32(out + 4, session);
    put_u32(out + 8, seq);
    if (body_len && body != out + TUN_HEADER_LEN) {
        memmove(out + TUN_HEADER_LEN, body, body_len);
    }
    hmac_sha256(key, out, TUN_HEADER_LEN + body_len, mac);
    memcpy(out + TUN_HEADER_LEN + body_len, mac, TUN_TAG_LEN);
    return TUN_HEADER_LEN + body_len + TUN_TAG_LEN;
}

static void emit_event(tun_client_t *t, tun_event_t ev)
{
    if (t->cfg.on_event) {
        t->cfg.on_event(t->cfg.ctx, ev, t->session);
    }
}

/* ---- anti-replay window (RFC 4303 style, 64 packets) */

static bool window_accept(tun_client_t *t, uint32_t seq)
{
    if (seq == 0) {
        return false;
    }
    if (seq > t->rx_top) {
        uint32_t shift = seq - t->rx_top;
        t->rx_mask = shift < 64 ? (t->rx_mask << shift) | 1 : 1;
        t->rx_top = seq;
        return true;
    }
    uint32_t age = t->rx_top - seq;
    if (age >= 64 || (t->rx_mask & ((uint64_t)1 << age))) {
        return false;
    }
    t->rx_mask |= (uint64_t)1 << age;
    return true;
}

/* ---- downlink loss from sequence gaps, per one-second slot */

static void loss_reset(tun_client_t *t)
{
    t->loss_top = 0;
    t->loss_slot = 0;
    memset(t->loss_expected, 0, sizeof(t->loss_expected));
    memset(t->loss_received, 0, sizeof(t->loss_received));
}

static void loss_packet(tun_client_t *t, uint32_t seq)
{
    if (seq > t->loss_top) {
        t->loss_expected[t->loss_slot] += t->loss_top ? seq - t->loss_top : 1;
        t->loss_top = seq;
    }
    t->loss_received[t->loss_slot]++;
}

static void loss_roll(tun_client_t *t)
{
    t->loss_slot = (uint8_t)((t->loss_slot + 1) % (TUN_LOSS_SECONDS + 1));
    t->loss_expected[t->loss_slot] = 0;
    t->loss_received[t->loss_slot] = 0;
}

uint16_t tun_loss_permille(const tun_client_t *t)
{
    uint32_t expected = 0, received = 0;
    for (int i = 0; i <= TUN_LOSS_SECONDS; i++) {
        if (i != t->loss_slot) { /* completed seconds only */
            expected += t->loss_expected[i];
            received += t->loss_received[i];
        }
    }
    if (expected == 0) {
        return TUN_U16_UNKNOWN;
    }
    if (received >= expected) {
        return 0;
    }
    return (uint16_t)(((uint64_t)(expected - received) * 1000 + expected / 2) / expected);
}

/* ---- session */

static void drop_session(tun_client_t *t)
{
    t->session = 0;
    t->have_nonce = false;
    t->hello_due = true;
    t->rtt_ms = TUN_U16_UNKNOWN;
    t->gcs_present = true;
}

void tun_init(tun_client_t *t, const tun_config_t *cfg, uint32_t now_ms)
{
    memset(t, 0, sizeof(*t));
    t->cfg = *cfg;
    if (!t->cfg.hello_interval_ms) {
        t->cfg.hello_interval_ms = 1000;
    }
    if (!t->cfg.ping_interval_ms) {
        t->cfg.ping_interval_ms = 1000;
    }
    if (!t->cfg.link_timeout_ms) {
        t->cfg.link_timeout_ms = 10000;
    }
    hmac_sha256_setkey(&t->key, cfg->key, cfg->key_len);
    t->rssi_dbm = TUN_RSSI_UNKNOWN;
    t->rat = TUN_RAT_UNKNOWN;
    t->last_roll = now_ms;
    drop_session(t);
}

void tun_restart(tun_client_t *t)
{
    drop_session(t);
}

void tun_set_radio(tun_client_t *t, int16_t rssi_dbm, uint8_t rat)
{
    t->rssi_dbm = rssi_dbm;
    t->rat = rat;
}

void tun_set_ping_flags(tun_client_t *t, uint8_t flags)
{
    t->ping_flags = flags;
}

static bool send_packet(tun_client_t *t, uint8_t type, const uint8_t *body, size_t body_len)
{
    uint32_t seq = 0;
    uint32_t session = 0;
    if (type != TUN_HELLO) {
        if (t->tx_seq == UINT32_MAX) { /* sequence numbers used up: get a new session */
            drop_session(t);
            return false;
        }
        seq = ++t->tx_seq;
        session = t->session;
    }
    size_t n = tun_encode(&t->key, t->pkt, type, t->cfg.role, session, seq, body, body_len);
    t->cfg.send(t->cfg.ctx, t->pkt, n);
    return true;
}

static void send_hello(tun_client_t *t)
{
    uint8_t body[TUN_NONCE_LEN + TUN_INFO_MAX];
    size_t info_len = t->cfg.info ? strlen(t->cfg.info) : 0;
    if (info_len > TUN_INFO_MAX) {
        info_len = TUN_INFO_MAX;
    }
    if (!t->have_nonce) { /* one nonce per attempt, so a late WELCOME to any retry still counts */
        t->cfg.random(t->cfg.ctx, t->nonce, sizeof(t->nonce));
        t->have_nonce = true;
    }
    memcpy(body, t->nonce, TUN_NONCE_LEN);
    if (info_len) {
        memcpy(body + TUN_NONCE_LEN, t->cfg.info, info_len);
    }
    send_packet(t, TUN_HELLO, body, TUN_NONCE_LEN + info_len);
}

static void send_ping(tun_client_t *t, uint32_t now_ms)
{
    uint8_t body[12];
    put_u32(body, now_ms);
    put_u16(body + 4, t->rtt_ms);
    put_u16(body + 6, tun_loss_permille(t));
    put_u16(body + 8, (uint16_t)t->rssi_dbm);
    body[10] = t->rat;
    body[11] = t->ping_flags;
    t->last_ping = now_ms;
    send_packet(t, TUN_PING, body, sizeof(body));
}

void tun_poll(tun_client_t *t, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - t->last_roll) >= 1000) {
        t->last_roll = now_ms;
        loss_roll(t);
    }
    if (!t->session) {
        if (t->hello_due || (uint32_t)(now_ms - t->last_hello) >= t->cfg.hello_interval_ms) {
            t->hello_due = false;
            t->last_hello = now_ms;
            if (++t->hellos >= 30) {
                t->hellos = 0;
                emit_event(t, TUN_EVENT_TIMEOUT);
            }
            send_hello(t);
        }
    } else if ((uint32_t)(now_ms - t->last_rx) > t->cfg.link_timeout_ms) {
        emit_event(t, TUN_EVENT_TIMEOUT);
        drop_session(t);
    } else if ((uint32_t)(now_ms - t->last_ping) >= t->cfg.ping_interval_ms) {
        send_ping(t, now_ms);
    }
}

bool tun_send_data(tun_client_t *t, const uint8_t *data, size_t len)
{
    if (!t->session || len > TUN_MAX_PAYLOAD || !send_packet(t, TUN_DATA, data, len)) {
        t->stats.dropped++;
        return false;
    }
    t->stats.tx_packets++;
    t->stats.tx_bytes += len;
    return true;
}

bool tun_send_packet(tun_client_t *t, uint8_t type, const uint8_t *body, size_t len)
{
    return t->session && type != TUN_HELLO && len <= TUN_MAX_PAYLOAD && send_packet(t, type, body, len);
}

static bool tag_ok(const tun_client_t *t, const uint8_t *pkt, size_t len)
{
    uint8_t mac[SHA256_DIGEST_SIZE];
    uint8_t diff = 0;
    hmac_sha256(&t->key, pkt, len - TUN_TAG_LEN, mac);
    for (int i = 0; i < TUN_TAG_LEN; i++) {
        diff |= mac[i] ^ pkt[len - TUN_TAG_LEN + i];
    }
    return diff == 0;
}

void tun_input(tun_client_t *t, const uint8_t *pkt, size_t len, uint32_t now_ms)
{
    if (len < TUN_HEADER_LEN + TUN_TAG_LEN || pkt[0] != TUN_MAGIC || pkt[1] != TUN_VERSION ||
        pkt[3] != TUN_ROLE_SERVER || !tag_ok(t, pkt, len)) {
        t->stats.bad++;
        return;
    }
    uint8_t type = pkt[2];
    uint32_t session = get_u32(pkt + 4);
    uint32_t seq = get_u32(pkt + 8);
    const uint8_t *body = pkt + TUN_HEADER_LEN;
    size_t body_len = len - TUN_HEADER_LEN - TUN_TAG_LEN;

    if (type == TUN_WELCOME) {
        if (!t->session && t->have_nonce && session != 0 && body_len >= TUN_NONCE_LEN &&
            memcmp(body, t->nonce, TUN_NONCE_LEN) == 0) {
            t->session = session;
            t->tx_seq = 0;
            t->rx_top = 0;
            t->rx_mask = 0;
            t->have_nonce = false;
            t->hellos = 0;
            t->last_rx = now_ms;
            loss_reset(t);
            t->stats.sessions++;
            t->server_ms = 0;
            if (body_len >= TUN_NONCE_LEN + 8) { /* the relay's clock, then: half our HELLO's round trip ago */
                t->server_ms = (uint64_t)get_u32(body + 8) | (uint64_t)get_u32(body + 12) << 32;
                t->server_ms += (uint32_t)(now_ms - t->last_hello) / 2;
                t->server_ms_at = now_ms;
            }
            send_ping(t, now_ms); /* makes the session active on the server */
            emit_event(t, TUN_EVENT_CONNECTED);
        }
        return;
    }
    if (!t->session || session != t->session) {
        return;
    }
    if (type == TUN_REJECT) {
        emit_event(t, TUN_EVENT_REJECTED);
        drop_session(t);
        return;
    }
    if (!window_accept(t, seq)) {
        t->stats.bad++;
        return;
    }
    t->last_rx = now_ms;
    loss_packet(t, seq);
    if (type == TUN_DATA) {
        t->stats.rx_packets++;
        t->stats.rx_bytes += body_len;
        t->cfg.on_data(t->cfg.ctx, body, body_len);
    } else if (type == TUN_PONG && body_len >= 5) {
        uint32_t rtt = now_ms - get_u32(body);
        t->rtt_ms = rtt < TUN_U16_UNKNOWN ? (uint16_t)rtt : TUN_U16_UNKNOWN - 1;
        t->gcs_present = (body[4] & TUN_PONG_GCS_PRESENT) != 0;
        t->voice_on = (body[4] & TUN_PONG_VOICE) != 0;
    } else if (type >= TUN_SNAP_REQ && t->cfg.on_packet) {
        t->cfg.on_packet(t->cfg.ctx, type, body, body_len);
    }
}
