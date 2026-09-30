/* Host tests for the portable firmware core (sha256, mavframe, tunnel). Build and run: make test */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mavframe.h"
#include "sha256.h"
#include "tunnel.h"

static int failures;
static int checks;

#define CHECK(cond)                                                          \
    do {                                                                     \
        checks++;                                                            \
        if (!(cond)) {                                                       \
            failures++;                                                      \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                    \
    } while (0)

static size_t unhex(const char *hex, uint8_t *out)
{
    size_t n = strlen(hex) / 2;
    for (size_t i = 0; i < n; i++) {
        unsigned v;
        sscanf(hex + 2 * i, "%2x", &v);
        out[i] = (uint8_t)v;
    }
    return n;
}

static void tohex(const uint8_t *p, size_t n, char *out)
{
    for (size_t i = 0; i < n; i++) {
        sprintf(out + 2 * i, "%02x", p[i]);
    }
}

static uint32_t rng_state = 12345;
static uint32_t rnd(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state >> 8;
}

/* ------------------------------------------------------------------ sha256 / hmac */

static void check_sha(const void *msg, size_t len, const char *expect)
{
    uint8_t d[32];
    char hex[65];
    sha256_ctx_t c;
    sha256_init(&c);
    sha256_update(&c, msg, len);
    sha256_final(&c, d);
    tohex(d, 32, hex);
    CHECK(strcmp(hex, expect) == 0);
    /* same result when fed in odd pieces */
    sha256_init(&c);
    for (size_t i = 0; i < len;) {
        size_t n = 1 + rnd() % 70;
        if (n > len - i) {
            n = len - i;
        }
        sha256_update(&c, (const uint8_t *)msg + i, n);
        i += n;
    }
    sha256_final(&c, d);
    tohex(d, 32, hex);
    CHECK(strcmp(hex, expect) == 0);
}

static void check_hmac(const uint8_t *key, size_t key_len, const void *msg, size_t len, const char *expect)
{
    hmac_sha256_key_t k;
    uint8_t mac[32];
    char hex[65];
    hmac_sha256_setkey(&k, key, key_len);
    hmac_sha256(&k, msg, len, mac);
    tohex(mac, 32, hex);
    CHECK(strcmp(hex, expect) == 0);
    hmac_sha256(&k, msg, len, mac); /* the prepared key is reusable */
    tohex(mac, 32, hex);
    CHECK(strcmp(hex, expect) == 0);
}

static void test_sha256(void)
{
    check_sha("", 0, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    check_sha("abc", 3, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    const char *m448 = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    check_sha(m448, strlen(m448), "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    static uint8_t million[1000000];
    memset(million, 'a', sizeof(million));
    check_sha(million, sizeof(million), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");

    /* RFC 4231 */
    uint8_t key[131], data[160];
    memset(key, 0x0b, 20);
    check_hmac(key, 20, "Hi There", 8, "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
    check_hmac((const uint8_t *)"Jefe", 4, "what do ya want for nothing?", 28,
               "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
    memset(key, 0xaa, 20);
    memset(data, 0xdd, 50);
    check_hmac(key, 20, data, 50, "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe");
    for (int i = 0; i < 25; i++) {
        key[i] = (uint8_t)(i + 1);
    }
    memset(data, 0xcd, 50);
    check_hmac(key, 25, data, 50, "82558a389a443c0ea4cc819899f2083a85f0faa3e578f8077a2e3ff46729665b");
    memset(key, 0xaa, 131);
    const char *m6 = "Test Using Larger Than Block-Size Key - Hash Key First";
    check_hmac(key, 131, m6, strlen(m6), "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54");
    const char *m7 = "This is a test using a larger than block-size key and a larger than block-size data. The key "
                     "needs to be hashed before being used by the HMAC algorithm.";
    check_hmac(key, 131, m7, strlen(m7), "9b09ffa71b942fcb27635fbcd5b0e944bfdc63644f0713938a7f51535c3a35e2");
}

/* ------------------------------------------------------------------ golden packet (shared with the Python tests) */

static void test_golden_packet(void)
{
    uint8_t key[32], body[64], out[128];
    char hex[300];
    unhex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f", key);
    size_t body_len = unhex("fd0900002a0101000000000000000103410303ca52", body);
    hmac_sha256_key_t k;
    hmac_sha256_setkey(&k, key, sizeof(key));
    size_t n = tun_encode(&k, out, TUN_DATA, TUN_ROLE_VEHICLE, 0xA1B2C3D4, 7, body, body_len);
    tohex(out, n, hex);
    CHECK(strcmp(hex, "a5010301d4c3b2a107000000fd0900002a0101000000000000000103410303ca52"
                      "96bcbb3483d85430869d9997548c9380") == 0);
    /* body already in place */
    memmove(out + TUN_HEADER_LEN, body, body_len);
    n = tun_encode(&k, out, TUN_DATA, TUN_ROLE_VEHICLE, 0xA1B2C3D4, 7, out + TUN_HEADER_LEN, body_len);
    tohex(out, n, hex);
    CHECK(strncmp(hex + 2 * (n - 16), "96bcbb3483d85430869d9997548c9380", 32) == 0);
}

/* ------------------------------------------------------------------ framer */

static size_t v2_frame(uint8_t *out, uint32_t msgid, const uint8_t *payload, uint8_t plen, uint8_t seq, int sign)
{
    size_t n = 0;
    out[n++] = 0xFD;
    out[n++] = plen;
    out[n++] = sign ? 1 : 0;
    out[n++] = 0;
    out[n++] = seq;
    out[n++] = 1;
    out[n++] = 1;
    out[n++] = (uint8_t)msgid;
    out[n++] = (uint8_t)(msgid >> 8);
    out[n++] = (uint8_t)(msgid >> 16);
    memcpy(out + n, payload, plen);
    n += plen;
    out[n++] = 0xab;
    out[n++] = 0xcd;
    if (sign) {
        memset(out + n, 0x5a, 13);
        n += 13;
    }
    return n;
}

static size_t v1_frame(uint8_t *out, uint8_t msgid, const uint8_t *payload, uint8_t plen, uint8_t seq)
{
    size_t n = 0;
    out[n++] = 0xFE;
    out[n++] = plen;
    out[n++] = seq;
    out[n++] = 1;
    out[n++] = 1;
    out[n++] = msgid;
    memcpy(out + n, payload, plen);
    n += plen;
    out[n++] = 0xab;
    out[n++] = 0xcd;
    return n;
}

/* stream of random frames whose payloads are full of start bytes */
static size_t random_stream(uint8_t *out, int frames)
{
    uint8_t payload[255];
    size_t n = 0;
    for (int i = 0; i < frames; i++) {
        uint8_t plen = (uint8_t)(rnd() % 256);
        for (int j = 0; j < plen; j++) {
            uint32_t r = rnd() % 3;
            payload[j] = r == 0 ? 0xFD : r == 1 ? 0xFE : (uint8_t)rnd();
        }
        switch (rnd() % 3) {
        case 0: n += v1_frame(out + n, (uint8_t)rnd(), payload, plen, (uint8_t)i); break;
        case 1: n += v2_frame(out + n, rnd() & 0xFFFFFF, payload, plen, (uint8_t)i, 0); break;
        default: n += v2_frame(out + n, rnd() & 0xFFFFFF, payload, plen, (uint8_t)i, 1); break;
        }
    }
    return n;
}

static int ends_between_frames(const uint8_t *p, size_t n)
{
    mav_framer_t f;
    mav_framer_reset(&f);
    for (size_t i = 0; i < n; i++) {
        mav_framer_push(&f, p[i]);
    }
    return !mav_framer_in_frame(&f);
}

static void test_framer(void)
{
    uint8_t s[400], pl[64];
    size_t ends[4], n = 0;
    mav_framer_t f;

    memset(pl, 0xFD, 9);
    n += v2_frame(s + n, 0, pl, 9, 0, 0);
    ends[0] = n;
    memset(pl, 0xFE, 9);
    n += v1_frame(s + n, 0, pl, 9, 0);
    ends[1] = n;
    memset(pl, 0, 28);
    n += v2_frame(s + n, 33, pl, 28, 0, 1);
    ends[2] = n;
    n += v2_frame(s + n, 1, pl, 0, 0, 0);
    ends[3] = n;
    mav_framer_reset(&f);
    int k = 0;
    for (size_t i = 0; i < n; i++) {
        if (mav_framer_push(&f, s[i])) {
            CHECK(k < 4 && i + 1 == ends[k]);
            k++;
        }
    }
    CHECK(k == 4);

    /* stray bytes are boundaries of their own; 0xFD with an unknown flag is not a frame */
    mav_framer_reset(&f);
    CHECK(mav_framer_push(&f, 0x00));
    CHECK(!mav_framer_push(&f, 0xFD));
    CHECK(!mav_framer_push(&f, 0x05));
    CHECK(mav_framer_push(&f, 0x80));
    CHECK(!mav_framer_in_frame(&f));
}

/* ------------------------------------------------------------------ batcher */

typedef struct {
    uint8_t data[200000];
    size_t len;
    size_t chunks;
    int bad_chunk;
    size_t max_chunk;
} sink_t;

static void sink_emit(void *ctx, const uint8_t *p, size_t n)
{
    sink_t *s = ctx;
    memcpy(s->data + s->len, p, n);
    s->len += n;
    s->chunks++;
    if (n > s->max_chunk) {
        s->max_chunk = n;
    }
    if (!ends_between_frames(p, n)) {
        s->bad_chunk = 1;
    }
}

static void test_batcher_stream(void)
{
    static uint8_t stream[200000];
    static sink_t sink;
    static uint8_t buf[TUN_MAX_PAYLOAD];
    size_t len = random_stream(stream, 300);
    mav_batcher_t b;
    mav_batcher_init(&b, buf, sizeof(buf), 500);
    memset(&sink, 0, sizeof(sink));
    uint32_t t = 0xFFFFF000u; /* also crosses the 32-bit millisecond wrap */
    for (size_t pos = 0; pos < len;) {
        size_t n = 1 + rnd() % 200;
        if (n > len - pos) {
            n = len - pos;
        }
        mav_batcher_feed(&b, stream + pos, n, t, sink_emit, &sink);
        pos += n;
        t += 10;
        mav_batcher_poll(&b, t, 50, sink_emit, &sink);
    }
    mav_batcher_poll(&b, t + 1000, 50, sink_emit, &sink);
    CHECK(sink.len == len);
    CHECK(memcmp(sink.data, stream, len) == 0);
    CHECK(!sink.bad_chunk);
    CHECK(sink.max_chunk <= TUN_MAX_PAYLOAD);
    CHECK(sink.chunks > 10);
}

static void test_batcher_timing(void)
{
    static sink_t sink;
    uint8_t buf[TUN_MAX_PAYLOAD], hb[32], big[64], pl[255] = {0};
    size_t hb_len = v2_frame(hb, 0, pl, 9, 0, 0);
    size_t big_len = v2_frame(big, 0, pl, 40, 0, 0);
    mav_batcher_t b;

    /* waits for max_age */
    mav_batcher_init(&b, buf, sizeof(buf), 500);
    memset(&sink, 0, sizeof(sink));
    mav_batcher_feed(&b, hb, hb_len, 10000, sink_emit, &sink);
    CHECK(mav_batcher_due_in(&b, 10000, 50) == 50);
    CHECK(!mav_batcher_poll(&b, 10049, 50, sink_emit, &sink));
    CHECK(mav_batcher_poll(&b, 10050, 50, sink_emit, &sink));
    CHECK(sink.len == hb_len && sink.chunks == 1);
    CHECK(mav_batcher_due_in(&b, 10050, 50) == UINT32_MAX);

    /* holds a partial frame; its age counts from its first byte */
    memset(&sink, 0, sizeof(sink));
    mav_batcher_feed(&b, hb, hb_len, 0, sink_emit, &sink);
    mav_batcher_feed(&b, big, 10, 0, sink_emit, &sink);
    CHECK(mav_batcher_poll(&b, 100, 50, sink_emit, &sink));
    CHECK(sink.len == hb_len);
    CHECK(!mav_batcher_poll(&b, 200, 50, sink_emit, &sink));
    mav_batcher_feed(&b, big + 10, big_len - 10, 300, sink_emit, &sink);
    CHECK(mav_batcher_poll(&b, 300, 50, sink_emit, &sink));
    CHECK(sink.len == hb_len + big_len && sink.chunks == 2);

    /* a frame start that never completes is released after the timeout */
    memset(&sink, 0, sizeof(sink));
    mav_batcher_init(&b, buf, sizeof(buf), 500);
    const uint8_t junk[3] = {0xFE, 0xFF, 0x00};
    mav_batcher_feed(&b, junk, 3, 0, sink_emit, &sink);
    CHECK(mav_batcher_due_in(&b, 100, 0) == 400);
    CHECK(!mav_batcher_poll(&b, 499, 0, sink_emit, &sink));
    CHECK(mav_batcher_poll(&b, 500, 0, sink_emit, &sink));
    CHECK(sink.len == 3);

    /* a full buffer is cut at the last frame boundary */
    uint8_t f200[260];
    size_t f200_len = v2_frame(f200, 0, pl, 200, 0, 0); /* 212 bytes */
    memset(&sink, 0, sizeof(sink));
    mav_batcher_init(&b, buf, sizeof(buf), 500);
    for (int i = 0; i < 6; i++) {
        mav_batcher_feed(&b, f200, f200_len, 0, sink_emit, &sink);
    }
    CHECK(sink.chunks == 1 && sink.len == 5 * f200_len);
    CHECK(mav_batcher_poll(&b, 1000, 50, sink_emit, &sink));
    CHECK(sink.len == 6 * f200_len);
}

/* ------------------------------------------------------------------ tunnel client against a fake server */

typedef struct {
    uint8_t sent[8][TUN_MAX_DATAGRAM];
    size_t sent_len[8];
    int nsent;
    uint8_t got[TUN_MAX_DATAGRAM];
    size_t got_len;
    int ndata;
    tun_event_t events[8];
    int nevents;
} fake_t;

static void fake_send(void *ctx, const uint8_t *p, size_t n)
{
    fake_t *f = ctx;
    if (f->nsent < 8) {
        memcpy(f->sent[f->nsent], p, n);
        f->sent_len[f->nsent] = n;
    }
    f->nsent++;
}

static void fake_data(void *ctx, const uint8_t *p, size_t n)
{
    fake_t *f = ctx;
    memcpy(f->got, p, n);
    f->got_len = n;
    f->ndata++;
}

static void fake_event(void *ctx, tun_event_t ev, uint32_t session)
{
    (void)session;
    fake_t *f = ctx;
    f->events[f->nevents++ % 8] = ev;
}

static void fake_random(void *ctx, uint8_t *p, size_t n)
{
    (void)ctx;
    for (size_t i = 0; i < n; i++) {
        p[i] = (uint8_t)rnd();
    }
}

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void test_tunnel(void)
{
    static tun_client_t t;
    static fake_t f;
    uint8_t key[32], pkt[TUN_MAX_DATAGRAM], nonce[8];
    size_t n;
    hmac_sha256_key_t k, wrong;
    for (int i = 0; i < 32; i++) {
        key[i] = (uint8_t)i;
    }
    hmac_sha256_setkey(&k, key, 32);
    key[0] ^= 1;
    hmac_sha256_setkey(&wrong, key, 32);
    key[0] ^= 1;
    memset(&f, 0, sizeof(f));
    tun_config_t cfg = {
        .role = TUN_ROLE_VEHICLE, .key = key, .key_len = 32, .info = "test/1.0",
        .send = fake_send, .on_data = fake_data, .on_event = fake_event, .random = fake_random, .ctx = &f,
    };
    uint32_t now = 5000;
    tun_init(&t, &cfg, now);

    /* first poll sends HELLO: session 0, seq 0, nonce + info */
    tun_poll(&t, now);
    CHECK(f.nsent == 1);
    CHECK(f.sent_len[0] == 12 + 8 + 8 + 16);
    CHECK(f.sent[0][2] == TUN_HELLO && f.sent[0][3] == TUN_ROLE_VEHICLE);
    CHECK(u32(f.sent[0] + 4) == 0 && u32(f.sent[0] + 8) == 0);
    CHECK(memcmp(f.sent[0] + 20, "test/1.0", 8) == 0);
    memcpy(nonce, f.sent[0] + 12, 8);

    /* retries keep the nonce */
    tun_poll(&t, now + 500);
    CHECK(f.nsent == 1);
    tun_poll(&t, now + 1000);
    CHECK(f.nsent == 2 && memcmp(f.sent[1] + 12, nonce, 8) == 0);

    /* WELCOME: wrong nonce, wrong key and wrong sender role are ignored */
    uint8_t bad_nonce[8];
    memcpy(bad_nonce, nonce, 8);
    bad_nonce[7] ^= 1;
    n = tun_encode(&k, pkt, TUN_WELCOME, TUN_ROLE_SERVER, 0x1234, 0, bad_nonce, 8);
    tun_input(&t, pkt, n, now);
    n = tun_encode(&wrong, pkt, TUN_WELCOME, TUN_ROLE_SERVER, 0x1234, 0, nonce, 8);
    tun_input(&t, pkt, n, now);
    n = tun_encode(&k, pkt, TUN_WELCOME, TUN_ROLE_VEHICLE, 0x1234, 0, nonce, 8);
    tun_input(&t, pkt, n, now);
    CHECK(!tun_connected(&t));
    CHECK(t.stats.bad == 2);

    /* the right WELCOME connects and triggers a PING with seq 1 */
    f.nsent = 0;
    n = tun_encode(&k, pkt, TUN_WELCOME, TUN_ROLE_SERVER, 0x1234, 0, nonce, 8);
    tun_input(&t, pkt, n, now);
    CHECK(tun_connected(&t) && t.session == 0x1234);
    CHECK(f.nevents == 1 && f.events[0] == TUN_EVENT_CONNECTED);
    CHECK(f.nsent == 1 && f.sent[0][2] == TUN_PING && u32(f.sent[0] + 4) == 0x1234 && u32(f.sent[0] + 8) == 1);
    CHECK(u32(f.sent[0] + 12) == now);
    /* a replayed WELCOME changes nothing */
    tun_input(&t, pkt, n, now);
    CHECK(t.session == 0x1234 && f.nsent == 1);

    /* DATA goes out with increasing sequence numbers */
    CHECK(tun_send_data(&t, (const uint8_t *)"abc", 3));
    CHECK(f.nsent == 2 && f.sent[1][2] == TUN_DATA && u32(f.sent[1] + 8) == 2);
    CHECK(f.sent_len[1] == 12 + 3 + 16 && memcmp(f.sent[1] + 12, "abc", 3) == 0);

    /* DATA from the server is delivered once; replays and forgeries are dropped */
    n = tun_encode(&k, pkt, TUN_DATA, TUN_ROLE_SERVER, 0x1234, 1, (const uint8_t *)"cmd", 3);
    tun_input(&t, pkt, n, now + 10);
    CHECK(f.ndata == 1 && f.got_len == 3 && memcmp(f.got, "cmd", 3) == 0);
    tun_input(&t, pkt, n, now + 20);
    CHECK(f.ndata == 1);
    pkt[12] ^= 1;
    tun_input(&t, pkt, n, now + 20);
    CHECK(f.ndata == 1);
    n = tun_encode(&k, pkt, TUN_DATA, TUN_ROLE_SERVER, 0x9999, 2, (const uint8_t *)"cmd", 3);
    tun_input(&t, pkt, n, now + 20);
    CHECK(f.ndata == 1);

    /* PONG: round-trip time and GCS presence */
    uint8_t pong[5];
    memcpy(pong, f.sent[0] + 12, 4);
    pong[4] = 0;
    n = tun_encode(&k, pkt, TUN_PONG, TUN_ROLE_SERVER, 0x1234, 2, pong, 5);
    tun_input(&t, pkt, n, now + 87);
    CHECK(t.rtt_ms == 87);
    CHECK(!t.gcs_present);
    pong[4] = TUN_PONG_GCS_PRESENT;
    n = tun_encode(&k, pkt, TUN_PONG, TUN_ROLE_SERVER, 0x1234, 3, pong, 5);
    tun_input(&t, pkt, n, now + 90);
    CHECK(t.gcs_present);

    /* PINGs once a second carry rtt and radio state */
    tun_set_radio(&t, -71, 7);
    f.nsent = 0;
    tun_poll(&t, now + 999);
    CHECK(f.nsent == 0);
    tun_poll(&t, now + 1000);
    CHECK(f.nsent == 1 && f.sent[0][2] == TUN_PING);
    CHECK(f.sent[0][16] == 90 && f.sent[0][17] == 0);            /* rtt 90 ms */
    CHECK((int16_t)(f.sent[0][20] | f.sent[0][21] << 8) == -71); /* rssi */
    CHECK(f.sent[0][22] == 7);                                   /* LTE */

    /* REJECT for another session is ignored, for ours it starts over */
    n = tun_encode(&k, pkt, TUN_REJECT, TUN_ROLE_SERVER, 0x4321, 0, (const uint8_t *)"\x01", 1);
    tun_input(&t, pkt, n, now + 1100);
    CHECK(tun_connected(&t));
    n = tun_encode(&k, pkt, TUN_REJECT, TUN_ROLE_SERVER, 0x1234, 0, (const uint8_t *)"\x01", 1);
    tun_input(&t, pkt, n, now + 1100);
    CHECK(!tun_connected(&t));
    CHECK(f.events[f.nevents - 1] == TUN_EVENT_REJECTED);
    CHECK(!tun_send_data(&t, (const uint8_t *)"x", 1));
    f.nsent = 0;
    tun_poll(&t, now + 1101); /* HELLO right away, with a fresh nonce */
    CHECK(f.nsent == 1 && f.sent[0][2] == TUN_HELLO && memcmp(f.sent[0] + 12, nonce, 8) != 0);

    /* reconnect, then let the link time out */
    memcpy(nonce, f.sent[0] + 12, 8);
    n = tun_encode(&k, pkt, TUN_WELCOME, TUN_ROLE_SERVER, 0x5678, 0, nonce, 8);
    tun_input(&t, pkt, n, now + 1200);
    CHECK(t.session == 0x5678);
    for (uint32_t ms = now + 1300; ms <= now + 1200 + 10000; ms += 100) {
        tun_poll(&t, ms);
    }
    CHECK(tun_connected(&t));
    tun_poll(&t, now + 1200 + 10001);
    CHECK(!tun_connected(&t));
    CHECK(f.events[f.nevents - 1] == TUN_EVENT_TIMEOUT);
}

static void test_tunnel_unanswered_hellos(void)
{
    static tun_client_t t;
    static fake_t f;
    uint8_t key[32] = {0};
    memset(&f, 0, sizeof(f));
    tun_config_t cfg = {.role = TUN_ROLE_VEHICLE, .key = key, .key_len = 32, .send = fake_send,
                        .on_data = fake_data, .on_event = fake_event, .random = fake_random, .ctx = &f};
    tun_init(&t, &cfg, 0);
    for (uint32_t s = 0; s < 29; s++) {
        tun_poll(&t, s * 1000);
    }
    CHECK(f.nsent == 29 && f.nevents == 0);
    tun_poll(&t, 29000); /* 30th HELLO: time to look the server up again */
    CHECK(f.nsent == 30 && f.nevents == 1 && f.events[0] == TUN_EVENT_TIMEOUT);
}

static void test_tunnel_loss(void)
{
    static tun_client_t t;
    static fake_t f;
    uint8_t key[32] = {0}, pkt[64], nonce[8];
    hmac_sha256_key_t k;
    hmac_sha256_setkey(&k, key, 32);
    memset(&f, 0, sizeof(f));
    tun_config_t cfg = {.role = TUN_ROLE_VEHICLE, .key = key, .key_len = 32, .send = fake_send,
                        .on_data = fake_data, .random = fake_random, .ctx = &f};
    tun_init(&t, &cfg, 0);
    tun_poll(&t, 0);
    memcpy(nonce, f.sent[0] + 12, 8);
    size_t n = tun_encode(&k, pkt, TUN_WELCOME, TUN_ROLE_SERVER, 7, 0, nonce, 8);
    tun_input(&t, pkt, n, 0);
    CHECK(tun_loss_permille(&t) == TUN_U16_UNKNOWN);
    for (uint32_t seq = 1; seq <= 100; seq++) { /* every 10th packet lost */
        if (seq % 10) {
            n = tun_encode(&k, pkt, TUN_DATA, TUN_ROLE_SERVER, 7, seq, (const uint8_t *)"x", 1);
            tun_input(&t, pkt, n, seq * 5);
        }
    }
    CHECK(tun_loss_permille(&t) == TUN_U16_UNKNOWN); /* second not complete yet */
    tun_poll(&t, 1000);
    CHECK(tun_loss_permille(&t) == 91); /* 9 of 99 */
}

int main(void)
{
    test_sha256();
    test_golden_packet();
    test_framer();
    test_batcher_stream();
    test_batcher_timing();
    test_tunnel();
    test_tunnel_unanswered_hellos();
    test_tunnel_loss();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
