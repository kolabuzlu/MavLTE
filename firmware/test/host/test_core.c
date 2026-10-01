/* Host tests for the portable firmware core (sha256, mavframe, tunnel, snapshot, mavpos, locator).
 * Build and run: make test */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "locator.h"
#include "mavframe.h"
#include "mavpos.h"
#include "sha256.h"
#include "snapshot.h"
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

/* ------------------------------------------------------------------ snapshots */

#define WIRE_MAX 64

typedef struct { /* what the tunnel sent, decoded */
    int n;
    uint8_t type[WIRE_MAX];
    uint8_t body[WIRE_MAX][TUN_MAX_PAYLOAD];
    size_t len[WIRE_MAX];
} wire_t;

static void wire_send(void *ctx, const uint8_t *p, size_t n)
{
    wire_t *w = ctx;
    if (w->n < WIRE_MAX) {
        w->type[w->n] = p[2];
        w->len[w->n] = n - TUN_HEADER_LEN - TUN_TAG_LEN;
        memcpy(w->body[w->n], p + TUN_HEADER_LEN, w->len[w->n]);
    }
    w->n++;
}

typedef struct {
    int takes, releases;
    uint32_t take_id, release_id;
    uint16_t width, height;
    bool sent;
} camera_log_t;

static void fake_take(void *ctx, uint32_t photo_id, uint16_t width, uint16_t height)
{
    camera_log_t *c = ctx;
    c->takes++;
    c->take_id = photo_id;
    c->width = width;
    c->height = height;
}

static void fake_release(void *ctx, uint32_t photo_id, bool sent)
{
    camera_log_t *c = ctx;
    c->releases++;
    c->release_id = photo_id;
    c->sent = sent;
}

static void connect_tunnel(tun_client_t *t, wire_t *w, uint32_t now)
{
    uint8_t key[32] = {0}, pkt[64], nonce[8];
    hmac_sha256_key_t k;
    hmac_sha256_setkey(&k, key, 32);
    tun_config_t cfg = {.role = TUN_ROLE_VEHICLE, .key = key, .key_len = 32, .send = wire_send,
                        .on_data = fake_data, .random = fake_random, .ctx = w};
    tun_init(t, &cfg, now);
    tun_poll(t, now);
    memcpy(nonce, w->body[0], 8);
    size_t n = tun_encode(&k, pkt, TUN_WELCOME, TUN_ROLE_SERVER, 7, 0, nonce, 8);
    tun_input(t, pkt, n, now);
    w->n = 0;
}

static void request(snap_outbox_t *o, uint32_t photo_id, uint8_t size, uint32_t now)
{
    uint8_t body[5] = {(uint8_t)photo_id, (uint8_t)(photo_id >> 8), (uint8_t)(photo_id >> 16),
                       (uint8_t)(photo_id >> 24), size};
    snap_input(o, TUN_SNAP_REQ, body, sizeof(body), now);
}

static void ack(snap_outbox_t *o, uint32_t photo_id, uint8_t flags, uint8_t bitmap, uint32_t now)
{
    uint8_t body[6] = {(uint8_t)photo_id, (uint8_t)(photo_id >> 8), (uint8_t)(photo_id >> 16),
                       (uint8_t)(photo_id >> 24), flags, bitmap};
    snap_input(o, TUN_SNAP_ACK, body, sizeof(body), now);
}

/* the status in the last SNAP_INFO sent, or -1 */
static int last_status(const wire_t *w, uint32_t photo_id)
{
    for (int i = w->n - 1; i >= 0; i--) {
        if (w->type[i] == TUN_SNAP_INFO && u32(w->body[i]) == photo_id) {
            return w->body[i][26];
        }
    }
    return -1;
}

static int count(const wire_t *w, uint8_t type)
{
    int c = 0;
    for (int i = 0; i < w->n && i < WIRE_MAX; i++) {
        c += w->type[i] == type;
    }
    return c;
}

static void test_snapshot(void)
{
    static tun_client_t t;
    static wire_t w;
    static snap_outbox_t o;
    static uint8_t photo[2500];
    camera_log_t cam = {0};
    uint32_t now = 0xFFFFF000u; /* the millisecond clock wraps around during this test */
    for (size_t i = 0; i < sizeof(photo); i++) {
        photo[i] = (uint8_t)(i * 7 + 3);
    }
    memset(&w, 0, sizeof(w));
    connect_tunnel(&t, &w, now);
    snap_config_t cfg = {.take = fake_take, .release = fake_release, .ctx = &cam};
    snap_init(&o, &t, &cfg);

    /* asked for a medium photo: the camera starts */
    request(&o, 1000, 1, now);
    CHECK(cam.takes == 1 && cam.take_id == 1000 && cam.width == 640 && cam.height == 480);
    CHECK(snap_busy(&o) && w.n == 0);
    request(&o, 1001, 0, now); /* another one meanwhile: busy */
    CHECK(last_status(&w, 1001) == SNAP_BUSY && cam.takes == 1);
    w.n = 0;
    request(&o, 1000, 1, now); /* the relay asking again: already being taken */
    CHECK(w.n == 0 && cam.takes == 1);

    /* the photo: its SNAP_INFO at once, the chunks as the rate allows (4 KB/s to start with) */
    snap_where_t where = {411234567, 289876543, 120000, 4500};
    snap_photo_taken(&o, 1000, SNAP_OK, photo, sizeof(photo), &where, now);
    snap_poll(&o, now, 32768);
    CHECK(w.n == 1 && w.type[0] == TUN_SNAP_INFO && w.len[0] == SNAP_INFO_LEN);
    CHECK(u32(w.body[0]) == 1000 && u32(w.body[0] + 4) == 2500);
    CHECK(w.body[0][8] == (640 & 0xFF) && w.body[0][9] == 640 >> 8 && w.body[0][10] == 480 - 256);
    CHECK(u32(w.body[0] + 12) == 411234567u && u32(w.body[0] + 20) == 120000u && w.body[0][26] == SNAP_OK);
    CHECK(w.body[0][24] == (4500 & 0xFF) && w.body[0][25] == 4500 >> 8);
    for (uint32_t ms = 100; ms <= 900; ms += 100) {
        snap_poll(&o, now + ms, 32768);
    }
    CHECK(count(&w, TUN_SNAP_DATA) == 3 && count(&w, TUN_SNAP_INFO) == 1);
    for (int i = 1; i < w.n; i++) {
        unsigned index = w.body[i][4] | w.body[i][5] << 8;
        size_t len = index < 2 ? 1024 : 452;
        CHECK(u32(w.body[i]) == 1000 && w.len[i] == 6 + len);
        CHECK(memcmp(w.body[i] + 6, photo + index * 1024, len) == 0);
    }
    /* chunk 1 lost: the ACK shows 0 and 2; after the wait (2 s while the round trip is unknown) chunk 1
     * goes again, and only it */
    ack(&o, 1000, SNAP_ACK_HAVE_INFO, 0x05, now + 1000);
    w.n = 0;
    snap_poll(&o, now + 1500, 32768);
    CHECK(w.n == 0);
    for (uint32_t ms = 2000; ms <= 3200; ms += 100) {
        snap_poll(&o, now + ms, 32768);
    }
    CHECK(w.n == 1 && w.type[0] == TUN_SNAP_DATA && w.body[0][4] == 1);
    /* all there: released, sent */
    ack(&o, 1000, SNAP_ACK_DONE | SNAP_ACK_HAVE_INFO, 0x07, now + 3300);
    snap_poll(&o, now + 3310, 32768);
    CHECK(!snap_busy(&o) && cam.releases == 1 && cam.release_id == 1000 && cam.sent);
    CHECK(o.last_id == 1000 && o.last_sent && o.last_ms == 3310);
    w.n = 0;
    request(&o, 1000, 1, now + 3400); /* a late copy of the request: not taken again */
    CHECK(w.n == 0 && cam.takes == 1);

    /* the camera fails: the relay hears so, and again if it asks again */
    request(&o, 1002, 2, now + 4000);
    CHECK(cam.takes == 2 && cam.width == 1024);
    snap_photo_taken(&o, 1002, SNAP_FAILED, NULL, 0, NULL, now + 5000);
    CHECK(last_status(&w, 1002) == SNAP_FAILED && !snap_busy(&o));
    w.n = 0;
    request(&o, 1002, 2, now + 5100);
    CHECK(last_status(&w, 1002) == SNAP_FAILED && cam.takes == 2);

    /* sent, but never acknowledged: given up after a minute */
    request(&o, 1003, 0, now + 6000);
    snap_photo_taken(&o, 1003, SNAP_OK, photo, 1000, NULL, now + 6000);
    CHECK(u32(o.info + 12) == (uint32_t)SNAP_UNKNOWN_I32 && o.info[24] == 0xFF && o.info[25] == 0xFF); /* no position */
    for (uint32_t ms = 6000; ms <= 6000 + 60000; ms += 500) {
        snap_poll(&o, now + ms, 2048);
    }
    CHECK(snap_busy(&o));
    snap_poll(&o, now + 6000 + 60001, 2048);
    CHECK(!snap_busy(&o) && cam.releases == 2 && cam.release_id == 1003 && !cam.sent);

    /* a new session (perhaps a new relay): the photo on its way goes again, all of it */
    request(&o, 1004, 0, now + 70000);
    snap_photo_taken(&o, 1004, SNAP_OK, photo, 2048, NULL, now + 70000);
    for (uint32_t ms = 70000; ms <= 71000; ms += 50) {
        snap_poll(&o, now + ms, 32768);
    }
    ack(&o, 1004, SNAP_ACK_HAVE_INFO, 0x03, now + 71000);
    w.n = 0;
    snap_restart(&o, now + 71100);
    for (uint32_t ms = 71100; ms <= 72000; ms += 50) {
        snap_poll(&o, now + ms, 32768);
    }
    CHECK(count(&w, TUN_SNAP_INFO) == 1 && count(&w, TUN_SNAP_DATA) == 2);

    /* a photo nobody asked for is let go at once; one too large is refused */
    snap_photo_taken(&o, 999, SNAP_OK, photo, 100, NULL, now + 72000);
    CHECK(cam.releases == 3 && cam.release_id == 999);
    ack(&o, 1004, SNAP_ACK_DONE | SNAP_ACK_HAVE_INFO, 0x03, now + 72000);
    snap_poll(&o, now + 72001, 32768);
    request(&o, 1005, 0, now + 73000);
    snap_photo_taken(&o, 1005, SNAP_OK, photo, (size_t)SNAP_MAX_CHUNKS * SNAP_CHUNK + 1, NULL, now + 73000);
    CHECK(last_status(&w, 1005) == SNAP_FAILED && cam.release_id == 1005 && !snap_busy(&o));

    /* no camera at all */
    snap_config_t none = {.release = fake_release, .ctx = &cam};
    snap_init(&o, &t, &none);
    w.n = 0;
    request(&o, 2000, 1, now);
    request(&o, 2000, 1, now + 2000);
    CHECK(count(&w, TUN_SNAP_INFO) == 2 && last_status(&w, 2000) == SNAP_NO_CAMERA);
}

static void test_snapshot_rate(void)
{
    snap_rate_t r;
    snap_rate_init(&r, 32768, 32768);
    snap_rate_sample(&r, 100, 0);
    snap_rate_sample(&r, 200, 1000); /* 100 ms more: within the allowance of 150 ms */
    CHECK(r.rate == 32768);
    snap_rate_sample(&r, 400, 2000); /* a queue: half as fast */
    CHECK(r.rate == 16384);
    snap_rate_sample(&r, 400, 2500); /* at most once a second */
    CHECK(r.rate == 16384);
    for (uint32_t s = 3; s < 20; s++) {
        snap_rate_sample(&r, 2000, s * 1000);
    }
    CHECK(r.rate == 256); /* never slower than this */
    for (uint32_t s = 20; s < 40; s++) {
        snap_rate_sample(&r, 100, s * 1000);
    }
    CHECK(r.rate == 32768); /* back up, a tenth of the cap a time */

    snap_rate_init(&r, 2048, 2048); /* 2G: a second's round trip is normal */
    snap_rate_sample(&r, 1000, 0);
    snap_rate_sample(&r, 1450, 1000);
    CHECK(r.rate == 2048);
    snap_rate_sample(&r, 1600, 2000);
    CHECK(r.rate == 1024);

    snap_rate_init(&r, 10000, 10000);
    CHECK(snap_rate_budget(&r, 0) == 0);
    CHECK(snap_rate_budget(&r, 100) == 1000);
    r.tokens -= 1000;
    CHECK(snap_rate_budget(&r, 10000) == 2048); /* no big burst after a pause */
    CHECK(snap_rto_ms(TUN_U16_UNKNOWN) == 2000 && snap_rto_ms(100) == 1000 && snap_rto_ms(1000) == 2500);
}

/* ------------------------------------------------------------------ position from the flight controller */

static size_t position_frame(uint8_t *f, uint8_t compid, int32_t lat, int32_t lon, int32_t rel_alt, uint16_t hdg)
{
    uint8_t p[28] = {0};
    memcpy(p + 4, &lat, 4);
    memcpy(p + 8, &lon, 4);
    memcpy(p + 16, &rel_alt, 4);
    memcpy(p + 26, &hdg, 2);
    size_t plen = 28;
    while (plen > 1 && p[plen - 1] == 0) { /* MAVLink 2 leaves out trailing zeros */
        plen--;
    }
    uint8_t head[10] = {0xFD, (uint8_t)plen, 0, 0, 42, 1, compid, 33, 0, 0};
    memcpy(f, head, 10);
    memcpy(f + 10, p, plen);
    uint8_t extra = 104;
    uint16_t crc = mav_crc(mav_crc(0xFFFF, f + 1, 9 + plen), &extra, 1);
    f[10 + plen] = (uint8_t)crc;
    f[11 + plen] = (uint8_t)(crc >> 8);
    return 12 + plen;
}

static void test_position(void)
{
    /* the CRC: the shared golden HEARTBEAT frame (CRC_EXTRA 50) */
    uint8_t hb[32];
    size_t n = unhex("fd0900002a0101000000000000000103410303ca52", hb);
    uint8_t extra = 50;
    CHECK(mav_crc(mav_crc(0xFFFF, hb + 1, 18), &extra, 1) == (hb[n - 2] | hb[n - 1] << 8));

    mav_position_t pos;
    mav_position_init(&pos);
    uint8_t stream[512], frame[64];
    size_t len = 0;
    memcpy(stream, "\x00\x11junk", 6); /* bytes between frames */
    len += 6;
    memcpy(stream + len, hb, n); /* a HEARTBEAT: not a position */
    len += n;
    size_t fl = position_frame(frame, 1, 411234567, 289876543, 120000, 4500);
    memcpy(stream + len, frame, fl);
    len += fl;
    for (size_t i = 0; i < len; i += 5) { /* in pieces, as the UART hands them over */
        mav_position_feed(&pos, stream + i, len - i < 5 ? len - i : 5, 1234);
    }
    CHECK(pos.valid && pos.when_ms == 1234);
    CHECK(pos.lat == 411234567 && pos.lon == 289876543 && pos.alt_mm == 120000 && pos.heading == 4500);

    fl = position_frame(frame, 1, -335000000, -704000000, -2000, 0); /* southern and western, below home */
    mav_position_feed(&pos, frame, fl, 2000);
    CHECK(pos.lat == -335000000 && pos.lon == -704000000 && pos.alt_mm == -2000 && pos.heading == 0);
    fl = position_frame(frame, 1, 1, 2, 3, 4);
    frame[12] ^= 0x40; /* damaged on the way: CRC wrong */
    mav_position_feed(&pos, frame, fl, 3000);
    CHECK(pos.lat == -335000000 && pos.when_ms == 2000);
    fl = position_frame(frame, 2, 1, 2, 3, 4); /* another component's (a camera gimbal, say) */
    mav_position_feed(&pos, frame, fl, 3000);
    CHECK(pos.lat == -335000000);
}

/* ------------------------------------------------------------------ locator */

static void test_gnss_parse(void)
{
    gnss_fix_t f;
    /* NMEA-style degrees and minutes (A76XX firmware of 2023-2024), with the answer's OK after it */
    CHECK(gnss_parse("\r\n+CGNSSINFO: 3,12,05,06,3113.343286,N,12121.234064,E,131124,091747.0,32.9,0.0,255.0,"
                     "1.1,0.8,0.7\r\n\r\nOK\r\n", &f));
    CHECK(f.fix == GNSS_FIX_3D && f.sats == 23);
    CHECK(f.lat == 312223881 && f.lon == 1213539010); /* 31 deg 13.343286', 121 deg 21.234064' */
    CHECK(f.time == 1731489467u);                     /* 2024-11-13 09:17:47 UTC */
    CHECK(f.alt_mm == 32900 && f.speed == 0 && f.course == 25500 && f.hdop == 80);

    /* decimal degrees (other firmware), 2D, southern and western, moving */
    CHECK(gnss_parse("+CGNSSINFO: 2,09,05,00,33.9123456,S,70.6123456,W,300926,120000.0,150.5,12.3,45.6,1.2,0.9,0.8",
                     &f));
    CHECK(f.fix == GNSS_FIX_2D && f.sats == 14);
    CHECK(f.lat == -339123456 && f.lon == -706123456);
    CHECK(f.time == 1790769600u); /* 2026-09-30 12:00:00 UTC */
    CHECK(f.alt_mm == 150500 && f.speed == 632 && f.course == 4560 && f.hdop == 90);

    /* answers seen from real modems (A7670SA-FASE A7670M7_V1.11.1 of 2022, A7670SA of 2025, SIM7670G,
     * SIM7670 of 2025): empty course, DOPs like "12.", 3 or 4 satellite fields, an extra field at the end */
    CHECK(gnss_parse("+CGNSSINFO: 3,08,,00,00,1947.80135,S,04354.86991,W,151122,124230.00,802.6,0.414,,12.,5.1,10.",
                     &f));
    CHECK(f.fix == GNSS_FIX_3D && f.sats == 8 && f.lat == -197966891 && f.lon == -439144985);
    CHECK(f.alt_mm == 802600 && f.speed == 21 && f.course == LOCATOR_U16_UNKNOWN && f.hdop == 510);
    CHECK(f.time == gnss_unix_time(2022, 11, 15, 12, 42, 30));
    CHECK(gnss_parse("+CGNSSINFO: 3,08,,,,22.5711975,N,113.8874359,E,010625,084112.00,20.9,0.000,,2.08,1.15,1.74,13",
                     &f));
    CHECK(f.lat == 225711975 && f.lon == 1138874359 && f.hdop == 115 && f.speed == 0 && f.sats == 8);
    CHECK(gnss_parse("+CGNSSINFO: 3,11,07,10,14,45.391761,N,122.797858,W,140726,161102.000,54.9,0.01,215.75,0.82,"
                     "0.43,0.70,36", &f));
    CHECK(f.lat == 453917610 && f.lon == -1227978580 && f.sats == 42 && f.course == 21575 && f.hdop == 43);
    CHECK(gnss_parse("+CGNSSINFO: 3,13,06,18,31.399003,N,73.175373,E,261225,191834.000,195.5,0.16,0.00,0.80,0.53,"
                     "0.60,25", &f));
    CHECK(f.lat == 313990030 && f.lon == 731753730 && f.sats == 37 && f.speed == 8 && f.hdop == 53);
    CHECK(f.time == gnss_unix_time(2025, 12, 26, 19, 18, 34));
    CHECK(gnss_parse("+CGNSSINFO: ,,,,,,,,", &f) && f.fix == GNSS_FIX_NONE);  /* no fix, the short form */
    CHECK(gnss_parse("+CGNSSINFO:,,,,,,,,,,,,,,,", &f) && f.fix == GNSS_FIX_NONE); /* no space after the colon */
    CHECK(gnss_parse("+CGNSSINFO: 3,05,00,00,0.000000,N,0.000000,E,010126,000000.0,0.0,0.0,0.0,99.,99.,99.", &f));
    CHECK(f.fix == GNSS_FIX_NONE && f.lat == LOCATOR_UNKNOWN_I32); /* a "fix" at 0, 0 */
    CHECK(gnss_parse("+CGNSSINFO: 3,08,,,,530.500000,N,2859.259258,E,010126,000000.0,11.0,0.0,0.0,1.0,1.0,1.0", &f));
    CHECK(f.lat == 55083333 && f.lon == 289876543); /* degrees and minutes without the leading zero */

    /* a longitude west of Greenwich by less than a degree, in NMEA form (leading zeros) */
    CHECK(gnss_parse("+CGNSSINFO: 3,08,,,5130.000000,N,00007.500000,W,010126,000000.0,11.0,0.0,0.0,1.0,1.0,1.0", &f));
    CHECK(f.lat == 515000000 && f.lon == -1250000);

    /* no fix yet: the fields are empty, or only the satellites are known */
    CHECK(gnss_parse("+CGNSSINFO: ,,,,,,,,,,,,,,,\r\nOK", &f));
    CHECK(f.fix == GNSS_FIX_NONE && f.sats == 0 && f.lat == LOCATOR_UNKNOWN_I32);
    CHECK(gnss_parse("+CGNSSINFO: 1,03,00,01,,,,,,,,,,,,", &f));
    CHECK(f.fix == GNSS_FIX_NONE && f.sats == 4);

    /* not an answer, or garbage where the position should be */
    CHECK(!gnss_parse("ERROR", &f));
    CHECK(!gnss_parse("", &f));
    CHECK(gnss_parse("+CGNSSINFO: 3,12,05,06,99xx.1,N,12121.234064,E,131124,091747.0,32.9,0.0,255.0,1.1,0.8,0.7", &f));
    CHECK(f.fix == GNSS_FIX_NONE && f.lat == LOCATOR_UNKNOWN_I32);
    CHECK(gnss_parse("+CGNSSINFO: 3,12,05,06,3175.0,N,12121.234064,E,131124,091747.0,32.9,0.0,255.0", &f));
    CHECK(f.fix == GNSS_FIX_NONE); /* 75 minutes */
    CHECK(gnss_parse("+CGNSSINFO: 3,12,05,06,95.5,N,121.2,E,131124,091747.0,,,,,", &f));
    CHECK(f.fix == GNSS_FIX_NONE); /* 95.5 degrees north */
    CHECK(gnss_parse("+CGNSSINFO: 3,12,05,06,41.1,N,28.9,E", &f)); /* cut short: a position, nothing more */
    CHECK(f.fix == GNSS_FIX_3D && f.lat == 411000000 && f.time == 0 && f.alt_mm == LOCATOR_UNKNOWN_I32);

    CHECK(gnss_unix_time(2020, 1, 1, 0, 0, 0) == 1577836800u);
    CHECK(gnss_unix_time(2028, 2, 29, 23, 59, 59) == 1835481599u);
    CHECK(gnss_unix_time(1999, 12, 31, 0, 0, 0) == 0 && gnss_unix_time(2026, 13, 1, 0, 0, 0) == 0);
}

static void test_locator_packet(void)
{
    /* the same bytes as mavrelay.Position(...).pack() in relay/mavrelay.py */
    uint8_t body[LOCATOR_BODY_LEN], expect[LOCATOR_BODY_LEN];
    gnss_fix_t f = {.time = 1790841600u, .lat = 411234567, .lon = -289876543, .alt_mm = 150500, .speed = 632,
                    .course = 4560, .hdop = 90, .sats = 14, .fix = GNSS_FIX_3D};
    locator_pack(body, &f, LOCATOR_FC_SILENT, 42, 3950, 78, 47);
    CHECK(unhex("0013be6a07f18218c1d5b8eee44b02007802d0115a000e03012a006e0f4e000000002f", expect) == sizeof(expect));
    CHECK(memcmp(body, expect, sizeof(body)) == 0);
    locator_pack(body, NULL, LOCATOR_NO_GNSS, LOCATOR_U16_UNKNOWN, LOCATOR_U16_UNKNOWN, LOCATOR_BATTERY_UNKNOWN,
                 LOCATOR_TEMP_UNKNOWN);
    unhex("00000000000000800000008000000080ffffffffffff000002ffffffffff0000000080", expect);
    CHECK(memcmp(body, expect, sizeof(body)) == 0);
    locator_pack(body, NULL, 0, 0, 0, 0, -5); /* below freezing */
    CHECK(body[34] == 0xFB);
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
    test_snapshot();
    test_snapshot_rate();
    test_position();
    test_gnss_parse();
    test_locator_packet();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
