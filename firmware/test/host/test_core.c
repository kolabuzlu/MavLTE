/* Host tests for the portable firmware core (sha256, mavframe, tunnel, snapshot, mavpos, locator, alarm).
 * Build and run: make test */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alarm.h"
#include "fileout.h"
#include "locator.h"
#include "logrow.h"
#include "mavframe.h"
#include "mavpos.h"
#include "sha256.h"
#include "snapshot.h"
#include "tunnel.h"
#include "usbproto.h"

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
    CHECK(!t.voice_on);

    /* the locator voice: switched on in a PONG */
    pong[4] = TUN_PONG_GCS_PRESENT | TUN_PONG_VOICE;
    n = tun_encode(&k, pkt, TUN_PONG, TUN_ROLE_SERVER, 0x1234, 4, pong, 5);
    tun_input(&t, pkt, n, now + 90);
    CHECK(t.voice_on && t.gcs_present && t.rtt_ms == 90);

    /* PINGs once a second carry rtt, radio state and flags */
    tun_set_radio(&t, -71, 7);
    tun_set_ping_flags(&t, TUN_PING_SPEAKING);
    f.nsent = 0;
    tun_poll(&t, now + 999);
    CHECK(f.nsent == 0);
    tun_poll(&t, now + 1000);
    CHECK(f.nsent == 1 && f.sent[0][2] == TUN_PING);
    CHECK(f.sent[0][16] == 90 && f.sent[0][17] == 0);            /* rtt 90 ms */
    CHECK((int16_t)(f.sent[0][20] | f.sent[0][21] << 8) == -71); /* rssi */
    CHECK(f.sent[0][22] == 7);                                   /* LTE */
    CHECK(f.sent[0][23] == TUN_PING_SPEAKING);

    /* REJECT for another session is ignored, for ours it starts over */
    n = tun_encode(&k, pkt, TUN_REJECT, TUN_ROLE_SERVER, 0x4321, 0, (const uint8_t *)"\x01", 1);
    tun_input(&t, pkt, n, now + 1100);
    CHECK(tun_connected(&t));
    n = tun_encode(&k, pkt, TUN_REJECT, TUN_ROLE_SERVER, 0x1234, 0, (const uint8_t *)"\x01", 1);
    tun_input(&t, pkt, n, now + 1100);
    CHECK(!tun_connected(&t));
    CHECK(f.events[f.nevents - 1] == TUN_EVENT_REJECTED);
    CHECK(!tun_send_data(&t, (const uint8_t *)"x", 1));
    CHECK(t.voice_on); /* kept without a session: the aircraft goes on speaking where it has no coverage */
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
    CHECK(pos.heartbeat && pos.heartbeat_ms == 1234); /* the HEARTBEAT: a flight controller talks */

    /* noise on an unconnected RX pin is no flight controller, nor is a damaged HEARTBEAT, nor another
     * component's */
    mav_position_t quiet;
    mav_position_init(&quiet);
    mav_position_feed(&quiet, (const uint8_t *)"\x00\x13junk\xff", 7, 100);
    uint8_t bad[32];
    memcpy(bad, hb, n);
    bad[12] ^= 0x01;
    mav_position_feed(&quiet, bad, n, 200);
    memcpy(bad, hb, n);
    bad[6] = 2; /* component 2 (a camera, say), its CRC right */
    uint16_t crc2 = mav_crc(mav_crc(0xFFFF, bad + 1, 18), &extra, 1);
    bad[n - 2] = (uint8_t)crc2;
    bad[n - 1] = (uint8_t)(crc2 >> 8);
    mav_position_feed(&quiet, bad, n, 300);
    CHECK(!quiet.heartbeat);
    mav_position_feed(&quiet, hb, n, 400); /* and then the autopilot's own */
    CHECK(quiet.heartbeat && quiet.heartbeat_ms == 400);

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
    /* this project's first board (A7670E-FASE, A7670M7_V1.11.1, 2026-10-01; position replaced): decimal
     * degrees, four satellite fields with one empty, no course, a field at the end */
    CHECK(gnss_parse("+CGNSSINFO: 3,10,,00,00,41.1234567,N,28.9876543,E,011026,145023.00,128.6,5.516,,5.48,4.36,"
                     "3.32,04", &f));
    CHECK(f.fix == GNSS_FIX_3D && f.sats == 10 && f.lat == 411234567 && f.lon == 289876543);
    CHECK(f.alt_mm == 128600 && f.speed == 283 && f.course == LOCATOR_U16_UNKNOWN && f.hdop == 436);
    CHECK(f.time == gnss_unix_time(2026, 10, 1, 14, 50, 23));

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

/* The locator alarm: the WAV file the user picked by ear (made then by a Python script): the same header,
 * and every sample within 1 of that script's, which works in double precision */
static void test_alarm(void)
{
    static uint8_t wav[ALARM_WAV_BYTES];
    alarm_wav(wav, 0, 100); /* in pieces, as the firmware uploads it */
    alarm_wav(wav + 100, 100, sizeof(wav) - 100);
    char hex[2 * 44 + 1];
    for (int i = 0; i < 44; i++) {
        sprintf(hex + 2 * i, "%02x", wav[i]);
    }
    CHECK(strcmp(hex, "52494646247d000057415645666d74201000000001000100401f0000803e00000200100064617461007d0000") == 0);
    int worst = 0;
    for (int i = 0; i < 16000; i++) {
        int beep = i / 2000, k = i % 2000;
        double env = k >= 1600 ? 0.0 : fmin(1.0, fmin(k / 40.0, (1599 - k) / 40.0));
        int expect = (int)(32000 * env * sin(2 * 3.141592653589793 * (beep % 2 ? 3000 : 2400) * k / 8000));
        int got = (int16_t)(wav[44 + 2 * i] | wav[45 + 2 * i] << 8);
        worst = abs(got - expect) > worst ? abs(got - expect) : worst;
    }
    CHECK(worst <= 1);
    /* a few of the script's own values */
    static const struct {
        int i, v;
    } known[] = {{1, 760}, {2, -940}, {39, -29672}, {41, 30433}, {1999, 0}, {2001, 565}, {2041, 22627}};
    for (size_t j = 0; j < sizeof(known) / sizeof(known[0]); j++) {
        int got = (int16_t)(wav[44 + 2 * known[j].i] | wav[45 + 2 * known[j].i] << 8);
        CHECK(abs(got - known[j].v) <= 1);
    }
    uint8_t piece[3];
    alarm_wav(piece, 44 + 2 * 41 - 1, 3); /* across a sample's two bytes */
    CHECK(piece[1] == wav[44 + 2 * 41] && piece[2] == wav[45 + 2 * 41] && piece[0] == wav[43 + 2 * 41]);
}

/* ------------------------------------------------------------------ the flight controller's telemetry (the log) */

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | p[1] << 8);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}

static void putf(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    put32(p, u);
}

/* A MAVLink 2 frame from the autopilot (system 1, component 1), its trailing zeros left out, with its CRC. */
static size_t telemetry_frame(uint8_t *f, uint32_t msgid, uint8_t extra, const uint8_t *payload, size_t plen)
{
    while (plen > 1 && payload[plen - 1] == 0) {
        plen--;
    }
    uint8_t head[10] = {0xFD, (uint8_t)plen, 0, 0, 7, 1, 1, (uint8_t)msgid, (uint8_t)(msgid >> 8), (uint8_t)(msgid >> 16)};
    memcpy(f, head, 10);
    memcpy(f + 10, payload, plen);
    uint16_t crc = mav_crc(mav_crc(0xFFFF, f + 1, 9 + plen), &extra, 1);
    f[10 + plen] = (uint8_t)crc;
    f[11 + plen] = (uint8_t)(crc >> 8);
    return 12 + plen;
}

/* An ArduPlane in FBWA, armed, on its battery, with its GPS, flying (all at now_ms). */
static void fly(mav_position_t *p, uint32_t now_ms)
{
    uint8_t f[80], m[52];
    memset(m, 0, sizeof(m));
    put32(m, 5); /* FBWA */
    m[4] = 1;    /* MAV_TYPE_FIXED_WING */
    m[5] = 3;    /* MAV_AUTOPILOT_ARDUPILOTMEGA */
    m[6] = MAV_ARMED | 0x01;
    m[7] = 4;
    m[8] = 3;
    mav_position_feed(p, f, telemetry_frame(f, MAV_MSG_HEARTBEAT, 50, m, 9), now_ms);
    memset(m, 0, sizeof(m));
    put16(m + 14, 12345); /* 12.345 V */
    put16(m + 16, 2345);  /* 23.45 A */
    m[30] = 67;
    mav_position_feed(p, f, telemetry_frame(f, MAV_MSG_SYS_STATUS, 124, m, 31), now_ms);
    memset(m, 0, sizeof(m));
    m[28] = 3;
    m[29] = 14;
    mav_position_feed(p, f, telemetry_frame(f, MAV_MSG_GPS_RAW_INT, 24, m, 30), now_ms);
    memset(m, 0, sizeof(m));
    putf(m, 18.5f);
    putf(m + 4, 21.25f);
    putf(m + 8, 150.0f);
    putf(m + 12, 1.5f);
    put16(m + 16, 90);
    put16(m + 18, 55);
    mav_position_feed(p, f, telemetry_frame(f, MAV_MSG_VFR_HUD, 20, m, 20), now_ms);
    memset(m, 0, sizeof(m));
    m[40] = 16;
    m[41] = 200;
    mav_position_feed(p, f, telemetry_frame(f, MAV_MSG_RC_CHANNELS, 118, m, 42), now_ms);
    memset(m, 0, sizeof(m));
    put32(m + 4, 411234567);
    put32(m + 8, 289876543);
    put32(m + 12, 250000);
    put32(m + 16, 120000);
    put16(m + 26, 4500);
    mav_position_feed(p, f, telemetry_frame(f, MAV_MSG_GLOBAL_POSITION_INT, 104, m, 28), now_ms);
}

static void test_telemetry(void)
{
    mav_position_t p;
    mav_position_init(&p);
    CHECK(p.battery_mv == 0xFFFF && p.battery_ca == -1 && p.battery_pct == -1 && p.gps_sats == 255 && p.rc_rssi == 255);
    fly(&p, 1000);
    CHECK(p.heartbeat && p.heartbeat_ms == 1000 && p.custom_mode == 5 && (p.base_mode & MAV_ARMED));
    CHECK(mav_is_ardupilot_plane(&p) && strcmp(mav_plane_mode(p.custom_mode), "FBWA") == 0);
    CHECK(strcmp(mav_plane_mode(10), "AUTO") == 0 && strcmp(mav_plane_mode(21), "QRTL") == 0);
    CHECK(mav_plane_mode(9) == NULL && mav_plane_mode(99) == NULL);
    CHECK(p.sys && p.battery_mv == 12345 && p.battery_ca == 2345 && p.battery_pct == 67);
    CHECK(p.gps && p.gps_fix == 3 && p.gps_sats == 14);
    CHECK(p.hud && p.airspeed == 18.5f && p.groundspeed == 21.25f && p.climb == 1.5f && p.throttle == 55);
    CHECK(p.rc && p.rc_rssi == 200);
    CHECK(p.valid && p.lat == 411234567 && p.alt_msl_mm == 250000 && p.alt_mm == 120000 && p.heading == 4500);

    /* a damaged SYS_STATUS changes nothing, nor does one with another message's CRC_EXTRA */
    uint8_t f[80], m[52];
    memset(m, 0, sizeof(m));
    put16(m + 14, 9999);
    size_t n = telemetry_frame(f, MAV_MSG_SYS_STATUS, 124, m, 31);
    f[25] ^= 1;
    mav_position_feed(&p, f, n, 2000);
    mav_position_feed(&p, f, telemetry_frame(f, MAV_MSG_SYS_STATUS, 125, m, 31), 2000);
    CHECK(p.battery_mv == 12345 && p.sys_ms == 1000);
    /* a copter's HEARTBEAT: its modes have other numbers */
    memset(m, 0, sizeof(m));
    put32(m, 5);
    m[4] = 2;
    m[5] = 3;
    mav_position_feed(&p, f, telemetry_frame(f, MAV_MSG_HEARTBEAT, 50, m, 9), 3000);
    CHECK(!mav_is_ardupilot_plane(&p) && p.custom_mode == 5);
}

/* ------------------------------------------------------------------ the log's line */

static int columns(const char *line)
{
    int c = 1;
    for (; *line; line++) {
        c += *line == ',';
    }
    return c;
}

static void join(char *out, const char *const *fields, int n)
{
    out[0] = '\0';
    for (int i = 0; i < n; i++) {
        if (i) {
            strcat(out, ",");
        }
        strcat(out, fields[i]);
    }
    strcat(out, "\n");
}

static void test_log_line(void)
{
    char t[24];
    log_time_text(t, 1790933712u);
    CHECK(strcmp(t, "2026-10-02T09:35:12Z") == 0);
    log_time_text(t, 951782400u);
    CHECK(strcmp(t, "2000-02-29T00:00:00Z") == 0);

    cell_info_t c;
    /* the A7670E's answer on the bench (modem firmware A7670M7_V1.11.1) */
    CHECK(cell_parse("\r\n+CPSI: LTE,Online,286-01,0x172B,387360,449,EUTRAN-BAND3,1651,5,0,22,64,64,10\r\n\r\nOK\r\n", &c));
    CHECK(strcmp(c.mode, "LTE") == 0 && strcmp(c.plmn, "286-01") == 0 && strcmp(c.band, "B3") == 0);
    CHECK(c.cell == 387360 && c.rsrp_dbm == -76 && c.rsrq_half_db == -18 && c.rssi_dbm == -46 && c.sinr_db == 10);
    /* the AT manual's example: 255 is unknown */
    CHECK(cell_parse("+CPSI: LTE,Online,460-01,0x230A,175499523,318,EUTRAN-BAND3,1650,5,0,21,67,255,19", &c));
    CHECK(c.rsrp_dbm == -73 && c.rsrq_half_db == -19 && c.rssi_dbm == LOG_I16_UNKNOWN && c.sinr_db == 19);
    CHECK(cell_parse("+CPSI: GSM,Online,286-01,0x2B5C,12401,27,-64,2110,42-42", &c));
    CHECK(strcmp(c.mode, "GSM") == 0 && strcmp(c.band, "GSM900") == 0 && c.cell == 12401);
    CHECK(c.rsrp_dbm == LOG_I16_UNKNOWN && c.rssi_dbm == LOG_I16_UNKNOWN);
    CHECK(cell_parse("+CPSI: GSM,Online,286-02,0x2B5C,12401,700,-64,2110,42-42", &c) && strcmp(c.band, "DCS1800") == 0);
    CHECK(cell_parse("+CPSI: NO SERVICE,Online", &c));
    CHECK(strcmp(c.mode, "NO SERVICE") == 0 && c.cell == 0 && c.band[0] == '\0');
    CHECK(!cell_parse("\r\nERROR\r\n", &c));

    /* all of it known */
    static log_row_t r;
    static char line[LOG_LINE_MAX], expect[LOG_LINE_MAX];
    memset(&r, 0, sizeof(r));
    r.utc = 1790933712u;
    r.uptime_s = 754;
    r.gnss_state = 1;
    r.gnss = (gnss_fix_t){1790933710u, 411234567, 289876543, 150000, 1234, 27350, 90, 11, GNSS_FIX_3D};
    r.gnss_age_s = 2;
    r.signal_dbm = -71;
    strcpy(r.operator_name, "Turk,cell");
    cell_parse("+CPSI: LTE,Online,286-01,0x172B,387360,449,EUTRAN-BAND3,1651,5,0,22,64,64,10", &r.cell);
    r.relay = true;
    r.rtt_ms = 54;
    r.loss_permille = 12;
    r.data_kb = 1234;
    r.gcs = true;
    mav_position_init(&r.fc);
    fly(&r.fc, 100000);
    r.now_ms = 101500;
    r.chip_c = 41;
    r.rail_mv = 3950;
    r.cell_pct = 78;
    r.voice = 2;
    r.events = "relay connected, photo 1790933700 sent\n";
    const char *full[] = {
        "2026-10-02T09:35:12Z", "754",
        "3", "11", "41.1234567", "28.9876543", "150.0", "12.3", "273.5", "0.90", "2",
        "LTE", "-71", "Turk;cell", "286-01", "B3", "387360", "-76", "-9.0", "-46", "10",
        "1", "54", "1.2", "1234", "1",
        "1", "FBWA", "1", "3", "14", "41.1234567", "28.9876543", "250.0", "120.0", "45.0",
        "21.3", "18.5", "1.5", "55", "12.345", "23.4", "67", "200",
        "41", "cell", "78", "3950", "sounding", "relay connected; photo 1790933700 sent ",
    };
    CHECK(sizeof(full) / sizeof(full[0]) == 50 && columns(log_header()) == 50);
    join(expect, full, 50);
    size_t n = log_format(line, sizeof(line), &r);
    CHECK(n == strlen(line) && strcmp(line, expect) == 0);

    /* the flight controller quiet for 6 s: only how long, nothing older than 5 s */
    r.now_ms = 106000;
    r.rail_mv = 4298; /* USB or the BEC: the gauge reads the rail, not the cell */
    r.events = "";
    r.gnss.fix = GNSS_FIX_NONE; /* the GNSS lost its fix */
    r.gnss.hdop = LOCATOR_U16_UNKNOWN;
    log_format(line, sizeof(line), &r);
    const char *quiet[] = {
        "2026-10-02T09:35:12Z", "754",
        "0", "11", "", "", "", "", "", "", "2",
        "LTE", "-71", "Turk;cell", "286-01", "B3", "387360", "-76", "-9.0", "-46", "10",
        "1", "54", "1.2", "1234", "1",
        "6", "", "", "", "", "", "", "", "", "",
        "", "", "", "", "", "", "", "",
        "41", "ext", "", "4298", "sounding", "",
    };
    join(expect, quiet, 50);
    CHECK(strcmp(line, expect) == 0);

    /* nothing known yet: just after power-on */
    memset(&r, 0x5A, sizeof(r));
    log_row_clear(&r);
    r.uptime_s = 3;
    log_format(line, sizeof(line), &r);
    const char *none[50];
    for (int i = 0; i < 50; i++) {
        none[i] = "";
    }
    none[1] = "3";
    none[21] = "0"; /* relay */
    none[24] = "0"; /* data */
    none[25] = "0"; /* gcs */
    none[48] = "off";
    join(expect, none, 50);
    CHECK(strcmp(line, expect) == 0);
}

/* A log file as relay/tests/test_logs.py makes them: a header, `untimed` lines without a time, then `lines` lines a
 * second apart from first_time. */
static size_t log_file(char *out, size_t n, unsigned lines, uint32_t first_time, unsigned untimed)
{
    size_t len = (size_t)snprintf(out, n, "time_utc,uptime_s,gnss_fix\n");
    for (unsigned i = 0; i < untimed; i++) {
        len += (size_t)snprintf(out + len, n - len, ",%u,0\n", i);
    }
    for (unsigned i = 0; i < lines; i++) {
        char t[24];
        log_time_text(t, first_time + i);
        len += (size_t)snprintf(out + len, n - len, "%s,%u,3,0123456789abcdef0123456789abcdef01234567\n", t,
                                untimed + i);
    }
    return len;
}

/* What sdlog.c does with a file: its first 2 KB, and its last 2 KB. */
static void file_times(const char *file, size_t len, uint32_t *start, uint32_t *end)
{
    static char buf[2049];
    uint32_t first_up, t, up;
    *start = *end = 0;
    size_t head = len < 2048 ? len : 2048;
    memcpy(buf, file, head);
    buf[head] = '\0';
    if (!log_first_uptime(buf, &first_up)) {
        return;
    }
    size_t from = len > 2048 ? len - 2048 : 0;
    memcpy(buf, file + from, len - from);
    buf[len - from] = '\0';
    if (log_last_time(buf, from > 0, &t, &up)) {
        *end = t;
        *start = up >= first_up ? t - (up - first_up) : t;
    }
}

static void test_log_times(void)
{
    static char file[64 * 1024];
    uint32_t start, end;
    size_t len = log_file(file, sizeof(file), 500, 1790000000u, 20); /* longer than what is read of each end */
    file_times(file, len, &start, &end);
    CHECK(start == 1789999980u && end == 1790000499u);
    len = log_file(file, sizeof(file), 50, 1790000000u, 200); /* no time at all in what is read of its start */
    file_times(file, len, &start, &end);
    CHECK(start == 1789999800u && end == 1790000049u);
    len = log_file(file, sizeof(file), 3, 1790000000u, 0); /* shorter than what is read */
    file_times(file, len, &start, &end);
    CHECK(start == 1790000000u && end == 1790000002u);
    len = log_file(file, sizeof(file), 0, 0, 2); /* no GNSS and no relay yet: no time at all */
    file_times(file, len, &start, &end);
    CHECK(start == 0 && end == 0);
    len = log_file(file, sizeof(file), 0, 0, 0); /* only the header: the power went within a second */
    file_times(file, len, &start, &end);
    CHECK(start == 0 && end == 0);
    /* a real line, all 50 columns */
    char line[] = "time_utc,uptime_s\n2026-10-02T09:35:12Z,754,3,11,41.1234567\n";
    char tail[sizeof(line)];
    memcpy(tail, line, sizeof(line));
    uint32_t up;
    CHECK(log_first_uptime(line, &up) && up == 754);
    CHECK(log_last_time(tail, false, &start, &up) && start == 1790933712u && up == 754);
}

/* ------------------------------------------------------------------ the USB link */

static void test_usb_lines(void)
{
    CHECK(usb_crc32(0, (const uint8_t *)"123456789", 9) == 0xCBF43926u);
    CHECK(usb_crc32(usb_crc32(0, (const uint8_t *)"1234", 4), (const uint8_t *)"56789", 5) == 0xCBF43926u);
    char out[64];
    const char *in[] = {"", "f", "fo", "foo", "foob", "fooba", "foobar"};
    const char *b64[] = {"", "Zg==", "Zm8=", "Zm9v", "Zm9vYg==", "Zm9vYmE=", "Zm9vYmFy"};
    for (int i = 0; i < 7; i++) {
        CHECK(usb_base64(out, (const uint8_t *)in[i], strlen(in[i])) == strlen(b64[i]) && strcmp(out, b64[i]) == 0);
    }
    static char line[USB_LINE_MAX];
    size_t n = usb_data_line(line, 4096, (const uint8_t *)"foobar", 6);
    CHECK(n == strlen(line) && strcmp(line, "@D 4096 Zm9vYmFy 9ef61f95\n") == 0);
    static uint8_t chunk[USB_DATA_CHUNK + 10];
    for (size_t i = 0; i < sizeof(chunk); i++) {
        chunk[i] = (uint8_t)(i * 13);
    }
    n = usb_data_line(line, 4294967295u, chunk, sizeof(chunk)); /* at most a chunk */
    CHECK(n == strlen(line) && n == 3 + 10 + 1 + 1024 + 1 + 8 + 1 && n < USB_LINE_MAX);

    char name[16];
    uint32_t num = 7;
    CHECK(usb_parse("MAVLTE HELLO", name, sizeof(name), &num) == USB_CMD_HELLO);
    CHECK(usb_parse("MAVLTE LIST\r", name, sizeof(name), &num) == USB_CMD_LIST && num == 0);
    CHECK(usb_parse("MAVLTE LIST 40", name, sizeof(name), &num) == USB_CMD_LIST && num == 40);
    CHECK(usb_parse("  MAVLTE  GET LOG00012.CSV 4096", name, sizeof(name), &num) == USB_CMD_GET);
    CHECK(strcmp(name, "LOG00012.CSV") == 0 && num == 4096);
    CHECK(usb_parse("MAVLTE GET LOG00012.CSV", name, sizeof(name), &num) == USB_CMD_GET && num == 0);
    CHECK(usb_parse("MAVLTE SPEED 2000000", name, sizeof(name), &num) == USB_CMD_SPEED && num == 2000000);
    CHECK(usb_parse("MAVLTE STOP", name, sizeof(name), &num) == USB_CMD_STOP);
    CHECK(usb_parse("I (1234) modem: registered with Turkcell", name, sizeof(name), &num) == USB_CMD_NONE);
    CHECK(usb_parse("MAVLTE GET", name, sizeof(name), &num) == USB_CMD_NONE);
    CHECK(usb_parse("MAVLTE SPEED fast", name, sizeof(name), &num) == USB_CMD_NONE);
    CHECK(usb_parse("mavlte hello", name, sizeof(name), &num) == USB_CMD_NONE);
    CHECK(usb_parse("", name, sizeof(name), &num) == USB_CMD_NONE);
    CHECK(usb_parse("MAVLTE GET LOG00012.CSV 0", name, 12, &num) == USB_CMD_NONE); /* no room for the name */
}

/* ------------------------------------------------------------------ logs over the tunnel (fileout.c) */

#define CARD_FILES 45
#define LOG_PACKETS 400

typedef struct {
    uint8_t data[2][40000]; /* LOG00045.CSV and LOG00044.CSV; the other files are empty */
    uint32_t size[2];
    bool no_card, fail_read, open;
    int which;
    int opens, closes;
    /* what was sent */
    int n;
    uint8_t type[LOG_PACKETS];
    uint16_t len[LOG_PACKETS];
    uint8_t body[LOG_PACKETS][FILE_DATA_HEAD_LEN + FILE_CHUNK];
} card_t;

static int card_list(void *ctx, unsigned first, file_entry_t *out, unsigned max, unsigned *total)
{
    card_t *c = ctx;
    if (c->no_card) {
        return -FILE_NO_CARD;
    }
    *total = CARD_FILES;
    unsigned n = 0;
    for (unsigned i = first; i < CARD_FILES && n < max; i++, n++) {
        snprintf(out[n].name, sizeof(out[n].name), "LOG%05u.CSV", (unsigned)((CARD_FILES - i % 100) % 100000));
        out[n].size = i < 2 ? c->size[i] : 0;
        out[n].start = i < 2 ? 1790000000u + i : 0;
        out[n].end = i < 2 ? 1790003600u + i : 0;
    }
    return (int)n;
}

static int card_open(void *ctx, const char *name, uint32_t *size)
{
    card_t *c = ctx;
    if (c->no_card) {
        return FILE_NO_CARD;
    }
    int which = strcmp(name, "LOG00045.CSV") == 0 ? 0 : strcmp(name, "LOG00044.CSV") == 0 ? 1 : -1;
    if (which < 0) {
        return FILE_NOT_FOUND;
    }
    CHECK(!c->open); /* one file at a time */
    c->open = true;
    c->which = which;
    c->opens++;
    *size = c->size[which];
    return FILE_OK;
}

static bool card_read(void *ctx, uint32_t offset, uint8_t *buf, size_t len)
{
    card_t *c = ctx;
    CHECK(c->open && offset + len <= c->size[c->which]);
    if (c->fail_read) {
        return false;
    }
    memcpy(buf, c->data[c->which] + offset, len);
    return true;
}

static void card_close(void *ctx)
{
    card_t *c = ctx;
    CHECK(c->open);
    c->open = false;
    c->closes++;
}

static bool card_send(void *ctx, uint8_t type, const uint8_t *body, size_t len)
{
    card_t *c = ctx;
    if (c->n < LOG_PACKETS) {
        c->type[c->n] = type;
        c->len[c->n] = (uint16_t)len;
        memcpy(c->body[c->n], body, len);
    }
    c->n++;
    return true;
}

static void file_req(file_outbox_t *o, uint16_t id, uint8_t op, uint32_t offset, const char *name, uint32_t now)
{
    uint8_t body[FILE_REQ_LEN] = {0};
    put16(body, id);
    body[2] = op;
    put32(body + 3, offset);
    memcpy(body + 7, name, strlen(name));
    fileout_input(o, TUN_FILE_REQ, body, sizeof(body), now);
}

static void file_ack(file_outbox_t *o, uint16_t id, uint32_t next, uint32_t now)
{
    uint8_t body[FILE_ACK_LEN];
    put16(body, id);
    put32(body + 2, next);
    fileout_input(o, TUN_FILE_ACK, body, sizeof(body), now);
}

static void test_fileout(void)
{
    static card_t c;
    static file_outbox_t o;
    memset(&c, 0, sizeof(c));
    for (int f = 0; f < 2; f++) {
        for (size_t i = 0; i < sizeof(c.data[f]); i++) {
            c.data[f][i] = (uint8_t)(i * (f ? 7 : 3) + (i >> 8));
        }
    }
    c.size[0] = 40000;
    c.size[1] = 2500;
    file_config_t cfg = {card_list, card_open, card_read, card_close, card_send, &c};
    fileout_init(&o, &cfg);
    uint32_t now = 0xFFFF0000u; /* the clock wraps during the test */

    /* LIST, a page at a time */
    file_req(&o, 7, FILE_OP_LIST, 0, "", now);
    CHECK(fileout_busy(&o) && c.n == 0); /* nothing from inside tun_input: the card waits for the poll */
    CHECK(!fileout_poll(&o, now, TUN_U16_UNKNOWN, 32768.0f) && !fileout_busy(&o));
    CHECK(c.n == 1 && c.type[0] == TUN_FILE_LIST && c.len[0] == FILE_LIST_HEAD_LEN + 40 * FILE_ENTRY_LEN);
    CHECK(get16(c.body[0]) == 7 && c.body[0][2] == FILE_OK && get16(c.body[0] + 3) == 45 && get16(c.body[0] + 5) == 0);
    CHECK(c.body[0][7] == 40 && memcmp(c.body[0] + 8, "LOG00045.CSV", 12) == 0);
    CHECK(u32(c.body[0] + 8 + 12) == 40000 && u32(c.body[0] + 8 + 16) == 1790000000u && u32(c.body[0] + 8 + 20) == 1790003600u);
    file_req(&o, 8, FILE_OP_LIST, 40, "", now);
    fileout_poll(&o, now, TUN_U16_UNKNOWN, 32768.0f);
    CHECK(c.n == 2 && c.body[1][7] == 5 && get16(c.body[1] + 5) == 40 && memcmp(c.body[1] + 8, "LOG00005.CSV", 12) == 0);

    /* a whole file, lost packets and all: the receiver ACKs every 200 ms what came in order */
    c.n = 0;
    file_req(&o, 9, FILE_OP_GET, 0, "log00045.csv", now); /* any case */
    fileout_poll(&o, now, 60, 65536.0f);
    CHECK(o.sending && c.opens == 1);
    static uint8_t got[40000];
    uint32_t have = 0, sent_ahead = 0, last_ack = now;
    int lost = 0, i = 0, ended = 0;
    while (have < 40000 && (uint32_t)(now - 0xFFFF0000u) < 120000) {
        now += 10;
        ended += fileout_poll(&o, now, 60, 65536.0f);
        for (; i < c.n && i < LOG_PACKETS; i++) {
            CHECK(c.type[i] == TUN_FILE_DATA && get16(c.body[i]) == 9 && c.body[i][2] == FILE_OK);
            uint32_t off = u32(c.body[i] + 3), size = u32(c.body[i] + 7), len = c.len[i] - FILE_DATA_HEAD_LEN;
            CHECK(size == 40000 && len >= 1 && len <= FILE_CHUNK && off + len <= size);
            if (off + len > have && off + len - have > sent_ahead) {
                sent_ahead = off + len - have;
            }
            if ((i % 7) == 3 && lost < 6) { /* lost on the way */
                lost++;
                continue;
            }
            if (off == have) {
                memcpy(got + off, c.body[i] + FILE_DATA_HEAD_LEN, len);
                have += len;
            }
        }
        if (i >= LOG_PACKETS) { /* keep the capture small: start counting again */
            c.n = i = 0;
        }
        if ((uint32_t)(now - last_ack) >= 200) {
            last_ack = now;
            file_ack(&o, 9, have, now);
        }
    }
    CHECK(have == 40000 && memcmp(got, c.data[0], 40000) == 0 && lost == 6);
    CHECK(sent_ahead <= FILE_WINDOW + FILE_CHUNK); /* never far beyond the last ACK */
    file_ack(&o, 9, 40000, now);
    CHECK(fileout_poll(&o, now, 60, 65536.0f) && !o.sending && c.closes == 1);
    CHECK(strcmp(o.last_name, "LOG00045.CSV") == 0 && o.last_bytes == 40000 && o.last_complete && ended == 0);

    /* no ACK: back to the last one after the retransmission time, then gives up */
    c.n = 0;
    file_req(&o, 10, FILE_OP_GET, 1024, "LOG00045.CSV", now);
    for (int k = 0; k < 300; k++) {
        now += 10;
        fileout_poll(&o, now, TUN_U16_UNKNOWN, 65536.0f); /* rate starts at 4 KB/s: a few chunks in 3 s */
    }
    int again = 0;
    for (int k = 1; k < c.n && k < LOG_PACKETS; k++) {
        again += u32(c.body[k] + 3) == 1024; /* from the start again */
    }
    CHECK(c.n >= 3 && u32(c.body[0] + 3) == 1024 && again >= 1);
    now += FILE_GIVE_UP_MS;
    CHECK(fileout_poll(&o, now, TUN_U16_UNKNOWN, 65536.0f) && !o.sending && !o.last_complete);

    /* asked again from where the receiver stopped: the same download goes on */
    file_req(&o, 11, FILE_OP_GET, 0, "LOG00045.CSV", now);
    fileout_poll(&o, now, 60, 65536.0f);
    file_ack(&o, 11, 2048, now + 100);
    file_req(&o, 11, FILE_OP_GET, 2048, "LOG00045.CSV", now + 100);
    CHECK(!fileout_poll(&o, now + 100, 60, 65536.0f) && o.sending && o.acked == 2048 && c.opens == 3);

    /* another download takes over: the first one hears STOPPED */
    c.n = 0;
    file_req(&o, 12, FILE_OP_GET, 0, "LOG00044.CSV", now + 200);
    CHECK(fileout_poll(&o, now + 200, 60, 65536.0f) && !o.last_complete && o.sending && o.id == 12);
    CHECK(c.type[0] == TUN_FILE_DATA && get16(c.body[0]) == 11 && c.body[0][2] == FILE_STOPPED && c.len[0] == FILE_DATA_HEAD_LEN);
    file_req(&o, 12, FILE_OP_STOP, 0, "LOG00044.CSV", now + 300); /* and stops */
    CHECK(fileout_poll(&o, now + 300, 60, 65536.0f) && !o.sending && c.closes == c.opens);

    /* problems: not there, from the end (nothing more), the card failing, no card */
    c.n = 0;
    file_req(&o, 13, FILE_OP_GET, 0, "LOG00099.CSV", now);
    fileout_poll(&o, now, 60, 65536.0f);
    file_req(&o, 14, FILE_OP_GET, 2500, "LOG00044.CSV", now);
    fileout_poll(&o, now, 60, 65536.0f);
    CHECK(c.n == 2 && c.body[0][2] == FILE_NOT_FOUND && get16(c.body[0]) == 13);
    CHECK(c.body[1][2] == FILE_OK && u32(c.body[1] + 3) == 2500 && u32(c.body[1] + 7) == 2500 && c.len[1] == FILE_DATA_HEAD_LEN);
    CHECK(!o.sending && c.closes == c.opens);
    c.fail_read = true;
    file_req(&o, 15, FILE_OP_GET, 0, "LOG00044.CSV", now);
    for (int k = 0; k < 50 && (o.sending || fileout_busy(&o)); k++) {
        now += 20;
        fileout_poll(&o, now, 60, 65536.0f);
    }
    CHECK(c.body[c.n - 1][2] == FILE_CARD_ERROR && !o.sending && c.closes == c.opens);
    c.fail_read = false;
    c.no_card = true;
    c.n = 0;
    file_req(&o, 16, FILE_OP_LIST, 0, "", now);
    file_req(&o, 17, FILE_OP_GET, 0, "LOG00044.CSV", now);
    fileout_poll(&o, now, 60, 65536.0f);
    CHECK(c.n == 2 && c.type[0] == TUN_FILE_LIST && c.body[0][2] == FILE_NO_CARD && c.body[0][7] == 0);
    CHECK(c.type[1] == TUN_FILE_DATA && c.body[1][2] == FILE_NO_CARD);
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
    test_alarm();
    test_telemetry();
    test_log_line();
    test_log_times();
    test_usb_lines();
    test_fileout();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
