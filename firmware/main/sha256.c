#include "sha256.h"

#include <string.h>

static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static void compress(uint32_t s[8], const uint8_t *p)
{
    uint32_t w[64];
    for (int i = 0; i < 16; i++) {
        w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 | p[4 * i + 3];
    }
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ROTR(w[i - 15], 7) ^ ROTR(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ROTR(w[i - 2], 17) ^ ROTR(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = s[0], b = s[1], c = s[2], d = s[3], e = s[4], f = s[5], g = s[6], h = s[7];
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = h + (ROTR(e, 6) ^ ROTR(e, 11) ^ ROTR(e, 25)) + ((e & f) ^ (~e & g)) + K[i] + w[i];
        uint32_t t2 = (ROTR(a, 2) ^ ROTR(a, 13) ^ ROTR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    s[0] += a;
    s[1] += b;
    s[2] += c;
    s[3] += d;
    s[4] += e;
    s[5] += f;
    s[6] += g;
    s[7] += h;
}

void sha256_init(sha256_ctx_t *ctx)
{
    static const uint32_t H0[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
    };
    memcpy(ctx->state, H0, sizeof(H0));
    ctx->length = 0;
    ctx->used = 0;
}

void sha256_update(sha256_ctx_t *ctx, const void *data, size_t len)
{
    const uint8_t *p = data;
    ctx->length += len;
    if (ctx->used) {
        size_t n = SHA256_BLOCK_SIZE - ctx->used;
        if (n > len) {
            n = len;
        }
        memcpy(ctx->block + ctx->used, p, n);
        ctx->used += n;
        p += n;
        len -= n;
        if (ctx->used < SHA256_BLOCK_SIZE) {
            return;
        }
        compress(ctx->state, ctx->block);
        ctx->used = 0;
    }
    while (len >= SHA256_BLOCK_SIZE) {
        compress(ctx->state, p);
        p += SHA256_BLOCK_SIZE;
        len -= SHA256_BLOCK_SIZE;
    }
    if (len) {
        memcpy(ctx->block, p, len);
        ctx->used = len;
    }
}

void sha256_final(sha256_ctx_t *ctx, uint8_t digest[SHA256_DIGEST_SIZE])
{
    uint64_t bits = ctx->length * 8;
    size_t used = ctx->used;
    ctx->block[used++] = 0x80;
    if (used > 56) {
        memset(ctx->block + used, 0, SHA256_BLOCK_SIZE - used);
        compress(ctx->state, ctx->block);
        used = 0;
    }
    memset(ctx->block + used, 0, 56 - used);
    for (int i = 0; i < 8; i++) {
        ctx->block[63 - i] = (uint8_t)(bits >> (8 * i));
    }
    compress(ctx->state, ctx->block);
    for (int i = 0; i < 8; i++) {
        digest[4 * i] = (uint8_t)(ctx->state[i] >> 24);
        digest[4 * i + 1] = (uint8_t)(ctx->state[i] >> 16);
        digest[4 * i + 2] = (uint8_t)(ctx->state[i] >> 8);
        digest[4 * i + 3] = (uint8_t)ctx->state[i];
    }
}

void hmac_sha256_setkey(hmac_sha256_key_t *k, const uint8_t *key, size_t key_len)
{
    uint8_t block[SHA256_BLOCK_SIZE] = {0};
    uint8_t pad[SHA256_BLOCK_SIZE];
    if (key_len > SHA256_BLOCK_SIZE) {
        sha256_ctx_t c;
        sha256_init(&c);
        sha256_update(&c, key, key_len);
        sha256_final(&c, block);
    } else {
        memcpy(block, key, key_len);
    }
    for (int i = 0; i < SHA256_BLOCK_SIZE; i++) {
        pad[i] = block[i] ^ 0x36;
    }
    sha256_init(&k->inner);
    sha256_update(&k->inner, pad, sizeof(pad));
    for (int i = 0; i < SHA256_BLOCK_SIZE; i++) {
        pad[i] = block[i] ^ 0x5c;
    }
    sha256_init(&k->outer);
    sha256_update(&k->outer, pad, sizeof(pad));
    memset(block, 0, sizeof(block));
    memset(pad, 0, sizeof(pad));
}

void hmac_sha256(const hmac_sha256_key_t *k, const void *msg, size_t len, uint8_t mac[SHA256_DIGEST_SIZE])
{
    uint8_t inner[SHA256_DIGEST_SIZE];
    sha256_ctx_t c = k->inner;
    sha256_update(&c, msg, len);
    sha256_final(&c, inner);
    c = k->outer;
    sha256_update(&c, inner, sizeof(inner));
    sha256_final(&c, mac);
}
