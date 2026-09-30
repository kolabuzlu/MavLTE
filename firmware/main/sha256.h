/* SHA-256 (FIPS 180-4) and HMAC-SHA256 (RFC 2104).
 *
 * Self-contained so the tunnel code builds the same way on the ESP32 and on a PC for tests,
 * independent of the mbedTLS version that ships with ESP-IDF. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define SHA256_BLOCK_SIZE 64
#define SHA256_DIGEST_SIZE 32

typedef struct {
    uint32_t state[8];
    uint64_t length; /* bytes hashed so far */
    uint8_t block[SHA256_BLOCK_SIZE];
    size_t used; /* bytes waiting in block */
} sha256_ctx_t;

void sha256_init(sha256_ctx_t *ctx);
void sha256_update(sha256_ctx_t *ctx, const void *data, size_t len);
void sha256_final(sha256_ctx_t *ctx, uint8_t digest[SHA256_DIGEST_SIZE]);

/* A key prepared for HMAC: hash states after absorbing the inner and outer pads. */
typedef struct {
    sha256_ctx_t inner;
    sha256_ctx_t outer;
} hmac_sha256_key_t;

void hmac_sha256_setkey(hmac_sha256_key_t *k, const uint8_t *key, size_t key_len);
void hmac_sha256(const hmac_sha256_key_t *k, const void *msg, size_t len, uint8_t mac[SHA256_DIGEST_SIZE]);
