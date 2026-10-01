#include "alarm.h"

#include <math.h>
#include <string.h>

#define RATE 8000
#define BEEP 1600 /* samples: 0.2 s */
#define GAP 400   /* 50 ms */
#define RAMP 40   /* 5 ms in and out: no clicks */
#define BEEPS 8
#define SAMPLES (BEEPS * (BEEP + GAP))

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    put_u16(p, (uint16_t)v);
    put_u16(p + 2, (uint16_t)(v >> 16));
}

static void header(uint8_t *h)
{
    memcpy(h, "RIFF", 4);
    put_u32(h + 4, 36 + 2 * SAMPLES);
    memcpy(h + 8, "WAVEfmt ", 8);
    put_u32(h + 16, 16);
    put_u16(h + 20, 1); /* PCM */
    put_u16(h + 22, 1); /* mono */
    put_u32(h + 24, RATE);
    put_u32(h + 28, 2 * RATE);
    put_u16(h + 32, 2);
    put_u16(h + 34, 16);
    memcpy(h + 36, "data", 4);
    put_u32(h + 40, 2 * SAMPLES);
}

static int16_t sample(size_t i)
{
    size_t beep = i / (BEEP + GAP), k = i % (BEEP + GAP);
    if (k >= BEEP) {
        return 0;
    }
    float env = 1.0f;
    if (k < RAMP) {
        env = (float)k / RAMP;
    } else if (BEEP - 1 - k < RAMP) {
        env = (float)(BEEP - 1 - k) / RAMP;
    }
    float cycles = (beep % 2 ? 3000.0f : 2400.0f) * (float)k / RATE;
    return (int16_t)(32000.0f * env * sinf(6.2831853f * (cycles - floorf(cycles))));
}

void alarm_wav(uint8_t *out, size_t offset, size_t n)
{
    uint8_t h[44];
    header(h);
    for (size_t i = 0; i < n; i++) {
        size_t at = offset + i;
        if (at < sizeof(h)) {
            out[i] = h[at];
        } else {
            uint16_t v = (uint16_t)sample((at - sizeof(h)) / 2);
            out[i] = (at - sizeof(h)) % 2 ? (uint8_t)(v >> 8) : (uint8_t)v;
        }
    }
}
