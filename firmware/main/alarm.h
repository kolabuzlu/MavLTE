/* The locator alarm: what the board's speaker plays while MavLTE's Voice switch is on, as a WAV file for
 * the modem (8 kHz, 16-bit mono PCM): beeps of 2.4 and 3 kHz in turn, 0.2 s each with 50 ms between them,
 * as loud as 16 bits go, 2 s in all. The sound the user picked by ear on the first board, from tones, a
 * spoken phrase and this. Platform independent: the bytes are made, not stored. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define ALARM_WAV_BYTES 32044 /* a 44-byte header, then 16000 samples */
#define ALARM_WAV_MS 2000

/* Bytes [offset, offset + n) of the file into out. */
void alarm_wav(uint8_t *out, size_t offset, size_t n);
