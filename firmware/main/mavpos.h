/* Where the aircraft is, read from the flight controller's MAVLink stream: GLOBAL_POSITION_INT of the
 * autopilot (component 1), for the notes on each photo. Checks the frame's CRC, for which it needs
 * this one message's CRC_EXTRA and nothing else. Platform independent. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mavframe.h"

#define MAV_MSG_GLOBAL_POSITION_INT 33

typedef struct {
    mav_framer_t framer;
    uint8_t frame[MAV_MAX_FRAME];
    uint16_t len;
    bool valid;       /* a position has come */
    uint32_t when_ms; /* when the last one came */
    int32_t lat, lon; /* 1e-7 degrees */
    int32_t alt_mm;   /* relative_alt: above home */
    uint16_t heading; /* centidegrees, 0xFFFF if unknown */
} mav_position_t;

void mav_position_init(mav_position_t *p);
/* Bytes from the flight controller, as they come. */
void mav_position_feed(mav_position_t *p, const uint8_t *data, size_t len, uint32_t now_ms);
/* The CRC MAVLink uses (X.25), over data, starting from crc (0xFFFF for a new frame). */
uint16_t mav_crc(uint16_t crc, const uint8_t *data, size_t len);
