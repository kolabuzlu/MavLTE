/* What the flight controller's MAVLink stream says about it, from the autopilot (component 1): its HEARTBEAT,
 * the proof that a flight controller talks (a byte of noise on an unconnected pin is not one), with its flight
 * mode; GLOBAL_POSITION_INT, for the notes on each photo; and for the flight log its battery (SYS_STATUS), its
 * GPS (GPS_RAW_INT), its speeds (VFR_HUD) and its RC signal (RC_CHANNELS). Checks each frame's CRC, for which
 * it needs these messages' CRC_EXTRA and nothing else. Platform independent. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mavframe.h"

#define MAV_MSG_HEARTBEAT 0
#define MAV_MSG_SYS_STATUS 1
#define MAV_MSG_GPS_RAW_INT 24
#define MAV_MSG_GLOBAL_POSITION_INT 33
#define MAV_MSG_RC_CHANNELS 65
#define MAV_MSG_VFR_HUD 74
#define MAV_ARMED 0x80 /* HEARTBEAT base_mode: MAV_MODE_FLAG_SAFETY_ARMED */

typedef struct {
    mav_framer_t framer;
    uint8_t frame[MAV_MAX_FRAME];
    uint16_t len;
    /* GLOBAL_POSITION_INT */
    bool valid;         /* a position has come */
    uint32_t when_ms;   /* when the last one came */
    int32_t lat, lon;   /* 1e-7 degrees */
    int32_t alt_mm;     /* relative_alt: above home */
    int32_t alt_msl_mm; /* above mean sea level */
    uint16_t heading;   /* centidegrees, 0xFFFF if unknown */
    /* HEARTBEAT */
    bool heartbeat;        /* the autopilot's HEARTBEAT has come */
    uint32_t heartbeat_ms; /* when the last one came */
    uint32_t custom_mode;  /* the flight mode, in the autopilot's numbers */
    uint8_t type, autopilot, base_mode;
    /* SYS_STATUS */
    bool sys;
    uint32_t sys_ms;
    uint16_t battery_mv;   /* 0xFFFF unknown */
    int16_t battery_ca;    /* centiamperes, -1 unknown */
    int8_t battery_pct;    /* -1 unknown */
    /* GPS_RAW_INT */
    bool gps;
    uint32_t gps_ms;
    uint8_t gps_fix;       /* GPS_FIX_TYPE: 0-1 none, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed */
    uint8_t gps_sats;      /* 255 unknown */
    /* VFR_HUD */
    bool hud;
    uint32_t hud_ms;
    float airspeed, groundspeed, climb; /* m/s */
    uint16_t throttle;     /* % */
    /* RC_CHANNELS */
    bool rc;
    uint32_t rc_ms;
    uint8_t rc_rssi;       /* 0-254, 255 unknown */
} mav_position_t;

void mav_position_init(mav_position_t *p);
/* Bytes from the flight controller, as they come. */
void mav_position_feed(mav_position_t *p, const uint8_t *data, size_t len, uint32_t now_ms);
/* The CRC MAVLink uses (X.25), over data, starting from crc (0xFFFF for a new frame). */
uint16_t mav_crc(uint16_t crc, const uint8_t *data, size_t len);
/* ArduPlane's name for a flight mode (HEARTBEAT custom_mode), as "FBWA"; NULL for a number it does not know. */
const char *mav_plane_mode(uint32_t custom_mode);
/* Whether the HEARTBEAT is an ArduPilot plane's (whose custom_mode mav_plane_mode() names). */
bool mav_is_ardupilot_plane(const mav_position_t *p);
