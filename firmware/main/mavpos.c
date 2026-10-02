#include "mavpos.h"

#include <string.h>

#define MAV_IFLAG_SIGNED 0x01
#define MAV_SIGNATURE_LEN 13
#define MAV_AUTOPILOT_ARDUPILOTMEGA 3
#define MAV_TYPE_FIXED_WING 1
#define MAV_TYPE_VTOL_FIRST 19 /* the VTOL types (19-25) are ArduPlane's quadplanes */
#define MAV_TYPE_VTOL_LAST 25

/* The messages read here: id, CRC_EXTRA, and their longest payload (MAVLink 2 extensions included) */
static const struct {
    uint32_t id;
    uint8_t extra, len;
} known[] = {
    {MAV_MSG_HEARTBEAT, 50, 9},
    {MAV_MSG_SYS_STATUS, 124, 43},
    {MAV_MSG_GPS_RAW_INT, 24, 52},
    {MAV_MSG_GLOBAL_POSITION_INT, 104, 28},
    {MAV_MSG_RC_CHANNELS, 118, 42},
    {MAV_MSG_VFR_HUD, 20, 20},
};

uint16_t mav_crc(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t tmp = data[i] ^ (uint8_t)crc;
        tmp ^= (uint8_t)(tmp << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4));
    }
    return crc;
}

static int32_t get_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | p[1] << 8);
}

static float get_float(const uint8_t *p)
{
    float f;
    uint32_t u = (uint32_t)get_i32(p);
    memcpy(&f, &u, sizeof(f));
    return f;
}

void mav_position_init(mav_position_t *p)
{
    memset(p, 0, sizeof(*p));
    p->heading = 0xFFFF;
    p->battery_mv = 0xFFFF;
    p->battery_ca = -1;
    p->battery_pct = -1;
    p->gps_sats = 255;
    p->rc_rssi = 255;
}

static void frame_done(mav_position_t *p, const uint8_t *f, size_t n, uint32_t now_ms)
{
    size_t head, plen = f[1];
    uint32_t msgid;
    uint8_t compid;
    if (f[0] == MAV_STX_V2) {
        if (n < 12 || n != 12 + plen + ((f[2] & MAV_IFLAG_SIGNED) ? MAV_SIGNATURE_LEN : 0)) {
            return;
        }
        head = 10;
        msgid = (uint32_t)f[7] | (uint32_t)f[8] << 8 | (uint32_t)f[9] << 16;
        compid = f[6];
    } else {
        if (n < 8 || n != 8 + plen) {
            return;
        }
        head = 6;
        msgid = f[5];
        compid = f[4];
    }
    if (compid != 1) {
        return;
    }
    size_t k = 0;
    while (k < sizeof(known) / sizeof(known[0]) && known[k].id != msgid) {
        k++;
    }
    if (k == sizeof(known) / sizeof(known[0]) || plen > known[k].len) {
        return;
    }
    uint16_t crc = mav_crc(mav_crc(0xFFFF, f + 1, head - 1 + plen), &known[k].extra, 1);
    if (crc != ((uint16_t)f[head + plen] | (uint16_t)f[head + plen + 1] << 8)) {
        return;
    }
    uint8_t m[52] = {0}; /* MAVLink 2 leaves out trailing zeros */
    memcpy(m, f + head, plen);
    switch (msgid) {
    case MAV_MSG_HEARTBEAT:
        p->heartbeat = true;
        p->heartbeat_ms = now_ms;
        p->custom_mode = (uint32_t)get_i32(m);
        p->type = m[4];
        p->autopilot = m[5];
        p->base_mode = m[6];
        break;
    case MAV_MSG_SYS_STATUS:
        p->sys = true;
        p->sys_ms = now_ms;
        p->battery_mv = get_u16(m + 14);
        p->battery_ca = (int16_t)get_u16(m + 16);
        p->battery_pct = (int8_t)m[30];
        break;
    case MAV_MSG_GPS_RAW_INT:
        p->gps = true;
        p->gps_ms = now_ms;
        p->gps_fix = m[28];
        p->gps_sats = m[29];
        break;
    case MAV_MSG_GLOBAL_POSITION_INT:
        p->lat = get_i32(m + 4);
        p->lon = get_i32(m + 8);
        p->alt_msl_mm = get_i32(m + 12);
        p->alt_mm = get_i32(m + 16);
        p->heading = get_u16(m + 26);
        p->valid = true;
        p->when_ms = now_ms;
        break;
    case MAV_MSG_RC_CHANNELS:
        p->rc = true;
        p->rc_ms = now_ms;
        p->rc_rssi = m[41];
        break;
    case MAV_MSG_VFR_HUD:
        p->hud = true;
        p->hud_ms = now_ms;
        p->airspeed = get_float(m);
        p->groundspeed = get_float(m + 4);
        p->climb = get_float(m + 12);
        p->throttle = get_u16(m + 18);
        break;
    }
}

void mav_position_feed(mav_position_t *p, const uint8_t *data, size_t len, uint32_t now_ms)
{
    for (size_t i = 0; i < len; i++) {
        bool in_frame = mav_framer_in_frame(&p->framer);
        bool boundary = mav_framer_push(&p->framer, data[i]);
        if (!in_frame && !mav_framer_in_frame(&p->framer)) {
            continue; /* a byte between frames */
        }
        if (!in_frame) {
            p->len = 0; /* a frame starts */
        }
        if (p->len < sizeof(p->frame)) {
            p->frame[p->len++] = data[i];
        }
        if (boundary) { /* the frame is complete (or was not one: frame_done checks its length) */
            frame_done(p, p->frame, p->len, now_ms);
            p->len = 0;
        }
    }
}

const char *mav_plane_mode(uint32_t custom_mode)
{
    static const char *const names[] = {
        "MANUAL", "CIRCLE", "STABILIZE", "TRAINING", "ACRO", "FBWA", "FBWB", "CRUISE", "AUTOTUNE", NULL,
        "AUTO", "RTL", "LOITER", "TAKEOFF", "AVOID_ADSB", "GUIDED", "INITIALISING", "QSTABILIZE", "QHOVER",
        "QLOITER", "QLAND", "QRTL", "QAUTOTUNE", "QACRO", "THERMAL", "LOITER_QLAND", "AUTOLAND",
    };
    return custom_mode < sizeof(names) / sizeof(names[0]) ? names[custom_mode] : NULL;
}

bool mav_is_ardupilot_plane(const mav_position_t *p)
{
    return p->heartbeat && p->autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA &&
           (p->type == MAV_TYPE_FIXED_WING || (p->type >= MAV_TYPE_VTOL_FIRST && p->type <= MAV_TYPE_VTOL_LAST));
}
