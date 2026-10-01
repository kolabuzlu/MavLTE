#include "mavpos.h"

#include <string.h>

#define CRC_EXTRA_HEARTBEAT 50
#define HEARTBEAT_LEN 9
#define CRC_EXTRA_GLOBAL_POSITION_INT 104
#define GLOBAL_POSITION_INT_LEN 28
#define MAV_IFLAG_SIGNED 0x01
#define MAV_SIGNATURE_LEN 13

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

void mav_position_init(mav_position_t *p)
{
    memset(p, 0, sizeof(*p));
    p->heading = 0xFFFF;
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
    uint8_t extra;
    if (compid != 1) {
        return;
    } else if (msgid == MAV_MSG_GLOBAL_POSITION_INT && plen <= GLOBAL_POSITION_INT_LEN) {
        extra = CRC_EXTRA_GLOBAL_POSITION_INT;
    } else if (msgid == MAV_MSG_HEARTBEAT && plen <= HEARTBEAT_LEN) {
        extra = CRC_EXTRA_HEARTBEAT;
    } else {
        return;
    }
    uint16_t crc = mav_crc(mav_crc(0xFFFF, f + 1, head - 1 + plen), &extra, 1);
    if (crc != ((uint16_t)f[head + plen] | (uint16_t)f[head + plen + 1] << 8)) {
        return;
    }
    if (msgid == MAV_MSG_HEARTBEAT) {
        p->heartbeat = true;
        p->heartbeat_ms = now_ms;
        return;
    }
    uint8_t m[GLOBAL_POSITION_INT_LEN] = {0}; /* MAVLink 2 leaves out trailing zeros */
    memcpy(m, f + head, plen);
    p->lat = get_i32(m + 4);
    p->lon = get_i32(m + 8);
    p->alt_mm = get_i32(m + 16);
    p->heading = (uint16_t)(m[26] | m[27] << 8);
    p->valid = true;
    p->when_ms = now_ms;
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
