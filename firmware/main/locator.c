#include "locator.h"

#include <string.h>

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

void gnss_clear(gnss_fix_t *out)
{
    memset(out, 0, sizeof(*out));
    out->lat = out->lon = out->alt_mm = LOCATOR_UNKNOWN_I32;
    out->speed = out->course = out->hdop = LOCATOR_U16_UNKNOWN;
}

/* ---- AT+CGNSSINFO
 *
 * +CGNSSINFO: <mode>,<satellites in view, one field per system>,<lat>,<N/S>,<lon>,<E/W>,<ddmmyy>,
 *             <hhmmss.s>,<alt m>,<speed knots>,<course>,<PDOP>,<HDOP>,<VDOP>[,...]
 *
 * How many satellite fields come first differs between the modem's firmware versions, and so does the
 * form of latitude and longitude: NMEA's ddmm.mmmm / dddmm.mmmm, or decimal degrees. So the fields are
 * found from the N/S and E/W letters, and the form from the number of digits before the point. Without
 * a fix the fields are empty. Parsed without floating point, so a bad field cannot turn into a NaN. */

#define GNSS_FIELDS 24

/* the first n characters of s are digits */
static bool digits(const char *s, int n)
{
    for (int i = 0; i < n; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
    }
    return true;
}

static int two(const char *s)
{
    return (s[0] - '0') * 10 + (s[1] - '0');
}

/* A decimal number with up to `places` digits after the point, times 10^places; false if the field is
 * empty or not a number. */
static bool fixed(const char *f, int places, int64_t *out)
{
    bool neg = *f == '-', digits = false;
    f += neg || *f == '+';
    int64_t v = 0;
    while (*f >= '0' && *f <= '9') {
        v = v * 10 + (*f++ - '0');
        digits = true;
        if (v > (INT64_MAX / 1000) / 10000000) {
            return false;
        }
    }
    int got = 0;
    if (*f == '.') {
        for (f++; *f >= '0' && *f <= '9'; f++) {
            if (got < places) {
                v = v * 10 + (*f - '0');
                got++;
            }
            digits = true;
        }
    }
    if (!digits || *f) {
        return false;
    }
    for (; got < places; got++) {
        v *= 10;
    }
    *out = neg ? -v : v;
    return true;
}

/* Latitude or longitude in 1e-7 degrees, from ddmm.mmmm (deg_digits 2) / dddmm.mmmm (3), or from
 * decimal degrees. */
static bool coordinate(const char *f, int deg_digits, int32_t *out)
{
    int int_digits = 0;
    for (const char *p = f; *p >= '0' && *p <= '9'; p++) {
        int_digits++;
    }
    int64_t v;
    if (!fixed(f, 7, &v) || v < 0) {
        return false;
    }
    const int64_t limit = (int64_t)(deg_digits == 2 ? 90 : 180) * 10000000;
    /* degrees and minutes: zero-padded (ddmm, dddmm), or any number too large to be degrees */
    if (int_digits >= deg_digits + 2 || (int_digits >= 3 && v > limit)) {
        int64_t deg = v / 1000000000, min = v % 1000000000; /* minutes x 1e7 */
        if (min >= 600000000) {
            return false;
        }
        v = deg * 10000000 + min / 60;
    }
    if (v > limit) {
        return false;
    }
    *out = (int32_t)v;
    return true;
}

bool gnss_parse(const char *answer, gnss_fix_t *out)
{
    gnss_clear(out);
    const char *line = strstr(answer, "+CGNSSINFO:");
    if (!line) {
        return false;
    }
    line += strlen("+CGNSSINFO:");
    while (*line == ' ') {
        line++;
    }
    /* split the line (up to the end of the line) into fields */
    char buf[160];
    size_t n = 0;
    while (line[n] && line[n] != '\r' && line[n] != '\n' && n < sizeof(buf) - 1) {
        buf[n] = line[n];
        n++;
    }
    buf[n] = '\0';
    char *field[GNSS_FIELDS];
    int count = 0;
    char *p = buf;
    field[count++] = p;
    for (; *p && count < GNSS_FIELDS; p++) {
        if (*p == ',') {
            *p = '\0';
            field[count++] = p + 1;
        }
    }
    /* the N/S field, with E/W two further on */
    int ns = -1;
    for (int i = 1; i + 2 < count; i++) {
        if ((strcmp(field[i], "N") == 0 || strcmp(field[i], "S") == 0) &&
            (strcmp(field[i + 2], "E") == 0 || strcmp(field[i + 2], "W") == 0)) {
            ns = i;
            break;
        }
    }
    /* satellites: the fields between the mode and the latitude (all of them without a fix) */
    int last_sv = ns > 0 ? ns - 2 : count - 1;
    unsigned sats = 0;
    for (int i = 1; i <= last_sv && i < count; i++) {
        int64_t v;
        if (fixed(field[i], 0, &v) && v > 0 && v < 100) {
            sats += (unsigned)v;
        }
    }
    out->sats = (uint8_t)(sats < 255 ? sats : 255);
    int32_t lat, lon;
    if (ns < 2 || !coordinate(field[ns - 1], 2, &lat) || !coordinate(field[ns + 1], 3, &lon) ||
        (lat == 0 && lon == 0)) { /* (some firmware reports a "fix" at 0, 0) */
        return true; /* no position */
    }
    out->lat = field[ns][0] == 'S' ? -lat : lat;
    out->lon = field[ns + 2][0] == 'W' ? -lon : lon;
    int64_t v;
    int64_t mode = 0;
    fixed(field[0], 0, &mode);
    out->fix = mode == 2 ? GNSS_FIX_2D : GNSS_FIX_3D; /* a position given is at least a 2D fix */
    if (ns + 4 < count && digits(field[ns + 3], 6) && digits(field[ns + 4], 6)) { /* ddmmyy, hhmmss.s */
        const char *d = field[ns + 3], *t = field[ns + 4];
        out->time = gnss_unix_time(2000 + two(d + 4), two(d + 2), two(d), two(t), two(t + 2), two(t + 4));
    }
    if (ns + 5 < count && fixed(field[ns + 5], 3, &v) && v > INT32_MIN / 2 && v < INT32_MAX / 2) {
        out->alt_mm = (int32_t)v; /* m to mm */
    }
    if (ns + 6 < count && fixed(field[ns + 6], 3, &v) && v >= 0) { /* knots x 1000 */
        int64_t cms = v * 514444 / 10000000; /* 1 knot = 51.4444 cm/s */
        out->speed = (uint16_t)(cms < LOCATOR_U16_UNKNOWN ? cms : LOCATOR_U16_UNKNOWN - 1);
    }
    if (ns + 7 < count && fixed(field[ns + 7], 2, &v) && v >= 0 && v < 36000) {
        out->course = (uint16_t)v;
    }
    if (ns + 9 < count && fixed(field[ns + 9], 2, &v) && v >= 0 && v < LOCATOR_U16_UNKNOWN) {
        out->hdop = (uint16_t)v; /* PDOP, HDOP, VDOP */
    }
    return true;
}

uint32_t gnss_unix_time(int year, int month, int day, int hour, int minute, int second)
{
    if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 60) {
        return 0;
    }
    /* days since 1970-01-01 of a proleptic Gregorian date (H. Hinnant's days_from_civil) */
    int y = year - (month <= 2);
    int era = y / 400;
    int yoe = y - era * 400;
    int doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = (int64_t)era * 146097 + doe - 719468;
    return (uint32_t)(days * 86400 + hour * 3600 + minute * 60 + second);
}

void locator_pack(uint8_t *out, const gnss_fix_t *fix, uint8_t flags, uint16_t fc_silent_s, uint16_t battery_mv,
                  uint8_t battery_pct, int8_t chip_c)
{
    gnss_fix_t none;
    if (!fix) {
        gnss_clear(&none);
        fix = &none;
    }
    put_u32(out, fix->time);
    put_u32(out + 4, (uint32_t)fix->lat);
    put_u32(out + 8, (uint32_t)fix->lon);
    put_u32(out + 12, (uint32_t)fix->alt_mm);
    put_u16(out + 16, fix->speed);
    put_u16(out + 18, fix->course);
    put_u16(out + 20, fix->hdop);
    out[22] = fix->sats;
    out[23] = fix->fix;
    out[24] = flags;
    put_u16(out + 25, fc_silent_s);
    put_u16(out + 27, battery_mv);
    out[29] = battery_pct;
    put_u32(out + 30, 0); /* time: the relay's clock */
    out[34] = (uint8_t)chip_c;
}
