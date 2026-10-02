#include "logrow.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EXTERNAL_POWER_MV 4250 /* as in relay/mavrelay.py: more than a Li-ion cell holds */

static const char HEADER[] =
    "time_utc,uptime_s,"
    "gnss_fix,gnss_sats,gnss_lat,gnss_lon,gnss_alt_m,gnss_speed_ms,gnss_course,gnss_hdop,gnss_age_s,"
    "net,signal_dbm,operator,plmn,band,cell_id,rsrp_dbm,rsrq_db,rssi_dbm,sinr_db,"
    "relay,rtt_ms,loss_pct,data_kb,gcs,"
    "fc_heard_s,fc_mode,fc_armed,fc_gps_fix,fc_gps_sats,fc_lat,fc_lon,fc_alt_m,fc_rel_alt_m,fc_heading,"
    "fc_groundspeed_ms,fc_airspeed_ms,fc_climb_ms,fc_throttle,fc_battery_v,fc_current_a,fc_battery_pct,fc_rssi,"
    "chip_c,power,cell_pct,rail_mv,voice,events\n";

const char *log_header(void)
{
    return HEADER;
}

/* ---- the modem's cell */

void cell_clear(cell_info_t *c)
{
    memset(c, 0, sizeof(*c));
    c->rsrp_dbm = c->rsrq_half_db = c->rssi_dbm = c->sinr_db = LOG_I16_UNKNOWN;
}

static void copy_trimmed(char *out, size_t n, const char *s, size_t len)
{
    while (len && (*s == ' ' || *s == '"')) {
        s++;
        len--;
    }
    while (len && (s[len - 1] == ' ' || s[len - 1] == '"')) {
        len--;
    }
    if (len >= n) {
        len = n - 1;
    }
    memcpy(out, s, len);
    out[len] = '\0';
}

/* A report value: unknown as 255 or not a number; else report + offset (RSRQ: in half dB). */
static int16_t report(const char *field, int offset)
{
    char *end;
    long v = strtol(field, &end, 10);
    if (end == field || v == 255 || v < -1000 || v > 1000) {
        return LOG_I16_UNKNOWN;
    }
    return (int16_t)(v + offset);
}

static const char *gsm_band(unsigned long arfcn)
{
    return arfcn <= 124 || (arfcn >= 975 && arfcn <= 1023) ? "GSM900"
           : arfcn >= 512 && arfcn <= 885                 ? "DCS1800"
           : arfcn >= 128 && arfcn <= 251                 ? "GSM850"
                                                          : "";
}

bool cell_parse(const char *answer, cell_info_t *out)
{
    cell_clear(out);
    const char *line = strstr(answer, "+CPSI:");
    if (line == NULL) {
        return false;
    }
    line += 6;
    const char *field[16];
    size_t len[16];
    int nf = 0;
    for (const char *p = line;;) {
        const char *end = p + strcspn(p, ",\r\n");
        if (nf < 16) {
            field[nf] = p;
            len[nf++] = (size_t)(end - p);
        }
        if (*end != ',') {
            break;
        }
        p = end + 1;
    }
    char text[24];
    copy_trimmed(out->mode, sizeof(out->mode), field[0], len[0]);
    if (nf >= 5) {
        copy_trimmed(out->plmn, sizeof(out->plmn), field[2], len[2]);
        copy_trimmed(text, sizeof(text), field[4], len[4]);
        out->cell = (uint32_t)strtoul(text, NULL, 0);
    }
    if (strcmp(out->mode, "LTE") == 0 && nf >= 14) {
        copy_trimmed(text, sizeof(text), field[6], len[6]); /* "EUTRAN-BAND3": "B3" */
        const char *b = strstr(text, "BAND");
        if (b) {
            out->band[0] = 'B';
            copy_trimmed(out->band + 1, sizeof(out->band) - 1, b + 4, strlen(b + 4));
        } else {
            copy_trimmed(out->band, sizeof(out->band), text, strlen(text));
        }
        copy_trimmed(text, sizeof(text), field[10], len[10]);
        out->rsrq_half_db = report(text, -40);
        copy_trimmed(text, sizeof(text), field[11], len[11]);
        out->rsrp_dbm = report(text, -140);
        copy_trimmed(text, sizeof(text), field[12], len[12]);
        out->rssi_dbm = report(text, -110);
        copy_trimmed(text, sizeof(text), field[13], len[13]);
        out->sinr_db = report(text, 0);
    } else if (strcmp(out->mode, "GSM") == 0 && nf >= 6) {
        copy_trimmed(text, sizeof(text), field[5], len[5]);
        const char *band = gsm_band(strtoul(text, NULL, 10));
        copy_trimmed(out->band, sizeof(out->band), band, strlen(band));
    }
    return true;
}

/* ---- time */

void log_time_text(char *out, uint32_t unix_s)
{
    /* civil_from_days (H. Hinnant) */
    int64_t z = unix_s / 86400 + 719468;
    uint32_t secs = unix_s % 86400;
    int64_t era = z / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    unsigned day = doy - (153 * mp + 2) / 5 + 1;
    unsigned month = mp < 10 ? mp + 3 : mp - 9;
    unsigned year = (unsigned)(yoe + era * 400 + (month <= 2));
    snprintf(out, 21, "%04u-%02u-%02uT%02u:%02u:%02uZ", year % 10000, month % 100, day % 100,
             (unsigned)(secs / 3600 % 100), (unsigned)(secs / 60 % 60), (unsigned)(secs % 60));
}

/* ---- a log file's times */

/* A log line's time (0: none) and uptime; false for what is not one (the header). */
static bool parse_line(const char *line, uint32_t *utc, uint32_t *uptime)
{
    const char *comma = strchr(line, ',');
    if (!comma || comma[1] < '0' || comma[1] > '9') {
        return false;
    }
    int y, mo, d, h, mi, s;
    char z = 0;
    *utc = comma > line && sscanf(line, "%4d-%2d-%2dT%2d:%2d:%2d%c", &y, &mo, &d, &h, &mi, &s, &z) == 7 && z == 'Z'
               ? gnss_unix_time(y, mo, d, h, mi, s)
               : 0;
    *uptime = (uint32_t)strtoul(comma + 1, NULL, 10);
    return true;
}

/* The next whole line of a buffer, cut out at *p; NULL when no whole line is left. */
static char *next_line(char **p)
{
    char *line = *p, *end = strchr(line, '\n');
    if (!end) {
        return NULL;
    }
    *end = '\0';
    *p = end + 1;
    return line;
}

bool log_first_uptime(char *head, uint32_t *uptime)
{
    uint32_t utc;
    for (char *p = head, *line; (line = next_line(&p));) {
        if (parse_line(line, &utc, uptime)) {
            return true;
        }
    }
    return false;
}

bool log_last_time(char *tail, bool cut, uint32_t *utc, uint32_t *uptime)
{
    bool found = false;
    uint32_t t, up;
    char *p = tail, *line;
    if (cut) {
        next_line(&p);
    }
    while ((line = next_line(&p))) {
        if (parse_line(line, &t, &up) && t) {
            *utc = t;
            *uptime = up;
            found = true;
        }
    }
    return found;
}

/* ---- the line */

void log_row_clear(log_row_t *r)
{
    memset(r, 0, sizeof(*r));
    gnss_clear(&r->gnss);
    r->signal_dbm = LOG_I16_UNKNOWN;
    cell_clear(&r->cell);
    r->rtt_ms = r->loss_permille = 0xFFFF;
    mav_position_init(&r->fc);
    r->chip_c = INT8_MIN;
    r->rail_mv = 0xFFFF;
    r->cell_pct = 0xFF;
    r->events = "";
}

typedef struct {
    char *p;
    size_t left;
} out_t;

static void add(out_t *o, const char *s)
{
    size_t n = strlen(s);
    if (n >= o->left) {
        n = o->left ? o->left - 1 : 0;
    }
    memcpy(o->p, s, n);
    o->p += n;
    o->left -= n;
    if (o->left) {
        *o->p = '\0';
    }
}

static void comma(out_t *o)
{
    add(o, ",");
}

static void add_int(out_t *o, long long v)
{
    char s[24];
    snprintf(s, sizeof(s), "%lld", v);
    add(o, s);
}

/* v / 10^decimals, with exactly that many decimals: (-5, 1) is "-0.5" */
static void add_fixed(out_t *o, long long v, int decimals)
{
    long long scale = 1;
    for (int i = 0; i < decimals; i++) {
        scale *= 10;
    }
    long long whole = (v < 0 ? -v : v) / scale, frac = (v < 0 ? -v : v) % scale;
    char s[48];
    snprintf(s, sizeof(s), "%s%lld.%0*lld", v < 0 ? "-" : "", whole, decimals, frac);
    add(o, s);
}

/* A float as tenths, or nothing if it is not a sensible number. */
static void add_tenths(out_t *o, float v)
{
    if (v == v && v > -1e6f && v < 1e6f) {
        add_fixed(o, (long long)(v * 10.0f + (v >= 0 ? 0.5f : -0.5f)), 1);
    }
}

/* Text without commas or line breaks, which would break the CSV. */
static void add_clean(out_t *o, const char *s, size_t most)
{
    char buf[LOG_EVENTS_MOST + 1];
    size_t i = 0;
    for (; s[i] && i < most && i < LOG_EVENTS_MOST; i++) {
        buf[i] = s[i] == ',' ? ';' : (s[i] == '\n' || s[i] == '\r') ? ' ' : s[i];
    }
    buf[i] = '\0';
    add(o, buf);
}

static bool fresh(bool have, uint32_t when_ms, uint32_t now_ms)
{
    return have && (uint32_t)(now_ms - when_ms) <= LOG_FRESH_MS;
}

size_t log_format(char *out, size_t n, const log_row_t *r)
{
    out_t o = {out, n};
    if (n) {
        out[0] = '\0';
    }
    char text[24];
    if (r->utc) {
        log_time_text(text, r->utc);
        add(&o, text);
    }
    comma(&o);
    add_int(&o, r->uptime_s);

    /* the module's GNSS */
    const gnss_fix_t *g = &r->gnss;
    bool read = r->gnss_state > 0, fix = read && g->fix >= GNSS_FIX_2D && g->lat != LOCATOR_UNKNOWN_I32;
    comma(&o);
    if (read) {
        add_int(&o, g->fix);
    }
    comma(&o);
    if (read) {
        add_int(&o, g->sats);
    }
    comma(&o);
    if (fix) {
        add_fixed(&o, g->lat, 7);
    }
    comma(&o);
    if (fix) {
        add_fixed(&o, g->lon, 7);
    }
    comma(&o);
    if (fix && g->alt_mm != LOCATOR_UNKNOWN_I32) {
        add_fixed(&o, g->alt_mm / 100, 1);
    }
    comma(&o);
    if (fix && g->speed != LOCATOR_U16_UNKNOWN) {
        add_fixed(&o, g->speed / 10, 1);
    }
    comma(&o);
    if (fix && g->course != LOCATOR_U16_UNKNOWN) {
        add_fixed(&o, g->course / 10, 1);
    }
    comma(&o);
    if (read && g->hdop != LOCATOR_U16_UNKNOWN) {
        add_fixed(&o, g->hdop, 2);
    }
    comma(&o);
    if (read) {
        add_int(&o, r->gnss_age_s);
    }

    /* the cellular network */
    const cell_info_t *c = &r->cell;
    comma(&o);
    add(&o, c->mode);
    comma(&o);
    if (r->signal_dbm != LOG_I16_UNKNOWN) {
        add_int(&o, r->signal_dbm);
    }
    comma(&o);
    add_clean(&o, r->operator_name, sizeof(r->operator_name));
    comma(&o);
    add(&o, c->plmn);
    comma(&o);
    add(&o, c->band);
    comma(&o);
    if (c->cell) {
        add_int(&o, c->cell);
    }
    comma(&o);
    if (c->rsrp_dbm != LOG_I16_UNKNOWN) {
        add_int(&o, c->rsrp_dbm);
    }
    comma(&o);
    if (c->rsrq_half_db != LOG_I16_UNKNOWN) {
        add_fixed(&o, c->rsrq_half_db * 5, 1);
    }
    comma(&o);
    if (c->rssi_dbm != LOG_I16_UNKNOWN) {
        add_int(&o, c->rssi_dbm);
    }
    comma(&o);
    if (c->sinr_db != LOG_I16_UNKNOWN) {
        add_int(&o, c->sinr_db);
    }

    /* the relay */
    comma(&o);
    add_int(&o, r->relay);
    comma(&o);
    if (r->relay && r->rtt_ms != 0xFFFF) {
        add_int(&o, r->rtt_ms);
    }
    comma(&o);
    if (r->relay && r->loss_permille != 0xFFFF) {
        add_fixed(&o, r->loss_permille, 1);
    }
    comma(&o);
    add_int(&o, r->data_kb);
    comma(&o);
    add_int(&o, r->relay && r->gcs);

    /* the flight controller */
    const mav_position_t *f = &r->fc;
    bool beat = fresh(f->heartbeat, f->heartbeat_ms, r->now_ms);
    comma(&o);
    if (f->heartbeat) {
        add_int(&o, (uint32_t)(r->now_ms - f->heartbeat_ms) / 1000);
    }
    comma(&o);
    if (beat) {
        const char *mode = mav_is_ardupilot_plane(f) ? mav_plane_mode(f->custom_mode) : NULL;
        if (mode) {
            add(&o, mode);
        } else {
            add_int(&o, f->custom_mode);
        }
    }
    comma(&o);
    if (beat) {
        add_int(&o, (f->base_mode & MAV_ARMED) != 0);
    }
    bool gps = fresh(f->gps, f->gps_ms, r->now_ms);
    comma(&o);
    if (gps) {
        add_int(&o, f->gps_fix);
    }
    comma(&o);
    if (gps && f->gps_sats != 255) {
        add_int(&o, f->gps_sats);
    }
    bool pos = fresh(f->valid, f->when_ms, r->now_ms);
    comma(&o);
    if (pos) {
        add_fixed(&o, f->lat, 7);
    }
    comma(&o);
    if (pos) {
        add_fixed(&o, f->lon, 7);
    }
    comma(&o);
    if (pos) {
        add_fixed(&o, f->alt_msl_mm / 100, 1);
    }
    comma(&o);
    if (pos) {
        add_fixed(&o, f->alt_mm / 100, 1);
    }
    comma(&o);
    if (pos && f->heading != 0xFFFF) {
        add_fixed(&o, f->heading / 10, 1);
    }
    bool hud = fresh(f->hud, f->hud_ms, r->now_ms);
    comma(&o);
    if (hud) {
        add_tenths(&o, f->groundspeed);
    }
    comma(&o);
    if (hud) {
        add_tenths(&o, f->airspeed);
    }
    comma(&o);
    if (hud) {
        add_tenths(&o, f->climb);
    }
    comma(&o);
    if (hud) {
        add_int(&o, f->throttle);
    }
    bool sys = fresh(f->sys, f->sys_ms, r->now_ms);
    comma(&o);
    if (sys && f->battery_mv != 0xFFFF) {
        add_fixed(&o, f->battery_mv, 3);
    }
    comma(&o);
    if (sys && f->battery_ca >= 0) {
        add_fixed(&o, f->battery_ca / 10, 1);
    }
    comma(&o);
    if (sys && f->battery_pct >= 0) {
        add_int(&o, f->battery_pct);
    }
    comma(&o);
    if (fresh(f->rc, f->rc_ms, r->now_ms) && f->rc_rssi != 255) {
        add_int(&o, f->rc_rssi);
    }

    /* the module */
    comma(&o);
    if (r->chip_c != INT8_MIN) {
        add_int(&o, r->chip_c);
    }
    bool rail = r->rail_mv != 0xFFFF, external = rail && r->rail_mv >= EXTERNAL_POWER_MV;
    comma(&o);
    if (rail) {
        add(&o, external ? "ext" : "cell");
    }
    comma(&o);
    if (rail && !external && r->cell_pct != 0xFF) {
        add_int(&o, r->cell_pct);
    }
    comma(&o);
    if (rail) {
        add_int(&o, r->rail_mv);
    }
    comma(&o);
    static const char *const voice[] = {"off", "on", "sounding", "cannot"};
    add(&o, voice[r->voice < 4 ? r->voice : 0]);
    comma(&o);
    add_clean(&o, r->events ? r->events : "", LOG_EVENTS_MOST);
    add(&o, "\n");
    return (size_t)(o.p - out);
}
