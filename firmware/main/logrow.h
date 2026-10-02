/* The flight log's line (README, "Flight log"): what the board knows, once a second, as CSV. Platform
 * independent: sdlog.c gathers the values and writes the lines. Also the modem's AT+CPSI? answer, parsed for
 * the line: the cell the modem is on, and its signal. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "locator.h"
#include "mavpos.h"

#define LOG_LINE_MAX 640
#define LOG_I16_UNKNOWN INT16_MIN
#define LOG_FRESH_MS 5000 /* a flight controller's message older than this is left out of the line */
#define LOG_EVENTS_MOST 200 /* characters of the events column, at most */

typedef struct {
    char mode[12];       /* "LTE", "GSM", "WCDMA", "NO SERVICE"; "" unknown */
    char plmn[8];        /* MCC-MNC, "286-01" */
    char band[12];       /* "B3" (LTE), "GSM900"; "" unknown */
    uint32_t cell;       /* the serving cell's id, 0 unknown */
    int16_t rsrp_dbm;    /* LTE only, LOG_I16_UNKNOWN if not known */
    int16_t rsrq_half_db; /* RSRQ in half dB: -9 dB is -18 */
    int16_t rssi_dbm;
    int16_t sinr_db;
} cell_info_t;

void cell_clear(cell_info_t *c);
/* Reads the modem's answer to AT+CPSI? (all of it, or its +CPSI: line); false if it has none. The A7670's
 * report values (its AT manual): RSRP - 140 dBm, (RSRQ - 40) / 2 dB, RSSI - 110 dBm; 255 is unknown. */
bool cell_parse(const char *answer, cell_info_t *out);

typedef struct {
    uint32_t utc;              /* unix seconds; 0: not known yet (no GNSS time, no relay yet) */
    uint32_t uptime_s;
    /* the LTE module's own GNSS */
    int gnss_state;            /* 0: not read yet, 1: read, -1: it cannot be read */
    gnss_fix_t gnss;
    uint32_t gnss_age_s;       /* since the reading */
    /* the cellular network */
    int16_t signal_dbm;        /* AT+CSQ; LOG_I16_UNKNOWN if not known */
    char operator_name[24];
    cell_info_t cell;
    /* the relay */
    bool relay;                /* a session with it */
    uint16_t rtt_ms;           /* 0xFFFF unknown */
    uint16_t loss_permille;    /* downlink; 0xFFFF unknown */
    uint32_t data_kb;          /* mobile data both ways since power-on, IP and UDP headers included */
    bool gcs;                  /* a GCS takes the telemetry */
    /* the flight controller */
    mav_position_t fc;
    uint32_t now_ms;           /* the clock fc's times (when_ms, ...) count on */
    /* the module */
    int8_t chip_c;             /* INT8_MIN unknown */
    uint16_t rail_mv;          /* the fuel gauge, 0xFFFF unknown */
    uint8_t cell_pct;          /* 0xFF unknown */
    uint8_t voice;             /* 0 off, 1 on, 2 sounding, 3 cannot sound */
    const char *events;        /* what happened since the last line ("relay connected; photo 123 sent"), or "" */
} log_row_t;

/* Sets *row to "nothing known": what the line leaves empty. */
void log_row_clear(log_row_t *row);
/* The CSV header line, with its "\n". */
const char *log_header(void);
/* The line for row, with its "\n", into out (LOG_LINE_MAX bytes are always enough). Returns its length. */
size_t log_format(char *out, size_t n, const log_row_t *row);
/* UTC date and time of unix seconds, "2026-10-02T09:35:12Z", into out (21 bytes). */
void log_time_text(char *out, uint32_t unix_s);

/* A log file's times, from its first and its last bytes (sdlog.c lists them; relay/mavrelay.py's log_times() does the
 * same): the last line's that has one, and the first line's, reckoned back from that one with the uptimes, so that a file
 * whose first lines came before the board knew the time still gets its start. Both take a NUL-terminated buffer and
 * cut it into lines. */
/* The first log line's uptime, from the file's first bytes; false if they hold none (only the header, say). */
bool log_first_uptime(char *head, uint32_t *uptime);
/* The time and the uptime of the last line that has a time, from the file's last bytes (cut: they begin in the middle
 * of a line); false if none has one. */
bool log_last_time(char *tail, bool cut, uint32_t *utc, uint32_t *uptime);
