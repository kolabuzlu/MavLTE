/* The locator: where the aircraft is, from the modem's own GNSS, reported to the relay every few seconds
 * whatever the flight controller does (docs/PROTOCOL.md, "Locator"). This part is platform independent:
 * it reads the modem's answer to AT+CGNSSINFO and builds the POSITION packet. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TUN_POSITION 13
#define LOCATOR_BODY_LEN 35
#define LOCATOR_UNKNOWN_I32 INT32_MIN
#define LOCATOR_U16_UNKNOWN 0xFFFF
#define LOCATOR_BATTERY_UNKNOWN 0xFF
#define LOCATOR_TEMP_UNKNOWN INT8_MIN
#define LOCATOR_FC_SILENT 0x01 /* nothing from the flight controller for LOCATOR_FC_SILENT_S or more */
#define LOCATOR_NO_GNSS 0x02   /* the GNSS cannot be read */
#define LOCATOR_FC_SILENT_S 10

enum { GNSS_FIX_NONE = 0, GNSS_FIX_2D = 2, GNSS_FIX_3D = 3 };

typedef struct {
    uint32_t time;     /* unix seconds (UTC), 0 if unknown */
    int32_t lat, lon;  /* 1e-7 degrees */
    int32_t alt_mm;    /* above mean sea level */
    uint16_t speed;    /* cm/s */
    uint16_t course;   /* centidegrees */
    uint16_t hdop;     /* x100 */
    uint8_t sats;      /* used in the fix */
    uint8_t fix;       /* GNSS_FIX_* */
} gnss_fix_t;

/* Sets *out to "no fix, nothing known". */
void gnss_clear(gnss_fix_t *out);

/* Reads the modem's answer to AT+CGNSSINFO (the whole answer, or just its +CGNSSINFO: line). Returns
 * false if there is no +CGNSSINFO: line in it; a line without a position gives a fix of GNSS_FIX_NONE. */
bool gnss_parse(const char *answer, gnss_fix_t *out);

/* The charge of the board's 18650 cell (0-100 %) from its voltage (mV): 4.10 V and above full, 3.40 V empty (the
 * user's endpoints: 4.10 V is the most the gauge reads on the cell after a full charge, the cell then running the board;
 * 3.40 V is the A7670E's lowest supply: below it the modem stops), along a Li-ion cell's discharge curve in between,
 * whose voltage stays flat through the middle and falls fast near empty. relay/mavrelay.py has the same CELL_CURVE. */
uint8_t locator_cell_percent(uint16_t mv);

/* Days-from-civil: unix seconds of a UTC date and time, 0 for dates before 2020. */
uint32_t gnss_unix_time(int year, int month, int day, int hour, int minute, int second);

/* The POSITION body (LOCATOR_BODY_LEN bytes): the fix, or none (NULL, with LOCATOR_NO_GNSS in flags),
 * and the rest of what the relay and MavLTE show. fc_silent_s: seconds since the flight controller was
 * last heard, LOCATOR_U16_UNKNOWN if never; chip_c: the ESP32-S3's temperature in degrees C. */
void locator_pack(uint8_t *out, const gnss_fix_t *fix, uint8_t flags, uint16_t fc_silent_s, uint16_t battery_mv,
                  uint8_t battery_pct, int8_t chip_c);
