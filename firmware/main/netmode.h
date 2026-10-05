/* The mobile network the modem uses (1.8.8): the one chosen at the relay, in MavLTE or on its web page (automatic, 2G
 * only or LTE only), and automatic's fallback to 2G. In the air the LTE signal stays strong while its quality falls with
 * the many cells heard at once, and the link with it, yet the modem stays on such an LTE: on the first flight (1.8.6)
 * the link was gone for up to 3 minutes above 850 m. So automatic moves to 2G when LTE fails, and tries LTE again once
 * the aircraft is back below where LTE worked, or after a while on 2G: 30 s, then 60 s, then 2 minutes at most.
 *
 * Platform independent: modem.c gathers what it needs, asks once a second or so, and sets the modem (AT+CNMP). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* The networks to choose from, as on the wire (tunnel.h TUN_NET_*) */
#define NET_AUTO 0 /* LTE, and 2G while LTE fails */
#define NET_2G 1   /* 2G only */
#define NET_LTE 2  /* LTE only: no 2G at all, not even the modem's own fallback */
#define NET_FALLBACK 0x04 /* in the report to the relay: on 2G because LTE failed */

/* The modem's settings: AT+CNMP <mode>. Each change ends the data call (the A7670E drops its connection to the network
 * and registers again, measured on the bench), so the modem is only set when the network must change. */
#define NET_MODE_AUTO 2
#define NET_MODE_GSM 13
#define NET_MODE_LTE 38

#define NET_SINR_UNKNOWN (-128)
#define NET_RAT_LTE 7               /* 3GPP AcT */
#define NET_POOR_SINR_DB (-9)       /* LTE's quality this low or lower is poor (first flight: the link held down to -9 dB, */
#define NET_FAIL_SILENCE_MS 20000   /* faltered below); and with nothing from the relay for this long, LTE has failed */
#define NET_UP_MS 5000              /* mobile data up this long at least before (the relay answers within a second) */
#define NET_GOOD_SILENCE_MS 3000    /* LTE works: the relay heard within this */
#define NET_FALLBACK_FIRST_MS 30000 /* on 2G this long at most the first time, each next time twice as long, */
#define NET_FALLBACK_MOST_MS 120000 /* up to this (the modem takes about 11 s to get there, not counted) */
#define NET_REACH_2G_MS 30000       /* no 2G this long after LTE failed: LTE again */
#define NET_PROVEN_MS 120000        /* LTE that has worked this long starts the waits afresh */
#define NET_MIN_2G_MS 10000         /* on 2G at least this long before the height brings LTE back */
#define NET_BELOW_M 50              /* LTE again once this far below both where it last worked well and where it failed */

typedef struct {
    uint32_t now_ms;
    bool online;         /* mobile data is up */
    uint32_t online_ms;  /* ... since this long */
    uint32_t silence_ms; /* since the relay last answered, counted across data calls */
    uint8_t rat;         /* the modem's access technology (3GPP AcT), 0xFF unknown */
    int8_t sinr_db;      /* LTE's quality, NET_SINR_UNKNOWN if not known (or not read lately) */
    bool alt_known;
    int32_t alt_m;       /* the flight controller's height above home */
} net_in_t;

typedef struct {
    uint8_t choice;           /* NET_AUTO, NET_2G or NET_LTE */
    bool fallback;            /* automatic, on 2G because LTE failed */
    bool on_2g;               /* ... and the modem got there: on 2G since fallback_ms (until then, LTE failed then) */
    uint32_t fallback_ms;
    uint32_t fallback_for_ms; /* on 2G for this long at most */
    uint8_t failures;         /* LTE failed this many times without working NET_PROVEN_MS in between */
    bool lte_working;         /* automatic, on LTE, the relay answering ... */
    uint32_t lte_working_ms;  /* ... since */
    bool good_known;
    int32_t good_alt_m;       /* the height where LTE last worked well */
    bool retry_known;
    int32_t retry_alt_m;      /* during a fallback: LTE again at or below this height */
    char why[128];            /* why the network last changed, for the logs */
} netmode_t;

void netmode_init(netmode_t *n, uint8_t choice);
/* The network chosen at the relay. A new choice starts afresh (no fallback); true if the modem's setting changes. */
bool netmode_choose(netmode_t *n, uint8_t choice);
/* Once a second or so; true if the modem's setting has changed (netmode_setting(), and why in n->why). */
bool netmode_tick(netmode_t *n, const net_in_t *in);
/* The modem's setting the network needs now (NET_MODE_*). */
int netmode_setting(const netmode_t *n);
/* The vehicle's report to the relay: NET_AUTO, NET_2G or NET_LTE, and NET_FALLBACK. */
uint8_t netmode_report(const netmode_t *n);
/* "automatic", "2G only", "LTE only" */
const char *netmode_name(uint8_t choice);
