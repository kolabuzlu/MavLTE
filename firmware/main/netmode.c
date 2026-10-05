#include "netmode.h"

#include <stdio.h>
#include <string.h>

const char *netmode_name(uint8_t choice)
{
    return choice == NET_2G ? "2G only" : choice == NET_LTE ? "LTE only" : "automatic";
}

void netmode_init(netmode_t *n, uint8_t choice)
{
    memset(n, 0, sizeof(*n));
    n->choice = choice <= NET_LTE ? choice : NET_AUTO;
}

int netmode_setting(const netmode_t *n)
{
    if (n->choice == NET_2G || (n->choice == NET_AUTO && n->fallback)) {
        return NET_MODE_GSM;
    }
    return n->choice == NET_LTE ? NET_MODE_LTE : NET_MODE_AUTO;
}

uint8_t netmode_report(const netmode_t *n)
{
    return (uint8_t)(n->choice | (n->fallback ? NET_FALLBACK : 0));
}

bool netmode_choose(netmode_t *n, uint8_t choice)
{
    choice = choice <= NET_LTE ? choice : NET_AUTO;
    if (choice == n->choice) {
        return false;
    }
    int before = netmode_setting(n);
    n->choice = choice;
    n->fallback = n->on_2g = false;
    n->failures = 0;
    n->lte_working = false;
    n->retry_known = false;
    snprintf(n->why, sizeof(n->why), "%s, as chosen", netmode_name(choice));
    return netmode_setting(n) != before;
}

/* How long the fallback after the given number of failures lasts: 30 s, 60 s, then 2 minutes */
static uint32_t fallback_for(uint8_t failures)
{
    uint32_t ms = NET_FALLBACK_FIRST_MS;
    for (uint8_t i = 1; i < failures && ms < NET_FALLBACK_MOST_MS; i++) {
        ms *= 2;
    }
    return ms < NET_FALLBACK_MOST_MS ? ms : NET_FALLBACK_MOST_MS;
}

bool netmode_tick(netmode_t *n, const net_in_t *in)
{
    if (n->choice != NET_AUTO) {
        return false;
    }
    bool lte = in->rat == NET_RAT_LTE;
    bool known = in->sinr_db != NET_SINR_UNKNOWN;
    if (n->fallback) { /* LTE again after a while on 2G, or as soon as the aircraft is lower down */
        if (!n->on_2g && (in->rat == 0 || in->rat == 1 || in->rat == 3)) { /* GSM, or with EDGE */
            n->on_2g = true; /* the wait starts now that the modem is there */
            n->fallback_ms = in->now_ms;
        }
        uint32_t since = in->now_ms - n->fallback_ms;
        if (!n->on_2g) {
            if (since < NET_REACH_2G_MS) {
                return false;
            }
            snprintf(n->why, sizeof(n->why), "LTE again: no 2G within %u s", (unsigned)(since / 1000));
        } else if (since >= n->fallback_for_ms) {
            snprintf(n->why, sizeof(n->why), "LTE again after %u s on 2G", (unsigned)(since / 1000));
        } else if (n->retry_known && in->alt_known && in->alt_m <= n->retry_alt_m && since >= NET_MIN_2G_MS) {
            snprintf(n->why, sizeof(n->why), "LTE again, the aircraft is down to %d m", (int)in->alt_m);
        } else {
            return false;
        }
        n->fallback = n->on_2g = false;
        n->lte_working = false;
        return true;
    }

    /* LTE that works, and where it works well (the relay answering, the quality better than poor) */
    if (in->online && lte && in->silence_ms < NET_GOOD_SILENCE_MS) {
        if (!n->lte_working) {
            n->lte_working = true;
            n->lte_working_ms = in->now_ms;
        }
        if (known && in->sinr_db > NET_POOR_SINR_DB && in->alt_known) {
            n->good_known = true;
            n->good_alt_m = in->alt_m;
        }
        if (n->failures && in->now_ms - n->lte_working_ms >= NET_PROVEN_MS) {
            n->failures = 0; /* the waits start afresh at 30 s */
        }
    } else {
        n->lte_working = false;
    }

    /* LTE that fails: nothing from the relay for a while, at a poor quality. (With a good one, the relay or the
     * internet may be at fault, which 2G would not mend: modem.c's redials see to it.) */
    if (!in->online || in->online_ms < NET_UP_MS || !lte || !known || in->sinr_db > NET_POOR_SINR_DB ||
        in->silence_ms < NET_FAIL_SILENCE_MS) {
        return false;
    }
    if (n->failures < 255) {
        n->failures++;
    }
    n->fallback = true;
    n->on_2g = false;
    n->fallback_ms = in->now_ms;
    n->fallback_for_ms = fallback_for(n->failures);
    n->lte_working = false;
    /* back to LTE at once below where it last worked well, and below where it failed */
    n->retry_known = n->good_known || in->alt_known;
    if (n->retry_known) {
        int32_t below = n->good_known ? n->good_alt_m : in->alt_m;
        if (in->alt_known && in->alt_m < below) {
            below = in->alt_m;
        }
        n->retry_alt_m = below - NET_BELOW_M;
    }
    int len = snprintf(n->why, sizeof(n->why), "2G: LTE failed (nothing from the relay for %u s at quality %d dB); "
                       "LTE again in %u s", (unsigned)(in->silence_ms / 1000), (int)in->sinr_db,
                       (unsigned)(n->fallback_for_ms / 1000));
    if (n->retry_known && len > 0 && (size_t)len < sizeof(n->why)) {
        snprintf(n->why + len, sizeof(n->why) - (size_t)len, " or below %d m", (int)n->retry_alt_m);
    }
    return true;
}
