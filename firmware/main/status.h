/* The board's RGB LED (WS2812B on GPIO38, beside the Waveshare logo) shows how far the link has come, from worst
 * to best (README, "The RGB LED"):
 *   red, blinking   no mobile data yet: the modem starts up, or searches for the network
 *   red             modem, SIM or network problem (the log says which)
 *   yellow          mobile data up, the relay does not answer (yet)
 *   purple          connected to the relay, but no flight controller talks (no HEARTBEAT for 10 s)
 *   blue            ready to fly: the relay and the flight controller, a GCS connected or not
 * At power-on it shows each colour once, a second each (a lamp test). */
#pragma once

typedef enum {
    STATUS_MODEM_STARTING,
    STATUS_MODEM_ERROR,
    STATUS_MODEM_ONLINE,
} status_modem_t;

void status_start(void);
void status_set_modem(status_modem_t state);
