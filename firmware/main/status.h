/* The board's RGB LED (WS2812B on GPIO38) shows how far the link has come:
 *   red, blinking      modem, SIM or network problem (the log says which)
 *   yellow, blinking   modem starting up or searching for the network
 *   blue               mobile data up, relay not answering (yet)
 *   green, flashing    connected to the relay, no GCS connected
 *   green              connected to the relay and a GCS is connected */
#pragma once

typedef enum {
    STATUS_MODEM_STARTING,
    STATUS_MODEM_ERROR,
    STATUS_MODEM_ONLINE,
} status_modem_t;

void status_start(void);
void status_set_modem(status_modem_t state);
