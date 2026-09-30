/* Flight controller UART <-> relay tunnel over the cellular data connection. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BRIDGE_RSSI_UNKNOWN 0x7FFF
#define BRIDGE_RAT_UNKNOWN 0xFF

/* Installs the UART driver and starts the bridge tasks. Traffic flows once the PPP interface
 * has an IP address (IP_EVENT_PPP_GOT_IP). */
void bridge_start(void);

/* Signal strength (dBm) and access technology (3GPP <AcT>, 7 = LTE) for the relay's link status. */
void bridge_set_radio(int16_t rssi_dbm, uint8_t rat);

/* Milliseconds since the relay was last heard from, or since mobile data came up if later. */
uint32_t bridge_relay_silence_ms(void);

typedef struct {
    bool relay; /* session with the relay */
    bool gcs;   /* the relay says a GCS is connected */
} bridge_state_t;

bridge_state_t bridge_state(void);
