/* Flight controller UART <-> relay tunnel over the cellular data connection. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "locator.h"

#define BRIDGE_RSSI_UNKNOWN 0x7FFF
#define BRIDGE_RAT_UNKNOWN 0xFF

/* Installs the UART driver and starts the bridge tasks. Traffic flows once the PPP interface
 * has an IP address (IP_EVENT_PPP_GOT_IP). */
void bridge_start(void);

/* Signal strength (dBm) and access technology (3GPP <AcT>, 7 = LTE) for the relay's link status. */
void bridge_set_radio(int16_t rssi_dbm, uint8_t rat);

/* The locator: the modem's latest GNSS reading (the bridge reports it to the relay every
 * CONFIG_BRIDGE_LOCATOR_INTERVAL seconds), or NULL when the GNSS cannot be read during the data call. */
void bridge_set_gnss(const gnss_fix_t *fix);

/* Milliseconds since the relay was last heard from, or since mobile data came up if later. */
uint32_t bridge_relay_silence_ms(void);

/* Valid packets received from the relay since boot: compare two readings to tell whether the relay
 * answered in between (bridge_relay_silence_ms() restarts when mobile data comes up). */
uint32_t bridge_relay_packets(void);

typedef struct {
    bool relay; /* session with the relay */
    bool gcs;   /* the relay says a GCS is connected */
} bridge_state_t;

bridge_state_t bridge_state(void);
