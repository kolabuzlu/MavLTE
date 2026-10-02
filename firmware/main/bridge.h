/* Flight controller UART <-> relay tunnel over the cellular data connection. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "locator.h"
#include "logrow.h"

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

/* The locator voice: whether the relay last asked for it. That holds while the relay is out of reach,
 * so an aircraft whose voice is on keeps speaking where it has no coverage. */
bool bridge_voice_wanted(void);

/* What the modem makes of it, for the relay (in every PING): speaking, or asked to but it does not. */
void bridge_set_voice(bool speaking, bool failed);

/* Milliseconds since the relay was last heard from, or since mobile data came up if later. */
uint32_t bridge_relay_silence_ms(void);

/* Valid packets received from the relay since boot: compare two readings to tell whether the relay
 * answered in between (bridge_relay_silence_ms() restarts when mobile data comes up). */
uint32_t bridge_relay_packets(void);

typedef struct {
    bool relay; /* session with the relay */
    bool gcs;   /* the relay says a GCS is connected */
    bool fc;    /* the flight controller's HEARTBEAT came within LOCATOR_FC_SILENT_S */
} bridge_state_t;

bridge_state_t bridge_state(void);

/* What the bridge knows, for the flight log's line: the time (once the relay or the GNSS has given it), the module's
 * GNSS, the relay, the mobile data used, the flight controller, the board's power and the locator voice. */
void bridge_log_row(log_row_t *row);
