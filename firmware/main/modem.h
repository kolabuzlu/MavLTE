/* A7670E: power, mobile data over PPP on the modem UART, and recovery. */
#pragma once

/* Powers the modem and starts the task that brings mobile data up and keeps it up.
 * The bridge follows the connection through IP_EVENT_PPP_GOT_IP / IP_EVENT_PPP_LOST_IP. */
void modem_start(void);
