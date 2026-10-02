/* A7670E: power, mobile data over PPP on the modem UART, and recovery. */
#pragma once

#include "logrow.h"

/* Powers the modem and starts the task that brings mobile data up and keeps it up.
 * The bridge follows the connection through IP_EVENT_PPP_GOT_IP / IP_EVENT_PPP_LOST_IP. */
void modem_start(void);

/* The network as the modem last read it, for the flight log's line: the signal, the operator and the cell (AT+CPSI?),
 * read every few seconds wherever the modem takes AT commands; nothing if the last reading is too old. */
void modem_log_row(log_row_t *row);
