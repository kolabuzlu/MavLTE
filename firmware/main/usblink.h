/* The USB link (usbproto.h): MavLTE on a PC lists and downloads the flight log's files over the board's USB serial
 * port, the CH343 behind the USB-C socket on the ESP32's UART0, which also carries the board's own log. */
#pragma once

/* Takes UART0 over from the console (its log goes on through the UART driver) and starts the link's task. */
void usblink_start(void);
