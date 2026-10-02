/* The USB link's lines: MavLTE on a PC asks over the board's USB serial port (the USB-C socket, the port it is
 * flashed through) for the log files on its card, in text commands, and gets them in text lines it checks
 * (README, "Flight log"). Platform independent; usblink.c runs it on the ESP32's UART0.
 *
 *   PC                              board
 *   MAVLTE HELLO                    @MAVLTE <version> <card: ok | none>
 *   MAVLTE LIST [<first>]           @FILE <name> <bytes> <start> <end>  (up to 40 files, newest first, from index
 *                                   <first> on), @END <files on the card>
 *   MAVLTE GET <name> <offset>      @SIZE <name> <bytes>, @D <offset> <base64> <crc32> ..., @DONE <name> <bytes>
 *                                   or @ERR <status> <why>  (status as in FILE_DATA)
 *   MAVLTE SPEED <baud>             @SPEED <baud>, then the board switches; back to 115200 if no command comes at
 *                                   the new rate within 2 s, and after 20 s without a command
 *   MAVLTE STOP                     @ERR 5 stopped, if a GET was going on
 *
 * <start> and <end> are unix seconds (0: no time in the file), <crc32> the CRC-32 (as zlib's) of the line's
 * bytes, in hex. Lines that do not start with "@" are the board's own log: the PC passes them by. The board keeps
 * its log quiet from the first command until 20 s after the last (a COM port has one user at a time anyway). */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define USB_DATA_CHUNK 768 /* bytes in an @D line: 1024 characters of base64 */
#define USB_LINE_MAX 1100
#define USB_IDLE_MS 20000  /* without a command for this long, the board goes back to 115200 baud */
#define USB_SPEED_CHECK_MS 2000 /* after SPEED: the first command at the new rate must come within this */
#define USB_BAUD 115200

/* CRC-32 as zlib has it (Python's zlib.crc32): usb_crc32(0, data, len), or go on from an earlier result. */
uint32_t usb_crc32(uint32_t crc, const uint8_t *data, size_t len);
/* Base64 of len bytes into out (4 * ((len + 2) / 3) characters, then a '\0'); returns the characters. */
size_t usb_base64(char *out, const uint8_t *data, size_t len);
/* "@D <offset> <base64> <crc32>\n" for len bytes (at most USB_DATA_CHUNK) at offset, into out (USB_LINE_MAX
 * bytes); returns its length. */
size_t usb_data_line(char *out, uint32_t offset, const uint8_t *data, size_t len);

typedef enum { USB_CMD_NONE, USB_CMD_HELLO, USB_CMD_LIST, USB_CMD_GET, USB_CMD_SPEED, USB_CMD_STOP } usb_cmd_t;
/* A command line (without its line end): which command, and for GET the name (into name) and the offset, for
 * LIST the first index (0 if not given), for SPEED the baud (into *number). USB_CMD_NONE for anything else. */
usb_cmd_t usb_parse(const char *line, char *name, size_t name_size, uint32_t *number);
