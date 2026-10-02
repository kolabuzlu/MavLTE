/* The flight log (README, "Flight log"): a CSV line a second (logrow.h) on the SD card in the board's TF slot, in
 * MAVLTE/LOGnnnnn.CSV, a new file at each power-on and after every 16 MB. V2 boards only: their slot is wired for
 * SD mode, 1 bit (CLK GPIO5, CMD GPIO4, D0 GPIO6). The card must be FAT32 (or FAT16): the firmware's FatFs has no
 * exFAT, which cards over 32 GB come with, so those need formatting as FAT32 once. It never formats a card itself.
 * A card that is missing, or fails, is looked for again every 30 s. MavLTE downloads the files over the USB cable
 * (usblink.c) and over 4G (fileout.h), through the functions below. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ff.h"

#include "fileout.h"

/* Starts the logger's task, which mounts the card. */
void sdlog_start(void);

/* Something that happened, for the events column of the next line ("relay connected"), as printf. From any task;
 * what no longer fits in the line is left out. */
void sdlog_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Whether a card is mounted and works. */
bool sdlog_card(void);

/* The log files, newest first, as file_config_t's list(): at most max of them from index first on into out, how
 * many there are into *total. Returns how many it wrote, or -FILE_NO_CARD. */
int sdlog_list(unsigned first, file_entry_t *out, unsigned max, unsigned *total);

typedef struct {
    FIL fil;
    bool open;
} sdlog_file_t;

/* Opens a log file ("LOG00012.CSV", any case) for reading: FILE_OK and its size (as of the logger's last sync, a
 * few seconds ago for the file being written), or FILE_NO_CARD or FILE_NOT_FOUND. */
int sdlog_open(sdlog_file_t *file, const char *name, uint32_t *size);
/* Reads exactly len bytes at offset; false if that failed. */
bool sdlog_read(sdlog_file_t *file, uint32_t offset, uint8_t *buf, size_t len);
void sdlog_close(sdlog_file_t *file);
