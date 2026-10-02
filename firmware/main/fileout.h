/* Logs, the aircraft's side (docs/PROTOCOL.md, "Logs"): answers the relay's FILE_REQ with the list of log
 * files on the card, or with the file asked for, sent from the offset asked for, at most FILE_WINDOW beyond
 * the last ACK, paced as a photo is (snap_rate_t), and from the last ACK again when no new one comes
 * (go-back-N). The same logic as FileOutbox in relay/mavrelay.py.
 *
 * Platform independent: the card comes through the callbacks. fileout_input() only takes note of what the
 * relay wants (it runs inside tun_input, with the tunnel's lock held); fileout_poll() does the work, the
 * card's included, and sends through send(). Not thread safe: call both from one task. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "snapshot.h"

#define TUN_FILE_REQ 15
#define TUN_FILE_LIST 16
#define TUN_FILE_DATA 17
#define TUN_FILE_ACK 18
#define FILE_NAME_LEN 12 /* LOG00012.CSV */
#define FILE_CHUNK 1024
#define FILE_LIST_MOST 40
#define FILE_WINDOW (32 * 1024)
#define FILE_GIVE_UP_MS 30000
#define FILE_REQ_LEN 19
#define FILE_LIST_HEAD_LEN 8
#define FILE_ENTRY_LEN 24
#define FILE_DATA_HEAD_LEN 11
#define FILE_ACK_LEN 6

enum { FILE_OP_LIST = 1, FILE_OP_GET, FILE_OP_STOP };
enum { FILE_OK, FILE_NO_CARD, FILE_NOT_FOUND, FILE_NO_AIRCRAFT, FILE_CARD_ERROR, FILE_STOPPED };

typedef struct {
    char name[FILE_NAME_LEN + 1];
    uint32_t size;       /* bytes */
    uint32_t start, end; /* unix seconds of the first and the last line that have a time; 0: none */
} file_entry_t;

typedef struct {
    /* The log files, newest first, from index `first` on: at most `max` of them into out, and how many there
     * are in all into *total. Returns how many it wrote, or -FILE_NO_CARD. */
    int (*list)(void *ctx, unsigned first, file_entry_t *out, unsigned max, unsigned *total);
    /* Opens the log file `name` for reading, its size into *size: FILE_OK, FILE_NO_CARD or FILE_NOT_FOUND. */
    int (*open)(void *ctx, const char *name, uint32_t *size);
    /* Reads len bytes at offset from the file open; false if that failed. */
    bool (*read)(void *ctx, uint32_t offset, uint8_t *buf, size_t len);
    void (*close)(void *ctx);
    /* Sends a packet on the session with the relay; false without one. */
    bool (*send)(void *ctx, uint8_t type, const uint8_t *body, size_t len);
    void *ctx;
} file_config_t;

typedef struct {
    file_config_t cfg;
    /* asked for by the relay, until fileout_poll() sees to it */
    bool list_asked, get_asked, stop_asked;
    uint16_t list_id, list_first, get_id;
    uint32_t get_offset;
    char get_name[FILE_NAME_LEN + 1];
    /* the download going on */
    bool sending, finished;
    uint16_t id;
    char name[FILE_NAME_LEN + 1];
    uint32_t size, acked, next, from;
    uint32_t last_ack_ms, moved_ms;
    snap_rate_t rate;
    uint32_t rtt_ms_at;
    /* the last download that ended, for the log: its name, bytes sent, and whether all of it went */
    char last_name[FILE_NAME_LEN + 1];
    uint32_t last_bytes;
    bool last_complete;
    uint8_t pkt[FILE_DATA_HEAD_LEN + FILE_CHUNK]; /* a FILE_DATA body; a FILE_LIST one fits too */
    file_entry_t entries[FILE_LIST_MOST];
} file_outbox_t;

void fileout_init(file_outbox_t *o, const file_config_t *cfg);
/* A FILE_REQ or FILE_ACK from the relay (the tunnel's on_packet): noted, for fileout_poll(). */
void fileout_input(file_outbox_t *o, uint8_t type, const uint8_t *body, size_t len, uint32_t now_ms);
/* Answers what was asked for and sends what is due. Call every 10-20 ms while fileout_busy(), never while a
 * photo goes. rtt_ms: the tunnel's round trip (TUN_U16_UNKNOWN if not known); cap: the most bytes/s a log may
 * take on the network the modem is on, as for a photo. Returns true when a download ended (last_*). */
bool fileout_poll(file_outbox_t *o, uint32_t now_ms, uint16_t rtt_ms, float cap);

static inline bool fileout_busy(const file_outbox_t *o)
{
    return o->sending || o->list_asked || o->get_asked || o->stop_asked;
}
