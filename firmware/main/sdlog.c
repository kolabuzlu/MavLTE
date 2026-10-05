#include "sdlog.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "diskio_sdmmc.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

#include "board.h"
#include "bridge.h"
#include "locator.h"
#include "logrow.h"
#include "modem.h"
#include "version.h"

#define MOUNT_POINT "/sd"
#define FOLDER "MAVLTE"
#define NAME_FORMAT "LOG%05" PRIu32 ".CSV"
#define NUMBER_MOST 99999
#define SPLIT_BYTES (16u << 20) /* a new file after this much */
#define SYNC_EVERY 5            /* lines: at most this many seconds of the log are lost when the power goes */
#define MOUNT_EVERY_MS 30000
#define LIST_MOST 4096 /* files a list reaches, newest first: older ones still download by name */
#define TIMES_CACHED 48
#define SCAN_BYTES 2048 /* read at a file's start and at its end, for its times: a line is at most LOG_LINE_MAX */

static const char *TAG = "sdlog";

/* the card: card_lock guards these, and is held for all work on the card */
static SemaphoreHandle_t card_lock;
static sdmmc_card_t *card; /* NULL: none mounted */
static char folder[16];    /* the log folder, as FatFs names it: "0:/MAVLTE" */
static bool card_failed;   /* it failed: unmounted once the readers have closed their files */
static bool card_full;     /* no room for the log: it stops, and the files still download */
static unsigned readers;   /* files open in sdlog_open() */
static FILINFO info;
static char scan[SCAN_BYTES + 1];

/* the times of files listed lately: finding them takes two reads of the file */
typedef struct {
    uint32_t number, size, start, end;
} times_t;
static times_t cached[TIMES_CACHED];
static unsigned cached_next;

/* the file being written (the logger's task only, and card_lock while it works on it) */
static FIL log_file;
static bool log_open;
static uint32_t log_number, log_bytes;
static unsigned unsynced;

/* what happened since the last line */
static SemaphoreHandle_t events_lock;
static char events[LOG_EVENTS_MOST + 1];
static size_t events_len;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void sdlog_event(const char *fmt, ...)
{
    if (!events_lock) {
        return;
    }
    char text[96];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    size_t n = strlen(text);
    xSemaphoreTake(events_lock, portMAX_DELAY);
    size_t sep = events_len ? 2 : 0;
    if (events_len + sep + n <= LOG_EVENTS_MOST) {
        memcpy(events + events_len, "; ", sep);
        memcpy(events + events_len + sep, text, n + 1);
        events_len += sep + n;
    }
    xSemaphoreGive(events_lock);
}

static void take_events(char *out)
{
    xSemaphoreTake(events_lock, portMAX_DELAY);
    memcpy(out, events, events_len + 1);
    events_len = 0;
    events[0] = '\0';
    xSemaphoreGive(events_lock);
}

/* ---- the card */

/* The number in a log file's name ("LOG00012.CSV", any case); false for any other name. */
static bool name_number(const char *name, uint32_t *number)
{
    if (strlen(name) != FILE_NAME_LEN || strncasecmp(name, "LOG", 3) != 0 || strcasecmp(name + 8, ".CSV") != 0) {
        return false;
    }
    uint32_t n = 0;
    for (int i = 3; i < 8; i++) {
        if (name[i] < '0' || name[i] > '9') {
            return false;
        }
        n = n * 10 + (uint32_t)(name[i] - '0');
    }
    *number = n;
    return true;
}

static void file_path(char *out, size_t n, uint32_t number)
{
    snprintf(out, n, "%s/" NAME_FORMAT, folder, number);
}

/* A FatFs result that says the card itself fails (called with card_lock held). */
static void trouble(FRESULT fr, const char *doing)
{
    if (fr == FR_DISK_ERR || fr == FR_NOT_READY || fr == FR_INT_ERR || fr == FR_NO_FILESYSTEM) {
        if (!card_failed) {
            ESP_LOGW(TAG, "the SD card failed %s (FatFs error %d): looking for it again in %d s", doing, (int)fr,
                     MOUNT_EVERY_MS / 1000);
            sdlog_event("SD card failed");
        }
        card_failed = true;
    }
}

/* The same for a reader (a download, the list): a failed read ends that reading only, and the logger's own
 * writes decide whether the card has failed (a glitch while downloading must not cut the flight's log). */
static void reader_trouble(FRESULT fr, const char *doing)
{
    static uint32_t said_ms;
    if ((fr == FR_DISK_ERR || fr == FR_NOT_READY || fr == FR_INT_ERR || fr == FR_NO_FILESYSTEM) &&
        (uint32_t)(now_ms() - said_ms) > 10000) {
        said_ms = now_ms();
        ESP_LOGW(TAG, "the SD card failed %s (FatFs error %d)", doing, (int)fr);
    }
}

/* Closes a file, also one on a failing card: f_close() keeps a file it cannot write open, with its buffer. */
static void close_file(FIL *f)
{
    if (f_close(f) != FR_OK && f->obj.fs) {
#if FF_USE_DYN_BUFFER && !FF_FS_TINY
        ff_memfree(f->buf);
        f->buf = NULL;
#endif
        f->obj.fs = NULL;
    }
}

static bool mount(void)
{
    static esp_err_t last_err = ESP_OK;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = GPIO_NUM_5;
    slot.cmd = GPIO_NUM_4;
    slot.d0 = GPIO_NUM_6;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    const esp_vfs_fat_sdmmc_mount_config_t config = {.format_if_mount_failed = false, .max_files = 2};
    sdmmc_card_t *c = NULL;
    esp_err_t err = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &config, &c);
    if (err != ESP_OK) {
        if (err != last_err) { /* once, not every 30 s */
            if (err == ESP_FAIL) {
                ESP_LOGW(TAG, "the SD card has no FAT32 file system (cards over 32 GB come as exFAT, which the board "
                              "cannot read): format it as FAT32. No flight log until then.");
            } else {
                ESP_LOGI(TAG, "no SD card in the TF slot, or it does not answer (%s): no flight log",
                         esp_err_to_name(err));
            }
        }
        last_err = err;
        return false;
    }
    last_err = ESP_OK;
    xSemaphoreTake(card_lock, portMAX_DELAY);
    card = c;
    card_failed = card_full = false;
    snprintf(folder, sizeof(folder), "%u:/" FOLDER, (unsigned)ff_diskio_get_pdrv_card(c));
    memset(cached, 0, sizeof(cached)); /* perhaps another card */
    xSemaphoreGive(card_lock);
    return true;
}

static void unmount(void)
{
    xSemaphoreTake(card_lock, portMAX_DELAY);
    if (card) {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        card = NULL;
    }
    card_failed = false;
    xSemaphoreGive(card_lock);
}

bool sdlog_card(void)
{
    if (!card_lock) {
        return false;
    }
    xSemaphoreTake(card_lock, portMAX_DELAY);
    bool ok = card && !card_failed;
    xSemaphoreGive(card_lock);
    return ok;
}

/* ---- reading: the list, and the files */

/* The times of a log file (unix seconds, 0: unknown; called with card_lock held), as log_first_uptime() and
 * log_last_time() find them. */
static void file_times(uint32_t number, uint32_t size, uint32_t *start, uint32_t *end)
{
    *start = *end = 0;
    for (unsigned i = 0; i < TIMES_CACHED; i++) {
        if (cached[i].size && cached[i].number == number && cached[i].size == size) {
            *start = cached[i].start;
            *end = cached[i].end;
            return;
        }
    }
    char path[32];
    file_path(path, sizeof(path), number);
    FIL f;
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr != FR_OK) {
        reader_trouble(fr, "opening a log");
        return;
    }
    UINT got = 0;
    uint32_t first_up = 0, t, up;
    fr = f_read(&f, scan, SCAN_BYTES, &got);
    if (fr == FR_OK) {
        scan[got] = '\0';
        uint32_t from = size > SCAN_BYTES ? size - SCAN_BYTES : 0;
        if (log_first_uptime(scan, &first_up) && (fr = f_lseek(&f, from)) == FR_OK &&
            (fr = f_read(&f, scan, size - from, &got)) == FR_OK) {
            scan[got] = '\0';
            if (log_last_time(scan, from > 0, &t, &up)) {
                *end = t;
                *start = up >= first_up ? t - (up - first_up) : t;
            }
        }
    }
    close_file(&f);
    reader_trouble(fr, "reading a log");
    if (fr == FR_OK) {
        cached[cached_next] = (times_t){number, size, *start, *end};
        cached_next = (cached_next + 1) % TIMES_CACHED;
    }
}

typedef struct {
    uint32_t number, size;
} listed_t;

/* Keeps the highest-numbered `cap` files of those offered: a min-heap on the number. */
static void keep_newest(listed_t *heap, unsigned *n, unsigned cap, listed_t item)
{
    unsigned i;
    if (*n < cap) {
        for (i = (*n)++; i > 0 && heap[(i - 1) / 2].number > item.number; i = (i - 1) / 2) {
            heap[i] = heap[(i - 1) / 2];
        }
        heap[i] = item;
        return;
    }
    if (cap == 0 || item.number <= heap[0].number) {
        return;
    }
    for (i = 0;;) {
        unsigned c = 2 * i + 1;
        if (c >= cap) {
            break;
        }
        if (c + 1 < cap && heap[c + 1].number < heap[c].number) {
            c++;
        }
        if (heap[c].number >= item.number) {
            break;
        }
        heap[i] = heap[c];
        i = c;
    }
    heap[i] = item;
}

static int newest_first(const void *a, const void *b)
{
    uint32_t x = ((const listed_t *)a)->number, y = ((const listed_t *)b)->number;
    return x < y ? 1 : x > y ? -1 : 0;
}

int sdlog_list(unsigned first, file_entry_t *out, unsigned max, unsigned *total)
{
    *total = 0;
    if (!card_lock) {
        return -FILE_NO_CARD;
    }
    unsigned cap = first < LIST_MOST - max ? first + max : LIST_MOST;
    listed_t *heap = malloc((cap ? cap : 1) * sizeof(*heap));
    if (!heap) {
        return -FILE_NO_CARD;
    }
    unsigned n = 0;
    xSemaphoreTake(card_lock, portMAX_DELAY);
    if (!card || card_failed) {
        xSemaphoreGive(card_lock);
        free(heap);
        return -FILE_NO_CARD;
    }
    FF_DIR dir;
    FRESULT fr = f_opendir(&dir, folder);
    if (fr == FR_OK) {
        uint32_t number;
        while ((fr = f_readdir(&dir, &info)) == FR_OK && info.fname[0]) {
            if (!(info.fattrib & AM_DIR) && name_number(info.fname, &number)) {
                (*total)++;
                keep_newest(heap, &n, cap, (listed_t){number, (uint32_t)info.fsize});
            }
        }
        f_closedir(&dir);
    }
    reader_trouble(fr, "listing the logs");
    bool failed = card_failed || fr != FR_OK;
    xSemaphoreGive(card_lock);
    if (failed) {
        free(heap);
        return -FILE_NO_CARD;
    }
    qsort(heap, n, sizeof(*heap), newest_first);
    int count = 0;
    for (unsigned i = first; i < n && count < (int)max; i++, count++) {
        file_entry_t *e = &out[count];
        snprintf(e->name, sizeof(e->name), NAME_FORMAT, heap[i].number);
        e->size = heap[i].size;
        e->start = e->end = 0;
        xSemaphoreTake(card_lock, portMAX_DELAY); /* a file at a time: the logger does not wait for the whole list */
        if (card && !card_failed) {
            file_times(heap[i].number, heap[i].size, &e->start, &e->end);
        }
        xSemaphoreGive(card_lock);
    }
    free(heap);
    return count;
}

int sdlog_open(sdlog_file_t *file, const char *name, uint32_t *size)
{
    uint32_t number;
    file->open = false;
    if (!name_number(name, &number)) {
        return FILE_NOT_FOUND;
    }
    if (!card_lock) {
        return FILE_NO_CARD;
    }
    int status = FILE_NO_CARD;
    xSemaphoreTake(card_lock, portMAX_DELAY);
    if (card && !card_failed) {
        char path[32];
        file_path(path, sizeof(path), number);
        FRESULT fr = f_open(&file->fil, path, FA_READ);
        if (fr == FR_OK) {
            file->open = true;
            readers++;
            *size = (uint32_t)f_size(&file->fil);
            status = FILE_OK;
        } else if (fr == FR_NO_FILE || fr == FR_NO_PATH) {
            status = FILE_NOT_FOUND;
        } else {
            reader_trouble(fr, "opening a log");
        }
    }
    xSemaphoreGive(card_lock);
    return status;
}

bool sdlog_read(sdlog_file_t *file, uint32_t offset, uint8_t *buf, size_t len)
{
    if (!file->open) {
        return false;
    }
    UINT got = 0;
    xSemaphoreTake(card_lock, portMAX_DELAY);
    FRESULT fr = card_failed ? FR_NOT_READY : f_lseek(&file->fil, offset);
    if (fr == FR_OK) {
        fr = f_read(&file->fil, buf, (UINT)len, &got);
    }
    reader_trouble(fr, "reading a log");
    xSemaphoreGive(card_lock);
    return fr == FR_OK && got == len;
}

void sdlog_close(sdlog_file_t *file)
{
    if (!file->open) {
        return;
    }
    xSemaphoreTake(card_lock, portMAX_DELAY);
    close_file(&file->fil);
    file->open = false;
    readers--;
    xSemaphoreGive(card_lock);
}

/* ---- writing */

/* Text to the log file (called with card_lock held). */
static bool put(const char *text, size_t len)
{
    UINT written = 0;
    FRESULT fr = f_write(&log_file, text, (UINT)len, &written);
    if (fr == FR_OK && written < len) {
        if (!card_full) {
            ESP_LOGW(TAG, "the SD card is full: the flight log stops (its files still download)");
        }
        card_full = true;
        return false;
    }
    trouble(fr, "writing the log");
    log_bytes += written;
    return fr == FR_OK;
}

/* A new log file, numbered `number` or the first free one after it, with the header line (card_lock held). */
static bool start_file(uint32_t number)
{
    char path[32];
    for (; number <= NUMBER_MOST; number++) {
        file_path(path, sizeof(path), number);
        FRESULT fr = f_open(&log_file, path, FA_WRITE | FA_CREATE_NEW);
        if (fr == FR_EXIST) {
            continue;
        }
        if (fr != FR_OK) {
            trouble(fr, "starting a log file");
            if (!card_failed) {
                ESP_LOGW(TAG, "cannot start a log file on the SD card (FatFs error %d): trying again in %d s", (int)fr,
                         MOUNT_EVERY_MS / 1000);
            }
            return false;
        }
        log_open = true;
        log_number = number;
        log_bytes = 0;
        unsynced = 0;
        const char *header = log_header();
        fr = FR_OK;
        if (!put(header, strlen(header)) || (fr = f_sync(&log_file)) != FR_OK) {
            trouble(fr, "starting a log file");
            close_file(&log_file);
            log_open = false;
            return false;
        }
        return true;
    }
    ESP_LOGW(TAG, "the SD card's " FOLDER " folder holds log number %d: no more fit. Make room for the flight log.",
             NUMBER_MOST);
    card_full = true; /* no file to log to on this card: not tried again until another one is in */
    return false;
}

/* Logging on a newly mounted card: the folder, and a file numbered after the last one there (card_lock held). */
static bool start_card(void)
{
    FRESULT fr = f_mkdir(folder);
    if (fr != FR_OK && fr != FR_EXIST) {
        trouble(fr, "making its " FOLDER " folder");
        if (!card_failed) {
            ESP_LOGW(TAG, "cannot make the " FOLDER " folder on the SD card (FatFs error %d): no flight log", (int)fr);
        }
        return false;
    }
    uint32_t highest = 0, number;
    unsigned files = 0;
    FF_DIR dir;
    fr = f_opendir(&dir, folder);
    if (fr == FR_OK) {
        while ((fr = f_readdir(&dir, &info)) == FR_OK && info.fname[0]) {
            if (!(info.fattrib & AM_DIR) && name_number(info.fname, &number)) {
                files++;
                highest = number > highest ? number : highest;
            }
        }
        f_closedir(&dir);
    }
    if (fr != FR_OK) {
        trouble(fr, "reading its " FOLDER " folder");
        return false;
    }
    if (!start_file(highest + 1)) {
        return false;
    }
    uint64_t bytes = (uint64_t)card->csd.capacity * card->csd.sector_size;
    ESP_LOGI(TAG, "SD card %s, %" PRIu64 " GB, %u log file%s: logging to " FOLDER "/" NAME_FORMAT, card->cid.name,
             (bytes + (1ULL << 29)) >> 30, files, files == 1 ? "" : "s", log_number);
    return true;
}

static const char *reset_reason(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "restart";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    default: return "reset";
    }
}

static void write_line(const char *happened)
{
    static log_row_t row;
    static char line[LOG_LINE_MAX];
    log_row_clear(&row);
    row.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    bridge_log_row(&row);
    modem_log_row(&row);
    row.events = happened;
    size_t len = log_format(line, sizeof(line), &row);
    xSemaphoreTake(card_lock, portMAX_DELAY);
    bool ok = !card_failed && put(line, len);
    if (!ok && card_full) {
        close_file(&log_file);
        log_open = false;
    }
    if (ok && ++unsynced >= SYNC_EVERY) {
        unsynced = 0;
        FRESULT fr = f_sync(&log_file);
        trouble(fr, "writing the log");
        ok = fr == FR_OK;
    }
    if (ok && log_bytes >= SPLIT_BYTES) { /* on in a new file */
        uint32_t done = log_number;
        close_file(&log_file);
        log_open = false;
        if (start_file(done + 1)) {
            sdlog_event("continued from " NAME_FORMAT, done);
        }
    }
    xSemaphoreGive(card_lock);
}

static void sdlog_task(void *arg)
{
    static char happened[LOG_EVENTS_MOST + 1];
    bool first_file = true;
    uint32_t mount_at = now_ms();
    uint32_t failed_ms = now_ms() - 2 * 60000, restart_at = now_ms();
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        xSemaphoreTake(card_lock, portMAX_DELAY);
        if (card_failed) {
            if (log_open) {
                close_file(&log_file);
                log_open = false;
            }
            if (!readers) { /* their files closed: a failed read ends a download */
                xSemaphoreGive(card_lock);
                unmount();
                /* again at once the first time (a glitch); if it fails again within a minute, every 30 s */
                uint32_t now = now_ms();
                mount_at = (uint32_t)(now - failed_ms) > 60000 ? now : now + MOUNT_EVERY_MS;
                failed_ms = now;
                xSemaphoreTake(card_lock, portMAX_DELAY);
            }
        }
        bool look = !card && (int32_t)(now_ms() - mount_at) >= 0;
        /* a card that works but took no log file (a full folder cluster, little memory at the 16 MB split, a file
         * named MAVLTE): tried again every 30 s */
        bool restart = card && !card_failed && !card_full && !log_open && (int32_t)(now_ms() - restart_at) >= 0;
        xSemaphoreGive(card_lock);
        if (look) {
            mount_at = now_ms() + MOUNT_EVERY_MS;
            restart_at = now_ms() + MOUNT_EVERY_MS;
            if (mount()) {
                xSemaphoreTake(card_lock, portMAX_DELAY);
                bool started = start_card();
                xSemaphoreGive(card_lock);
                if (started && first_file) { /* the uptime column says how long ago */
                    sdlog_event("MavLTE " FIRMWARE_VERSION " on board V%d started (%s)", board_get()->version,
                                reset_reason());
                    first_file = false;
                } else if (started) {
                    sdlog_event("SD card found");
                }
            }
            wake = xTaskGetTickCount(); /* no catching up on the seconds that took */
        } else if (restart) {
            restart_at = now_ms() + MOUNT_EVERY_MS;
            xSemaphoreTake(card_lock, portMAX_DELAY);
            bool started = start_card();
            xSemaphoreGive(card_lock);
            if (started) {
                sdlog_event(first_file ? "MavLTE " FIRMWARE_VERSION " started; the log could start only now"
                                       : "the log goes on");
                first_file = false;
            }
            wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
        take_events(happened); /* those of a second without a log file are lost */
        if (log_open) {
            write_line(happened);
        }
    }
}

void sdlog_start(void)
{
#if CONFIG_BRIDGE_SD_LOG
    if (board_get()->version != 2) {
        ESP_LOGI(TAG, "no flight log on V1 boards: their TF slot is wired differently");
        return;
    }
    card_lock = xSemaphoreCreateMutex();
    events_lock = xSemaphoreCreateMutex();
    xTaskCreate(sdlog_task, "sdlog", 5120, NULL, 4, NULL);
#endif
}
