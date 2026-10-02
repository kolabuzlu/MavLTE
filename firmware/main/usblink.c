#include "usblink.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "fileout.h"
#include "sdlog.h"
#include "usbproto.h"
#include "version.h"

#define USB_UART CONFIG_ESP_CONSOLE_UART_NUM
#define SPEED_LEAST 9600
#define SPEED_MOST 3000000

static const char *TAG = "usblink";
static const char *const PROBLEMS[] = {"ok", "no SD card", "no such file", "no aircraft", "card read failed", "stopped"};

static uint32_t baud = USB_BAUD;
static bool speed_unconfirmed; /* switched with SPEED: no command at the new rate yet */
static uint32_t speed_at, last_command;

/* the download going on */
static struct {
    bool on;
    sdlog_file_t file;
    char name[FILE_NAME_LEN + 1];
    uint32_t size, next;
} get;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ---- the board's log, quiet while a PC talks to the link: its lines would get into the link's */

static vprintf_like_t log_out;
static bool quiet;

static int no_log(const char *fmt, va_list args)
{
    (void)fmt;
    (void)args;
    return 0;
}

static void keep_quiet(bool on)
{
    if (on == quiet) {
        return;
    }
    if (on) {
        ESP_LOGI(TAG, "MavLTE on the USB port: the log stays quiet until %d s after its last command",
                 USB_IDLE_MS / 1000);
        log_out = esp_log_set_vprintf(no_log);
    } else {
        esp_log_set_vprintf(log_out);
        ESP_LOGI(TAG, "USB port idle: the log goes on");
    }
    quiet = on;
}

/* ---- the link */

static void reply(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void reply(const char *fmt, ...)
{
    char text[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    if (n > 0) {
        uart_write_bytes(USB_UART, text, (size_t)n < sizeof(text) ? (size_t)n : sizeof(text) - 1);
    }
}

static void problem(int status)
{
    reply("@ERR %d %s\n", status, PROBLEMS[status]);
}

static void set_baud(uint32_t rate)
{
    uart_wait_tx_done(USB_UART, pdMS_TO_TICKS(200)); /* the answer still at the old rate */
    uart_set_baudrate(USB_UART, rate);
    uart_flush_input(USB_UART);
    baud = rate;
}

static void stop_get(void)
{
    if (get.on) {
        sdlog_close(&get.file);
        get.on = false;
    }
}

/* The download's next line, and its end after the last one. */
static void send_more(void)
{
    static uint8_t chunk[USB_DATA_CHUNK];
    static char line[USB_LINE_MAX];
    if (get.next >= get.size) {
        reply("@DONE %s %" PRIu32 "\n", get.name, get.size);
        sdlog_event("log %s sent over USB", get.name);
        stop_get();
        return;
    }
    size_t n = get.size - get.next < USB_DATA_CHUNK ? get.size - get.next : USB_DATA_CHUNK;
    if (!sdlog_read(&get.file, get.next, chunk, n)) {
        problem(FILE_CARD_ERROR);
        stop_get();
        return;
    }
    uart_write_bytes(USB_UART, line, usb_data_line(line, get.next, chunk, n));
    get.next += (uint32_t)n;
}

static void command(const char *line)
{
    static file_entry_t entries[FILE_LIST_MOST];
    char name[FILE_NAME_LEN + 1];
    uint32_t number = 0;
    usb_cmd_t cmd = usb_parse(line, name, sizeof(name), &number);
    if (cmd == USB_CMD_NONE) {
        return;
    }
    last_command = now_ms();
    speed_unconfirmed = false;
    keep_quiet(true);
    switch (cmd) {
    case USB_CMD_HELLO:
        reply("@MAVLTE %s %s\n", FIRMWARE_VERSION, sdlog_card() ? "ok" : "none");
        break;
    case USB_CMD_LIST: {
        unsigned total = 0;
        int n = sdlog_list(number, entries, FILE_LIST_MOST, &total);
        if (n < 0) {
            problem(-n);
            break;
        }
        for (int i = 0; i < n; i++) {
            reply("@FILE %s %" PRIu32 " %" PRIu32 " %" PRIu32 "\n", entries[i].name, entries[i].size,
                  entries[i].start, entries[i].end);
        }
        reply("@END %u\n", total);
        break;
    }
    case USB_CMD_GET: {
        stop_get(); /* a new GET ends the one before */
        for (char *p = name; *p; p++) {
            *p = (char)toupper((unsigned char)*p);
        }
        uint32_t size = 0;
        int status = sdlog_open(&get.file, name, &size);
        if (status != FILE_OK) {
            problem(status);
            break;
        }
        get.on = true;
        memcpy(get.name, name, sizeof(get.name));
        get.size = size;
        get.next = number < size ? number : size;
        reply("@SIZE %s %" PRIu32 "\n", get.name, size);
        break;
    }
    case USB_CMD_SPEED:
        if (number < SPEED_LEAST || number > SPEED_MOST) {
            reply("@SPEED %" PRIu32 "\n", baud); /* staying at this one */
            break;
        }
        reply("@SPEED %" PRIu32 "\n", number);
        if (number != baud) {
            set_baud(number);
            speed_unconfirmed = number != USB_BAUD;
            speed_at = now_ms();
        }
        break;
    case USB_CMD_STOP:
        if (get.on) {
            stop_get();
            problem(FILE_STOPPED);
        }
        break;
    default:
        break;
    }
}

static void usb_task(void *arg)
{
    static char line[96];
    static uint8_t in[64];
    size_t len = 0;
    bool overlong = false;
    for (;;) {
        int n = uart_read_bytes(USB_UART, in, sizeof(in), get.on ? 0 : pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            char c = (char)in[i];
            if (c == '\n' || c == '\r') {
                if (len && !overlong) {
                    line[len] = '\0';
                    command(line);
                }
                len = 0;
                overlong = false;
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else {
                overlong = true;
            }
        }
        uint32_t now = now_ms();
        if (get.on) {
            send_more(); /* waits while the UART's buffer is full */
            last_command = now;
        }
        if (speed_unconfirmed && (uint32_t)(now - speed_at) >= USB_SPEED_CHECK_MS) {
            speed_unconfirmed = false; /* nothing came through at the new rate */
            set_baud(USB_BAUD);
        }
        if (quiet && (uint32_t)(now - last_command) >= USB_IDLE_MS) {
            if (baud != USB_BAUD) {
                set_baud(USB_BAUD);
            }
            keep_quiet(false);
        }
    }
}

void usblink_start(void)
{
    fflush(stdout);
    fsync(fileno(stdout));
    esp_err_t err = uart_driver_install(USB_UART, 1024, 4096, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot take the USB serial port over (%s): no log downloads over USB", esp_err_to_name(err));
        return;
    }
    uart_vfs_dev_use_driver(USB_UART); /* the log goes on through the driver */
    xTaskCreate(usb_task, "usblink", 4096, NULL, 3, NULL);
}
