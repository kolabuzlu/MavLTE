#include "usbproto.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t usb_crc32(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

size_t usb_base64(char *out, const uint8_t *data, size_t len)
{
    static const char abc[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t n = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = (uint32_t)data[i] << 16 | (i + 1 < len ? (uint32_t)data[i + 1] << 8 : 0) |
                     (i + 2 < len ? data[i + 2] : 0);
        out[n++] = abc[v >> 18 & 63];
        out[n++] = abc[v >> 12 & 63];
        out[n++] = i + 1 < len ? abc[v >> 6 & 63] : '=';
        out[n++] = i + 2 < len ? abc[v & 63] : '=';
    }
    out[n] = '\0';
    return n;
}

size_t usb_data_line(char *out, uint32_t offset, const uint8_t *data, size_t len)
{
    if (len > USB_DATA_CHUNK) {
        len = USB_DATA_CHUNK;
    }
    size_t n = (size_t)snprintf(out, 20, "@D %u ", (unsigned)offset);
    n += usb_base64(out + n, data, len);
    n += (size_t)snprintf(out + n, 12, " %08x\n", (unsigned)usb_crc32(0, data, len));
    return n;
}

usb_cmd_t usb_parse(const char *line, char *name, size_t name_size, uint32_t *number)
{
    char word[4][16] = {{0}};
    int words = 0;
    for (const char *p = line; *p && words < 4;) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        size_t n = strcspn(p, " \t\r\n");
        if (n == 0) {
            break;
        }
        if (n < sizeof(word[0])) {
            memcpy(word[words], p, n);
        }
        words++;
        p += n;
    }
    if (words < 2 || strcmp(word[0], "MAVLTE") != 0) {
        return USB_CMD_NONE;
    }
    if (strcmp(word[1], "HELLO") == 0) {
        return USB_CMD_HELLO;
    }
    if (strcmp(word[1], "LIST") == 0) {
        *number = words >= 3 ? (uint32_t)strtoul(word[2], NULL, 10) : 0;
        return USB_CMD_LIST;
    }
    if (strcmp(word[1], "STOP") == 0) {
        return USB_CMD_STOP;
    }
    if (strcmp(word[1], "SPEED") == 0 && words >= 3) {
        *number = (uint32_t)strtoul(word[2], NULL, 10);
        return *number ? USB_CMD_SPEED : USB_CMD_NONE;
    }
    if (strcmp(word[1], "GET") == 0 && words >= 3 && word[2][0] && strlen(word[2]) < name_size) {
        strcpy(name, word[2]);
        *number = words >= 4 ? (uint32_t)strtoul(word[3], NULL, 10) : 0;
        return USB_CMD_GET;
    }
    return USB_CMD_NONE;
}
