#include "camera.h"

#include "sdkconfig.h"

#if CONFIG_BRIDGE_CAMERA
#include <inttypes.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "battery.h"
#include "board.h"
#include "snapshot.h"

#define SETTLE_MS 1000 /* frames thrown away after a start, while exposure and white balance settle */

static const char *TAG = "camera";
static bool running;
static camera_fb_t *photo;

/* the camera connector's pins, which the two board versions wire differently (Waveshare's
 * schematics and demos): D0-D7 on GPIO7-14 and SCCB on GPIO15/16 on both */
typedef struct {
    int xclk, pclk, vsync, href;
} camera_pins_t;

static const camera_pins_t V1_PINS = {.xclk = 34, .pclk = 37, .vsync = 36, .href = 35};
static const camera_pins_t V2_PINS = {.xclk = 39, .pclk = 46, .vsync = 42, .href = 41};

static esp_err_t start(framesize_t frame_size, size_t buffer)
{
    const bool v1 = board_get()->version == 1;
    const camera_pins_t *pins = v1 ? &V1_PINS : &V2_PINS;
    /* On V2 boards the fuel gauge shares the SCCB bus: the camera uses the bus battery.c made */
    const int shared = v1 ? -1 : battery_i2c_port();
    const camera_config_t config = {
        .pin_pwdn = -1, /* tied low on the board: the camera is on whenever DIP switch CAM is */
        .pin_reset = -1,
        .pin_xclk = pins->xclk,
        .pin_sccb_sda = shared >= 0 ? -1 : 15,
        .pin_sccb_scl = shared >= 0 ? -1 : 16,
        .sccb_i2c_port = shared >= 0 ? shared : 0,
        .pin_d7 = 14,
        .pin_d6 = 13,
        .pin_d5 = 12,
        .pin_d4 = 11,
        .pin_d3 = 10,
        .pin_d2 = 9,
        .pin_d1 = 8,
        .pin_d0 = 7,
        .pin_vsync = pins->vsync,
        .pin_href = pins->href,
        .pin_pclk = pins->pclk,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = frame_size,
        .jpeg_quality = CONFIG_BRIDGE_CAMERA_QUALITY,
        .fb_count = 1,
        /* internal RAM: the firmware leaves the PSRAM off, so that one image runs on both board versions */
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .jpeg_buffer_size = buffer,
    };
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
        running = true;
#if CONFIG_BRIDGE_CAMERA_FLIP || CONFIG_BRIDGE_CAMERA_MIRROR
        /* as the camera is mounted, by the sensor itself: flipped, mirrored, or both (turned half round), each from
         * what the driver set up, which differs between sensors and modules */
        sensor_t *s = esp_camera_sensor_get();
        if (s && s->set_vflip && s->set_hmirror) {
#if CONFIG_BRIDGE_CAMERA_FLIP
            s->set_vflip(s, !s->status.vflip);
#endif
#if CONFIG_BRIDGE_CAMERA_MIRROR
            s->set_hmirror(s, !s->status.hmirror);
#endif
        }
#endif
    }
    return err;
}

/* The exposure asked for, before the frames thrown away settle it: the brightness the camera's automatic
 * exposure aims for, about 1.4 times brighter or darker each step. On the OV5640 its target (AEC: the average
 * luminance, 44 by default, held within about 8 % either way, and changed faster outside twice that); others
 * (the OV2640) by their own exposure level, as far as it goes. 0 leaves the camera as it sets itself. */
static void set_exposure(int8_t steps)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s || steps == 0) {
        return;
    }
    if (s->id.PID == OV5640_PID && s->set_reg) {
        static const uint8_t targets[2 * SNAP_EXPOSURE_MOST + 1] = {16, 22, 31, 44, 62, 88, 124};
        int target = targets[steps + SNAP_EXPOSURE_MOST];
        int high = target * 27 / 25, low = target * 23 / 25;
        int fast_high = high * 2 > 255 ? 255 : high * 2;
        if (s->set_reg(s, 0x3a0f, 0xff, high) || s->set_reg(s, 0x3a10, 0xff, low) ||
            s->set_reg(s, 0x3a1b, 0xff, high) || s->set_reg(s, 0x3a1e, 0xff, low) ||
            s->set_reg(s, 0x3a11, 0xff, fast_high) || s->set_reg(s, 0x3a1f, 0xff, low / 2)) {
            ESP_LOGW(TAG, "the camera did not take the exposure %+d", steps);
        }
    } else if (s->set_ae_level) {
        s->set_ae_level(s, steps < -2 ? -2 : steps > 2 ? 2 : steps);
    }
}

uint8_t camera_take(uint16_t width, uint16_t height, int8_t exposure, const uint8_t **jpeg, size_t *len)
{
    camera_release();
    framesize_t frame_size;
    size_t buffer; /* room for the JPEG: bright, detailed scenes make the largest ones */
    if (width <= 320) {
        frame_size = FRAMESIZE_QVGA;
        buffer = 32 * 1024;
    } else if (width <= 640) {
        frame_size = FRAMESIZE_VGA;
        buffer = 80 * 1024;
    } else {
        frame_size = FRAMESIZE_XGA;
        buffer = 128 * 1024;
    }
    esp_err_t err = start(frame_size, buffer);
    if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_NOT_SUPPORTED) { /* no sensor answered, or not one it knows */
        ESP_LOGW(TAG, "no camera (%s): is one on the connector, and DIP switch CAM on?", esp_err_to_name(err));
        return SNAP_NO_CAMERA;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot start the camera for %ux%u (%s; largest free block %u KB)", width, height,
                 esp_err_to_name(err),
                 (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024));
        camera_release();
        return SNAP_FAILED;
    }
    set_exposure(exposure);
    /* the first frames after a start are too dark or too bright: throw them away */
    int64_t until = esp_timer_get_time() + SETTLE_MS * 1000LL;
    camera_fb_t *fb = NULL;
    do {
        if (fb) {
            esp_camera_fb_return(fb);
        }
        fb = esp_camera_fb_get();
    } while (fb && esp_timer_get_time() < until);
    if (!fb || fb->format != PIXFORMAT_JPEG || fb->len == 0) {
        ESP_LOGW(TAG, "the camera gave no picture");
        if (fb) {
            esp_camera_fb_return(fb);
        }
        camera_release();
        return SNAP_FAILED;
    }
    photo = fb;
    *jpeg = fb->buf;
    *len = fb->len;
    ESP_LOGI(TAG, "photo %ux%u, %u KB, exposure %+d", (unsigned)fb->width, (unsigned)fb->height,
             (unsigned)((fb->len + 512) / 1024), exposure);
    return SNAP_OK;
}

void camera_release(void)
{
    if (photo) {
        esp_camera_fb_return(photo);
        photo = NULL;
    }
    if (running) {
        /* The OV5640's outputs off before it stops (its next start, the driver's reset, turns them on again): its PCLK
         * is on GPIO46 of V2 boards, a strapping pin, which it held high once a photo was taken, so that an upload over
         * USB failed ("Wrong boot mode") until DIP switch CAM was off. */
        sensor_t *s = esp_camera_sensor_get();
        if (s && s->id.PID == OV5640_PID && s->set_reg) {
            s->set_reg(s, 0x3017, 0xff, 0x00);
            s->set_reg(s, 0x3018, 0xff, 0x00);
        }
        esp_camera_deinit();
        running = false;
    }
}
#endif /* CONFIG_BRIDGE_CAMERA */
