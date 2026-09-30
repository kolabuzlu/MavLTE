/* The board's camera (an OV5640 on the 24-pin connector, powered with DIP switch CAM): started for
 * each photo and stopped once it is sent, so that between photos it takes no memory and no power. */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* Starts the camera, lets its exposure settle and takes a JPEG of width x height (320x240, 640x480 or
 * 1024x768). Returns SNAP_OK with the photo, which stays valid until camera_release(), SNAP_NO_CAMERA
 * if no camera answers, or SNAP_FAILED. Blocks for a second or two. */
uint8_t camera_take(uint16_t width, uint16_t height, const uint8_t **jpeg, size_t *len);
/* Frees the photo and stops the camera. */
void camera_release(void);
