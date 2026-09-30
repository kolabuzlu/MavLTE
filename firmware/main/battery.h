/* The board's fuel gauge (a MAX17048 on I2C): the charge of the 18650 cell in its holder, for the
 * locator's reports. V1 boards wire it to GPIO3 (SDA) and GPIO2 (SCL), V2 boards to GPIO15/16, which
 * is the camera's SCCB bus: this module owns that bus, and the camera uses it. On V2 the bus's pull-up
 * resistors only have power with DIP switch CAM on; without them the gauge may not answer. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Sets up the bus and looks for the gauge. */
void battery_start(void);

/* The cell's voltage (mV) and charge (0-100 %); false if the gauge does not answer. Without a cell,
 * it reads what the charger holds the cell's contacts at. */
bool battery_read(uint16_t *mv, uint8_t *pct);

/* The I2C port of the gauge's bus, for the camera to share on V2 boards; -1 if there is no bus. */
int battery_i2c_port(void);
