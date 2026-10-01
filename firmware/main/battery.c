#include "battery.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "board.h"

#define GAUGE_PORT I2C_NUM_0 /* the camera makes its own bus on port 1 where it has its own pins (V1) */
#define GAUGE_ADDRESS 0x36
#define REG_VCELL 0x02 /* 78.125 uV per bit, big-endian */
#define REG_SOC 0x04   /* 1/256 % per bit */

static const char *TAG = "battery";
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t gauge;

void battery_start(void)
{
    const bool v1 = board_get()->version == 1;
    const i2c_master_bus_config_t config = {
        .i2c_port = GAUGE_PORT,
        .sda_io_num = v1 ? 3 : 15,
        .scl_io_num = v1 ? 2 : 16,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* weak, but some help when the board's have no power */
    };
    if (i2c_new_master_bus(&config, &bus) != ESP_OK) {
        bus = NULL;
        ESP_LOGW(TAG, "no I2C bus for the fuel gauge");
        return;
    }
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GAUGE_ADDRESS,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus, &device, &gauge) != ESP_OK) {
        gauge = NULL;
        return;
    }
    if (i2c_master_probe(bus, GAUGE_ADDRESS, 50) != ESP_OK) {
        ESP_LOGI(TAG, "the fuel gauge does not answer%s: the aircraft reports no battery charge",
                 v1 ? "" : " (its bus needs DIP switch CAM on)");
    }
}

static bool read_register(uint8_t reg, uint16_t *value)
{
    uint8_t raw[2];
    if (!gauge || i2c_master_transmit_receive(gauge, &reg, 1, raw, sizeof(raw), 50) != ESP_OK) {
        return false;
    }
    *value = (uint16_t)(raw[0] << 8 | raw[1]);
    return true;
}

bool battery_read(uint16_t *mv, uint8_t *pct)
{
    uint16_t vcell, soc;
    if (!read_register(REG_VCELL, &vcell) || !read_register(REG_SOC, &soc)) {
        return false;
    }
    *mv = (uint16_t)((uint32_t)vcell * 5 / 64); /* 78.125 uV = 5/64 mV (vcell * 78125 overflowed above 4.295 V) */
    unsigned whole = soc >> 8;
    *pct = (uint8_t)(whole > 100 ? 100 : whole);
    return true;
}

int battery_i2c_port(void)
{
    return bus ? GAUGE_PORT : -1;
}
