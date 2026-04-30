/*
 * cutebot.c — ELECFREAKS Cutebot Mini driver (I2C)
 *
 * Cutebot I2C register map (7-bit address 0x10):
 *
 *   Motor control — two separate 4-byte writes, one per motor:
 *     [0x01, dir, speed, 0x00]  — left motor (M1)
 *     [0x02, dir, speed, 0x00]  — right motor (M2)
 *     direction: 0x02 = forward, 0x01 = backward
 *     speed:     0–100
 *
 *   RGB LED control — two separate 4-byte writes, one per LED:
 *     [0x04, R, G, B]  — right LED
 *     [0x08, R, G, B]  — left LED
 *
 * Both writes are issued as standard I2C write transactions with no repeated
 * START.  The Cutebot STM8S firmware acknowledges on address match.
 */

#include "cutebot.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cutebot, LOG_LEVEL_INF);

#define CUTEBOT_I2C_ADDR    0x10u

#define MOTOR_LEFT          0x01u
#define MOTOR_RIGHT         0x02u

#define DIR_FORWARD         0x02u
#define DIR_BACKWARD        0x01u

#define LED_RIGHT_CMD       0x04u
#define LED_LEFT_CMD        0x08u

static const struct device *i2c_dev;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

void cutebot_set_leds(uint8_t r, uint8_t g, uint8_t b)
{
    if (!i2c_dev) {
        return;
    }
    uint8_t buf_r[] = { LED_RIGHT_CMD, r, g, b };
    uint8_t buf_l[] = { LED_LEFT_CMD,  r, g, b };
    int ret;

    ret = i2c_write(i2c_dev, buf_r, sizeof(buf_r), CUTEBOT_I2C_ADDR);
    if (ret) { LOG_WRN("LED right write failed: %d", ret); }
    ret = i2c_write(i2c_dev, buf_l, sizeof(buf_l), CUTEBOT_I2C_ADDR);
    if (ret) { LOG_WRN("LED left write failed: %d", ret); }
}

static void set_motors(int left_pct, int right_pct)
{
    /* Clamp */
    if (left_pct  >  100) { left_pct  =  100; }
    if (left_pct  < -100) { left_pct  = -100; }
    if (right_pct >  100) { right_pct =  100; }
    if (right_pct < -100) { right_pct = -100; }

    uint8_t l_dir   = (left_pct  < 0) ? DIR_BACKWARD : DIR_FORWARD;
    uint8_t r_dir   = (right_pct < 0) ? DIR_BACKWARD : DIR_FORWARD;
    uint8_t l_speed = (uint8_t)(left_pct  < 0 ? -left_pct  : left_pct);
    uint8_t r_speed = (uint8_t)(right_pct < 0 ? -right_pct : right_pct);

    uint8_t buf_l[] = { MOTOR_LEFT,  l_dir, l_speed, 0x00 };
    uint8_t buf_r[] = { MOTOR_RIGHT, r_dir, r_speed, 0x00 };
    int ret;

    ret = i2c_write(i2c_dev, buf_l, sizeof(buf_l), CUTEBOT_I2C_ADDR);
    if (ret) { LOG_WRN("motor left write failed: %d", ret); }
    ret = i2c_write(i2c_dev, buf_r, sizeof(buf_r), CUTEBOT_I2C_ADDR);
    if (ret) { LOG_WRN("motor right write failed: %d", ret); }
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int cutebot_init(void)
{
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(cutebot_i2c));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready — Cutebot unavailable");
        i2c_dev = NULL;
        return -ENODEV;
    }

    set_motors(0, 0);
    cutebot_set_leds(0, 0, 0);

    LOG_INF("Cutebot ready on I2C addr 0x%02X", CUTEBOT_I2C_ADDR);
    return 0;
}

void cutebot_drive(int left_pct, int right_pct)
{
    if (!i2c_dev) {
        return;
    }
    set_motors(left_pct, right_pct);
}

