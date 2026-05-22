/*
 * cutebot.h — ELECFREAKS Cutebot Mini motor and LED driver
 *
 * The Cutebot uses an STM8S microcontroller as I2C peripheral at address
 * 0x10 (7-bit).  All commands are sent as a register write sequence.
 *
 * I2C bus: bbc_microbit_v2 exposes the edge-connector I2C on &i2c0, which is
 * the same bus that hosts the on-board LSM303AGR sensors.  The Cutebot address
 * 0x10 does not conflict with LSM303AGR (0x19/0x1E).
 *
 * Motor speed convention: [-100, 100] percent.
 *   > 0  = forward
 *   < 0  = backward
 *   = 0  = stop (coast)
 */

#ifndef TAPESTRY_CUTEBOT_H
#define TAPESTRY_CUTEBOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Obtain the I2C device handle and verify the Cutebot is accessible.
 * Returns 0 on success, negative errno if the I2C bus or device is missing. */
int cutebot_init(void);

/* Set left and right motor speeds.  Values clamped to [-100, 100]. */
void cutebot_drive(int left_pct, int right_pct);

/* Set both RGB LEDs to the given color directly. */
void cutebot_set_leds(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CUTEBOT_H */
