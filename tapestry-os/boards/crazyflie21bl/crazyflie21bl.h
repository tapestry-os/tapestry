/*
 * crazyflie21bl.h — Crazyflie 2.1 motor and LED driver API
 *
 * Controls the four brushless ESCs and the status LED on the
 * STM32F405-based Crazyflie 2.1 via Zephyr PWM drivers.
 *
 * ESC signal convention (RC PWM 400 Hz default; OneShot125 via Kconfig
 * choice CF21BL_ESC_PROTOCOL divides all widths by 8):
 *   idle width (1 000 µs) = armed, prop stopped — only for motor value ≤ 0
 *   motor value (0.0, 1.0] maps linearly onto the live range
 *     [1180, 2000] µs (spin threshold → full throttle), so any positive
 *     command keeps the prop spinning (see motor_to_ns() in crazyflie21bl.c)
 *
 * The driver requires a DTS node alias "cf21-motors" that exposes
 * four PWM channels (see crazyflie21bl.overlay).
 */

#ifndef TAPESTRY_CRAZYFLIE21BL_H
#define TAPESTRY_CRAZYFLIE21BL_H

#include <stdbool.h>
#include <stdint.h>
#include "crazyflie21bl_mix.h"   /* cf21bl_motors_t */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cf21bl_init — Verify PWM devices are ready and arm all four ESCs.
 *
 * Sends the idle pulse to each ESC for CF21BL_ARM_MS milliseconds
 * so the ESC arming sequence completes before the first thrust command.
 *
 * Returns 0 on success, negative errno if any PWM device is unreachable.
 * All other cf21bl_* calls are no-ops until init succeeds.
 */
int cf21bl_init(void);

/*
 * cf21bl_set_motors — Write all four motor outputs in a single call.
 *
 * Values in cf21bl_motors_t are already clamped to [0.0, 1.0] by cf21bl_mix().
 * This function converts each float to a PWM pulse width and writes it.
 */
void cf21bl_set_motors(const cf21bl_motors_t *motors);

/*
 * cf21bl_set_armed — Enable or disable motor outputs.
 *
 * When disarmed (armed=false), all motors are driven to the idle pulse
 * regardless of subsequent cf21bl_set_motors() calls until re-armed.
 * The substrate calls this from substrate_set_power().
 */
void cf21bl_set_armed(bool armed);

/*
 * cf21bl_set_led — Set the Crazyflie status LED brightness [0, 255].
 * Pass 0 to turn the LED off.
 */
void cf21bl_set_led(uint8_t brightness);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CRAZYFLIE21BL_H */
