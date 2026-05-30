/*
 * crazyflie21.h — Crazyflie 2.1 motor and LED driver API
 *
 * Controls the four brushless ESCs and the status LED on the
 * STM32F405-based Crazyflie 2.1 via Zephyr PWM drivers.
 *
 * ESC PWM convention (standard 50/400 Hz RC signal):
 *   1 000 µs = armed / idle (motors spin at minimum)
 *   2 000 µs = full throttle
 *   Motor value [0.0, 1.0] maps linearly to [1000, 2000] µs.
 *
 * The driver requires a DTS node alias "cf21-motors" that exposes
 * four PWM channels (see crazyflie21.overlay).
 */

#ifndef TAPESTRY_CRAZYFLIE21_H
#define TAPESTRY_CRAZYFLIE21_H

#include <stdbool.h>
#include <stdint.h>
#include "crazyflie21_mix.h"   /* cf21_motors_t */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cf21_init — Verify PWM devices are ready and arm all four ESCs.
 *
 * Sends the idle pulse (1 000 µs) to each ESC for CF21_ARM_MS milliseconds
 * so the ESC arming sequence completes before the first thrust command.
 *
 * Returns 0 on success, negative errno if any PWM device is unreachable.
 * All other cf21_* calls are no-ops until init succeeds.
 */
int cf21_init(void);

/*
 * cf21_set_motors — Write all four motor outputs in a single call.
 *
 * Values in cf21_motors_t are already clamped to [0.0, 1.0] by cf21_mix().
 * This function converts each float to a PWM pulse width and writes it.
 */
void cf21_set_motors(const cf21_motors_t *motors);

/*
 * cf21_set_armed — Enable or disable motor outputs.
 *
 * When disarmed (armed=false), all motors are driven to idle (1 000 µs)
 * regardless of subsequent cf21_set_motors() calls until re-armed.
 * The substrate calls this from substrate_set_power().
 */
void cf21_set_armed(bool armed);

/*
 * cf21_set_led — Set the Crazyflie status LED brightness [0, 255].
 * Pass 0 to turn the LED off.
 */
void cf21_set_led(uint8_t brightness);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CRAZYFLIE21_H */
