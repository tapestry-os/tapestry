/*
 * tapestry-os/boards/crazyflie21/substrate_crazyflie21.c
 * Tapestry L1 substrate implementation for the Bitcraze Crazyflie 2.1 brushless.
 *
 * Implements <tapestry/substrate.h> by delegating to the Crazyflie 2.1
 * PWM driver (crazyflie21.c) via the motor mixing math (crazyflie21_mix.h).
 *
 * Motion model — quadrotor, 6-DOF:
 *   linear.x   forward (+) / backward (-)  → nose-down/up pitch  → P term
 *   linear.y   left    (+) / right    (-)   → left/right roll     → R term
 *   linear.z   up      (+) / down     (-)   → collective thrust   → T term
 *   angular.x  roll  rate (right-side-down positive)              → R term
 *   angular.y  pitch rate (nose-up positive)                      → P term
 *   angular.z  yaw   rate (CCW positive)                          → Y term
 *
 * linear.x and linear.y contribute to P and R respectively, allowing the BSE
 * to command velocity in the horizontal plane without computing attitude angles.
 * For attitude-rate control, set linear.x/y to zero and use angular.x/y/z.
 *
 * Power state mapping:
 *   ACTIVE   → armed; substrate_move() commands apply immediately.
 *   IDLE     → armed; BSE pauses motion; motors at idle via cf21_set_motors(0,0,0,0).
 *   SLEEP    → disarmed; all motors cut to idle pulse.
 *   HARVEST  → disarmed; same as SLEEP.
 *
 * Signal → LED brightness:
 *   NONE     → 0   (off)
 *   IDLE     → 64  (dim blue-ish — status LED is single-color on CF2.1)
 *   ACTIVE   → 255 (bright)
 *   DEGRADED → 128 (medium)
 *   FAILED   → fast blink driven by Zephyr LED blink API (TODO)
 */

#include <stdbool.h>
#include <tapestry/substrate.h>
#include "crazyflie21.h"
#include "crazyflie21_mix.h"

/* ── API ─────────────────────────────────────────────────────────────────── */

int substrate_init(void)
{
    return cf21_init();
}

void substrate_move(const substrate_twist_t *twist)
{
    cf21_motors_t motors;
    cf21_mix(twist, &motors);
    cf21_set_motors(&motors);
}

void substrate_set_signal(substrate_signal_t signal)
{
    switch (signal) {
    case SUBSTRATE_SIGNAL_ACTIVE:   cf21_set_led(255); break;
    case SUBSTRATE_SIGNAL_DEGRADED: cf21_set_led(128); break;
    case SUBSTRATE_SIGNAL_IDLE:     cf21_set_led(64);  break;
    case SUBSTRATE_SIGNAL_FAILED:   cf21_set_led(32);  break;  /* TODO: blink */
    case SUBSTRATE_SIGNAL_NONE:
    default:                        cf21_set_led(0);   break;
    }
}

void substrate_set_power(substrate_power_state_t state)
{
    switch (state) {
    case SUBSTRATE_POWER_ACTIVE:
        cf21_set_armed(true);
        break;

    case SUBSTRATE_POWER_IDLE: {
        /* Keep ESCs armed but zero all motor outputs so motors spin at idle.
         * The BSE continues to run; motion resumes on the next substrate_move(). */
        static const substrate_twist_t zero = {0};
        substrate_move(&zero);
        break;
    }

    case SUBSTRATE_POWER_SLEEP:
    case SUBSTRATE_POWER_HARVEST:
        cf21_set_armed(false);
        break;
    }
}

int substrate_sense(substrate_sensor_t type, float *out)
{
    /* Crazyflie 2.1 sensors (barometer, IMU) require separate driver paths
     * outside the motor substrate.  Returning -1 lets callers fall back to
     * world-model estimates, which is correct for current Tapestry deployments. */
    (void)type;
    (void)out;
    return -1;
}

void substrate_bond(void)    {}
void substrate_release(void) {}
void substrate_emit(void)    {}
