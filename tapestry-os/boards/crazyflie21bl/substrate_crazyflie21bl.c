/*
 * tapestry-os/boards/crazyflie21bl/substrate_crazyflie21bl.c
 * Tapestry L1 substrate implementation for the Bitcraze Crazyflie 2.1 brushless.
 *
 * Implements <tapestry/substrate.h> by delegating to the Crazyflie 2.1
 * PWM driver (crazyflie21bl.c) via the motor mixing math (crazyflie21bl_mix.h).
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
 *   IDLE     → armed; BSE pauses motion; motors at idle via cf21bl_set_motors(0,0,0,0).
 *   SLEEP    → disarmed; all motors cut to idle pulse.
 *   HARVEST  → disarmed; same as SLEEP.
 *
 * Signal → LED brightness:
 *   NONE     → 0     (off)
 *   IDLE     → 64    (dim blue-ish — status LED is single-color on CF2.1)
 *   ACTIVE   → 255   (bright)
 *   DEGRADED → 128   (medium)
 *   FAILED   → blink (250 ms, driven by a k_timer — cf21bl_set_led() only
 *              toggles the LED pin on/off, so FAILED cannot be distinguished
 *              from the other always-on states by brightness alone)
 */

#include <stdbool.h>
#include <tapestry/substrate.h>
#include "crazyflie21bl.h"
#include "crazyflie21bl_mix.h"
#ifdef CONFIG_CF21BL_STABILIZER
#include "cf21bl_stabilizer.h"
#endif
#include <zephyr/kernel.h>

#define CF21BL_FAILED_BLINK_MS 250

/* ── API ─────────────────────────────────────────────────────────────────── */

int substrate_init(void)
{
    return cf21bl_init();
}

void substrate_move(const substrate_twist_t *twist)
{
#ifdef CONFIG_CF21BL_STABILIZER
    cf21bl_stabilizer_set_setpoint(twist);
#else
    cf21bl_motors_t motors;
    cf21bl_mix(twist, &motors);
    cf21bl_set_motors(&motors);
#endif
}

static void failed_blink_fn(struct k_timer *t)
{
    ARG_UNUSED(t);
    static bool s_led_on;
    s_led_on = !s_led_on;
    cf21bl_set_led(s_led_on ? 255 : 0);
}

static K_TIMER_DEFINE(failed_blink_timer, failed_blink_fn, NULL);

void substrate_set_signal(substrate_signal_t signal)
{
    if (signal == SUBSTRATE_SIGNAL_FAILED) {
        k_timer_start(&failed_blink_timer,
                      K_MSEC(CF21BL_FAILED_BLINK_MS),
                      K_MSEC(CF21BL_FAILED_BLINK_MS));
        return;
    }
    /* No-op if not running. */
    k_timer_stop(&failed_blink_timer);

    switch (signal) {
    case SUBSTRATE_SIGNAL_ACTIVE:   cf21bl_set_led(255); break;
    case SUBSTRATE_SIGNAL_DEGRADED: cf21bl_set_led(128); break;
    case SUBSTRATE_SIGNAL_IDLE:     cf21bl_set_led(64);  break;
    case SUBSTRATE_SIGNAL_NONE:
    default:                        cf21bl_set_led(0);   break;
    }
}

void substrate_set_power(substrate_power_state_t state)
{
    switch (state) {
    case SUBSTRATE_POWER_ACTIVE:
        cf21bl_set_armed(true);
        break;

    case SUBSTRATE_POWER_IDLE: {
        /* Keep ESCs armed but drive collective to T=0 so motors spin at minimum.
         * Must use linear.z=-1 (T=0, 1 ms pulse); zero twist gives T=0.5 (50%). */
        static const substrate_twist_t idle = { .linear = { .z = -1.0f } };
        substrate_move(&idle);
        break;
    }

    case SUBSTRATE_POWER_SLEEP:
    case SUBSTRATE_POWER_HARVEST:
        cf21bl_set_armed(false);
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
