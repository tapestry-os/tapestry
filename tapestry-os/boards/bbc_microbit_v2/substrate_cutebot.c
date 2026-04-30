/*
 * tapestry-os/boards/bbc_microbit_v2/substrate_cutebot.c
 * Tapestry L1 substrate implementation for the ELECFREAKS Cutebot Mini.
 *
 * Implements <tapestry/substrate.h> by delegating to the Cutebot I2C driver.
 * Selected at build time when CONFIG_I2C=y (see app CMakeLists.txt).
 *
 * Motion model — differential drive:
 *   twist.linear.x  = forward velocity, normalized [-1, 1]
 *   twist.angular.z = yaw rate,          normalized [-1, 1]
 *                     positive = counterclockwise (turn left)
 *
 *   left_pct  = (linear.x - angular.z) * 100   [clamped to ±100]
 *   right_pct = (linear.x + angular.z) * 100   [clamped to ±100]
 *
 * All other twist axes (linear.y/z, angular.x/y) are ignored — the Cutebot
 * is a ground vehicle constrained to 2-DOF motion.
 */

#include <tapestry/substrate.h>
#include "cutebot.h"

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static int to_pct(float v)
{
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    return (int)(v * 100.0f);
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int substrate_init(void)
{
    return cutebot_init();
}

void substrate_move(const substrate_twist_t *twist)
{
    int left_pct  = to_pct(twist->linear.x - twist->angular.z);
    int right_pct = to_pct(twist->linear.x + twist->angular.z);
    cutebot_drive(left_pct, right_pct);
}

void substrate_set_signal(substrate_signal_t signal)
{
    switch (signal) {
    case SUBSTRATE_SIGNAL_ACTIVE:   cutebot_set_leds(0,   200, 0);   break; /* green  */
    case SUBSTRATE_SIGNAL_DEGRADED: cutebot_set_leds(200, 200, 0);   break; /* yellow */
    case SUBSTRATE_SIGNAL_FAILED:   cutebot_set_leds(200,   0, 0);   break; /* red    */
    case SUBSTRATE_SIGNAL_IDLE:     cutebot_set_leds(0,     0, 200); break; /* blue   */
    case SUBSTRATE_SIGNAL_NONE:
    default:                        cutebot_set_leds(0,     0, 0);   break; /* off    */
    }
}

void substrate_set_power(substrate_power_state_t state)
{
    switch (state) {

    case SUBSTRATE_POWER_ACTIVE:
        /* Returning to full operation — nothing to do at L1; L2 resumes
         * normal gossip cadence. */
        break;

    case SUBSTRATE_POWER_IDLE:
        /* Communication only; actuation paused by L2.  Zephyr's idle
         * thread enters PM_STATE_RUNTIME_IDLE automatically when no
         * runnable work is pending — no explicit call required here. */
        break;

    case SUBSTRATE_POWER_SLEEP:
        /* Stop actuators; L2 (power.c) issues PM_STATE_SUSPEND_TO_IDLE. */
        cutebot_drive(0, 0);
        break;

    case SUBSTRATE_POWER_HARVEST:
        /* Stop actuators; L2 (power.c) arms threshold-gpios and issues
         * PM_STATE_SOFT_OFF (or falls back to SUSPEND_TO_IDLE if no DT node). */
        cutebot_drive(0, 0);
        break;
    }
}

int substrate_sense(substrate_sensor_t type, float *out)
{
    /* Cutebot Mini has no proximity or battery sense via the I2C interface. */
    (void)type;
    (void)out;
    return -1;
}

void substrate_bond(void)    {}
void substrate_release(void) {}
void substrate_emit(void)    {}
