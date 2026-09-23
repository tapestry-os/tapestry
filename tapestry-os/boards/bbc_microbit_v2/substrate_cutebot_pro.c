/*
 * tapestry-os/boards/bbc_microbit_v2/substrate_cutebot_pro.c
 * Tapestry L1 substrate implementation for the ELECFREAKS Cutebot Pro.
 *
 * Implements <tapestry/substrate.h> by delegating to the Cutebot Pro I2C
 * driver (cutebot_pro.h). Selected at build time when CONFIG_CUTEBOT_VARIANT_PRO=y
 * (see app CMakeLists.txt) — mirrors substrate_cutebot.c's (Mini) role and
 * motion model exactly:
 *
 *   twist.linear.x  = forward velocity, normalized [-1, 1]
 *   twist.angular.z = yaw rate,          normalized [-1, 1]
 *                     positive = counterclockwise (turn left)
 *
 *   left_cms  = (linear.x - angular.z) * CUTEBOT_PRO_MAX_SPEED_CMS
 *   right_cms = (linear.x + angular.z) * CUTEBOT_PRO_MAX_SPEED_CMS
 *
 * Same simplified symmetric-differential mix substrate_cutebot.c's to_pct()
 * uses for Mini (angular.z=1.0 means "wheels opposite by the full range,"
 * not a wheel-track-derived physical rad/s) — kept deliberately consistent
 * with the established pattern rather than introducing a more "precise"
 * model on Pro alone. All other twist axes are ignored — ground vehicle,
 * 2-DOF motion, same as Mini.
 *
 * Unlike Mini, cutebot_pro_drive_cms() talks to a REAL closed-loop firmware
 * PID (cmd 0x80) with a legal nonzero range of [20, 50] cm/s per wheel —
 * see cutebot_pro.h. CUTEBOT_PRO_MAX_SPEED_CMS is an UNVERIFIED PLACEHOLDER
 * (no Pro hardware bench-measured yet; see Kconfig's CUTEBOT_VARIANT_PRO
 * help) picked as a reasonable mid-range value inside that legal range so a
 * full-speed twist command lands solidly within firmware limits rather than
 * needing to lean on the driver's own clamp.
 */

#include <tapestry/substrate.h>
#include "cutebot_pro.h"
#include <zephyr/kernel.h>

/* UNVERIFIED PLACEHOLDER — see this file's top comment and
 * DEMO_MODE_ENCODER_BENCH (Kconfig). Override at build time with
 * -DCUTEBOT_PRO_MAX_SPEED_CMS=<measured value> once real hardware pins
 * this down, same override convention formation.h's DEMO_* constants use. */
#ifndef CUTEBOT_PRO_MAX_SPEED_CMS
#define CUTEBOT_PRO_MAX_SPEED_CMS 35.0f
#endif

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float to_cms(float v)
{
    v = clampf(v, -1.0f, 1.0f);
    return v * CUTEBOT_PRO_MAX_SPEED_CMS;
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int substrate_init(void)
{
    return cutebot_pro_init();
}

void substrate_move(const substrate_twist_t *twist)
{
    float left_cms  = to_cms(twist->linear.x - twist->angular.z);
    float right_cms = to_cms(twist->linear.x + twist->angular.z);
    cutebot_pro_drive_cms(left_cms, right_cms);
}

void substrate_set_signal(substrate_signal_t signal)
{
    switch (signal) {
    case SUBSTRATE_SIGNAL_ACTIVE:   cutebot_pro_set_leds(0,   200, 0);   break; /* green  */
    case SUBSTRATE_SIGNAL_DEGRADED: cutebot_pro_set_leds(200,  80, 0);   break; /* orange */
    case SUBSTRATE_SIGNAL_FAILED:   cutebot_pro_set_leds(200,   0, 0);   break; /* red    */
    case SUBSTRATE_SIGNAL_IDLE:     cutebot_pro_set_leds(0,     0, 200); break; /* blue   */
    case SUBSTRATE_SIGNAL_NONE:
    default:                        cutebot_pro_set_leds(0,     0, 0);   break; /* off    */
    }
}

void substrate_set_power(substrate_power_state_t state)
{
    switch (state) {

    case SUBSTRATE_POWER_ACTIVE:
        break;

    case SUBSTRATE_POWER_IDLE:
        break;

    case SUBSTRATE_POWER_SLEEP:
        cutebot_pro_drive_cms(0.0f, 0.0f);
        break;

    case SUBSTRATE_POWER_HARVEST:
        cutebot_pro_drive_cms(0.0f, 0.0f);
        break;
    }
}

int substrate_sense(substrate_sensor_t type, float *out)
{
    /* Real wheel-speed/distance feedback exists (cutebot_pro_read_encoders())
     * but doesn't fit substrate_sensor_t's PROXIMITY/BATTERY enum — exposed
     * as a Pro-specific API directly to main.c instead, same precedent as
     * Mini's cutebot_line.h bypassing substrate.h for line-sensor data. */
    (void)type;
    (void)out;
    return -1;
}

void substrate_bond(void)    {}
void substrate_release(void) {}
void substrate_emit(void)    {}

/* ── Identity announcement ───────────────────────────────────────────────── */
/* Identical blink pattern/timing to substrate_cutebot.c's (Mini) — see that
 * file's doc for the full rationale; repeated here rather than shared since
 * substrate.h implementations are independent per board/substrate. */
#define IDENTIFY_FLASH_MS      250
#define IDENTIFY_GAP_MS        250
#define IDENTIFY_REP_PAUSE_MS 1500
#define IDENTIFY_REPEATS         4

void substrate_identify(uint8_t ordinal)
{
    for (int rep = 0; rep < IDENTIFY_REPEATS; rep++) {
        for (int flash = 0; flash <= ordinal; flash++) {
            cutebot_pro_set_leds(180, 180, 180);
            k_msleep(IDENTIFY_FLASH_MS);
            cutebot_pro_set_leds(0, 0, 0);
            k_msleep(IDENTIFY_GAP_MS);
        }
        k_msleep(IDENTIFY_REP_PAUSE_MS);
    }
}
