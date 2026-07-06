/*
 * lh2-hover — minimal altitude-hold + lighthouse XY-hold debug flight
 *
 * REWRITTEN 2026-07-06 to isolate a horizontal-instability bug: both the
 * July-4 and 2026-07-06 base-station calibrations produced wild XY
 * wandering (not a clean runaway — an oscillation, swinging back and
 * forth over 0.3-0.5 m) that persisted even after flipping the boot nose
 * heading 180 degrees, which rules out a simple 180-degree mirrored frame.
 * Altitude was ALSO overshooting its target by 50-80% in both flights
 * (target 0.3 m, actual up to 0.54 m) — a real confound, since a
 * tilting/oscillating drone loses vertical thrust and can couple Z
 * instability into XY and vice versa.
 *
 * This version removes that confound: instead of lh2-hover's old
 * hand-rolled Z PID on lighthouse-Z (never migrated off since WS3 landed,
 * a known-deferred item), it uses the already flight-validated closed-loop
 * CONFIG_CF21BL_ALTITUDE_HOLD (baro, independent of lighthouse) for Z,
 * exactly like altitude-hold-tether. Lighthouse X/Y position hold
 * (CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD) is still active — linear.x/y stay at
 * 0 throughout, i.e. "just hold at home" — so any horizontal wandering
 * observed here is attributable to the XY loop alone, not Z coupling.
 *
 * Control architecture:
 *   Attitude:     BMI088 rate + angle PIDs (unchanged)
 *   Yaw heading:  CONFIG_CF21BL_YAW_HOLD — locked to boot orientation
 *   Altitude:     CONFIG_CF21BL_ALTITUDE_HOLD (baro, closed-loop)
 *   X/Y:          CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD, commanded to (0,0)
 *                 i.e. hold at the home position captured at first arm
 *
 * PLACEMENT REQUIREMENT (unchanged, still applies): the stabilizer maps
 * world-frame position error onto body pitch/roll assuming the drone's
 * nose points along the lighthouse world +X axis at boot — Mahony yaw is
 * boot-relative with no absolute reference, so getting this wrong means
 * every correction is applied in the wrong direction. If this flight still
 * wanders significantly, do the SAFE hand-test before flying again: power
 * on (motors need not even be armed), place the drone in its intended
 * orientation, then physically pick it up and translate it in the
 * direction its nose points while watching the "fix (x,y,z)" log line.
 * Moving nose-first should increase X smoothly with Y roughly constant —
 * if it instead changes Y, or decreases X, that tells you the actual
 * angle between "nose" and this calibration's world +X directly, which a
 * 180-degree flip cannot diagnose or fix by itself.
 *
 * Sequence (mirrors altitude-hold-tether):
 *   0  Init lighthouse, then substrate_init() (ESC arm silent, gyro cal,
 *      baro home average, stabilizer start)
 *   1  Wait for lighthouse fix (up to 30 s)
 *   2  PLACE ON GROUND AND STAND CLEAR — 5 s countdown
 *   3  Arm ESCs at idle (2 s)
 *   4  Ramp altitude setpoint SP_LOW -> SP_HIGH (~0.15 m -> ~0.50 m) over
 *      RAMP_S seconds; X/Y setpoint held at 0 (home) throughout
 *   5  Hold ~0.50 m for HOLD_S seconds — watch cf21bl_stabilizer's
 *      periodic "pos x=... y=... z=... ex=... ey=..." log line: x/y
 *      should stay near home with small, damped ex/ey, not grow or
 *      oscillate with growing amplitude
 *   6  Ramp back down to SP_LOW over RAMP_S seconds
 *   7  Cut to idle, settle, disarm
 *
 * TETHERED FLIGHT ONLY until this is understood — same safety posture as
 * altitude-hold-tether.
 *
 * Console: CRTP radio (USART3 taken by lighthouse deck at 230400 baud)
 * Read:    python3 ~/code/tapestry/read_console.py
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/examples/lh2-hover
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/substrate.h>
#include "cf21bl_lighthouse.h"

LOG_MODULE_REGISTER(lh2_hover, LOG_LEVEL_INF);

/* ── Calibrated BS poses + OOTX calibration ──────────────────────────────── */
/* From lighthouse_cal_office_260706.yaml (2026-07-06, second recalibration same day —
 * user suspected BS1 was partially occluded, tilted it further and
 * re-ran Estimate Geometry). Supersedes both the July-4 poses and the
 * first 2026-07-06 poses. OOTX sweep calibration unchanged (same uid for
 * both BS's across all three calibrations — factory calibration is
 * per-physical-BS, tied to uid, not to where/how the BS is mounted). */
 static const lh2_bs_pose_t BS0 = {
    .origin = {-0.6803646087646484,0.6335355639457703,1.615210771560669},
    .rot    = {0.8344101905822754,-0.08563866466283798,0.5444498062133789,
        0.13026301562786102,0.9905099272727966,-0.04383661970496178,
        -0.535528838634491,0.1074993908405304,0.8376471400260925}
  };
static const lh2_bs_pose_t BS1 = {
    .origin = {0.09399518370628357,-2.2131965160369873,1.4227608442306519},
    .rot    = {0.049453821033239365,-0.9982976317405701,0.03092208132147789,
        0.9098809957504272,0.057799000293016434,0.41082337498664856,
        -0.4119112491607666,0.00781862810254097,0.911190390586853}
};

/* OOTX sweep calibration — from the same YAML's "calibs:" section, sweep
 * list order = sweep[0], sweep[1]. */
static const lh2_bs_calib_t BS0_CALIB = {
    .sweep = {
        { .phase = 0.0f,                  .tilt = -0.0482177734375f,
          .curve = -0.139892578125f,      .gibphase = 2.232421875f,
          .gibmag = -0.001861572265625f,  .ogeephase = 1.1142578125f,
          .ogeemag = -0.1802978515625f },
        { .phase = -0.0070343017578125f,  .tilt = 0.038848876953125f,
          .curve = -0.047149658203125f,   .gibphase = 1.4541015625f,
          .gibmag = -0.0013513565063476562f, .ogeephase = 2.359375f,
          .ogeemag = -0.25439453125f },
    },
    .uid = 3438823989u
};
static const lh2_bs_calib_t BS1_CALIB = {
    .sweep = {
        { .phase = 0.0f,                  .tilt = -0.047393798828125f,
          .curve = -0.3046875f,           .gibphase = 1.1494140625f,
          .gibmag = -0.004795074462890625f, .ogeephase = 0.0887451171875f,
          .ogeemag = 0.09014892578125f },
        { .phase = -0.0010623931884765625f, .tilt = 0.051727294921875f,
          .curve = -0.1802978515625f,     .gibphase = 1.525390625f,
          .gibmag = -0.007568359375f,     .ogeephase = 0.97998046875f,
          .ogeemag = 0.24072265625f },
    },
    .uid = 3211055830u
};

#define BS0_CHANNEL  0
#define BS1_CHANNEL  1

/* ── Mission parameters ───────────────────────────────────────────────────── */

/* 50 Hz setpoint update — comfortably inside CF21BL_SP_STALE_MS (default
 * 500 ms), so the watchdog never fires mid-test. */
#define LOOP_DT_S       0.02f

/* target_alt = linear.z + 1.0 m (CONFIG_CF21BL_ALTITUDE_HOLD's
 * linear.z in [-1,+1] -> [0,2] m). SP_LOW must stay clearly above -0.9
 * (the is_idle threshold) or the altitude cascade never engages.
 * Override at build time, e.g. -DHOLD_S=30.0f for a longer hold. */
#define SP_LOW          -0.85f   /* ~0.15 m — gentle first-liftoff point */
#ifndef SP_HIGH
#define SP_HIGH         -0.50f   /* ~0.50 m — hold height, per debug request */
#endif

#define COUNTDOWN_S       5
#define ARM_SETTLE_S      2
#define RAMP_S            4.0f
#ifndef HOLD_S
#define HOLD_S           15.0f
#endif
#define LAND_SETTLE_S     2

static void send_setpoint(float lz)
{
    substrate_twist_t sp = { .linear = { .x = 0.0f, .y = 0.0f, .z = lz } };
    substrate_move(&sp);
}

/* Resend a constant setpoint every LOOP_DT_S for duration_s. */
static void hold_setpoint(float lz, float duration_s)
{
    int steps = (int)(duration_s / LOOP_DT_S);
    for (int i = 0; i < steps; i++) {
        send_setpoint(lz);
        k_msleep((int32_t)(LOOP_DT_S * 1000.0f));
    }
}

/* Linearly ramp the setpoint from 'from' to 'to' over duration_s. */
static void ramp_setpoint(float from, float to, float duration_s)
{
    int steps = (int)(duration_s / LOOP_DT_S);
    for (int i = 0; i <= steps; i++) {
        float frac = (float)i / (float)steps;
        send_setpoint(from + (to - from) * frac);
        k_msleep((int32_t)(LOOP_DT_S * 1000.0f));
    }
}

int main(void)
{
    LOG_INF("=== lh2-hover: minimal altitude-hold + lighthouse XY-hold debug ===");
    LOG_INF("*** TETHERED FLIGHT ONLY *** confirm the leash is attached "
            "and stand clear before the countdown finishes.");

    /* Stage 0: lighthouse init — must precede substrate_init(), which
     * starts the stabilizer thread. */
    cf21bl_lighthouse_set_bs_pose(0, &BS0);
    cf21bl_lighthouse_set_bs_pose(1, &BS1);
    cf21bl_lighthouse_set_bs_calib(0, &BS0_CALIB);
    cf21bl_lighthouse_set_bs_calib(1, &BS1_CALIB);
    cf21bl_lighthouse_set_bs_channel(0, BS0_CHANNEL);
    cf21bl_lighthouse_set_bs_channel(1, BS1_CHANNEL);
    if (cf21bl_lighthouse_init() != 0) {
        LOG_ERR("lighthouse init failed — aborting");
        return -1;
    }

    if (substrate_init() != 0) {
        LOG_ERR("substrate_init failed (BMP388/BMI088 not ready?) — aborting");
        return -1;
    }
    /* Gyro cal + baro home average happen inside cf21bl_stabilizer_start()
     * (called from substrate_init()) — keep the drone still and level. */
    k_msleep(2000);

    /* Stage 1: wait for lighthouse fix */
    LOG_INF("Stage 1: waiting for lighthouse fix ...");
    uint32_t deadline = k_uptime_get_32() + 30000u;
    while (!cf21bl_lighthouse_is_valid()) {
        if (k_uptime_get_32() > deadline) {
            LOG_ERR("No lighthouse fix after 30 s — base stations on? poses correct?");
            return -1;
        }
        k_msleep(200);
    }
    LOG_INF("Fix acquired");

    /* Stage 2: countdown */
    LOG_INF("Stage 2: PLACE ON GROUND AND STAND CLEAR — arming in %d s ...",
            COUNTDOWN_S);
    for (int i = COUNTDOWN_S; i > 0; i--) {
        LOG_INF("  %d ...", i);
        k_msleep(1000);
    }

    /* Stage 3: arm at idle */
    LOG_INF("Stage 3: arming ESCs (idle, %d s) — motors should just idle-spin",
            ARM_SETTLE_S);
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);
    send_setpoint(-1.0f);
    k_msleep(ARM_SETTLE_S * 1000);

    /* Stage 4: ramp altitude setpoint up. This is also the moment the
     * stabilizer's LIGHTHOUSE_POS_HOLD captures "home" (first tick with
     * linear.z > -0.9) — the drone should not be moved between now and
     * Stage 3's countdown finishing. */
    LOG_INF("Stage 4: ramping altitude %.2f -> %.2f m over %.1f s "
            "(X/Y held at home)",
            (double)(SP_LOW + 1.0f), (double)(SP_HIGH + 1.0f), (double)RAMP_S);
    ramp_setpoint(SP_LOW, SP_HIGH, RAMP_S);

    /* Stage 5: hold — this is the actual test. Watch cf21bl_stabilizer's
     * "pos x=... y=... z=... ex=... ey=..." log line. */
    LOG_INF("Stage 5: holding %.2f m for %.1f s — watch pos/ex/ey on the console",
            (double)(SP_HIGH + 1.0f), (double)HOLD_S);
    hold_setpoint(SP_HIGH, HOLD_S);

    /* Stage 6: ramp back down */
    LOG_INF("Stage 6: ramping altitude back down over %.1f s", (double)RAMP_S);
    ramp_setpoint(SP_HIGH, SP_LOW, RAMP_S);

    /* Stage 7: cut to idle — safe here, target is only ~0.15 m and tracked */
    LOG_INF("Stage 7: idle");
    send_setpoint(-1.0f);
    k_msleep(LAND_SETTLE_S * 1000);

    /* Stage 8: disarm */
    substrate_set_power(SUBSTRATE_POWER_SLEEP);
    LOG_INF("Stage 8: disarmed — complete");
    return 0;
}
