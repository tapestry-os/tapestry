/*
 * altitude-hold-tether — WS3 first closed-loop flight test (TETHERED ONLY)
 *
 * Exercises the real closed-loop CONFIG_CF21BL_ALTITUDE_HOLD cascade
 * (cf21bl_stabilizer.c: accel/baro complementary-filter velocity estimate +
 * position/velocity/thrust PIDs) with motors actually armed, for the first
 * time. altitude-hold-bench proved the estimator tracks sanely with motors
 * disarmed; this is the next step up.
 *
 * SAFETY — READ BEFORE RUNNING:
 *   This WILL arm the ESCs and the drone WILL attempt to climb and hold
 *   altitude. Physically tether the drone with a short leash anchored so it
 *   cannot climb past roughly 30-40 cm even if the controller misbehaves
 *   completely — the tether is the real safety net here, not the code.
 *   Have a spotter present and a kill switch / battery lead ready to pull.
 *   Do this over a soft surface, clear of people and pets.
 *   DO NOT RUN THIS UNTETHERED.
 *
 * Unlike altitude-hold-test (which deliberately drives collective open-loop,
 * because the pre-WS3 controller porpoised in closed loop), this ramps the
 * ALTITUDE SETPOINT and lets the stabilizer's own closed loop fly itself
 * there — that loop is exactly what's under test.
 *
 * Existing safety nets already active underneath this app, from WS1/WS2:
 *   - idle sentinel zeroes all PID integrators and cuts thrust immediately
 *     whenever linear.z < -0.9
 *   - setpoint-staleness watchdog forces idle if this app stops sending
 *     setpoints for CF21BL_SP_STALE_MS (default 500 ms) — e.g. if this app
 *     hangs or crashes
 *   - tumble supervisor latches motors off if body-Z accel suggests a
 *     tip-over/crash, until power-cycled
 * None of these protect against "climbs too high" on their own — that is
 * what the physical tether is for.
 *
 * Sequence:
 *   0  boot + gyro cal + baro home average (~1 s, inside substrate_init())
 *   1  PLACE ON GROUND AND STAND CLEAR — countdown (5 s)
 *   2  arm ESCs at idle (2 s) — motors should just idle-spin, nothing more
 *   3  ramp setpoint from ~0.15 m to ~0.30 m target over 4 s
 *   4  hold ~0.30 m for 10 s — this is what's actually being tested
 *   5  ramp setpoint back down to ~0.15 m over 4 s
 *   6  cut to idle (immediate thrust-to-minimum; safe at ~0.15 m target)
 *   7  idle settle (2 s)
 *   8  disarm
 *
 * Watch the console for cf21bl_stabilizer's periodic
 *   "alt=... raw=... vz=... target=... vz_sp=... T=..."
 * line (~2 Hz) during stages 3-5: vz should rise then settle back near 0 as
 * alt approaches target, not oscillate in sign repeatedly (that would be
 * porpoising — abort via the kill switch if you see it).
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/tapestry/examples/altitude-hold-tether
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu
 * Read:   minicom -D /dev/ttyUSB0 -b 115200
 *   or:   picocom /dev/ttyACM0
 *   or:   python3 ~/code/tapestry/read_console.py
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/substrate.h>

LOG_MODULE_REGISTER(alt_hold_tether, LOG_LEVEL_INF);

/* 50 Hz setpoint update — comfortably inside CF21BL_SP_STALE_MS (default
 * 500 ms in cf21bl_stabilizer.c), so the watchdog never fires mid-test. */
#define LOOP_DT_S       0.02f

/*
 * target_alt = linear.z + CF21BL_ALT_SP_OFFSET (1.0 m, cf21bl_stabilizer.c).
 * SP_LOW must stay clearly above -0.9 (the is_idle threshold there) or the
 * stabilizer treats it as idle and the altitude cascade never engages.
 */
#define SP_LOW          -0.85f   /* ~0.15 m target — gentle first-liftoff point */
#define SP_HIGH         -0.70f   /* ~0.30 m target — hold height                */

#define COUNTDOWN_S       5
#define ARM_SETTLE_S      2
#define RAMP_S            4.0f
#define HOLD_S           10.0f
#define LAND_SETTLE_S     2

static void send_setpoint(float lz)
{
    substrate_twist_t sp = { .linear = { .z = lz } };
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
    LOG_INF("=== altitude-hold-tether: WS3 closed-loop flight test ===");
    LOG_INF("*** TETHERED FLIGHT ONLY *** confirm the leash is attached "
            "and stand clear before the countdown finishes.");

    if (substrate_init() != 0) {
        LOG_ERR("substrate_init failed (BMP388/BMI088 not ready?) — aborting");
        return -1;
    }

    /* Stage 0: gyro cal + baro home average happen inside
     * cf21bl_stabilizer_start() (called from substrate_init()) — keep the
     * drone still and level on the ground through this. */
    k_msleep(2000);

    /* Stage 1: countdown */
    LOG_INF("Stage 1: PLACE ON GROUND AND STAND CLEAR — arming in %d s ...",
            COUNTDOWN_S);
    for (int i = COUNTDOWN_S; i > 0; i--) {
        LOG_INF("  %d ...", i);
        k_msleep(1000);
    }

    /* Stage 2: arm at idle */
    LOG_INF("Stage 2: arming ESCs (idle, %d s) — motors should just idle-spin",
            ARM_SETTLE_S);
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);
    send_setpoint(-1.0f);
    k_msleep(ARM_SETTLE_S * 1000);

    /* Stage 3: ramp setpoint up to hold height */
    LOG_INF("Stage 3: ramping setpoint %.2f -> %.2f m over %.1f s",
            (double)(SP_LOW + 1.0f), (double)(SP_HIGH + 1.0f), (double)RAMP_S);
    ramp_setpoint(SP_LOW, SP_HIGH, RAMP_S);

    /* Stage 4: hold — this is the actual test */
    LOG_INF("Stage 4: holding %.2f m for %.1f s — watch alt/vz on the console",
            (double)(SP_HIGH + 1.0f), (double)HOLD_S);
    hold_setpoint(SP_HIGH, HOLD_S);

    /* Stage 5: ramp back down */
    LOG_INF("Stage 5: ramping setpoint back down over %.1f s", (double)RAMP_S);
    ramp_setpoint(SP_HIGH, SP_LOW, RAMP_S);

    /* Stage 6: cut to idle — immediate thrust-to-minimum, safe at this point
     * since the target is only ~0.15 m and we're already tracking it. */
    LOG_INF("Stage 6: idle");
    send_setpoint(-1.0f);
    k_msleep(LAND_SETTLE_S * 1000);

    /* Stage 7: disarm */
    substrate_set_power(SUBSTRATE_POWER_SLEEP);
    LOG_INF("Stage 7: disarmed — complete");
    return 0;
}
