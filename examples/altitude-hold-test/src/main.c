/*
 * altitude-hold-test — Open-loop ramp / liftoff-detect / graceful-descent test
 *
 * Does NOT use the tapestry runtime (no quorum, no auto-arming) and does
 * NOT use the in-stabilizer altitude-hold PID (CONFIG_CF21BL_ALTITUDE_HOLD
 * is disabled in prj.conf for this example) — a closed-loop P+I hold on a
 * ~120ms-lagged baro filter with no velocity damping overshoots and then
 * porpoises once airborne. Until that loop gets a proper velocity-damped
 * redesign (WS3), this test instead drives collective open-loop:
 *
 *   Stage 0  boot + ESC arming                          (~3 s, motors silent)
 *   Stage 1  PLACE ON GROUND AND STAND CLEAR — countdown (5 s)
 *   Stage 2  arm ESCs (idle speed) + calibrate baro home (~1 s, motors silent)
 *   Stage 3  slow open-loop ramp UP from just-above-spin-threshold,
 *            watching the barometer for a sustained 30 cm rise that can
 *            only mean real liftoff (not pressure noise)
 *   Stage 4  hold steady for 10 s: a fast PD loop (gains weighted toward
 *            climb-rate, not just position error) clamps further climb the
 *            instant it's detected instead of waiting for a big position
 *            error to build up first
 *   Stage 5  graceful ramp DOWN at the same rate back to spin threshold,
 *            then cut to idle
 *   Stage 6  disarm
 *
 * Attitude (roll/pitch self-leveling, yaw rate) is still closed-loop via
 * cf21bl_stabilizer's rate+angle PIDs (CONFIG_CF21BL_ANGLE_MODE) — only
 * the Z axis is driven directly here.
 *
 * Safety nets:
 *   - RAMP_MAX: abort straight to idle if no liftoff is detected by the
 *     time collective reaches this level — the drone is still on the
 *     ground in this case, so an immediate cut is safe (graceful_land()
 *     would just be a no-op descent from a standstill).
 *   - MAX_CLIMB_M: hard ceiling, enforced during both the Stage 3 ramp and
 *     the Stage 4 hold. The drone IS airborne by the time this can trip,
 *     so the response is graceful_land() (ramp down, then idle), never an
 *     abrupt cut — see ALT_OUTLIER_M above for why a single bad baro read
 *     shouldn't be able to trip this in the first place.
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/tapestry/examples/altitude-hold-test
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu
 * Read:   minicom -D /dev/ttyUSB0 -b 115200
 *   or:   python3 ~/code/tapestry/read_console.py
 */

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/substrate.h>
#include "cf21bl_lighthouse.h"

LOG_MODULE_REGISTER(alt_hold_test, LOG_LEVEL_INF);

/* From lighthouse_cal_office_260706.yaml (2026-07-06, second recalibration same day —
 * BS1 suspected partially occluded, tilted further and re-run). Lighthouse
 * is used here purely as an informational cross-check logged alongside
 * the baro estimate (if a fix is available) — it does not drive any
 * control decision, so the test still runs fine with no base stations
 * powered on. */
static const lh2_bs_pose_t BS0 = {
    .origin = {-0.6803646087646484f, 0.6335355639457703f, 1.615210771560669f},
    .rot    = {0.8344101905822754f, -0.08563866466283798f, 0.5444498062133789f,
               0.13026301562786102f, 0.9905099272727966f, -0.04383661970496178f,
               -0.535528838634491f, 0.1074993908405304f, 0.8376471400260925f}
};
static const lh2_bs_pose_t BS1 = {
    .origin = {0.09399518370628357f, -2.2131965160369873f, 1.4227608442306519f},
    .rot    = {0.049453821033239365f, -0.9982976317405701f, 0.03092208132147789f,
               0.9098809957504272f, 0.057799000293016434f, 0.41082337498664856f,
               -0.4119112491607666f, 0.00781862810254097f, 0.911190390586853f}
};
#define BS0_CHANNEL  0
#define BS1_CHANNEL  1

/* ── Barometer (BMP390 at 0x77) ───────────────────────────────────────────── */

static const struct device *const baro_dev = DEVICE_DT_GET(DT_NODELABEL(bmp388_baro));

#define BARO_LP_ALPHA   0.15f    /* IIR smoothing — matches cf21bl_stabilizer.c */
#define PA_PER_M        11.77f   /* Pa per metre, standard atmosphere           */

/*
 * Outlier guard: this test's baro poll runs in the app's own 50 Hz loop,
 * independent of the BMI088's 1 kHz interrupt-driven I2C3 traffic — unlike
 * cf21bl_stabilizer.c's internal altitude-hold poll, which is phase-locked
 * to land between gyro transactions specifically to avoid bus contention.
 * An occasional read here can race the shared bus and come back torn/
 * stale. No real climb/descent moves this far in one 20 ms step (the
 * fastest genuine jump observed during a liftoff transient was ~0.25 m),
 * so a bigger single-step jump is treated as a corrupted read and dropped
 * — repeat the last filtered value rather than feed garbage into the
 * filter or any safety check built on top of it.
 */
#define ALT_OUTLIER_M   0.40f

static float read_alt_filtered(float *filt, float p_home)
{
    sensor_sample_fetch(baro_dev);
    struct sensor_value sv;
    sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &sv);
    float p_Pa    = sensor_value_to_float(&sv) * 1000.0f;   /* kPa → Pa */
    float alt_raw = (p_home - p_Pa) / PA_PER_M;

    if (fabsf(alt_raw - *filt) > ALT_OUTLIER_M) {
        return *filt;
    }

    *filt = (1.0f - BARO_LP_ALPHA) * (*filt) + BARO_LP_ALPHA * alt_raw;
    return *filt;
}

/* ── Open-loop ramp parameters ────────────────────────────────────────────── */

/*
 * linear.z is literal collective here (CONFIG_CF21BL_ALTITUDE_HOLD is off):
 * T = (linear.z + 1) / 2.  Spin threshold measured at T=0.18 (linear.z=-0.64).
 *
 * SPIN_START: just above the spin threshold so motors begin turning gently.
 * RAMP_RATE:  collective added per second — deliberately slow (the previous
 *             version of this test jumped straight to its target, which is
 *             what caused the fast climb / overshoot).
 * RAMP_MAX:   abort the ramp if still not airborne by this collective.
 */
#define SPIN_START_LZ     -0.55f
#define RAMP_RATE           0.06f   /* collective / second */
#define RAMP_MAX             0.25f  /* abort if no liftoff by collective 0.25 (62.5%) */
#define LOOP_DT              0.02f  /* 50 Hz, matches BMP390 practical poll rate */

/* Liftoff = a rise clearly bigger than baro noise, not "altitude is exactly
 * at the target." 30 cm comfortably clears typical baro jitter while still
 * being conservative — a real liftoff overshoots this fast. Debounce
 * shortened from 10 to 5 readings (~100 ms) so the ramp hands off to the
 * Stage 4 hold loop sooner — every extra reading spent confirming is
 * climb the open-loop ramp doesn't yet have any brake on. */
#define LIFTOFF_ALT_M        0.30f
#define LIFTOFF_HOLD_N          5   /* consecutive readings above threshold (~100 ms) */

/* Hard ceiling — independent of the liftoff debounce above, so a baro
 * glitch or genuine runaway can't ride the ramp (or the Stage 4 hold)
 * past a safe height. */
#define MAX_CLIMB_M           0.60f
#define CEILING_HOLD_N          5   /* consecutive readings above ceiling (~100 ms) */

/* ── Stage 4: hold-steady PD parameters ───────────────────────────────────── */

/*
 * "Reduce motors as quickly as necessary" → lean on the climb-rate (D) term,
 * not just position error (P), since rate is what actually answers "are we
 * still climbing, and how fast" without waiting for position error to build.
 *
 * The velocity estimate is a finite difference over VEL_BASE_N samples
 * (~100 ms), not adjacent 20 ms reads — a single-step diff of the IIR-
 * filtered altitude is still dominated by baro sample noise (the ramp log
 * shows ~5-10 cm jitter sample-to-sample even sitting still, which a 20 ms
 * diff turns into an apparent 2.5-5 m/s "velocity"). Differencing over a
 * longer baseline averages that out while still reacting within ~100 ms.
 *
 * Target and feedforward are both captured at the moment liftoff is
 * confirmed (see hold_target/hold_ff below) — "keep the height stable"
 * means hold wherever it is right now, not chase a separate setpoint.
 *
 * HOLD_SLEW_PER_S caps how fast the commanded collective itself can change,
 * independent of the P/D math — a second line of defense so a single bad
 * reading (whatever its source) can only nudge the output a little instead
 * of snapping it across its full range in one 20 ms step.
 */
/*
 * TUNING (manual, iterate on hardware):
 *   1. D-term zeroed for this pass. Oscillation = gain too high relative to
 *      the system's real response lag, and D computed off a ~100ms-lagged
 *      baro signal can be out of phase with true instantaneous motion —
 *      adding more of it doesn't reliably damp oscillation, it can feed it.
 *      Find the largest HOLD_KP that settles without bouncing first.
 *   2. Only then reintroduce HOLD_KD in small steps to shave off residual
 *      overshoot — each increment should visibly help, not just change the
 *      character of the bounce.
 *   3. HOLD_DURATION_S shortened to 5s for faster/safer tuning iterations;
 *      put back to 10s once a stable Kp (and any Kd) is found.
 */
#define HOLD_DURATION_S       5.0f
#define HOLD_KP               0.60f  /* collective per metre of position error */
#define HOLD_KD               0.00f  /* re-enable in small steps once Kp is stable */
#define HOLD_KI                0.05f /* small bias trim only, light I-limit    */
#define HOLD_I_LIMIT           0.10f
#define HOLD_OUT_MIN          -0.90f  /* willing to cut hard toward idle     */
#define HOLD_SLEW_PER_S        1.00f  /* max |Δcollective| per second        */

#define VEL_BASE_N               5    /* velocity baseline, samples (~100 ms at 50 Hz) */

static const substrate_twist_t IDLE = { .linear = { .z = -1.0f } };

static void emergency_idle(void)
{
    substrate_twist_t sp = IDLE;
    substrate_move(&sp);
    k_msleep(1000);
    substrate_set_power(SUBSTRATE_POWER_SLEEP);
}

static void log_lighthouse(void)
{
    lh2_position_t pos;
    if (cf21bl_lighthouse_get_position(&pos) == 0) {
        LOG_INF("  lh2  x=%+.3f  y=%+.3f  z=%+.3f  m",
                (double)pos.x, (double)pos.y, (double)pos.z);
    }
}

/*
 * Ramp collective down from wherever it is now to the spin threshold, then
 * cut to true idle and disarm. Used both for the normal end-of-hold landing
 * and for every mid-flight safety trip — an abrupt idle cut while airborne
 * is a real fall, not a landing, so every abort from liftoff onward lands
 * the same gentle way the normal flow does.
 */
static void graceful_land(substrate_twist_t *sp, float *alt_filt, float p_home)
{
    LOG_INF("Graceful descent — ramping collective down at %.2f/s from z=%.3f ...",
            (double)RAMP_RATE, (double)sp->linear.z);

    for (int i = 0; i < 2000; i++) {   /* generous cap; loop exits via break below */
        sp->linear.z -= RAMP_RATE * LOOP_DT;
        if (sp->linear.z <= SPIN_START_LZ) {
            sp->linear.z = SPIN_START_LZ;
            substrate_move(sp);
            break;
        }
        substrate_move(sp);

        float alt = read_alt_filtered(alt_filt, p_home);
        if (i % 50 == 0) {
            LOG_INF("  descend z_cmd=%.3f  alt=%.3f m", (double)sp->linear.z, (double)alt);
            log_lighthouse();
        }
        k_msleep((int32_t)(LOOP_DT * 1000.0f));
    }

    LOG_INF("Descent complete — cutting to full idle");
    sp->linear.z = -1.0f;
    substrate_move(sp);
    k_msleep(2000);
    substrate_set_power(SUBSTRATE_POWER_SLEEP);
}

int main(void)
{
    LOG_INF("=== Altitude ramp test: slow up / detect liftoff / graceful down ===");
    LOG_INF("Console: CRTP radio (USART3 taken by lighthouse deck)");

    cf21bl_lighthouse_set_bs_pose(0, &BS0);
    cf21bl_lighthouse_set_bs_pose(1, &BS1);
    cf21bl_lighthouse_set_bs_channel(0, BS0_CHANNEL);
    cf21bl_lighthouse_set_bs_channel(1, BS1_CHANNEL);
    cf21bl_lighthouse_init();

    LOG_INF("Stage 0: ESC arming (~3 s) ...");
    if (substrate_init() != 0) {
        LOG_ERR("substrate_init failed — aborting");
        return -1;
    }
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("BMP390 not ready — aborting");
        return -1;
    }
    LOG_INF("Stage 0 complete");

    /* ── Stage 1: tether window ─────────────────────────────────────── */
    LOG_INF("Stage 1: PLACE ON GROUND AND STAND CLEAR — arming in 5 s ...");
    for (int i = 5; i > 0; i--) {
        LOG_INF("  %d ...", i);
        k_msleep(1000);
    }

    /* ── Stage 2: arm at idle + calibrate baro home ─────────────────────
     * Motors are silent at idle (T=0, below the 18% spin threshold), so
     * this is a clean window to average the home pressure — same approach
     * cf21bl_stabilizer.c uses internally for CONFIG_CF21BL_ALTITUDE_HOLD. */
    LOG_INF("Stage 2: arming ESCs (idle) + calibrating baro home (~1 s) — keep still");
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);
    substrate_move(&IDLE);
    k_msleep(500);

    float p_home = 0.0f;
    for (int n = 0; n < 50; n++) {
        sensor_sample_fetch(baro_dev);
        struct sensor_value sv;
        sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &sv);
        p_home += sensor_value_to_float(&sv) * 1000.0f;
        k_msleep(20);
    }
    p_home /= 50.0f;
    LOG_INF("Baro home: %.1f Pa", (double)p_home);

    /* ── Stage 3: slow ramp up, watching for confirmed liftoff ─────────── */
    LOG_INF("Stage 3: ramp up from z=%.2f at %.2f/s, watching for %.0f cm "
            "sustained rise ...",
            (double)SPIN_START_LZ, (double)RAMP_RATE, (double)(LIFTOFF_ALT_M * 100.0f));

    substrate_twist_t sp = { .linear = { .z = SPIN_START_LZ } };
    substrate_move(&sp);
    k_msleep(500);   /* brief pause at first-spin so motors are running steadily */

    float alt_filt = 0.0f;
    bool  lifted = false;
    int   liftoff_count = 0;
    int   ceiling_count = 0;
    int   ramp_steps = (int)((RAMP_MAX - SPIN_START_LZ) / (RAMP_RATE * LOOP_DT)) + 10;

    for (int i = 0; i < ramp_steps; i++) {
        sp.linear.z += RAMP_RATE * LOOP_DT;
        if (sp.linear.z > RAMP_MAX) { sp.linear.z = RAMP_MAX; }
        substrate_move(&sp);

        float alt = read_alt_filtered(&alt_filt, p_home);

        if (alt > MAX_CLIMB_M) {
            ceiling_count++;
            if (ceiling_count >= CEILING_HOLD_N) {
                LOG_ERR("Hard ceiling exceeded (%.3f m > %.2f m) before liftoff "
                        "confirmed — gracefully landing", (double)alt, (double)MAX_CLIMB_M);
                graceful_land(&sp, &alt_filt, p_home);
                return -1;
            }
        } else {
            ceiling_count = 0;
        }

        if (alt > LIFTOFF_ALT_M) {
            liftoff_count++;
            if (liftoff_count >= LIFTOFF_HOLD_N) {
                lifted = true;
                LOG_INF("Liftoff confirmed: alt=%.3f m at collective z=%.3f",
                        (double)alt, (double)sp.linear.z);
                break;
            }
        } else {
            liftoff_count = 0;
        }

        if (i % 50 == 0) {   /* ~1 Hz */
            LOG_INF("  ramp z_cmd=%.3f  alt=%.3f m", (double)sp.linear.z, (double)alt);
            log_lighthouse();
        }
        k_msleep((int32_t)(LOOP_DT * 1000.0f));
    }

    if (!lifted) {
        LOG_ERR("Ramp reached max (%.2f) without a %.0f cm rise detected — "
                "aborting to idle", (double)RAMP_MAX, (double)(LIFTOFF_ALT_M * 100.0f));
        emergency_idle();
        return -1;
    }

    /* ── Stage 4: hold steady — fast PD, weighted toward climb-rate ─────── */
    /* Step back one ramp increment before the hold loop takes over. The
     * altitude reading that just triggered liftoff confirmation reflects
     * conditions ~100 ms ago (baro lag + debounce window); by then the
     * ramp has already delivered one step too many. Back off immediately
     * so hold_ff starts from one step below the trip point instead of
     * one step above it. */
    sp.linear.z -= RAMP_RATE * LOOP_DT;
    substrate_move(&sp);

    float hold_target = alt_filt;      /* freeze current altitude as target */
    float hold_ff      = sp.linear.z;  /* stepped-back collective as feedforward */

    LOG_INF("Stage 4: holding steady at %.3f m for %.0f s (ff=%.3f) ...",
            (double)hold_target, (double)HOLD_DURATION_S, (double)hold_ff);

    float hold_integral = 0.0f;
    int   hold_steps    = (int)(HOLD_DURATION_S / LOOP_DT);
    int   hold_ceiling_count = 0;
    float prev_out      = sp.linear.z;
    float max_step      = HOLD_SLEW_PER_S * LOOP_DT;

    /* Pre-fill the velocity ring buffer with the current altitude so the
     * first VEL_BASE_N iterations don't see a spurious startup transient. */
    float vel_hist[VEL_BASE_N];
    for (int k = 0; k < VEL_BASE_N; k++) { vel_hist[k] = alt_filt; }
    int vel_hist_idx = 0;

    for (int i = 0; i < hold_steps; i++) {
        float alt = read_alt_filtered(&alt_filt, p_home);
        float err = hold_target - alt;

        float alt_old = vel_hist[vel_hist_idx];
        float vel = (alt - alt_old) / (VEL_BASE_N * LOOP_DT);
        vel_hist[vel_hist_idx] = alt;
        vel_hist_idx = (vel_hist_idx + 1) % VEL_BASE_N;

        if (alt > MAX_CLIMB_M) {
            hold_ceiling_count++;
            if (hold_ceiling_count >= CEILING_HOLD_N) {
                LOG_ERR("Hard ceiling exceeded during hold (%.3f m > %.2f m) — "
                        "gracefully landing", (double)alt, (double)MAX_CLIMB_M);
                graceful_land(&sp, &alt_filt, p_home);
                return -1;
            }
        } else {
            hold_ceiling_count = 0;
        }

        hold_integral += err * LOOP_DT;
        if (hold_integral >  HOLD_I_LIMIT) { hold_integral =  HOLD_I_LIMIT; }
        if (hold_integral < -HOLD_I_LIMIT) { hold_integral = -HOLD_I_LIMIT; }

        float out = hold_ff + HOLD_KP * err + HOLD_KI * hold_integral - HOLD_KD * vel;
        if (out > RAMP_MAX)     { out = RAMP_MAX; }
        if (out < HOLD_OUT_MIN) { out = HOLD_OUT_MIN; }

        /* Slew-limit the commanded step itself, independent of the P/D math
         * above — bounds how much any single bad reading can move the motors. */
        float step = out - prev_out;
        if (step >  max_step) { out = prev_out + max_step; }
        if (step < -max_step) { out = prev_out - max_step; }
        prev_out = out;

        sp.linear.z = out;
        substrate_move(&sp);

        if (i % 50 == 0) {   /* ~1 Hz */
            LOG_INF("  hold z_cmd=%.3f  alt=%.3f m  vel=%.2f m/s  err=%.3f",
                    (double)sp.linear.z, (double)alt, (double)vel, (double)err);
            log_lighthouse();
        }
        k_msleep((int32_t)(LOOP_DT * 1000.0f));
    }

    /* ── Stage 5: graceful descent — ramp back down, then cut ──────────── */
    LOG_INF("Stage 5: graceful descent");
    graceful_land(&sp, &alt_filt, p_home);

    /* ── Stage 6: disarm ─────────────────────────────────────────────── */
    LOG_INF("Stage 6: disarmed — test complete");
    return 0;
}
