/*
 * lh2-hover — Phase 2: lighthouse-controlled 3D position hold
 *
 * Control architecture:
 *   Attitude:       BMI088 → rate + angle PIDs in stabilizer (1 kHz)
 *   Yaw heading:    CONFIG_CF21BL_YAW_HOLD — heading locked to the boot
 *                   orientation (Mahony yaw integrates from wherever the
 *                   drone points at gyro calibration)
 *   X/Y position:   lighthouse → position PD in stabilizer
 *                   (CF21BL_LIGHTHOUSE_POS_HOLD; D term from the lighthouse
 *                   velocity estimate damps the orbit/limit-cycle a P-only
 *                   loop produces)
 *   Z (altitude):   lighthouse → manual ramp then PID here (50 Hz)
 *
 * PLACEMENT REQUIREMENT: the stabilizer maps world-frame position error
 * directly onto body pitch/roll — it assumes the drone's nose points along
 * the lighthouse world +X axis.  In the cfclient geometry, the world origin
 * and axes are the DRONE'S pose during geometry estimation: +X is wherever
 * the nose pointed while calibrating.  So: place the drone at power-on in
 * the same spot/orientation it had during cfclient geometry estimation.
 * Yaw hold then keeps that alignment valid for the whole flight.  (A yaw-
 * rotation of the error vector would lift this requirement — future work.)
 *
 * Sanity check after any recalibration: with the drone on the floor at the
 * calibration spot, Stage 2 home should read x≈0, y≈0, z≈0.  A large home
 * offset (e.g. the z=+1.9 m seen 2026-07-04) means the geometry is being
 * applied in the wrong frame — do not fly.
 *
 * Takeoff philosophy — "ratchet up until it just hovers":
 *   Stage 5 slowly ramps collective from first-spin level upward.
 *   No Z PID is active yet; the drone is free to lift off when thrust
 *   exceeds its weight.  The lighthouse detects liftoff (Z rises > 4 cm)
 *   and records the collective at that moment as the natural hover
 *   feedforward.  Stage 6 then uses this feedforward plus a gentle PID
 *   to hold the target altitude.  This avoids the aggressive overshoot
 *   that happens when a large initial Z error hits a PID cold.
 *
 * Sequence:
 *   0  Init lighthouse + motors + gyro calibration  (~8 s, keep still)
 *   1  Wait for lighthouse fix  (up to 30 s)
 *   2  Record home position
 *   3  PLACE ON GROUND — 5 s countdown
 *   4  Arm ESCs at idle  (2 s)
 *   5  Slow ramp: collective climbs from SPIN_START at RAMP_RATE/s
 *      until lighthouse Z shows liftoff (> LIFTOFF_M above ground)
 *   6  Hold: PID from natural hover FF, target HOME_Z + HOVER_HEIGHT
 *      for 10 s, position logged at 2 Hz
 *   7  Land: symmetric slow ramp DOWN at RAMP_RATE/s until Z is back
 *      near ground, then cut to idle
 *   8  Idle 2 s
 *   9  Disarm
 *
 * Console: CRTP radio (USART3 taken by lighthouse deck at 230400 baud)
 * Read:    python3 ~/code/tapestry/read_console.py
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/examples/lh2-hover
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/substrate.h>
#include "cf21bl_lighthouse.h"

LOG_MODULE_REGISTER(lh2_hover, LOG_LEVEL_INF);

/* ── Calibrated BS poses ──────────────────────────────────────────────────── */
/* July 4 2026 */
 static const lh2_bs_pose_t BS0 = {
    .origin = {1.7085734605789185,-0.43685105443000793,1.8661978244781494},
    .rot    = {-0.8205968737602234,0.055611010640859604,-0.568795382976532,
        -0.08727049082517624,-0.9957755208015442,0.02854771353304386,
        -0.5648049712181091,0.07306522130966187,0.8219834566116333}
  };
static const lh2_bs_pose_t BS1 = {
    .origin = {0.9165827035903931,2.814929246902466,1.7187764644622803},
    .rot    = {0.09428899735212326,0.9954821467399597,-0.011177653446793556,
        -0.8471673130989075,0.0743338093161583,-0.526100754737854,
        -0.5228930711746216,0.05907485634088516,0.8503487706184387}
};

/*
static const lh2_bs_pose_t BS0 = {
    .origin = {-0.1968589723110199f, 2.560563087463379f, 1.2248899936676025f},
    .rot    = { 0.24485638737678528f,  0.9695349335670471f, -0.006882685702294111f,
               -0.9386824369430542f,   0.23527540266513824f, -0.25203338265419006f,
               -0.2427358329296112f,   0.0681726410984993f,   0.9676940441131592f}
};
static const lh2_bs_pose_t BS1 = {
    .origin = {1.066727876663208f, -1.6772024631500244f, 0.9965450167655945f},
    .rot    = {-0.25859639048576355f, -0.9628901481628418f,  -0.07726871222257614f,
                0.9137280583381653f,  -0.2697758674621582f,   0.30384543538093567f,
               -0.3134150207042694f,   0.007970745675265789f,  0.9495828151702881f}
};
*/
#define BS0_CHANNEL  0
#define BS1_CHANNEL  1

/* ── Mission parameters ───────────────────────────────────────────────────── */

#define HOVER_HEIGHT_M  0.30f   /* target altitude above ground (metres) */

/* ── Throttle ramp parameters ─────────────────────────────────────────────── */

/*
 * linear.z → collective thrust T = (z+1)/2, deadband-free mapping: any
 * T > 0 is already in the live ESC range (motor_to_ns() maps (0,1] onto
 * [spin threshold, full]), so "first spin" is any z above the stabilizer's
 * idle sentinel (-0.9), not an ESC property.
 * Hover (tether-measured 2026-07-03, battery-compensated at 3.7 V):
 *   T ≈ 0.46 → linear.z ≈ -0.08.
 *
 * SPIN_START: motors clearly spinning and loaded (T=0.30) but still ~35%
 *             below hover thrust — safe to jump straight to now that the
 *             hover point is battery-compensated and precisely known.
 * RAMP_RATE:  collective added per second.  0.06/s covers SPIN_START →
 *             hover (z≈-0.08) in ~5 s.
 * RAMP_MAX:   abort ramp if still not airborne by this collective
 *             (T=0.55, well above the measured 0.46 hover point).
 * LIFTOFF_M:  Z rise above home that confirms the drone is airborne.
 */
#define SPIN_START       -0.40f   /* ramp start (T=0.30, well below hover)              */
#define RAMP_RATE         0.06f   /* collective / second during ramp                    */
#define RAMP_MAX          0.10f   /* abort if no liftoff by z=0.10 (T=0.55)             */
#define LIFTOFF_M         0.10f   /* 10 cm sustained Z rise = airborne                  */
#define LIFTOFF_HOLD_N      10    /* consecutive Z readings required above threshold     */
#define MAX_CLIMB_M       0.40f   /* hard ceiling: abort if Z exceeds home + 40 cm      */
#define CEILING_HOLD_N      5     /* consecutive readings required before ceiling abort */

/*
 * Physics gate: below this collective the drone CANNOT have climbed, full
 * stop — thrust is below what's needed to overcome weight.  Any Z reading
 * suggesting a climb while collective is this low is sensor noise/corruption,
 * not a real measurement, and must be ignored entirely (not just debounced).
 *
 * Measured hover is z≈-0.08 (T=0.46); -0.30 (T=0.35) is comfortably below
 * any plausible liftoff point but still lets the gate open well before hover
 * so genuine early liftoff (light drone, fresh battery) isn't masked.
 */
#define MIN_THRUST_FOR_CLIMB_CHECK  -0.30f

#define LOOP_DT       0.020f  /* 50 Hz control loop                 */

/* ── Z PID (active only after liftoff) ───────────────────────────────────── */

/*
 * The feedforward (g_ff_hover) is set to the actual collective at liftoff,
 * so no warmup is needed — the integral starts at zero and only corrects
 * small deviations from the natural hover point.
 */
/* Rescaled ×1.22 (=1000/820) from the values flown under the old motor
 * mapping, matching the stabilizer's gain rescale: one z-unit now spans
 * the 820 µs live ESC range instead of 1000 µs. */
#define Z_KP        0.37f   /* 0.10 m error → 0.037 collective correction */
#define Z_KI        0.05f   /* ~8 s to fully correct 0.10 m steady error  */
#define Z_KD        0.10f   /* light damping; median filter adds 170 ms lag*/
#define Z_I_LIMIT   0.25f
#define Z_OUT_MIN  -0.70f
#define Z_OUT_MAX   0.90f
#define Z_IIR_A     0.15f   /* IIR smoothing on top of the median filter   */

static float g_ff_hover;    /* collective at liftoff — set in Stage 5      */
static float g_z_filt;
static float g_z_prev;
static float g_z_integral;
static float g_z_home;
static float g_home_x;     /* home X/Y for liftoff sanity gate            */
static float g_home_y;
static float g_z_target;

/* Maximum distance from home in X or Y for a reading to count as liftoff.
 * Wrong rotor-pair estimates jump 3-4 m in X; real hover stays within 1 m. */
#define HOME_XY_RADIUS  1.5f

static float z_pid(float z_meas)
{
    g_z_filt = (1.0f - Z_IIR_A) * g_z_filt + Z_IIR_A * z_meas;

    float err = g_z_target - g_z_filt;

    g_z_integral += err * LOOP_DT;
    if (g_z_integral >  Z_I_LIMIT) { g_z_integral =  Z_I_LIMIT; }
    if (g_z_integral < -Z_I_LIMIT) { g_z_integral = -Z_I_LIMIT; }

    float vel = (g_z_filt - g_z_prev) / LOOP_DT;
    g_z_prev = g_z_filt;

    float out = g_ff_hover + Z_KP * err + Z_KI * g_z_integral - Z_KD * vel;

    if (out > Z_OUT_MAX) { out = Z_OUT_MAX; }
    if (out < Z_OUT_MIN) { out = Z_OUT_MIN; }
    return out;
}

/* Advance g_z_target toward goal by at most step per call */
static void ramp_z_target(float goal, float step)
{
    float d = goal - g_z_target;
    if (fabsf(d) <= step) { g_z_target = goal; return; }
    g_z_target += (d > 0.0f ? step : -step);
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    LOG_INF("=== lh2-hover: gentle ramp-to-hover ===");
    LOG_INF("Console: CRTP radio (USART3 taken by lighthouse deck)");

    /* ── Stage 0: init lighthouse + motors ──────────────────────────────── */
    LOG_INF("Stage 0: lighthouse init + motor arm + gyro cal (~8 s, keep still)");

    cf21bl_lighthouse_set_bs_pose(0, &BS0);
    cf21bl_lighthouse_set_bs_pose(1, &BS1);
    cf21bl_lighthouse_set_bs_channel(0, BS0_CHANNEL);
    cf21bl_lighthouse_set_bs_channel(1, BS1_CHANNEL);
    cf21bl_lighthouse_init();

    if (substrate_init() != 0) {
        LOG_ERR("substrate_init failed");
        return -1;
    }
    k_msleep(2000);
    LOG_INF("Stage 0 complete");

    /* ── Stage 1: wait for fix ───────────────────────────────────────────── */
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

    /* ── Stage 2: record home — median of 21 readings, not mean ───────────
     * The lighthouse driver already median-filters over 5 samples, but a
     * wrong-rotor-pair geometry solution can still occasionally slip through
     * for several consecutive outputs.  A MEAN over 10 samples lets 2-3 such
     * outliers skew the home baseline permanently.  A MEDIAN over 21 samples
     * needs 11 outliers to move the result at all — effectively immune.
     */
    {
        #define HOME_SAMPLES  21
        float xs[HOME_SAMPLES], ys[HOME_SAMPLES], zs[HOME_SAMPLES];
        int   n = 0;
        for (int i = 0; i < HOME_SAMPLES * 3 && n < HOME_SAMPLES; i++) {
            lh2_position_t s;
            if (cf21bl_lighthouse_get_position(&s) == 0) {
                xs[n] = s.x; ys[n] = s.y; zs[n] = s.z;
                n++;
            }
            k_msleep(100);
        }
        if (n < HOME_SAMPLES / 2) {
            LOG_ERR("Could not read enough position samples for home (%d/%d)",
                    n, HOME_SAMPLES);
            return -1;
        }
        /* Insertion-sort each axis in place, take the middle element */
        for (int axis = 0; axis < 3; axis++) {
            float *a = (axis == 0) ? xs : (axis == 1) ? ys : zs;
            for (int i = 1; i < n; i++) {
                float key = a[i];
                int j = i - 1;
                while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
                a[j + 1] = key;
            }
        }
        g_home_x    = xs[n / 2];
        g_home_y    = ys[n / 2];
        g_z_home    = zs[n / 2];
        g_z_filt    = g_z_home;
        g_z_prev    = g_z_home;
        g_z_target  = g_z_home;
        g_z_integral = 0.0f;
        LOG_INF("Stage 2: home x=%+.3f  y=%+.3f  z=%+.3f  m  (median of %d)",
                (double)g_home_x, (double)g_home_y, (double)g_z_home, n);
    }

    /* ── Stage 3: countdown ─────────────────────────────────────────────── */
    LOG_INF("Stage 3: PLACE ON GROUND AND STAND CLEAR — arming in 5 s ...");
    for (int i = 5; i > 0; i--) { LOG_INF("  %d ...", i); k_msleep(1000); }

    /* ── Stage 4: arm at idle ────────────────────────────────────────────── */
    LOG_INF("Stage 4: arming ESCs (idle, 2 s) — motors should be silent");
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);
    substrate_twist_t sp = { .linear = { .x = 0, .y = 0, .z = -1.0f } };
    substrate_move(&sp);
    k_msleep(2000);

    /* ── Stage 5: slow ramp — "ratchet up until it just hovers" ─────────── */
    LOG_INF("Stage 5: slow ramp from z=%.2f upward at %.2f/s "
            "(liftoff when Z rises %.0f cm) ...",
            (double)SPIN_START, (double)RAMP_RATE,
            (double)(LIFTOFF_M * 100.0f));

    /* Brief pause at first-spin so motors are running steadily.  Re-send the
     * setpoint during the pause: SPIN_START is above the idle sentinel, so a
     * single fire-and-forget move would trip the stabilizer's 500 ms
     * staleness watchdog right at the boundary. */
    sp.linear.z = SPIN_START;
    for (int i = 0; i < 10; i++) {
        substrate_move(&sp);
        k_msleep(50);
    }

    bool lifted = false;
    int  liftoff_count = 0;   /* consecutive readings above LIFTOFF_M  */
    int  ceiling_count = 0;   /* consecutive readings above MAX_CLIMB_M */
    int  ramp_steps = (int)((RAMP_MAX - SPIN_START) / (RAMP_RATE * LOOP_DT)) + 10;

    for (int i = 0; i < ramp_steps; i++) {
        /* Abort immediately if lighthouse fix is lost — without a fix we
         * cannot detect liftoff and the drone would fly blind then cut out.
         * (Fix is checked BEFORE incrementing so we land from spin, not air.) */
        if (!cf21bl_lighthouse_is_valid()) {
            LOG_ERR("Lighthouse fix lost during ramp — cutting to idle");
            sp.linear.z = -1.0f;
            substrate_move(&sp);
            k_msleep(1000);
            substrate_set_power(SUBSTRATE_POWER_SLEEP);
            return -1;
        }

        /* Increment collective by one step */
        sp.linear.z += RAMP_RATE * LOOP_DT;
        if (sp.linear.z > RAMP_MAX) { sp.linear.z = RAMP_MAX; }
        substrate_move(&sp);

        /*
         * Liftoff detection — Z only, no X/Y gate.
         *
         * We previously gated on X/Y proximity to home to filter out
         * wrong-rotor-pair geometry estimates (which jump 3-4 m in X).
         * That worked on the ground but backfired once the drone actually
         * lifted off: lighthouse X/Y estimates go wild at height, so the
         * gate rejected every valid reading and liftoff was never declared.
         * The ramp then continued to RAMP_MAX, the drone flew to 2+ m, and
         * the abort cut power mid-air.
         *
         * Z is more reliable than X/Y for liftoff detection because Z rises
         * monotonically during a real ascent.  Pre-liftoff Z noise is < 5 cm;
         * the 10 cm threshold with 10 consecutive readings provides adequate
         * rejection of on-ground spikes.
         *
         * A hard ceiling (MAX_CLIMB_M) below provides the fail-safe: if Z
         * rises past 40 cm without liftoff detection firing, abort.
         */
        lh2_position_t pos;
        bool have_pos = (cf21bl_lighthouse_get_position(&pos) == 0);

        /*
         * Physics gate: ignore Z entirely below MIN_THRUST_FOR_CLIMB_CHECK.
         * A real flight test showed a sustained false reading (z jumped ~0.9 m
         * and stayed there) while collective was still -0.55 to -0.07 — far
         * too low to lift the drone.  That reading flip-flopped with good
         * values, eventually winning CEILING_HOLD_N in a row and aborting
         * ~14 s into the ramp, well after the false climb appeared.
         *
         * Below this collective, no Z reading can be physically real, so we
         * skip both checks entirely rather than debounce them — debouncing
         * only delays a guaranteed-wrong trigger, it can't prevent it.
         */
        bool thrust_high_enough = (sp.linear.z >= MIN_THRUST_FOR_CLIMB_CHECK);

        /*
         * Hard ceiling — debounced the same way as liftoff detection.
         * CEILING_HOLD_N consecutive readings (100 ms) distinguishes a real
         * runaway climb from a single bad sample while still reacting fast.
         */
        if (thrust_high_enough && have_pos && pos.z > g_z_home + MAX_CLIMB_M) {
            ceiling_count++;
            if (ceiling_count >= CEILING_HOLD_N) {
                LOG_ERR("Z exceeded safe ceiling (%.3f m > home + %.0f cm) for "
                        "%d readings — liftoff detection failed; emergency idle",
                        (double)pos.z, (double)(MAX_CLIMB_M * 100.0f),
                        CEILING_HOLD_N);
                sp.linear.z = -1.0f;
                substrate_move(&sp);
                k_msleep(1000);
                substrate_set_power(SUBSTRATE_POWER_SLEEP);
                return -1;
            }
        } else {
            ceiling_count = 0;
        }

        if (thrust_high_enough && have_pos && pos.z > g_z_home + LIFTOFF_M) {
            liftoff_count++;
            if (liftoff_count >= LIFTOFF_HOLD_N) {
                g_ff_hover = sp.linear.z;
                lifted = true;
                LOG_INF("Liftoff confirmed (%d Z readings >%.0f cm)  "
                        "z=%.3f m  collective=%.3f",
                        LIFTOFF_HOLD_N, (double)(LIFTOFF_M * 100.0f),
                        (double)pos.z, (double)g_ff_hover);
                break;
            }
        } else {
            liftoff_count = 0;
        }

        /* Log every 2 s so we can see the ramp progressing */
        if (i % 100 == 0) {
            LOG_INF("  ramp z_cmd=%.3f  z_meas=%.3f",
                    (double)sp.linear.z,
                    (double)(cf21bl_lighthouse_get_position(&pos) == 0
                             ? pos.z : 0.0f));
        }

        k_msleep((int32_t)(LOOP_DT * 1000.0f));
    }

    if (!lifted) {
        LOG_ERR("Ramp reached RAMP_MAX (%.2f) without liftoff detected — "
                "base stations visible? drone not tethered down?",
                (double)RAMP_MAX);
        sp.linear.z = -1.0f;
        substrate_move(&sp);
        k_msleep(1000);
        substrate_set_power(SUBSTRATE_POWER_SLEEP);
        return -1;
    }

    /* ── Stage 6: hold at target height ─────────────────────────────────── */
    LOG_INF("Stage 6: holding %.0f cm for 10 s (ff=%.3f, PID active) ...",
            (double)(HOVER_HEIGHT_M * 100.0f), (double)g_ff_hover);

    /* Initialise PID state: Z target ramps from current position to goal.
     * Retry until we get a validated reading (within HOME_XY_RADIUS) so an
     * outlier can't set the initial target to 1.6 m and send cmd negative. */
    lh2_position_t cur;
    for (int retry = 0; retry < 20; retry++) {
        if (cf21bl_lighthouse_get_position(&cur) == 0
                && fabsf(cur.x - g_home_x) < HOME_XY_RADIUS
                && fabsf(cur.y - g_home_y) < HOME_XY_RADIUS) {
            break;
        }
        k_msleep(20);
    }
    g_z_target  = cur.z;            /* start from where we actually are    */
    g_z_integral = 0.0f;            /* integral zero: FF handles baseline  */
    float hold_goal = g_z_home + HOVER_HEIGHT_M;
    float hold_step = HOVER_HEIGHT_M * LOOP_DT / 5.0f;  /* reach target in 5 s */

    /* Tolerate up to this many consecutive fix-loss iterations before landing */
    #define MAX_FIX_LOSS_ITERS  25   /* 25 × 20 ms = 0.5 s */
    int fix_loss_count = 0;

    int log_tick = 0;
    for (int i = 0; i < 500; i++) {    /* 500 × 20 ms = 10 s */
        ramp_z_target(hold_goal, hold_step);

        lh2_position_t pos;
        bool have_fix = (cf21bl_lighthouse_get_position(&pos) == 0);
        float z_meas  = have_fix ? pos.z : g_z_filt;

        if (!have_fix) {
            fix_loss_count++;
            if (fix_loss_count >= MAX_FIX_LOSS_ITERS) {
                LOG_ERR("Lighthouse fix lost for %.1f s — initiating emergency land",
                        (double)(fix_loss_count * LOOP_DT));
                break;  /* fall through to Stage 7 landing */
            }
        } else {
            fix_loss_count = 0;
        }

        sp.linear.z = z_pid(z_meas);
        substrate_move(&sp);

        if (++log_tick >= 25) {   /* 2 Hz log */
            log_tick = 0;
            LOG_INF("pos  x=%+.3f  y=%+.3f  z=%+.3f  tgt=%.3f  cmd=%.3f",
                    (double)pos.x, (double)pos.y, (double)pos.z,
                    (double)g_z_target, (double)sp.linear.z);
        }
        k_msleep((int32_t)(LOOP_DT * 1000.0f));
    }

    /* ── Stage 7: land — symmetric slow ramp DOWN ────────────────────────── */
    LOG_INF("Stage 7: landing (slow ramp down at %.2f/s) ...", (double)RAMP_RATE);

    /* Continue logging; ramp z_target toward ground and simultaneously
     * ramp the feedforward down so collective reduces gently. */
    float land_ff = g_ff_hover;
    float ff_step = RAMP_RATE * LOOP_DT;   /* same rate as takeoff ramp */
    bool  landed  = false;

    for (int i = 0; i < 2000; i++) {   /* max 40 s */
        /* Pull both ff and target toward ground together */
        land_ff    -= ff_step;
        if (land_ff < SPIN_START) { land_ff = SPIN_START; }
        g_ff_hover  = land_ff;         /* lower the hover baseline   */
        ramp_z_target(g_z_home, hold_step);

        lh2_position_t pos;
        float z_meas = cf21bl_lighthouse_get_position(&pos) == 0
                       ? pos.z : g_z_filt;

        sp.linear.z = z_pid(z_meas);
        substrate_move(&sp);

        /* Touchdown: Z is back near ground AND collective is low */
        if (z_meas < g_z_home + LIFTOFF_M && land_ff <= SPIN_START + 0.02f) {
            LOG_INF("Touchdown at z=%.3f m", (double)z_meas);
            landed = true;
            break;
        }

        if (i % 100 == 0) {
            LOG_INF("  landing z=%.3f ff=%.3f cmd=%.3f",
                    (double)z_meas, (double)land_ff, (double)sp.linear.z);
        }

        k_msleep((int32_t)(LOOP_DT * 1000.0f));
    }
    if (!landed) {
        LOG_WRN("Landing timeout — going to idle anyway");
    }

    /* ── Stage 8: idle — settle on ground ───────────────────────────────── */
    LOG_INF("Stage 8: idle (2 s)");
    sp.linear.z = -1.0f;
    substrate_move(&sp);
    k_msleep(2000);

    /* ── Stage 9: disarm ─────────────────────────────────────────────────── */
    substrate_set_power(SUBSTRATE_POWER_SLEEP);
    LOG_INF("Stage 9: disarmed — complete");
    return 0;
}
