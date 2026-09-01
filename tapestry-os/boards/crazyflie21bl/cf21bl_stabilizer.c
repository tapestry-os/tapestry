/*
 * cf21bl_stabilizer.c — Cascaded attitude controller for the Crazyflie 2.1 brushless
 *
 * See cf21bl_stabilizer.h for architecture, enabling instructions, and setpoint
 * conventions.
 *
 * PID gain derivation
 * -------------------
 * Starting values are converted from the Crazyflie stock firmware cf21bl
 * platform defaults (platform_defaults_cf21bl.h), which expresses rate-loop
 * gains as:
 *
 *   Kp/Ki/Kd targeting:  error in deg/s  →  output in ~INT16 motor units
 *   CF21BL stock roll/pitch rate:  Kp=200, Ki=400,  Kd=2.5
 *   CF21BL stock yaw rate:         Kp=120, Ki=16.7, Kd=0
 *   (stock also runs a yaw *attitude* PID Kp=6/Ki=1/Kd=0.35 above the yaw
 *   rate loop — see CONFIG_CF21BL_YAW_HOLD)
 *
 * Our system:  error in rad/s  →  output normalized [-1, +1]
 *   Conversion: K_ours = K_cf * (180/π) / INT16_MAX
 *   where (180/π) converts rad/s error to deg/s, INT16_MAX=32767 normalizes.
 *
 * One output unit spans the ESC's live range [1180, 2000] µs (motor_to_ns()
 * deadband-free mapping), i.e. 820 µs.  Gains tuned before that remap (when
 * one unit spanned 1000 µs) carry a ×(1000/820) ≈ ×1.22 rescale so the
 * physical loop gain is unchanged.
 *
 * Outer angle loop: CF stock Kp_angle=6.0 (deg/s per deg) converted to
 *   (rad/s per deg): Kp_angle_ours = 6.0 * (π/180) ≈ 0.105.
 *
 * All values are starting points; tune on hardware.
 */

#include "cf21bl_stabilizer.h"
#include "cf21bl_imu.h"
#include "crazyflie21bl.h"      /* cf21bl_set_motors() */
#include "crazyflie21bl_mix.h"  /* cf21bl_mix(), cf21bl_motors_t */
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
#include "cf21bl_lighthouse.h"
#endif
#ifdef CONFIG_CF21BL_PM
#include "cf21bl_pm.h"
#endif

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
#include <zephyr/drivers/sensor.h>
#endif

/* CONFIG_CF21BL_STABILIZER_LOG_LEVEL lets an application quiet the 2 Hz
 * pos/alt streams on a shared console.  The console is not free here: it
 * rides the same USART6 syslink link as P2P gossip, and the STM32F4's
 * 1-byte UART FIFO means every byte is an ISR competing with the 1 kHz
 * control loop.  Measured 2026-08-26: with logging on, ck_fail ran
 * ~20-25/s, quorum flapped, and two drones running one script finished
 * 59.6 s apart; with CONFIG_LOG=n the same flight landed them under 10 s
 * apart.  Apps that do not define this keep LOG_LEVEL_INF. */
#ifndef CONFIG_CF21BL_STABILIZER_LOG_LEVEL
#define CONFIG_CF21BL_STABILIZER_LOG_LEVEL LOG_LEVEL_INF
#endif
LOG_MODULE_REGISTER(cf21bl_stabilizer, CONFIG_CF21BL_STABILIZER_LOG_LEVEL);

/* ── Configuration ─────────────────────────────────────────────────────────── */

#define CF21BL_STAB_STACK_SIZE    2048
#define CF21BL_STAB_PRIO          K_PRIO_PREEMPT(0)

/* Fixed dt matching the BMI088 gyro INT3 rate (~1 kHz).
 * A constant dt avoids hardware-cycle-counter reads each iteration and is
 * accurate enough for a sensor-slaved loop with a stable interrupt source. */
#define CF21BL_LOOP_DT            (1.0f / 1000.0f)

/* Physical rate and angle limits */
#define CF21BL_MAX_RATE_RPS       3.49f   /* ±200 deg/s */
#define CF21BL_MAX_YAW_RATE_RPS   1.75f   /* ±100 deg/s */
#define CF21BL_MAX_ANGLE_DEG      30.0f   /* ±30° for angle mode full-stick */
#define CF21BL_MAX_ANGLE_RATE_RPS CF21BL_MAX_RATE_RPS

/* Rate PID gains — roll and pitch (symmetric)
 * CF21BL stock converted: Kp=200×(180/π)/32767=0.350, Ki=0.699, Kd=0.00437.
 * PWM actuator latency (~5-10ms at 400 Hz RC PWM) + LPF phase delay (~2ms
 * gyro, ~5ms accel) requires lower gains than DSHOT-based CF firmware.
 * Values below are the hardware-tuned 400 Hz-PWM set (0.11 pre-remap)
 * rescaled ×1.22 for the deadband-free motor mapping (see header). */
#ifdef CONFIG_CF21BL_ESC_ONESHOT125
/* OneShot125 halves-to-fifths the actuator latency (2 kHz frame rate);
 * start halfway between the PWM400 set and the stock-converted values,
 * then tune upward toward Kp≈0.42 (0.350 × 1.22) on hardware. */
#define CF21BL_RP_KP      0.20f
#define CF21BL_RP_KI      0.40f
#define CF21BL_RP_KD      0.0020f
#else
#define CF21BL_RP_KP      0.134f
#define CF21BL_RP_KI      0.268f
#define CF21BL_RP_KD      0.00134f
#endif
#define CF21BL_RP_ILIM    0.37f   /* max ki·integral contribution, output units */
#define CF21BL_RP_OLIM    0.61f   /* output clamp (≈ ±500 µs at the ESC) */

/* Rate PID gains — yaw (pre-remap 0.21/0.029, rescaled ×1.22) */
#define CF21BL_YAW_KP     0.256f
#define CF21BL_YAW_KI     0.035f
#define CF21BL_YAW_KD     0.0f
#define CF21BL_YAW_ILIM   0.122f
#define CF21BL_YAW_OLIM   0.366f

/* Outer angle loop (PD_ROLL_KP=6.0, KI=3.0, deg/s per deg → rad/s per deg via × π/180) */
#define CF21BL_ANGLE_KP   0.105f  /* rad/s per degree error */
#define CF21BL_ANGLE_KI   0.052f  /* rad/s per degree·s — corrects steady trim offsets */
#define CF21BL_ANGLE_ILIM 0.35f   /* ±20 deg/s equivalent (CF21BL integration limit) */

/* Position hold (CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD) */
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
/* linear.x/y ∈ [-1,+1] → setpoint in [−POS_MAX, +POS_MAX] meters from origin */
#define CF21BL_POS_MAX_M       ((float)CONFIG_CF21BL_POS_MAX_M)
/* Position P gain: meters error → angle correction.
 * 0.08 rad/m → 4.6°/m.  Raised from 0.05 after the 2026-07-05 flight: a
 * small level-reference bias needs pos error ≈ bias/KP to cancel, and at
 * 0.05 the equilibrium offset (~0.3+ m for ~1° bias) sat near the old
 * error gate.  With saturation at 0.75 m the max correction is
 * 0.08 × 0.75 ≈ 3.4°, well inside the 10° limit. */
#define CF21BL_POS_KP          0.08f   /* rad/m  — converts meters error to rad */
/* Velocity damping: tilt θ produces accel ≈ g·θ, so the closed loop is
 * s² + g·Kd·s + g·Kp; at Kp=0.08, ωn=√(g·Kp)≈0.89 rad/s and ζ≈0.7 needs
 * Kd = 2·ζ·ωn/g ≈ 0.13.  Set slightly above (ζ≈0.78 nominal) because the
 * lighthouse velocity estimate lags (median-5 + LPF), which erodes
 * effective damping near ωn.  The old 0.10 was derived for Kp=0.05 and
 * left ζ≈0.55 after the Kp raise — tether flight 2026-07-05 showed a
 * sustained ~±1 m oscillation at the matching ~7 s natural period.
 * Without this term the P-only loop has no damping at all and
 * orbits/limit-cycles around the setpoint. */
#define CF21BL_POS_KD          0.14f   /* rad per m/s */
/* Slow position integrator (WS6 item 3): trims the standing offset a
 * level-reference bias creates (offset = bias/KP; ~0.35 m observed
 * 2026-07-05 for ~1.6° of bias).  Deliberately slow relative to the loop
 * (ωn≈0.9 rad/s): a 0.35 m error winds in ≈8 s, so it adds no meaningful
 * phase lag at the loop frequency.  Integrates the BODY-frame error —
 * the bias being trimmed lives in the body/IMU frame, so the trim must
 * rotate with the airframe, not the world.  Clamped to ±5° (larger than
 * any plausible level bias).  PERSISTS across idle and fix loss — the
 * bias is a physical constant of the airframe, so the learned trim stays
 * valid between hops; reset only at boot. */
#define CF21BL_POS_KI          0.010f  /* rad per m·s */
#define CF21BL_POS_ILIM_RAD    (5.0f * (float)M_PI / 180.0f)
/* The error fed to the trim integrator is CLAMPED to ±this radius (a hard
 * freeze-beyond-radius until 2026-07-10).  Near the setpoint the trim
 * learns the level bias at full fidelity, unchanged.  Beyond it the
 * winding RATE is capped (KI × 0.30 m ≈ 0.17°/s) instead of zeroed:
 * a symmetric limit-cycle ride (the 2026-07-05 iy-pumping problem the
 * freeze was added for) still averages to ≈0 net winding over a cycle,
 * but a sustained ONE-SIDED push keeps learning in the right direction —
 * the hard freeze locked the trim out exactly when a large static bias
 * made it most needed (drone #2 post-crash, 2026-07-10: ix froze at
 * −0.69° the moment ex saturated and stayed locked through a runaway to
 * the geofence; the bias needed more trim than P's ~3.4°-at-saturation
 * could supply and the trim was forbidden to provide it).  Wrong-way
 * winding during a sensor-artifact excursion is bounded by the same rate
 * cap, the ±5° ILIM, and unlearns at the same rate once tracking is
 * true. */
#define CF21BL_POS_KI_CLAMP_M  0.30f
#define CF21BL_POS_OLIM_DEG    10.0f   /* max angle correction from position loop */
#define CF21BL_POS_OLIM_RAD    (CF21BL_POS_OLIM_DEG * (float)M_PI / 180.0f)

/* ── Fix-loss braking ─────────────────────────────────────────────────────
 * With no active correction on fix loss, whatever horizontal velocity
 * existed at the instant the fix dropped just continues — no P/I term can
 * run without a live position, but the loop's own D term only needs a
 * velocity, and the lighthouse driver already computes one every tick.
 * 2026-09-01 flight 49: 1.56 m of real drift in ~5.3 s blind, breaching the
 * 2.0 m geofence — with FIX_LOSS_GRACE_MS raised to 10 s (see main.c), an
 * uncorrected coast has room to travel several meters before the grace
 * timer or a lucky reacquisition catches it.
 *
 * This reuses CF21BL_POS_KD unchanged — the same, already flight-validated
 * damping gain the live loop applies, continuing to act on the last
 * velocity actually measured rather than a new, untested constant — as a
 * short open-loop pulse: ramped linearly from the captured velocity down
 * to zero over CF21BL_BRAKE_MS, so trust in the snapshot decays as it
 * ages, then pure level for whatever remains of the outage.  Clamped by
 * the position loop's own CF21BL_POS_OLIM_RAD regardless of how large (or
 * how wrong) the captured velocity turns out to be.
 *
 * Best-effort, not exact: there is nothing to close the loop on once
 * blind, so this is a single calibrated pulse, not a controller.  800 ms
 * covers a full stop from a typical DEMO_MAX_SPEED_MPS-scale (0.3 m/s)
 * velocity with margin (θ≈KD·v0≈2.4° → accel≈g·sinθ≈0.41 m/s² →
 * time-to-stop≈0.3/0.41≈0.7 s), while staying a small fraction of the 10 s
 * grace period. */
#define CF21BL_BRAKE_MS        800u
#endif /* CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD */

/* Yaw heading hold (CONFIG_CF21BL_YAW_HOLD) — stock runs a yaw attitude PID
 * (Kp=6, Ki=1, Kd=0.35, deg/s per deg) above the yaw rate loop, integrating
 * rate commands into a heading target so heading is locked between commands
 * instead of random-walking with gyro noise.  Converted ×(π/180) to
 * rad/s per deg.  The D term acts on the measured yaw rate (deg/s). */
#ifdef CONFIG_CF21BL_YAW_HOLD
#define CF21BL_YAWH_KP           0.105f   /* rad/s per deg   (stock 6.0)    */
#define CF21BL_YAWH_KI           0.0175f  /* rad/s per deg·s (stock 1.0)    */
#define CF21BL_YAWH_KD           0.0061f  /* rad/s per deg/s (stock 0.35)   */
#define CF21BL_YAWH_ILIM         0.35f    /* max ki·integral, rad/s          */
#define CF21BL_YAWH_MAX_DELTA_DEG 30.0f   /* clamp target near current yaw
                                           * (stock yawMaxDelta semantics)   */
#endif /* CONFIG_CF21BL_YAW_HOLD */

/* Altitude hold (CONFIG_CF21BL_ALTITUDE_HOLD) */
#define CF21BL_BARO_POLL_DIV   20      /* poll every 20 loop iters ≈ 50 Hz           */
#define CF21BL_PA_PER_M        12.01f  /* Pa per meter = ρ·g at 15 °C sea level      */
/* Hover collective, referenced to CF21BL_PM_VREF (3.7 V).  Tether flight
 * 2026-07-03 (fresh pack, ~4.0 V loaded) sustained hover at T≈0.41–0.42;
 * converting that through the ESC throttle fraction to 3.7 V gives ≈0.46.
 * (The earlier 1650 µs / 0.65-of-old-mapping measurement was taken on a
 * sagged pack — using it on a fresh pack made the drone climb hard and
 * saturate the ±CF21BL_ALT_VEL_OLIM correction.)  With CONFIG_CF21BL_PM
 * compensation active this value is battery-independent; without it,
 * expect ~±0.05 of residual depending on charge state. */
#define CF21BL_ALT_SP_OFFSET   1.0f    /* linear.z=0 → 1 m above home               */

/* Idle sentinel: linear.z at or below this means "application wants motors
 * off", as opposed to "application is commanding a low altitude".
 *
 * This was an unnamed -0.9f literal at four sites.  With
 * CF21BL_ALT_SP_OFFSET = 1.0, linear.z = -0.9 is a COMMANDED ALTITUDE OF
 * 0.10 m — so a landing that walked its target down through 0.10 m tripped
 * the sentinel and cut thrust while the airframe, which lags the target,
 * was still above it.  The drone then fell the remainder.  (2026-07-19
 * flight 10 fixed the disarm half of this — see LAND_TOUCHDOWN_Z_M in
 * examples/cf21bl-formation/src/main.c — but left the thrust half, because
 * that gate only decides when to DECLARE the landing done; by the time it
 * is evaluated the motors have already been idle for ~0.27 s.)
 *
 * -0.98 keeps the dead band (2 cm) narrow enough that any real descent
 * target stays on the closed-loop side of it, while still absorbing float
 * imprecision around an intended exact -1.0.  Applications that want the
 * motors off send exactly -1.0, which the default setpoint already is. */
#define CF21BL_IDLE_SP_Z      (-0.98f)
#define CF21BL_HOVER_T         0.46f
#define CF21BL_T_FLOOR         0.10f   /* min collective mid-flight (≈1262 µs — now a
                                        * real spinning floor; pre-remap 0.10 mapped
                                        * to 1100 µs, inside the ESC dead zone) */
#define CF21BL_G_MPS2          9.80665f

/* Two-state complementary filter: accel-integrated velocity supplies fast
 * dynamics between barometer samples, the barometer supplies a slow but
 * drift-free absolute reference. Every ~50 Hz baro sample nudges both
 * alt_est and vel_est toward the measurement by these fractions of the
 * innovation (the position error between the baro reading and the current
 * estimate) — correcting vel_est too keeps accel-bias drift from growing
 * unbounded between corrections. KP=1 would snap fully to the raw baro
 * reading each sample (no filtering at all).
 *
 * CF21BL_VEL_BARO_KP couples every raw baro sample straight into vel_est,
 * which is then re-integrated into alt_est at 1 kHz — so noise on this path
 * pollutes position too, unlike a plain position-only IIR. Keep it well
 * below CF21BL_ALT_BARO_KP; bench testing (2026-07-02, drone stationary)
 * showed alt/vel_est swinging ~±0.15 m / ±0.15 m/s at KP=0.20 with no motion
 * at all — reduced here pending confirmation of how noisy the raw BMP390
 * reading actually is (see the alt_baro field added to the log line). */
#define CF21BL_ALT_BARO_KP     0.06f
#define CF21BL_VEL_BARO_KP     0.02f

/* Reject a single baro sample that implies an alt_baro jump too large to be
 * real for one ~20 ms tick — almost certainly a torn/corrupted I2C read
 * racing the shared bus with BMI088 gyro traffic, not real motion. Same
 * guard and threshold as read_alt_filtered() in
 * examples/altitude-hold-test/src/main.c, which hit this exact failure mode. */
#define CF21BL_ALT_OUTLIER_M   0.40f

/* Lighthouse z as a second altitude correction source (CONFIG_CF21BL_
 * LIGHTHOUSE_POS_HOLD only — reuses the home-z already captured for the
 * position-hold liftoff check). Baro can be fooled by ground-effect
 * pressure changes near the floor — confirmed 2026-07-xx: raw baro read
 * ~0.56 m against a 0.30 m cruise target while lighthouse z read ~0 m
 * (the drone was actually on the ground), driving a real unscheduled
 * descent. Lighthouse triangulation doesn't share that failure mode, so
 * fusing it in stops baro from unilaterally driving the descent. Same
 * poll cadence as baro (CF21BL_BARO_POLL_DIV) rather than every 1 kHz
 * tick — the lighthouse position cache updates asynchronously at roughly
 * baro's own rate, and applying a KP-fraction correction every tick
 * against a stale cached value would silently multiply the effective
 * gain by however many ticks pass between real updates. Gains start
 * equal to baro's (KP=0.06/0.02) — deliberately not favoring either
 * source yet; raise the lighthouse gains later if it should dominate
 * more, given it doesn't share baro's ground-effect failure mode. */
#define CF21BL_ALT_LH_KP        0.06f
#define CF21BL_VEL_LH_KP        0.02f

/* Outer loop: altitude error [m] → climb-rate setpoint [m/s]. */
#define CF21BL_ALT_POS_KP      0.8f
#define CF21BL_ALT_POS_KI      0.1f
#define CF21BL_ALT_POS_ILIM    0.2f    /* max ki·integral contribution, m/s          */
#define CF21BL_ALT_VEL_SP_MAX  0.5f    /* climb/descend rate clamp, m/s              */

/* Freeze the outer loop's integral while target_alt is far from alt_est.
 * (The lighthouse XY trim used the same hard-freeze pattern until
 * 2026-07-10, now softened to a rate clamp — see CF21BL_POS_KI_CLAMP_M.
 * This one stays a TRUE freeze deliberately: the XY trim is learning a
 * physical airframe bias that persists and must eventually be learned
 * even mid-excursion, whereas a large altitude gap here is the commanded
 * takeoff ramp working as intended — pure windup, nothing to learn.)
 * Without this, the takeoff ramp (main.c walks target_alt from
 * ALT_RAMP_START_M up to cruise at a fixed rate, so ramp DURATION scales
 * with cruise altitude, i.e. with CONFIG_TAPESTRY_ELEMENT_ID via
 * ALT_STEP_PER_ID_M) keeps a large, sustained tracking gap in front of
 * this PID for several seconds on higher-ID drones, winding the integral
 * toward its ILIM cap. That wound-up I-term then keeps commanding extra
 * climb rate well after cruise altitude is reached — confirmed 2026-07-xx:
 * element_id=2 (0.80 m cruise, 6.5 s ramp) climbed past 1.0 m and was
 * still rising when the mission ended it; element_id=0 (0.30 m cruise,
 * 1.5 s ramp) never showed this. Kp=0.8 already tracks the ramp
 * aggressively on its own; the integral only exists to trim small
 * steady-state bias, so it has no business accumulating while genuinely
 * far from target — a large gap is the ramp working as intended, not a
 * bias to learn. Conservative starting point (larger than either
 * element_id=0's 0.15 m ramp gap, which never caused a problem, so this
 * shouldn't change that drone's behavior) — tune on hardware. */
#define CF21BL_ALT_POS_KI_FREEZE_M  0.15f

/* Inner loop: climb-rate error [m/s] → collective correction [fraction].
 * This is the velocity-damping stage the single-stage baro-only P+I lacked —
 * it reacts to the fast accel-derived velocity estimate rather than only the
 * ~50 Hz-sampled position, which is what let closed-loop altitude hold
 * porpoise. Conservative starting gains; tune up on hardware. */
#define CF21BL_ALT_VEL_KP      0.195f  /* pre-remap 0.16, rescaled ×1.22             */
#define CF21BL_ALT_VEL_KI      0.061f
#define CF21BL_ALT_VEL_ILIM    0.061f  /* max ki·integral contribution, thrust fraction */
#define CF21BL_ALT_VEL_OLIM    0.183f  /* collective correction range (≈ ±150 µs)    */

/* ── WS2: Tumble supervisor ─────────────────────────────────────────────────── */

/* Threshold: body-frame Z-accel below this value (in g) signals a tumble.
 * At ±30° max tilt the minimum in-flight accel_z is cos(30°)≈0.87 g, so
 * 0.5 g gives comfortable margin against normal maneuver dynamics. */
#define CF21BL_TUMBLE_ACCEL_Z_G    0.5f

/* How many consecutive 1 kHz samples must be below the threshold before we
 * commit to a tumble call (~50 ms of continuous tilt beyond 60°). */
#define CF21BL_TUMBLE_COUNT        50

/* ── WS2: Setpoint staleness watchdog ──────────────────────────────────────── */

/* Compile-time constants from Kconfig (STALE_MS = 0 disables the feature). */
#define CF21BL_SP_STALE_MS         ((int64_t)CONFIG_CF21BL_SP_STALE_MS)
#define CF21BL_SP_CUTOFF_MS        ((int64_t)CONFIG_CF21BL_SP_CUTOFF_MS)

/* ── Forced landing (critical battery / stale setpoints) ───────────────────── */

/* Defensive numeric floor for the descent target below — see the block
 * comment further down for why it is not the touchdown criterion.
 * Deliberately a standalone figure, not CONFIG_CF21BL_POS_MAX_M: that
 * Kconfig only exists under CF21BL_LIGHTHOUSE_POS_HOLD, and this floor
 * must also make sense for the altitude-hold-only consumers of this file
 * (altitude-hold-tether, altitude-hold-bench, motor-test) that build
 * without it. */
#define CF21BL_LAND_ABS_FLOOR_M    (-5.0f)

/* With CONFIG_CF21BL_ALTITUDE_HOLD, a forced landing walks the altitude
 * target down from the measured alt_est at CF21BL_LAND_RATE_MPS and cuts
 * the motors only once the MEASURED altitude has stopped decreasing, held
 * for CF21BL_LAND_SETTLE_MS.  Cutting on a fixed clock — the first
 * implementation — killed the motors while still airborne, because the
 * setpoint reached the idle sentinel long before the drone reached the
 * ground.
 *
 * Touchdown is deliberately NOT "target_alt reached the ground-relative
 * altitude the descent started from" (0, on the assumption the floor is
 * flat and level with wherever the drone happened to take off).  A
 * platform, a step, a slope, a rug edge, or a takeoff point that was not
 * itself at floor level all break that assumption in either direction —
 * short (declares landed while still airborne over a valley) or long
 * (never declares landed at all over a rise, since the target races on
 * past ground level toward a "0" that isn't there, until
 * CF21BL_LAND_SETTLE_MS forces the issue on a stale error rather than a
 * genuine settle).  Instead this walks the target down UNCONDITIONALLY —
 * the terrain under the airframe decides where it stops, not a value
 * computed before the descent began — and calls it ground when the walk
 * keeps commanding a lower altitude but alt_est stops following: over a
 * rolling CF21BL_LAND_STALL_WINDOW_MS window, less than
 * CF21BL_LAND_STALL_EPS_M of actual descent means something solid is
 * under the airframe, whatever height that turns out to be.  A window
 * that DOES show real descent resets the settle clock, so a brief
 * startup lag (target and alt_est both stationary for the first window,
 * before the ramp has had time to act) self-corrects on the next window
 * rather than needing special-cased at the start.
 *
 * CF21BL_LAND_STALL_EPS_M is a noise floor, not a physical constant —
 * baro/accel noise and ground-effect turbulence at low altitude both eat
 * into the margin between "genuinely still descending" and "resting."
 * 2 cm / 300 ms (≈0.067 m/s) is comfortably under the 0.3 m/s commanded
 * rate but is a bench-tuning starting point, not a validated figure. */
#define CF21BL_LAND_RATE_MPS         0.3f
#define CF21BL_LAND_SETTLE_MS        2000
#define CF21BL_LAND_STALL_WINDOW_MS  300u
#define CF21BL_LAND_STALL_EPS_M      0.02f

/* Without altitude hold there is no altitude estimate, so fall back to
 * ramping the collective to idle over this window (crude, but the only
 * option open-loop). */
#define CF21BL_PM_LAND_MS          3000

/* ── In-flight motor floor ─────────────────────────────────────────────────── */

/* Lowest per-motor command while airborne.  motor_to_ns() maps any v > 0 into
 * the ESC's live range, so this floor only needs margin against signal jitter
 * (0.03 → ≈1205 µs).  Prevents a hard roll/pitch/yaw correction from stopping
 * a prop mid-flight (BLHeli_S re-spin-up costs tens of ms) — the stock
 * firmware's powerDistributionCap()/idleThrust serves the same purpose. */
#define CF21BL_MOTOR_FLOOR         0.03f

/* ── PID ────────────────────────────────────────────────────────────────────── */

typedef struct {
    float kp, ki, kd;
    float integral;
    float m_prev;      /* previous measurement (derivative-on-measurement) */
    float i_limit;
    float out_limit;
} cf21bl_pid_t;

static void pid_init(cf21bl_pid_t *p, float kp, float ki, float kd,
                     float i_limit, float out_limit)
{
    p->kp = kp;  p->ki = ki;  p->kd = kd;
    p->integral  = 0.0f;
    p->m_prev    = 0.0f;
    p->i_limit   = i_limit;
    p->out_limit = out_limit;
}

/* Zero the integrator and re-seed the derivative history at the current
 * measurement so the first post-reset update produces no D spike. */
static void pid_reset(cf21bl_pid_t *p, float measurement)
{
    p->integral = 0.0f;
    p->m_prev   = measurement;
}

/*
 * Discrete PID:  u = Kp·e + Ki·∫e·dt − Kd·(Δmeasurement/dt)
 *
 * The derivative acts on the measurement, not the error, so setpoint steps
 * (e.g. the angle loop handing the rate loop a new target every 1 ms) do not
 * kick the D term — same approach as stock pid.c ("derivative of the measured
 * process variable instead of the error").
 *
 * Anti-windup: the integral is clamped so that ki·integral stays within
 * ±i_limit (same units as the output).  This prevents the integral from
 * saturating the output during extended disturbances (e.g. constrained
 * tethered hover) and ensures the I-term contribution remains bounded
 * even when Kd is zero and rate errors persist.
 */
static float pid_update(cf21bl_pid_t *p, float setpoint, float measurement,
                        float dt)
{
    float error = setpoint - measurement;
    float pterm = p->kp * error;

    p->integral += error * dt;
    if (p->ki != 0.0f) {
        float i_max = p->i_limit / p->ki;
        if      (p->integral >  i_max) p->integral =  i_max;
        else if (p->integral < -i_max) p->integral = -i_max;
    } else {
        p->integral = 0.0f;
    }
    float iterm = p->ki * p->integral;

    float dterm = -p->kd * (measurement - p->m_prev) / dt;
    p->m_prev = measurement;

    float out = pterm + iterm + dterm;
    if      (out >  p->out_limit) out =  p->out_limit;
    else if (out < -p->out_limit) out = -p->out_limit;
    return out;
}

/* ── Shared setpoint (spinlock-protected) ──────────────────────────────────── */

static struct k_spinlock g_sp_lock;

/* Default: linear.z=-1 → T=0 (idle throttle), all angular zero.
 * Keeps motors at minimum until the application explicitly commands thrust. */
static substrate_twist_t g_setpoint = { .linear = { .z = -1.0f } };

/* Timestamp of the last cf21bl_stabilizer_set_setpoint() call (ms since boot).
 * Protected by g_sp_lock.  Initialized in cf21bl_stabilizer_start() so the
 * watchdog does not fire before the first real setpoint has been sent. */
static int64_t g_sp_last_ms;

/* Tumble supervisor state — written only by the stabilizer thread. */
static bool g_tumbled;       /* latched on crash; cleared only by power-cycle */
static bool g_airborne;      /* liftoff detected; drives the Mahony two-stage
                              * accel gain (cf21bl_imu_set_airborne).  Set when
                              * altitude rises >0.15 m above home, cleared only
                              * at the idle sentinel — mid-flight z dips don't
                              * flap the gain. */
static int  g_tumble_count;  /* consecutive below-threshold samples            */

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
/* Forced-landing touchdown latch: set once a forced landing has settled on
 * the ground; keeps the drone idle while the trigger (critical battery /
 * stale setpoints) persists.  Cleared when a stale-setpoint trigger goes
 * away (link recovered → the application's commands apply again); a
 * critical-battery trigger never clears, so the drone stays down. */
static bool    g_landed;
/* Set by cf21bl_stabilizer_request_land() — an application asking for the
 * same closed-loop descent the stale-setpoint and critical-battery paths
 * already use, instead of walking its own altitude target down and hoping
 * the idle sentinel lands it. */
static volatile bool g_land_requested;
/* Descent state — g_land_t0_ms == 0 means "no landing in progress". */
static int64_t g_land_t0_ms;
static float   g_land_alt0;
static int64_t g_land_ground_ms;
/* Rolling stall-detection sample: alt_est and its timestamp at the start
 * of the current CF21BL_LAND_STALL_WINDOW_MS window. */
static float   g_land_alt_ref;
static int64_t g_land_ref_ms;
#endif

/* ── PID instances ──────────────────────────────────────────────────────────── */

static cf21bl_pid_t g_pid_roll;
static cf21bl_pid_t g_pid_pitch;
static cf21bl_pid_t g_pid_yaw;

#ifdef CONFIG_CF21BL_ANGLE_MODE
static cf21bl_pid_t g_pid_roll_angle;
static cf21bl_pid_t g_pid_pitch_angle;
#endif

#ifdef CONFIG_CF21BL_YAW_HOLD
static float g_yaw_target_deg;   /* heading target, wrapped to ±180° */
static float g_yawh_integral;    /* heading-error integral, deg·s    */

static float wrap180f(float a)
{
    while (a >  180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}
#endif

#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
/* Home position captured at first valid lighthouse fix (meters, world frame).
 * Position setpoints are offsets from this origin. */
static float    g_pos_home_x;
static float    g_pos_home_y;
static float    g_pos_home_z;
static bool     g_pos_home_set;
/* "LH2 fix lost" is logged once per OUTAGE.  This used to be latched by
 * clearing g_pos_home_set, which had a much larger side effect than the
 * log throttle it was standing in for — see the fix-lost branch below. */
static bool     g_lh_lost_logged;
static float    g_pos_ix;        /* position integrator, body frame, rad */
static float    g_pos_iy;
static float    g_yaw_now_deg;   /* last Mahony yaw (previous 1 kHz tick) —
                                  * pos-hold runs before this tick's
                                  * filter_update; 1 ms staleness is
                                  * irrelevant at these dynamics */
/* Body-frame velocity at the most recent VALID fix — kept updated every
 * live tick (see the position-hold block below) so a snapshot is already
 * on hand the instant the fix drops; recomputing it after the fact would
 * need the very velocity reading that just stopped arriving. */
static float    g_last_vx_b;
static float    g_last_vy_b;
/* Fix-loss brake-pulse state (see CF21BL_BRAKE_MS above) — captured once
 * per outage from g_last_v{x,y}_b.  g_brake_t0_ms == 0 means no pulse
 * pending or in progress. */
static float    g_brake_vx0;
static float    g_brake_vy0;
static int64_t  g_brake_t0_ms;
#endif

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
static const struct device *const baro_dev =
    DEVICE_DT_GET(DT_NODELABEL(bmp388_baro));
static cf21bl_pid_t g_pid_alt_pos;   /* outer: altitude error → climb-rate setpoint */
static cf21bl_pid_t g_pid_alt_vel;   /* inner: climb-rate error → thrust correction */
#endif

/* ── Stabilizer thread ──────────────────────────────────────────────────────── */

K_THREAD_STACK_DEFINE(g_stab_stack, CF21BL_STAB_STACK_SIZE);
static struct k_thread g_stab_thread;

static void stabilizer_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /* Consume the first sample; the IMU may hold stale data from before
     * the trigger was armed, which would produce a spurious derivative spike. */
    cf21bl_imu_sample_t sample;
    cf21bl_imu_read(&sample);

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
    /* Record home altitude: average 50 BMP388 readings (~1 s at 50 Hz).
     * IMU interrupt fires normally during this period; semaphore accumulates
     * and is drained on the first main-loop iteration. */
    float p_home = 0.0f;
    for (int n = 0; n < 50; n++) {
        sensor_sample_fetch(baro_dev);
        struct sensor_value sv;
        sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &sv);
        p_home += sensor_value_to_float(&sv) * 1000.0f;  /* kPa → Pa */
        k_msleep(20);
    }
    p_home /= 50.0f;
    LOG_INF("Baro home: %.1f Pa", (double)p_home);
    float alt_est   = 0.0f;   /* world-frame altitude above home, m (complementary filter) */
    float vel_est   = 0.0f;   /* world-frame vertical velocity, m/s (complementary filter) */
    float alt_baro  = 0.0f;   /* last raw (unfiltered) baro reading, m — logged for diagnosis */
    int   baro_cnt  = 0;
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
    float alt_lh    = 0.0f;   /* last lighthouse-derived altitude above home, m — logged for diagnosis */
#endif
#endif

    while (true) {
        if (cf21bl_imu_read(&sample) != 0) {
            continue;
        }

        substrate_twist_t sp;
#if CONFIG_CF21BL_SP_STALE_MS > 0
        int64_t sp_age_ms;
#endif
        {
            k_spinlock_key_t key = k_spin_lock(&g_sp_lock);
            sp = g_setpoint;
#if CONFIG_CF21BL_SP_STALE_MS > 0
            sp_age_ms = k_uptime_get() - g_sp_last_ms;
#endif
            k_spin_unlock(&g_sp_lock, key);
        }

        /* ── Forced-landing triggers ──────────────────────────────────────── */
        /* Stale setpoints and critical battery both: level out (zero angular
         * setpoints + lateral feedforward), then bring the drone down.
         * With CONFIG_CF21BL_ALTITUDE_HOLD the descent is closed-loop: the
         * force_land flag makes the altitude section walk its target down
         * from the measured alt_est and cut only after ground settle.
         * Without it, fall back to an open-loop timed collective ramp. */
#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
        bool force_land = false;
#endif

#if CONFIG_CF21BL_SP_STALE_MS > 0
        if (sp_age_ms > CF21BL_SP_STALE_MS && sp.linear.z > CF21BL_IDLE_SP_Z) {
            static int stale_log_div;
            if (++stale_log_div >= 200) {   /* log at ~5 Hz, not 1 kHz */
                stale_log_div = 0;
                LOG_WRN("setpoint stale (%lld ms) — leveling + descending",
                        (long long)sp_age_ms);
            }
            sp.angular.x = 0.0f; sp.angular.y = 0.0f; sp.angular.z = 0.0f;
            sp.linear.x  = 0.0f; sp.linear.y  = 0.0f;

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
            force_land = true;
#else
            /* Open-loop: ramp collective to idle across STALE→CUTOFF
             * (mirrors stock COMMANDER_WDT stabilize/shutdown windows). */
            int64_t window = CF21BL_SP_CUTOFF_MS - CF21BL_SP_STALE_MS;
            float ramp = 0.0f;
            if (window > 0 && sp_age_ms < CF21BL_SP_CUTOFF_MS) {
                ramp = 1.0f - (float)(sp_age_ms - CF21BL_SP_STALE_MS)
                            / (float)window;
            }
            sp.linear.z = -1.0f + (sp.linear.z + 1.0f) * ramp;
#endif
        }
#endif

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
        /* Application-requested landing.  Unlike the two failure triggers
         * below, the lateral setpoint is deliberately left alone: the
         * application is still commanding WHERE to come down (this demo
         * holds its latched land-in-place point), and only the vertical
         * profile is handed over. */
        if (g_land_requested && sp.linear.z > CF21BL_IDLE_SP_Z) {
            force_land = true;
        }
#endif

#ifdef CONFIG_CF21BL_PM
        if (cf21bl_pm_battery_critical() && sp.linear.z > CF21BL_IDLE_SP_Z) {
            static bool crit_logged;
            if (!crit_logged) {
                crit_logged = true;
                LOG_ERR("battery critical (%.2f V) — forced landing",
                        (double)cf21bl_pm_vbat());
            }
            sp.angular.x = 0.0f; sp.angular.y = 0.0f; sp.angular.z = 0.0f;
            sp.linear.x  = 0.0f; sp.linear.y  = 0.0f;

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
            force_land = true;
#else
            static int64_t crit_t0_ms;
            if (crit_t0_ms == 0) {
                crit_t0_ms = k_uptime_get();
            }
            int64_t el = k_uptime_get() - crit_t0_ms;
            float ramp = 0.0f;
            if (el < CF21BL_PM_LAND_MS) {
                ramp = 1.0f - (float)el / (float)CF21BL_PM_LAND_MS;
            }
            sp.linear.z = -1.0f + (sp.linear.z + 1.0f) * ramp;
#endif
        } else if (cf21bl_pm_battery_low()) {
            static int low_log_div;
            if (++low_log_div >= 5000) {   /* ~0.2 Hz */
                low_log_div = 0;
                LOG_WRN("battery low: %.2f V", (double)cf21bl_pm_vbat());
            }
        }
#endif

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
        /* Touchdown latch: hold idle while the landing trigger persists.
         * A recovered link (trigger gone) releases the latch — the
         * application's commands apply again, same as the pre-landing
         * watchdog semantics.  Critical battery never releases. */
        if (g_landed) {
            if (force_land) {
                sp.linear.z = -1.0f;
            } else {
                g_landed = false;
            }
        }
#endif

        /* ── WS2: Tumble lockout ───────────────────────────────────────────── */
        /* Force idle for the rest of this loop iteration and all subsequent
         * ones — the is_idle block below then resets integrators automatically. */
        if (g_tumbled) { sp.linear.z = -1.0f; }

        float roll_rate_sp, pitch_rate_sp;
        float yaw_rate_sp = sp.angular.z * CF21BL_MAX_YAW_RATE_RPS;

#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
        /*
         * ── Position hold (outermost loop) ────────────────────────────────
         *
         * When a lighthouse fix is available, replace linear.x/y (which the
         * caller treats as velocity feedforward in angle mode) with an angle
         * correction derived from position error in the world frame.
         *
         * linear.x/y ∈ [-1,+1] → XY position setpoint ∈ ±CF21BL_POS_MAX_M
         * relative to the home position captured at first valid fix.
         *
         * The position loop is a P controller: position error [m] → angle
         * correction [rad].  The correction is added to the angular setpoint
         * so the angle loop drives the error to zero.
         *
         * Axis sign convention (matching mix.h):
         *   forward motion (+X world) requires negative pitch → subtract from
         *   pitch_sp_deg below.
         *   left motion    (+Y world) requires negative roll  → subtract from
         *   roll_sp_deg below.
         * So we negate the position correction before adding to the angle sp.
         *
         * No I or D term here: the angle/rate loops below already integrate
         * the position error implicitly (cascaded loops).  Adding integral at
         * the position level causes windup during the initial transient.
         */
        float pos_pitch_correction_deg = 0.0f;
        float pos_roll_correction_deg  = 0.0f;

        /* Fetch ONCE and branch on the result, rather than asking
         * is_valid() and then reading.  Validity is now time-based (see
         * LH2_POS_STALE_MS), so those two calls can straddle the staleness
         * boundary — and the read would then leave lhpos untouched while
         * the caller believed it held a position.  Zero-init and a checked
         * return close that. */
        lh2_position_t lhpos  = { 0 };
        bool           lh_ok  = (cf21bl_lighthouse_get_position(&lhpos) == 0);

        if (lh_ok && sp.linear.z > CF21BL_IDLE_SP_Z) {

            if (!g_pos_home_set) {
                g_pos_home_x   = lhpos.x;
                g_pos_home_y   = lhpos.y;
                g_pos_home_z   = lhpos.z;
                g_pos_home_set = true;
            }
            g_lh_lost_logged = false;   /* re-arm the once-per-outage warning */

            /* Liftoff detection for the Mahony two-stage accel gain: keep
             * full gain through arm/spin-up/ramp (vibration disturbances
             * re-level in ~2.5 s), drop to the slow airborne gain only
             * once genuinely flying. */
            if (!g_airborne && (lhpos.z - g_pos_home_z) > 0.15f) {
                g_airborne = true;
                cf21bl_imu_set_airborne(true);
                LOG_INF("airborne — Mahony accel gain -> flight value");
            }

            float sp_x = sp.linear.x * CF21BL_POS_MAX_M;
            float sp_y = sp.linear.y * CF21BL_POS_MAX_M;

            float ex = (g_pos_home_x + sp_x) - lhpos.x;
            float ey = (g_pos_home_y + sp_y) - lhpos.y;

            /* Saturate the position error rather than skipping large ones:
             * an earlier version gated out errors > 0.75 m entirely, which
             * silenced position hold exactly when the drone had drifted
             * furthest (2026-07-05 flight: drift past 0.75 m → corrections
             * stopped → runaway off the tracking volume).  Ghost fixes are
             * now rejected upstream (driver jump gate + miss-distance
             * gate), so a large error here is real — push back at the
             * saturated rate. */
            if (ex >  0.75f) { ex =  0.75f; }
            if (ex < -0.75f) { ex = -0.75f; }
            if (ey >  0.75f) { ey =  0.75f; }
            if (ey < -0.75f) { ey = -0.75f; }
            {
                /* PID correction in radians, converted to degrees for the
                 * angle loop.  The velocity term (see CF21BL_POS_KD) damps
                 * the otherwise-undamped P loop; the slow integrator (see
                 * CF21BL_POS_KI) trims the level-bias standing offset. */
                lh2_position_t lhvel = { 0 };
                (void)cf21bl_lighthouse_get_velocity(&lhvel);

                /* Rotate world-frame error/velocity into the body-aligned
                 * control frame by the current Mahony yaw (boot-relative).
                 * Under YAW_HOLD this is ≈0 for the whole flight; it
                 * matters when yaw drifts or is later commanded.  NOTE:
                 * Mahony yaw has no absolute reference, so the boot
                 * placement requirement (nose along world +X at power-on)
                 * STILL STANDS — this only keeps corrections mapped
                 * correctly if the heading moves after boot. */
                float psi  = g_yaw_now_deg * ((float)M_PI / 180.0f);
                float cpsi = cosf(psi), spsi = sinf(psi);
                float ex_b =  cpsi * ex      + spsi * ey;
                float ey_b = -spsi * ex      + cpsi * ey;
                float vx_b =  cpsi * lhvel.x + spsi * lhvel.y;
                float vy_b = -spsi * lhvel.x + cpsi * lhvel.y;

                /* Kept current every live tick — see the fix-loss braking
                 * block comment above for why this needs to already be a
                 * tick old rather than computed after the fact. */
                g_last_vx_b = vx_b;
                g_last_vy_b = vy_b;

                /* Rate-capped trim winding (see CF21BL_POS_KI_CLAMP_M) */
                float exi = ex_b, eyi = ey_b;
                if (exi >  CF21BL_POS_KI_CLAMP_M) { exi =  CF21BL_POS_KI_CLAMP_M; }
                if (exi < -CF21BL_POS_KI_CLAMP_M) { exi = -CF21BL_POS_KI_CLAMP_M; }
                if (eyi >  CF21BL_POS_KI_CLAMP_M) { eyi =  CF21BL_POS_KI_CLAMP_M; }
                if (eyi < -CF21BL_POS_KI_CLAMP_M) { eyi = -CF21BL_POS_KI_CLAMP_M; }
                g_pos_ix += CF21BL_POS_KI * exi * CF21BL_LOOP_DT;
                g_pos_iy += CF21BL_POS_KI * eyi * CF21BL_LOOP_DT;
                if (g_pos_ix >  CF21BL_POS_ILIM_RAD) { g_pos_ix =  CF21BL_POS_ILIM_RAD; }
                if (g_pos_ix < -CF21BL_POS_ILIM_RAD) { g_pos_ix = -CF21BL_POS_ILIM_RAD; }
                if (g_pos_iy >  CF21BL_POS_ILIM_RAD) { g_pos_iy =  CF21BL_POS_ILIM_RAD; }
                if (g_pos_iy < -CF21BL_POS_ILIM_RAD) { g_pos_iy = -CF21BL_POS_ILIM_RAD; }

                float cx_rad = CF21BL_POS_KP * ex_b + g_pos_ix
                             - CF21BL_POS_KD * vx_b;
                float cy_rad = CF21BL_POS_KP * ey_b + g_pos_iy
                             - CF21BL_POS_KD * vy_b;

                if (cx_rad >  CF21BL_POS_OLIM_RAD) { cx_rad =  CF21BL_POS_OLIM_RAD; }
                if (cx_rad < -CF21BL_POS_OLIM_RAD) { cx_rad = -CF21BL_POS_OLIM_RAD; }
                if (cy_rad >  CF21BL_POS_OLIM_RAD) { cy_rad =  CF21BL_POS_OLIM_RAD; }
                if (cy_rad < -CF21BL_POS_OLIM_RAD) { cy_rad = -CF21BL_POS_OLIM_RAD; }

                /* NEGATED here per the "Axis sign convention" comment above
                 * this block: forward (+X) motion requires negative pitch,
                 * left (+Y) motion requires negative roll.  cx_rad/cy_rad
                 * are positive when MORE +X/+Y motion is needed (ex_b/ey_b
                 * positive), so the correction must flip sign before being
                 * added to pitch_sp_deg/roll_sp_deg below — this negation
                 * was missing (found 2026-07-06 after extensive independent
                 * ruling-out of calibration, hardware, yaw, and lighthouse-
                 * sensor causes for a reproducible position-hold runaway;
                 * the sibling non-POS_HOLD branch two blocks down already
                 * correctly subtracts sp.linear.x/y, confirming this is the
                 * intended convention everywhere else it's applied). */
                pos_pitch_correction_deg = -cx_rad * (180.0f / (float)M_PI);
                pos_roll_correction_deg  = -cy_rad * (180.0f / (float)M_PI);

                static int pos_log_div;
                if (++pos_log_div >= 500) {
                    pos_log_div = 0;
                    /* yaw added 2026-07-06: altitude-hold-tether (no
                     * LIGHTHOUSE_POS_HOLD, no YAW_HOLD) flew stable on both
                     * "bad" drones, isolating the problem to this subsystem
                     * — but this log line never showed whether the Mahony
                     * yaw estimate (boot-relative, no absolute reference,
                     * used a few lines above to rotate ex/ey/vx/vy into
                     * body frame) drifts during flight.  A large in-flight
                     * yaw drift would misdirect every position correction
                     * without touching roll/pitch self-leveling or
                     * altitude at all — invisible to the altitude-hold-
                     * tether test, and invisible to every prior flight log
                     * since yaw was never logged here before now. */
                    LOG_INF("pos x=%.3f y=%.3f z=%.3f ex=%.3f ey=%.3f vx=%.3f vy=%.3f ix=%.2f iy=%.2f yaw=%.1f",
                            (double)lhpos.x, (double)lhpos.y, (double)lhpos.z,
                            (double)ex, (double)ey,
                            (double)lhvel.x, (double)lhvel.y,
                            (double)(g_pos_ix * 180.0f / (float)M_PI),
                            (double)(g_pos_iy * 180.0f / (float)M_PI),
                            (double)g_yaw_now_deg);
                }
            }
        } else if (g_pos_home_set && !lh_ok) {
            /* Fix lost mid-flight: log once, brake, then hold level.
             *
             * With CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD (required to reach
             * this branch — see the Kconfig `depends on`)
             * CF21BL_MAX_FWD_TILT_DEG is hardwired to 0.0f below, so
             * sp.linear.x/y contribute no tilt either — there is no
             * velocity feedforward path to "fall back" to in this
             * configuration.  roll_sp_deg/pitch_sp_deg resolve to whatever
             * sp.angular.x/y alone commands (0 for every caller in this
             * tree) PLUS pos_pitch/roll_correction_deg, which this branch
             * now sets for CF21BL_BRAKE_MS after the loss (see that
             * constant's block comment) rather than leaving at their
             * 0.0f init the whole outage.  Past the pulse it IS pure
             * level — stop accelerating and coast on drag, not steer
             * using a last-known heading — which is what an earlier
             * version of this branch did unconditionally, and what an
             * even earlier version of this log line called "feedforward"
             * without actually doing it.  A pulse alone does not make
             * this an active hold: there is still no position or velocity
             * feedback once blind, only a best-effort correction for the
             * velocity that existed at the moment the fix dropped.
             *
             * The integrator is kept: the trim is body-frame level bias,
             * not home-relative, so it stays valid across re-acquisition
             * (and holding the learned tilt during the outage beats
             * reverting to the biased level).
             *
             * g_pos_home_set is deliberately NOT cleared here.  It used to
             * be — as the once-per-outage latch for the warning above —
             * and the side effect was severe: home was re-captured by the
             * block above on the NEXT valid fix, i.e. re-latched to
             * wherever the drone had drifted to during the outage.  Since
             * the whole position loop is home-relative (ex/ey below), that
             * silently re-origined the control frame on every dropout, and
             * every commanded position afterwards inherited the offset.
             * cf21bl_stabilizer_get_pos_home() feeds the demo's
             * return-to-home too, so a drone could fly "home" to a point
             * it had never taken off from (2026-08-31 flight 42: RTH ~0.4 m
             * off, after a single dropout during the altitude ramp).
             *
             * Home is a point in the lighthouse WORLD frame, and that frame
             * does not move when the fix drops — the reference stays valid
             * across an outage by construction.  It is released when the
             * drone returns to idle (see the is_idle block below), so the
             * next takeoff captures a fresh one. */
            if (!g_lh_lost_logged) {
                g_brake_vx0   = g_last_vx_b;
                g_brake_vy0   = g_last_vy_b;
                g_brake_t0_ms = k_uptime_get();
                LOG_WRN("LH2 fix lost — braking (vx=%.2f vy=%.2f m/s) then "
                        "holding level", (double)g_brake_vx0,
                        (double)g_brake_vy0);
                g_lh_lost_logged = true;
            }

            /* See CF21BL_BRAKE_MS above: a short, decaying, D-term-only
             * pulse from the captured velocity, then pure level (0,0) —
             * pos_pitch/roll_correction_deg are already 0.0f from their
             * declaration above and this simply leaves them there once
             * the pulse completes. */
            if (g_brake_t0_ms != 0) {
                int64_t elapsed = k_uptime_get() - g_brake_t0_ms;
                if (elapsed < (int64_t)CF21BL_BRAKE_MS) {
                    float decay = 1.0f - (float)elapsed / (float)CF21BL_BRAKE_MS;
                    float cx_rad = -CF21BL_POS_KD * (g_brake_vx0 * decay);
                    float cy_rad = -CF21BL_POS_KD * (g_brake_vy0 * decay);
                    if (cx_rad >  CF21BL_POS_OLIM_RAD) { cx_rad =  CF21BL_POS_OLIM_RAD; }
                    if (cx_rad < -CF21BL_POS_OLIM_RAD) { cx_rad = -CF21BL_POS_OLIM_RAD; }
                    if (cy_rad >  CF21BL_POS_OLIM_RAD) { cy_rad =  CF21BL_POS_OLIM_RAD; }
                    if (cy_rad < -CF21BL_POS_OLIM_RAD) { cy_rad = -CF21BL_POS_OLIM_RAD; }
                    /* Same negation as the live position loop's D term —
                     * see the "Axis sign convention" / 2026-07-06 comment
                     * near the top of this section. */
                    pos_pitch_correction_deg = -cx_rad * (180.0f / (float)M_PI);
                    pos_roll_correction_deg  = -cy_rad * (180.0f / (float)M_PI);
                } else {
                    g_brake_t0_ms = 0;   /* pulse complete — pure level for the rest */
                }
            }
        }
#endif /* CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD */

#if defined(CONFIG_CF21BL_ANGLE_MODE) || defined(CONFIG_CF21BL_ALTITUDE_HOLD)
        /* Needed for the angle loop (roll_deg/pitch_deg) below and/or for the
         * altitude-hold velocity estimator (accel_up_g), so run it whenever
         * either feature is enabled — not just in angle mode. */
        cf21bl_imu_attitude_t att;
        cf21bl_imu_filter_update(&sample, CF21BL_LOOP_DT, &att);
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
        g_yaw_now_deg = att.yaw_deg;   /* consumed by pos-hold next tick */
#endif
#endif

#ifdef CONFIG_CF21BL_ANGLE_MODE
        /* ── Outer angle loop ─────────────────────────────────────────────── */
        /* Velocity feedforward or position correction (see position hold above).
         * Sign convention from mix.h:
         *   P = angular.y - linear.x → forward (linear.x>0) → pitch negative
         *   R = angular.x - linear.y → left    (linear.y>0) → roll  negative
         *
         * When position hold is active, pos_*_correction_deg supersedes the
         * feedforward tilt so we do not double-count linear.x/y. */
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
#define CF21BL_MAX_FWD_TILT_DEG  0.0f   /* position loop handles lateral motion */
        float roll_sp_deg  = sp.angular.x * CF21BL_MAX_ANGLE_DEG
                           + pos_roll_correction_deg;
        float pitch_sp_deg = sp.angular.y * CF21BL_MAX_ANGLE_DEG
                           + pos_pitch_correction_deg;
#else
#define CF21BL_MAX_FWD_TILT_DEG  10.0f
        float roll_sp_deg  = sp.angular.x * CF21BL_MAX_ANGLE_DEG
                           - sp.linear.y  * CF21BL_MAX_FWD_TILT_DEG;
        float pitch_sp_deg = sp.angular.y * CF21BL_MAX_ANGLE_DEG
                           - sp.linear.x  * CF21BL_MAX_FWD_TILT_DEG;
#endif /* CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD */

        roll_rate_sp  = pid_update(&g_pid_roll_angle,
                                   roll_sp_deg,  att.roll_deg,  CF21BL_LOOP_DT);
        pitch_rate_sp = pid_update(&g_pid_pitch_angle,
                                   pitch_sp_deg, att.pitch_deg, CF21BL_LOOP_DT);

#ifdef CONFIG_CF21BL_YAW_HOLD
        /* ── Yaw heading hold ─────────────────────────────────────────────
         * Integrate the commanded yaw rate into a heading target and close a
         * PID around the Mahony yaw estimate (stock controller_pid.c does the
         * same for modeVelocity yaw setpoints).  The target is kept within
         * ±CF21BL_YAWH_MAX_DELTA_DEG of the current heading so it cannot run
         * far ahead if the drone is physically prevented from turning. */
        g_yaw_target_deg = wrap180f(g_yaw_target_deg
                                    + sp.angular.z * CF21BL_MAX_YAW_RATE_RPS
                                      * (180.0f / (float)M_PI) * CF21BL_LOOP_DT);
        float yaw_err_deg = wrap180f(g_yaw_target_deg - att.yaw_deg);
        if (yaw_err_deg > CF21BL_YAWH_MAX_DELTA_DEG) {
            yaw_err_deg = CF21BL_YAWH_MAX_DELTA_DEG;
            g_yaw_target_deg = wrap180f(att.yaw_deg + yaw_err_deg);
        } else if (yaw_err_deg < -CF21BL_YAWH_MAX_DELTA_DEG) {
            yaw_err_deg = -CF21BL_YAWH_MAX_DELTA_DEG;
            g_yaw_target_deg = wrap180f(att.yaw_deg + yaw_err_deg);
        }

        g_yawh_integral += yaw_err_deg * CF21BL_LOOP_DT;
        float yawh_i_max = CF21BL_YAWH_ILIM / CF21BL_YAWH_KI;
        if      (g_yawh_integral >  yawh_i_max) { g_yawh_integral =  yawh_i_max; }
        else if (g_yawh_integral < -yawh_i_max) { g_yawh_integral = -yawh_i_max; }

        /* D on the measured yaw rate (deg/s), not the error — no target kick */
        yaw_rate_sp = CF21BL_YAWH_KP * yaw_err_deg
                    + CF21BL_YAWH_KI * g_yawh_integral
                    - CF21BL_YAWH_KD * sample.gyro_rps[2] * (180.0f / (float)M_PI);
        if      (yaw_rate_sp >  CF21BL_MAX_YAW_RATE_RPS) { yaw_rate_sp =  CF21BL_MAX_YAW_RATE_RPS; }
        else if (yaw_rate_sp < -CF21BL_MAX_YAW_RATE_RPS) { yaw_rate_sp = -CF21BL_MAX_YAW_RATE_RPS; }
#endif /* CONFIG_CF21BL_YAW_HOLD */
#else
        /* ── Rate mode: direct rate setpoint ─────────────────────────────── */
        roll_rate_sp  = sp.angular.x * CF21BL_MAX_RATE_RPS;
        pitch_rate_sp = sp.angular.y * CF21BL_MAX_RATE_RPS;
#endif

        /* ── Inner rate loop ──────────────────────────────────────────────── */
        float u_roll  = pid_update(&g_pid_roll,
                                   roll_rate_sp,  sample.gyro_rps[0],
                                   CF21BL_LOOP_DT);
        float u_pitch = pid_update(&g_pid_pitch,
                                   pitch_rate_sp, sample.gyro_rps[1],
                                   CF21BL_LOOP_DT);
        float u_yaw   = pid_update(&g_pid_yaw,
                                   yaw_rate_sp,   sample.gyro_rps[2],
                                   CF21BL_LOOP_DT);

        bool is_idle = (sp.linear.z < CF21BL_IDLE_SP_Z);
        if (is_idle) {
            /* Idle: motors must sit at pure minimum, not react to tilt/noise
             * while armed-but-grounded. Zero the commanded correction and
             * reset every rate/angle integrator so no windup carries into
             * the next takeoff — mirrors CF stock controllerPid()'s
             * thrust==0 branch (controller_pid.c), which zeros roll/pitch/
             * yaw and resets all attitude PIDs whenever thrust is zero. */
            u_roll = 0.0f;
            u_pitch = 0.0f;
            u_yaw = 0.0f;
            pid_reset(&g_pid_roll,  sample.gyro_rps[0]);
            pid_reset(&g_pid_pitch, sample.gyro_rps[1]);
            pid_reset(&g_pid_yaw,   sample.gyro_rps[2]);
#ifdef CONFIG_CF21BL_ANGLE_MODE
            pid_reset(&g_pid_roll_angle,  att.roll_deg);
            pid_reset(&g_pid_pitch_angle, att.pitch_deg);
#endif
            /* NOTE: g_pos_ix/iy deliberately NOT reset here — the trim is
             * a physical constant of the airframe (IMU level bias), so the
             * value learned in flight stays valid across landings.  Every
             * takeoff after the first starts pre-trimmed instead of
             * re-learning the bias airborne (the untrimmed first ~10 s
             * caused a 1.5 m departure on a fresh pack, 2026-07-05).
             * Reset only at boot (static zero-init). */
            /* Back on the ground: restore the fast Mahony accel gain so
             * the level reference re-converges before the next takeoff. */
            g_airborne = false;
            cf21bl_imu_set_airborne(false);
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
            /* Release the home reference here — at the takeoff/landing
             * boundary — rather than on a fix dropout (see the fix-lost
             * branch above).  The next non-idle tick with a valid fix
             * captures a fresh home, which is the behavior the original
             * code intended; what it actually did was re-capture mid-air. */
            g_pos_home_set   = false;
            g_lh_lost_logged = false;
            g_brake_t0_ms    = 0;   /* discard any pulse mid-flight left running */
#endif
#ifdef CONFIG_CF21BL_YAW_HOLD
            /* Re-seed the heading target at the current yaw so takeoff never
             * starts with a stale heading error. */
            g_yaw_target_deg = att.yaw_deg;
            g_yawh_integral  = 0.0f;
#endif
        }

        /* ── WS2: Tumble detection ─────────────────────────────────────────── */
        /* Count consecutive samples where body-Z accel is too low to be level
         * flight.  Clear the counter when idle (on ground, tilt doesn't matter).
         * Once the count exceeds the threshold, latch g_tumbled and log once.
         * The lockout at the top of this loop then forces idle indefinitely. */
        if (is_idle) {
            g_tumble_count = 0;
        } else if (!g_tumbled) {
            if (sample.accel_g[2] < CF21BL_TUMBLE_ACCEL_Z_G) {
                if (++g_tumble_count >= CF21BL_TUMBLE_COUNT) {
                    LOG_ERR("tumble: accel_z=%.2f g — cutting motors (power-cycle to reset)",
                            (double)sample.accel_g[2]);
                    g_tumbled = true;
                }
            } else {
                g_tumble_count = 0;
            }
        }

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
        float linear_z_out;
        if (is_idle) {
            /* Idle sentinel: disarm both altitude PID stages and the
             * complementary-filter estimator so nothing winds up or drifts
             * while the drone sits on the ground. */
            pid_reset(&g_pid_alt_pos, 0.0f);
            pid_reset(&g_pid_alt_vel, 0.0f);
            alt_est   = 0.0f;
            vel_est   = 0.0f;
            alt_baro  = 0.0f;
            baro_cnt  = 0;
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
            alt_lh    = 0.0f;
#endif
            g_land_t0_ms = 0;   /* abandon any in-progress descent state */
            linear_z_out = -1.0f;
        } else {
            /* Integrate world-frame vertical acceleration every 1 kHz tick so
             * the velocity/altitude estimate tracks fast dynamics between the
             * much slower barometer samples. att.accel_up_g reads +1 g when
             * level and stationary at any tilt, so subtracting 1 g and
             * scaling by g isolates net vertical acceleration. */
            float net_az_ms2 = (att.accel_up_g - 1.0f) * CF21BL_G_MPS2;
            vel_est += net_az_ms2 * CF21BL_LOOP_DT;
            alt_est += vel_est * CF21BL_LOOP_DT;

            /* Liftoff detection for the Mahony two-stage accel gain
             * (baro path; the lighthouse path does the same above). */
            if (!g_airborne && alt_est > 0.15f) {
                g_airborne = true;
                cf21bl_imu_set_airborne(true);
                LOG_INF("airborne — Mahony accel gain -> flight value");
            }

            /* Poll BMP388 every CF21BL_BARO_POLL_DIV iterations (~50 Hz) and
             * correct both filter states from the resulting position
             * innovation. sensor_sample_fetch blocks ~100 µs for the I2C
             * transaction; I2C3 bus is idle at this point (BMI088 RTIO
             * completes before the drdy semaphore is posted, so no bus
             * contention). */
            if (++baro_cnt >= CF21BL_BARO_POLL_DIV) {
                baro_cnt = 0;
                sensor_sample_fetch(baro_dev);
                struct sensor_value sv;
                sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &sv);
                float p_Pa = sensor_value_to_float(&sv) * 1000.0f;
                alt_baro   = (p_home - p_Pa) / CF21BL_PA_PER_M;
                float baro_err = alt_baro - alt_est;
                /* Drop a single-sample reading that implies an implausible
                 * jump for one ~20 ms tick instead of feeding a probably
                 * torn/corrupted I2C read into both filter states. */
                if (fabsf(baro_err) <= CF21BL_ALT_OUTLIER_M) {
                    alt_est += CF21BL_ALT_BARO_KP * baro_err;
                    vel_est += CF21BL_VEL_BARO_KP * baro_err;
                }

#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
                /* Second, independent altitude correction (see
                 * CF21BL_ALT_LH_KP above) — same poll cadence, same
                 * single-sample outlier gate as baro, just a different
                 * measurement of the same alt_est/vel_est pair. */
                if (g_pos_home_set && cf21bl_lighthouse_is_valid()) {
                    lh2_position_t lhpos_alt;
                    if (cf21bl_lighthouse_get_position(&lhpos_alt) == 0) {
                        alt_lh = lhpos_alt.z - g_pos_home_z;
                        float lh_err = alt_lh - alt_est;
                        if (fabsf(lh_err) <= CF21BL_ALT_OUTLIER_M) {
                            alt_est += CF21BL_ALT_LH_KP * lh_err;
                            vel_est += CF21BL_VEL_LH_KP * lh_err;
                        }
                    }
                }
#endif
            }

            /* ── Cascaded position → velocity → thrust ────────────────────
             * Outer PID turns altitude error into a climb-rate setpoint;
             * inner PID turns climb-rate error (against the fast accel-
             * fused vel_est) into the collective correction. The inner
             * stage is the velocity damping the old single-stage
             * baro-only controller lacked. */
            float target_alt = sp.linear.z + CF21BL_ALT_SP_OFFSET;

            /* ── Forced landing: closed-loop descent to touchdown ─────────
             * Walk the target down from the altitude measured at trigger
             * time, UNCONDITIONALLY — see the CF21BL_LAND_* block comment
             * above for why this no longer stops at a ground-relative
             * altitude of 0.  Latch g_landed once alt_est has stopped
             * following the descending target for CF21BL_LAND_SETTLE_MS —
             * the loop top forces idle from the next iteration on. */
            if (force_land && !g_landed) {
                int64_t now_ms = k_uptime_get();
                if (g_land_t0_ms == 0) {
                    g_land_t0_ms     = now_ms;
                    g_land_alt0      = (alt_est > 0.0f) ? alt_est : 0.0f;
                    g_land_ground_ms = 0;
                    g_land_alt_ref   = g_land_alt0;
                    g_land_ref_ms    = now_ms;
                    LOG_WRN("forced landing: descending from %.2f m",
                            (double)g_land_alt0);
                }
                float down = CF21BL_LAND_RATE_MPS
                             * (float)(now_ms - g_land_t0_ms) / 1000.0f;
                target_alt = g_land_alt0 - down;
                /* Defensive floor only, not the touchdown criterion below —
                 * keeps the target from running away to an absurd value if
                 * the stall check somehow never latches.  main.c's own
                 * LAND_FORCE_DISARM_MS backstop (an independent, measured-
                 * lighthouse check) covers that case regardless of what
                 * happens here. */
                if (target_alt < CF21BL_LAND_ABS_FLOOR_M) {
                    target_alt = CF21BL_LAND_ABS_FLOOR_M;
                }

                if (now_ms - g_land_ref_ms >= (int64_t)CF21BL_LAND_STALL_WINDOW_MS) {
                    float descended = g_land_alt_ref - alt_est;
                    bool  stalled    = descended < CF21BL_LAND_STALL_EPS_M;
                    g_land_alt_ref = alt_est;
                    g_land_ref_ms  = now_ms;

                    if (stalled) {
                        if (g_land_ground_ms == 0) {
                            g_land_ground_ms = now_ms;
                        }
                    } else {
                        /* Real descent this window — not resting on
                         * anything yet, whatever a prior window thought. */
                        g_land_ground_ms = 0;
                    }
                }

                if (g_land_ground_ms != 0 &&
                    now_ms - g_land_ground_ms > CF21BL_LAND_SETTLE_MS) {
                    g_landed = true;
                    LOG_WRN("forced landing: touchdown — motors off "
                            "(alt %.2f m)", (double)alt_est);
                }
            } else if (!force_land) {
                g_land_t0_ms = 0;   /* trigger cleared before touchdown */
            }

            /* Freeze-above-threshold anti-windup (see CF21BL_ALT_POS_KI_FREEZE_M):
             * pid_update() already accumulated this tick's integral by the
             * time it returns, so roll it back when the gap is too large to
             * trust as steady-state bias. One tick's worth of accumulation
             * (dt=1ms) is negligible either way — this only matters over the
             * many consecutive ticks of a real ramp. */
            float alt_integral_before = g_pid_alt_pos.integral;
            float target_vz  = pid_update(&g_pid_alt_pos,
                                          target_alt, alt_est, CF21BL_LOOP_DT);
            if (fabsf(target_alt - alt_est) > CF21BL_ALT_POS_KI_FREEZE_M) {
                g_pid_alt_pos.integral = alt_integral_before;
            }
            float delta_T    = pid_update(&g_pid_alt_vel,
                                          target_vz, vel_est, CF21BL_LOOP_DT);

            float T_cmd = CF21BL_HOVER_T + delta_T;
            if (T_cmd < CF21BL_T_FLOOR) { T_cmd = CF21BL_T_FLOOR; }
            if (T_cmd > 1.0f)           { T_cmd = 1.0f; }
            linear_z_out = T_cmd * 2.0f - 1.0f;   /* [0,1] → [-1,+1] for mix */

            /* Log at ~2 Hz so progress is visible on console. */
            static int alt_log_div;
            if (++alt_log_div >= 500) {
                alt_log_div = 0;
#if defined(CONFIG_CF21BL_PM) && defined(CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD)
                LOG_INF("alt=%.3f m  raw=%.3f m  lh=%.3f m  vz=%.3f m/s  target=%.3f m  vz_sp=%.3f  T=%.2f  vbat=%.2f",
                        (double)alt_est, (double)alt_baro, (double)alt_lh, (double)vel_est,
                        (double)target_alt, (double)target_vz, (double)T_cmd,
                        (double)cf21bl_pm_vbat());
#elif defined(CONFIG_CF21BL_PM)
                LOG_INF("alt=%.3f m  raw=%.3f m  vz=%.3f m/s  target=%.3f m  vz_sp=%.3f  T=%.2f  vbat=%.2f",
                        (double)alt_est, (double)alt_baro, (double)vel_est,
                        (double)target_alt, (double)target_vz, (double)T_cmd,
                        (double)cf21bl_pm_vbat());
#elif defined(CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD)
                LOG_INF("alt=%.3f m  raw=%.3f m  lh=%.3f m  vz=%.3f m/s  target=%.3f m  vz_sp=%.3f  T=%.2f",
                        (double)alt_est, (double)alt_baro, (double)alt_lh, (double)vel_est,
                        (double)target_alt, (double)target_vz, (double)T_cmd);
#else
                LOG_INF("alt=%.3f m  raw=%.3f m  vz=%.3f m/s  target=%.3f m  vz_sp=%.3f  T=%.2f",
                        (double)alt_est, (double)alt_baro, (double)vel_est,
                        (double)target_alt, (double)target_vz, (double)T_cmd);
#endif
            }
        }
#else
        float linear_z_out = sp.linear.z;
#endif

        substrate_twist_t out = {
            .linear  = { .x = 0.0f, .y = 0.0f, .z = linear_z_out },
            .angular = { .x = u_roll, .y = u_pitch, .z = u_yaw },
        };

        cf21bl_motors_t motors;
        cf21bl_mix(&out, &motors);

#ifdef CONFIG_CF21BL_PM_COMPENSATE
        if (!is_idle) {
            /* Battery compensation: scale all four commands by VREF/vbat so
             * delivered motor voltage (duty × pack voltage) — and thus thrust
             * and loop gain — stays constant as the pack sags.  First-order
             * equivalent of stock motorsCompensateBatteryVoltage(). */
            float bscale = cf21bl_pm_thrust_scale();
            motors.m1 *= bscale;  if (motors.m1 > 1.0f) { motors.m1 = 1.0f; }
            motors.m2 *= bscale;  if (motors.m2 > 1.0f) { motors.m2 = 1.0f; }
            motors.m3 *= bscale;  if (motors.m3 > 1.0f) { motors.m3 = 1.0f; }
            motors.m4 *= bscale;  if (motors.m4 > 1.0f) { motors.m4 = 1.0f; }
        }
#endif

        if (!is_idle) {
            /* Airborne: never let a prop stop (see CF21BL_MOTOR_FLOOR). */
            if (motors.m1 < CF21BL_MOTOR_FLOOR) { motors.m1 = CF21BL_MOTOR_FLOOR; }
            if (motors.m2 < CF21BL_MOTOR_FLOOR) { motors.m2 = CF21BL_MOTOR_FLOOR; }
            if (motors.m3 < CF21BL_MOTOR_FLOOR) { motors.m3 = CF21BL_MOTOR_FLOOR; }
            if (motors.m4 < CF21BL_MOTOR_FLOOR) { motors.m4 = CF21BL_MOTOR_FLOOR; }
        }

        cf21bl_set_motors(&motors);
    }
}

/* ── API ────────────────────────────────────────────────────────────────────── */

int cf21bl_stabilizer_start(void)
{
    int ret = cf21bl_imu_init();
    if (ret) {
        LOG_ERR("cf21bl_imu_init failed: %d", ret);
        return ret;
    }

    /* Measure static gyro bias (~1.1 s, motors at idle below spin threshold).
     * This must be done before filter init so the bias is zeroed for the
     * Mahony integral feedback, which then only needs to correct residual
     * temperature drift rather than the full static offset. */
    LOG_INF("Calibrating gyro bias — keep drone stationary on ground ...");
    cf21bl_imu_calibrate_gyro(1000);

    cf21bl_imu_filter_init();

    pid_init(&g_pid_roll,  CF21BL_RP_KP,  CF21BL_RP_KI,  CF21BL_RP_KD,
             CF21BL_RP_ILIM,  CF21BL_RP_OLIM);
    pid_init(&g_pid_pitch, CF21BL_RP_KP,  CF21BL_RP_KI,  CF21BL_RP_KD,
             CF21BL_RP_ILIM,  CF21BL_RP_OLIM);
    pid_init(&g_pid_yaw,   CF21BL_YAW_KP, CF21BL_YAW_KI, CF21BL_YAW_KD,
             CF21BL_YAW_ILIM, CF21BL_YAW_OLIM);

#ifdef CONFIG_CF21BL_ANGLE_MODE
    pid_init(&g_pid_roll_angle,  CF21BL_ANGLE_KP, CF21BL_ANGLE_KI, 0.0f,
             CF21BL_ANGLE_ILIM, CF21BL_MAX_ANGLE_RATE_RPS);
    pid_init(&g_pid_pitch_angle, CF21BL_ANGLE_KP, CF21BL_ANGLE_KI, 0.0f,
             CF21BL_ANGLE_ILIM, CF21BL_MAX_ANGLE_RATE_RPS);
#endif

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("BMP388 not ready");
        return -ENODEV;
    }
    pid_init(&g_pid_alt_pos, CF21BL_ALT_POS_KP, CF21BL_ALT_POS_KI, 0.0f,
             CF21BL_ALT_POS_ILIM, CF21BL_ALT_VEL_SP_MAX);
    pid_init(&g_pid_alt_vel, CF21BL_ALT_VEL_KP, CF21BL_ALT_VEL_KI, 0.0f,
             CF21BL_ALT_VEL_ILIM, CF21BL_ALT_VEL_OLIM);
#endif

#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
    /* Pre-seed the XY level trim from per-build constants (0.01° units,
     * default 0 = no change).  The trim otherwise starts at zero every
     * boot and learns at the KI_CLAMP-capped ~0.17°/s, so an airframe
     * with a large static bias spends its first ~10-20 s of flight
     * diving off in the bias direction while the trim catches up (drone
     * #2 solo flight 2026-07-11: 1.3 m departure with the error
     * saturated before recovery).  Seeding with values read from a
     * previous flight's ix/iy log skips that transient.
     * CAUTION: only valid while the airframe's bias is unchanged — this
     * project's problem unit changes bias direction when handled/
     * repaired, and a wrong-DIRECTION seed doubles the transient instead
     * of removing it.  Re-read ix/iy from a fresh log after any physical
     * work, and keep these at 0 for healthy airframes. */
    g_pos_ix = (float)CONFIG_CF21BL_POS_TRIM_X_CDEG * 0.01f
               * (float)M_PI / 180.0f;
    g_pos_iy = (float)CONFIG_CF21BL_POS_TRIM_Y_CDEG * 0.01f
               * (float)M_PI / 180.0f;
    if (g_pos_ix >  CF21BL_POS_ILIM_RAD) { g_pos_ix =  CF21BL_POS_ILIM_RAD; }
    if (g_pos_ix < -CF21BL_POS_ILIM_RAD) { g_pos_ix = -CF21BL_POS_ILIM_RAD; }
    if (g_pos_iy >  CF21BL_POS_ILIM_RAD) { g_pos_iy =  CF21BL_POS_ILIM_RAD; }
    if (g_pos_iy < -CF21BL_POS_ILIM_RAD) { g_pos_iy = -CF21BL_POS_ILIM_RAD; }
    if (CONFIG_CF21BL_POS_TRIM_X_CDEG != 0 || CONFIG_CF21BL_POS_TRIM_Y_CDEG != 0) {
        LOG_INF("XY trim pre-seeded: ix=%.2f iy=%.2f deg",
                (double)(g_pos_ix * 180.0f / (float)M_PI),
                (double)(g_pos_iy * 180.0f / (float)M_PI));
    }
#endif

#if CONFIG_CF21BL_SP_STALE_MS > 0
    /* Arm the watchdog clock immediately before the thread starts so the
     * gyro-calibration window (~1 s above) does not count as stale time. */
    g_sp_last_ms = k_uptime_get();
#endif

    k_thread_create(&g_stab_thread, g_stab_stack,
                    K_THREAD_STACK_SIZEOF(g_stab_stack),
                    stabilizer_fn, NULL, NULL, NULL,
                    CF21BL_STAB_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&g_stab_thread, "cf21bl_stab");

    LOG_INF("CF21 stabilizer started (%s mode)",
            IS_ENABLED(CONFIG_CF21BL_ANGLE_MODE) ? "angle" : "rate");
    return 0;
}

void cf21bl_stabilizer_set_setpoint(const substrate_twist_t *sp)
{
    k_spinlock_key_t key = k_spin_lock(&g_sp_lock);
    g_setpoint = *sp;
#if CONFIG_CF21BL_SP_STALE_MS > 0
    g_sp_last_ms = k_uptime_get();
#endif
    k_spin_unlock(&g_sp_lock, key);
}

void cf21bl_stabilizer_request_land(bool active)
{
#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
    g_land_requested = active;
#else
    /* No closed-loop altitude to walk down — see the header comment.
     * g_land_requested only exists under CONFIG_CF21BL_ALTITUDE_HOLD. */
    (void)active;
#endif
}

bool cf21bl_stabilizer_is_landed(void)
{
#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
    return g_landed;
#else
    return false;
#endif
}

bool cf21bl_stabilizer_get_pos_home(float *x, float *y)
{
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
    /* Unlocked: g_pos_home_x/y change only a few times per flight (captured
     * once per fix acquisition) and the reader tolerates a stale-by-one-tick
     * value, same tradeoff as other slowly-varying cross-thread reads in
     * this file (e.g. cf21bl_pm_vbat()). */
    if (!g_pos_home_set) {
        return false;
    }
    *x = g_pos_home_x;
    *y = g_pos_home_y;
    return true;
#else
    ARG_UNUSED(x);
    ARG_UNUSED(y);
    return false;
#endif
}
