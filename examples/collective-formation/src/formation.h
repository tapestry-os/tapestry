/*
 * formation.h — Demo: Collective Formation (L4 only)
 *
 * Spring-field repulsion/attraction over the L4 world model, with
 * dead-reckoning odometry to keep own_state.position current.
 *
 * Physical calibration:
 *   DEMO_MAX_SPEED — effective linearisation constant, NOT true 100% speed.
 *     The motor curve is non-linear: at 22% commanded speed Cutebots already
 *     do ~32% of max velocity.  Calibrate at the actual commanded speed (22%)
 *     so dead-reckoning is correct in practice:
 *       DEMO_MAX_SPEED = speed_mm_per_s_at_22pct / 0.22 * 100 / arena_mm
 *     Measured fleet average 238 mm/s at 22%, 800 mm arena (1 unit = 8 mm):
 *       238 / 0.22 * 100 / 800 = 135.0
 *     Per-robot values (compile with -DDEMO_MAX_SPEED=N):
 *       Bot 0: 120  Bot 1: 137  Bot 2: 145  Bot 3: 139
 *
 *   DEMO_WHEEL_TRACK — wheel-center-to-wheel-center in logical units.
 *     Cutebot Mini track ≈ 85 mm. In an 800 mm arena: 85/8 = 10.6.
 *
 * Formation tuning:
 *   DEMO_TARGET_SPACING — desired peer-to-peer spacing in logical units.
 *     25 units ≈ 200 mm in an 800 mm arena. Starting point — tune up to
 *     spread robots further or down to tighten the cluster.
 *
 * Override any constant at compile time:
 *   west build ... -- -DDEMO_TARGET_SPACING=40.0f
 */

#ifndef TAPESTRY_DEMO_FORMATION_H
#define TAPESTRY_DEMO_FORMATION_H

#include <stdint.h>
#include <tapestry/csm.h>
#include <tapestry/substrate.h>

/* ── Calibration defaults ───────────────────────────────────────────────── */

#ifndef DEMO_WHEEL_TRACK
#define DEMO_WHEEL_TRACK   10.6f   /* logical units, wheel-to-wheel (800 mm arena) */
#endif

#ifndef DEMO_MAX_SPEED
#define DEMO_MAX_SPEED    135.0f   /* linearisation constant at 22% commanded speed
                                     * Formula: speed_mm_per_s_at_22pct / 0.22 * 100 / arena_mm
                                     * 800 mm arena, fleet avg 238 mm/s at 22%: 238/0.22*100/800 = 135 */
#endif

/* Maximum yaw rate (rad/s) at rate_norm=1.0.
 * Although DEMO_MAX_SPEED is a low-speed linearisation constant (not true 100%
 * speed), using it here works because the motor non-linearity at 22% raises
 * effective speed by the same factor (~1.43×), so DEMO_MAX_OMEGA correctly
 * predicts physical omega for both in-place and arc turns at stiction speed.
 * Not overridable: change DEMO_MAX_SPEED or DEMO_WHEEL_TRACK instead. */
#define DEMO_MAX_OMEGA    (2.0f * DEMO_MAX_SPEED / DEMO_WHEEL_TRACK)

#ifndef DEMO_TARGET_SPACING
#define DEMO_TARGET_SPACING 50.0f  /* desired peer spacing, logical units →
                                     * equilibrium side = T × 0.854 (4-robot square geometry)
                                     * 50 units → equilibrium ≈ 43 units = 341 mm in 800 mm arena */
#endif

/* ── Dead-reckoning state ───────────────────────────────────────────────── */

typedef struct {
    float x;        /* Current position in logical world coords [0, WORLD_SIZE] */
    float y;
    float heading;  /* Radians, 0 = +x direction */
    bool  moving;   /* Hysteresis state for demo_compute_drive */
} demo_odometry_t;

/* Initialize odometry at (x, y) with heading 0 (+x direction). */
void demo_odometry_init(demo_odometry_t *odo, float x, float y);

/*
 * Update dead-reckoning estimate from the last motion command.
 *   speed_norm: forward velocity [-1.0, 1.0], passed to substrate_move().
 *   rate_norm:  yaw rate         [-1.0, 1.0], positive = CCW (turn left).
 *   Also resets odo->moving when peer count transitions from 0 → non-zero.
 *   dt_ms: elapsed milliseconds since last call (typically WM_CYCLE_MS).
 */
void demo_odometry_update(demo_odometry_t *odo,
                           float speed_norm, float rate_norm,
                           uint32_t dt_ms);

/* ── Formation control ──────────────────────────────────────────────────── */

/*
 * Compute motion command from the L4 world model.
 *
 * For each active, non-stale, non-self peer in wm, a spring force is applied:
 *   force = (distance - TARGET_SPACING) * SPRING_K
 *   direction = unit vector from own position toward peer
 *
 * The summed force vector is projected onto the robot frame and written to
 * *speed_out (forward velocity) and *rate_out (yaw rate), both normalized
 * [-1.0, 1.0].  Pass these directly to substrate_move() via substrate_twist_t.
 * When no peers are visible, the robot holds position (both outputs zero).
 */
void demo_compute_drive(const world_model_t *wm,
                         demo_odometry_t *odo,
                         float *speed_out,
                         float *rate_out);

/* ── Signal feedback ────────────────────────────────────────────────────── */

/*
 * Set substrate signal to reflect L4 world model peer visibility.
 *   >=2 fresh peers → SUBSTRATE_SIGNAL_ACTIVE   (formation viable)
 *    1 fresh peer   → SUBSTRATE_SIGNAL_DEGRADED (partial)
 *    0 fresh peers  → SUBSTRATE_SIGNAL_FAILED   (isolated / starting up)
 */
void demo_set_leds(const world_model_t *wm);

/*
 * Display dead-reckoning position on the micro:bit 5×5 LED matrix.
 * Maps the 100×100 logical world onto the 5×5 grid (20 units per cell).
 * One lit pixel shows where this robot thinks it is.  Only redraws when
 * the pixel cell changes, so it is safe to call every main-loop cycle.
 *
 * Orientation (connector at bottom, as held during demo):
 *   col 0 = right  col 4 = left   (x-axis flipped)
 *   row 0 = top    row 4 = bottom  (y-axis flipped: large-y → top)
 */
void demo_display_position(const demo_odometry_t *odo);

#endif /* TAPESTRY_DEMO_FORMATION_H */
