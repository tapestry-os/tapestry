/*
 * formation.h — Demo: Collective Formation (L4 only)
 *
 * Spring-field repulsion/attraction over the L4 world model, with
 * dead-reckoning odometry to keep own_state.position current.
 *
 * Physical calibration:
 *   DEMO_SPEED_SCALE — logical units per second at 100% motor speed.
 *     Formula: SPEED_SCALE = robot_speed_mm_per_s * 100 / WORLD_MM
 *     where WORLD_MM is the physical width of your arena in mm.
 *     Calibrated for 0.3 m arena and ~0.15 m/s Cutebot speed:
 *       750 mm/s × 100 / 300 mm = 250.0 units/s  (1 unit = 3 mm)
 *     (750 mm/s extrapolated from measured 150 mm/s at 20%.)
 *
 *   DEMO_WHEEL_TRACK — wheel-center-to-wheel-center in logical units.
 *     Cutebot Mini track ≈ 85 mm. In a 0.3 m arena: 85/3 ≈ 28.0.
 *
 * Formation tuning:
 *   DEMO_TARGET_SPACING — desired peer-to-peer spacing in logical units.
 *     59 units → equilibrium side ≈ 50 units = 150 mm in a 300 mm arena.
 *     Smaller = tighter cluster. Larger = robots spread to arena edges.
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
#define DEMO_WHEEL_TRACK   28.0f   /* logical units, wheel-to-wheel (0.3 m arena) */
#endif

#ifndef DEMO_MAX_SPEED
#define DEMO_MAX_SPEED    250.0f   /* logical units/s at speed_norm=1.0
                                     * Formula: robot_speed_mm_per_s * 100 / WORLD_MM
                                     * 0.3 m arena, ~0.75 m/s max: 750 * 100 / 300 = 250 */
#endif

/* Maximum yaw rate (rad/s) at rate_norm=1.0 — derived from wheel track and max speed.
 * Not overridable: change DEMO_MAX_SPEED or DEMO_WHEEL_TRACK instead. */
#define DEMO_MAX_OMEGA    (2.0f * DEMO_MAX_SPEED / DEMO_WHEEL_TRACK)

#ifndef DEMO_TARGET_SPACING
#define DEMO_TARGET_SPACING 59.0f  /* desired peer spacing, logical units →
                                     * equilibrium side = 59 × 0.854 ≈ 50 units = 150 mm */
#endif

/* ── Dead-reckoning state ───────────────────────────────────────────────── */

typedef struct {
    float x;        /* Current position in logical world coords [0, WORLD_SIZE] */
    float y;
    float heading;  /* Radians, 0 = +x direction */
} demo_odometry_t;

/* Initialize odometry at (x, y) with heading 0 (+x direction). */
void demo_odometry_init(demo_odometry_t *odo, float x, float y);

/*
 * Update dead-reckoning estimate from the last motion command.
 *   speed_norm: forward velocity [-1.0, 1.0], passed to substrate_move().
 *   rate_norm:  yaw rate         [-1.0, 1.0], positive = CCW (turn left).
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
                         const demo_odometry_t *odo,
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
