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
 *   DEMO_WHEEL_TRACK — wheel-centre-to-wheel-centre in logical units.
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

/* ── Calibration defaults ───────────────────────────────────────────────── */

#ifndef DEMO_SPEED_SCALE
#define DEMO_SPEED_SCALE  250.0f   /* logical units/s at 100% speed (0.3 m arena,
                                     * measured: 450 mm in 3 s at 20% → 750 mm/s @ 100%) */
#endif

#ifndef DEMO_WHEEL_TRACK
#define DEMO_WHEEL_TRACK   28.0f   /* logical units, wheel-to-wheel (0.3 m arena) */
#endif

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

/* Initialise odometry at (x, y) with heading 0 (+x direction). */
void demo_odometry_init(demo_odometry_t *odo, float x, float y);

/*
 * Update dead-reckoning estimate from the last motor command.
 *   left_pct / right_pct: [-100, 100] percent.
 *   dt_ms: elapsed milliseconds since last call (typically WM_CYCLE_MS).
 */
void demo_odometry_update(demo_odometry_t *odo,
                           int left_pct, int right_pct,
                           uint32_t dt_ms);

/* ── Formation control ──────────────────────────────────────────────────── */

/*
 * Compute differential drive command from the L4 world model.
 *
 * For each active, non-stale, non-self peer in wm, a spring force is applied:
 *   force = (distance - TARGET_SPACING) * SPRING_K
 *   direction = unit vector from own position toward peer
 *
 * The summed force vector is projected onto the robot frame and mapped to
 * differential motor speeds.  When no peers are visible, the robot wanders
 * slowly forward.
 *
 * Writes motor commands to *left_out and *right_out in [-100, 100].
 */
void demo_compute_drive(const world_model_t *wm,
                         const demo_odometry_t *odo,
                         int *left_out,
                         int *right_out);

/* ── LED feedback ───────────────────────────────────────────────────────── */

/*
 * Set Cutebot LEDs to reflect L4 world model peer visibility.
 *   3 fresh peers → white   (full 4-robot formation)
 *   2 fresh peers → green
 *   1 fresh peer  → yellow
 *   0 fresh peers → red     (isolated / starting up)
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
