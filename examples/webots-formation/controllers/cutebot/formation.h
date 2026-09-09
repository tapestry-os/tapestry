/*
 * formation.h — Cutebot Choreo target tracking, trimmed for Webots
 *
 * Trimmed from examples/cutebot-formation/src/formation.h: keeps only
 * demo_track_target (Choreo-mode turn-then-drive differential-drive
 * tracking) and its supporting types/constants — demo_compute_drive
 * (showcase-mode spring field), demo_drive_straight (motion-primitive
 * test), demo_display_position (micro:bit LED matrix), demo_grid_correct
 * (line-sensor drift correction) and demo_set_leds all have no Webots
 * equivalent or are exercised by other test modes not ported here, so
 * they and their only-used constants (SPRING_K, FORCE_STOP/START,
 * DEMO_WHEEL_TRACK, DEMO_MAX_SPEED/OMEGA, DEMO_START_RADIUS,
 * DEMO_GRID_AXIS_COS_MIN, ...) are dropped — same pattern
 * ../common/tracker.h already used trimming cf21bl-formation's version.
 *
 * demo_track_target/demo_arena_fence/demo_force_to_twist are unmodified
 * logic — the same turn-then-drive law, emergency-repulsion backstop
 * (including this session's DEMO_TRACK_MIN_SEP/EMERGENCY_K/REPEL_DEADBAND
 * fix), and arena-clamp already hardware-validated on the real Cutebot
 * (2026-08-24 flight, 2026-09-08 converge-test). All constants below stay
 * in the SAME abstract [0, WORLD_SIZE] unit space the real hardware demo
 * uses — unrelated to and untouched by whatever physical meters-per-unit
 * scale this Webots substrate renders that space at (see
 * substrate_webots_cutebot.h's METERS_PER_UNIT), so hardware tuning
 * carries over byte-for-byte.
 *
 * demo_odometry_t here holds only {x, y, heading} — `moving` (hysteresis
 * state for the dropped demo_compute_drive) isn't read by anything kept
 * here. Unlike hardware, main.c writes x/y/heading from real Webots GPS/
 * InertialUnit ground truth every tick instead of dead-reckoning them —
 * see main.c's coordination loop.
 */

#ifndef TAPESTRY_WEBOTS_CUTEBOT_FORMATION_H
#define TAPESTRY_WEBOTS_CUTEBOT_FORMATION_H

#include <stdint.h>
#include <tapestry/csm.h>
#include <tapestry/substrate.h>

/* ── Arena scale ─────────────────────────────────────────────────────────
 * Reused verbatim from cutebot-formation/src/formation.h's hardware
 * calibration (measured chessboard, 2026-09-08). This is purely an
 * abstract-unit quantity (feeds DEMO_ARENA_FENCE_MARGIN below) — it has
 * no relationship to Webots' own rendered arena size. */
#ifndef DEMO_ARENA_MM
#define DEMO_ARENA_MM      1266.25f
#endif

#define DEMO_MM_PER_UNIT   (DEMO_ARENA_MM / WORLD_SIZE)
#define DEMO_MM(mm)        ((mm) / DEMO_MM_PER_UNIT)

/* See cutebot-formation/src/formation.h's DEMO_ARENA_FENCE_MARGIN doc for
 * the full rationale (command-level containment, distinct from the
 * position-estimate clamp inside csm.h). */
#ifndef DEMO_ARENA_FENCE_MARGIN
#define DEMO_ARENA_FENCE_MARGIN DEMO_MM(80.0f)   /* 6.3 units */
#endif

/* ── Choreo target-tracking tuning ──────────────────────────────────────
 * Values unchanged from cutebot-formation/src/formation.h — see that
 * file for the full derivation/history of each constant (in particular
 * DEMO_TRACK_MIN_SEP/EMERGENCY_K/REPEL_DEADBAND's doc explaining the
 * two real collision-safety bugs this session's converge-test found and
 * fixed on hardware). */

#define DEMO_TRACK_MAX_FORCE      40.0f
#define DEMO_TRACK_SLOW_RADIUS    DEMO_MM(120.0f)   /* 9.5 units */
#define DEMO_TRACK_ARRIVE_EPS     DEMO_MM(25.0f)    /* 2.0 units */
#define DEMO_TRACK_MIN_SEP        DEMO_MM(320.0f)   /* 25.3 units */
#define DEMO_TRACK_EMERGENCY_K    4.0f
#define DEMO_TRACK_REPEL_DEADBAND 5.0f

/* demo_force_to_twist's turn-rate law divides by this even when called
 * from demo_track_target (not just the showcase spring field it's
 * documented for in the hardware header) — see tracker.c. Kept at the
 * same value so the turn/speed balance matches hardware exactly. */
#define DEMO_TARGET_SPACING DEMO_MM(500.0f)   /* 39.5 units */

/* ── Position estimate ──────────────────────────────────────────────── */

typedef struct {
    float x;        /* World-frame position, abstract [0, WORLD_SIZE] units */
    float y;
    float heading;  /* Radians, 0 = +x direction */
} demo_odometry_t;

/* Initialize at (x, y), heading 0. Webots main.c overwrites all three
 * fields from real sensors every tick thereafter — this only seeds a
 * sane value before the first sensor read. */
void demo_odometry_init(demo_odometry_t *odo, float x, float y);

/*
 * demo_track_target — Choreo tracking (L6/L7): drive toward a single
 * commanded world point (target_x, target_y) — e.g. a FORM step's
 * vertex or a HOLD step's captured station. Turn-then-drive
 * differential-drive law with a trapezoidal approach profile and an
 * emergency-repulsion backstop against any fresh peer closer than
 * DEMO_TRACK_MIN_SEP. See cutebot-formation/src/formation.h's doc for
 * the full behavioral description — unmodified here.
 */
void demo_track_target(const world_model_t *wm,
                        const demo_odometry_t *odo,
                        float target_x,
                        float target_y,
                        float *speed_out,
                        float *rate_out);

#endif /* TAPESTRY_WEBOTS_CUTEBOT_FORMATION_H */
