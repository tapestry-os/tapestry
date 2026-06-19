/*
 * formation.h — CF21BL collective formation (L4 only, no SCR)
 *
 * Spring-field formation control adapted from examples/collective-formation
 * for the Crazyflie 2.1 brushless quadrotor.
 *
 * Key differences from the Cutebot version:
 *   - Dead-reckoning integrates linear.x (forward tilt → horizontal velocity)
 *     rather than wheel odometry.
 *   - DEMO_MAX_SPEED is a tuning constant for the tilt-to-velocity mapping.
 *   - demo_display_position() and the micro:bit LED matrix are removed.
 *   - demo_set_leds() still calls substrate_set_signal() for the CF21BL LED.
 *
 * Initial formation tuning for 3 drones in a ~3 m × 3 m arena:
 *   DEMO_TARGET_SPACING = 30 units  → ~1.5 m equilateral triangle
 *   SPRING_K            = 2.0       → gentle restoring force
 *   DEMO_MAX_SPEED      = 30.0      → conservative first test
 */

#ifndef TAPESTRY_CF21BL_FORMATION_H
#define TAPESTRY_CF21BL_FORMATION_H

#include <stdint.h>
#include <stdbool.h>
#include <tapestry/csm.h>
#include <tapestry/substrate.h>

#ifndef DEMO_MAX_SPEED
#define DEMO_MAX_SPEED     30.0f   /* logical units/s at linear.x = 1.0 */
#endif

#ifndef DEMO_MAX_OMEGA
#define DEMO_MAX_OMEGA      1.0f   /* rad/s at angular.z = 1.0 */
#endif

#ifndef DEMO_TARGET_SPACING
#define DEMO_TARGET_SPACING 30.0f  /* desired peer spacing, logical units */
#endif

/* ── Dead-reckoning state ───────────────────────────────────────────────── */

typedef struct {
    float x;
    float y;
    float heading;
    bool  moving;
} demo_odometry_t;

void demo_odometry_init(demo_odometry_t *odo, float x, float y);
void demo_odometry_update(demo_odometry_t *odo,
                           float speed_norm, float rate_norm,
                           uint32_t dt_ms);

/* ── Formation control ──────────────────────────────────────────────────── */

void demo_compute_drive(const world_model_t *wm,
                         demo_odometry_t *odo,
                         float *speed_out,
                         float *rate_out);

/* ── Signal feedback ────────────────────────────────────────────────────── */

void demo_set_leds(const world_model_t *wm);

#endif /* TAPESTRY_CF21BL_FORMATION_H */
