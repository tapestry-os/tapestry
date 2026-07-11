/*
 * formation.h — CF21BL collective formation (L4 only, no SCR)
 *
 * Holonomic spring-field formation control driven by real lighthouse
 * position, not dead reckoning.
 *
 * Units: element_state_t.position (and everything in this file) is in
 * METRES, home-relative in the shared lighthouse world frame — NOT the
 * abstract [0,100] WORLD_SIZE convention csm.h's other constants
 * (MIN_SEPARATION, REPULSION_RADIUS, GOSSIP_RADIUS) assume.  Those CSM
 * constants are unused by this example for exactly that reason; formation.c
 * and main.c define their own metre-scale thresholds instead.
 *
 * REQUIRES all drones to share the SAME lighthouse base-station poses and
 * OOTX calibration (main.c) — gossiped positions are only comparable if
 * every drone's "home" is the same physical point in the same physical
 * frame.  Flash all drones from the same build of this example.
 *
 * Key differences from the old dead-reckoning version:
 *   - No heading/odometry integration: cf21bl_stabilizer's
 *     CF21BL_LIGHTHOUSE_POS_HOLD loop is holonomic (absolute X/Y setpoint,
 *     rotated into body frame internally by Mahony yaw), so formation
 *     control just outputs a target XY point, no forward/turn decomposition.
 *   - Peer distances and separation math use REAL measured position
 *     (broadcast by main.c from cf21bl_lighthouse_get_position()), not an
 *     integrated estimate — no drift accumulation.
 *   - demo_setpoint_t is a virtual, slowly-moving target the stabilizer's
 *     position PID chases; it is deliberately NOT the same as the drone's
 *     real position, so peers always gossip ground truth.
 */

#ifndef TAPESTRY_CF21BL_FORMATION_H
#define TAPESTRY_CF21BL_FORMATION_H

#include <stdint.h>
#include <stdbool.h>
#include <tapestry/csm.h>
#include <tapestry/substrate.h>

/* Desired peer spacing at equilibrium, metres. */
#ifndef DEMO_TARGET_SPACING_M
#define DEMO_TARGET_SPACING_M  1.0f
#endif

/* Max commanded approach speed, m/s — conservative for a first campaign. */
#ifndef DEMO_MAX_SPEED_MPS
#define DEMO_MAX_SPEED_MPS     0.3f
#endif

/* Spring constant — force per metre of spacing error. */
#define SPRING_K            1.0f

/* Hard-floor separation, metres — below this an extra repulsion term (on
 * top of the smooth spring) reacts faster than SPRING_K alone would.
 * 0.5 m is a props-clearance margin, not a contact distance. */
#define DEMO_MIN_SEP_M      0.5f
#define EMERGENCY_K         4.0f

/* Maps net spring force magnitude to a commanded speed fraction of
 * DEMO_MAX_SPEED_MPS. */
#define FORCE_TO_SPEED      0.15f

/* Hysteresis thresholds on net spring force magnitude (avoids dithering
 * right at equilibrium). */
#define FORCE_STOP          0.15f
#define FORCE_START         0.30f

/* Arena clamp for the commanded target — keeps formation.c's output well
 * inside the stabilizer's CF21BL_POS_MAX_M range (main.c enforces the
 * tighter, landing-triggering geofence on the drone's REAL position). */
#define DEMO_ARENA_LIMIT_M  ((float)CONFIG_CF21BL_POS_MAX_M - 0.3f)

/* ── Formation target state ──────────────────────────────────────────────── */

typedef struct {
    float x;   /* commanded X setpoint, metres, home-relative */
    float y;   /* commanded Y setpoint, metres, home-relative */
    bool  moving;
} demo_setpoint_t;

void demo_setpoint_init(demo_setpoint_t *sp, float x, float y);

/* Advance *target toward the spring-field equilibrium by dt_ms, using
 * REAL peer positions from wm and this drone's REAL position (own_pos_m,
 * metres) for the force calculation.  Also returns the minimum distance
 * observed to any fresh peer this call (for main.c's separation warning),
 * or -1.0f if no fresh peers were found.  own_id is only used to tag the
 * LOG_DBG line — with multiple drones sharing one radio channel (no
 * per-drone channel plan yet), interleaved console output is otherwise
 * impossible to attribute to a specific drone. */
float demo_compute_drive(const world_model_t *wm,
                          const position_t *own_pos_m,
                          demo_setpoint_t *target,
                          uint32_t dt_ms,
                          element_id_t own_id);

/* ── Signal feedback (LED) ────────────────────────────────────────────────── */

void demo_set_leds(const world_model_t *wm);

#endif /* TAPESTRY_CF21BL_FORMATION_H */
