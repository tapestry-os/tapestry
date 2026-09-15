/*
 * substrate_webots.h — Tapestry L1 Physical Substrate Interface, Webots
 * differential-drive backend (warehouse AMR)
 *
 * Implements tapestry/substrate.h against the WarehouseAMR PROTO
 * (../../protos/WarehouseAMR.proto) using the Webots C controller API
 * (webots/motor.h, webots/gps.h, webots/inertial_unit.h, webots/led.h).
 *
 * This is the SECOND substrate written to the examples/webots-formation
 * pattern, and the first that is not a flying element — it is the evidence
 * for that example's claim that everything in its controllers/common/ is
 * substrate-agnostic. L3-L7 (gossip.c, world_model.c, scr.c, bse.c,
 * choreo.c) are compiled here byte-for-byte identically to the drone build;
 * this file and main.c are the entire delta.
 *
 * TWIST CONVENTION — substrate.h says of substrate_twist_t: "Differential
 * drive platforms use linear.x and angular.z exclusively." That is taken
 * literally here:
 *
 *   linear.x   [-1, 1]  forward speed fraction  -> MAX_SPEED_MPS
 *   angular.z  [-1, 1]  yaw rate fraction       -> MAX_YAW_RATE_RADPS
 *   linear.y            IGNORED (non-holonomic — the platform cannot
 *                       strafe, and silently accepting a strafe command
 *                       would misreport what the substrate did)
 *   linear.z            IGNORED (no altitude)
 *
 * main.c is responsible for turning a world-frame position error into that
 * pair (a unicycle steering law); this file owns only the twist -> wheel
 * velocity conversion. The split is the same one the cf21bl substrate makes
 * between main.c's world->body transform and its own PID cascade.
 *
 * Two rates, same as the cf21bl backend:
 *   substrate_move()        — called at the L4-L7 coordination cadence
 *                             (WM_CYCLE_MS, 100 ms). Latches only; no I/O.
 *   substrate_webots_step() — called every Webots physics step. Reads
 *                             sensors and writes both wheel motors.
 */

#ifndef TAPESTRY_WAREHOUSE_SUBSTRATE_WEBOTS_H
#define TAPESTRY_WAREHOUSE_SUBSTRATE_WEBOTS_H

#include <tapestry/substrate.h>   /* substrate_quat_t */

/* Advance the physics-rate loop by dt seconds. Call once per
 * wb_robot_step(), after substrate_init(). */
void substrate_webots_step(double dt);

/* Ground-truth pose from the last substrate_webots_step() — Webots GPS and
 * InertialUnit readings.
 *
 * Unlike examples/cutebot-formation (whose "absolute position" is
 * dead reckoning from an ID-derived seed, and drifts), this is a real
 * absolute sensor. That is what makes this example's SCR_CAP_ABS_POSITION
 * claim and its scripts' frame = "absolute" targets honest rather than a
 * shared-fiction convention between elements. */
void substrate_webots_get_position(float *x, float *y, float *z);
float substrate_webots_get_yaw(void);

/* Ground-truth orientation as a unit quaternion, converted from the
 * InertialUnit's roll/pitch/yaw. Satisfies orientation_t's reference-frame
 * convention (csm.h) by construction: the InertialUnit reports in the same
 * world frame the GPS reports position in. */
void substrate_webots_get_orientation(substrate_quat_t *q);

/* Stop both wheels immediately and latch zero, bypassing the twist path.
 * Used by main.c's failure injection (scene 3) and its geofence backstop —
 * both need the platform stopped NOW, not at the next coordination tick. */
void substrate_webots_halt(void);

#endif /* TAPESTRY_WAREHOUSE_SUBSTRATE_WEBOTS_H */
