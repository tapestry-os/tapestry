/*
 * cf21bl_stabilizer.h — Cascaded attitude controller for the Crazyflie 2.1 brushless
 *
 * Two-loop architecture:
 *
 *   Inner rate loop (~1 kHz, always active):
 *     Three independent PID controllers on gyro_rps[0/1/2] (rad/s).
 *     Slaved to the BMI088 INT3 data-ready interrupt via cf21bl_imu_read().
 *
 *   Outer angle loop (CONFIG_CF21BL_ANGLE_MODE=y):
 *     P controller on roll/pitch error from the complementary filter.
 *     Produces inner-loop rate setpoints; yaw remains rate-controlled.
 *
 * Enabling the stabilizer
 * -----------------------
 * Add to your application's boards/crazyflie21bl.conf:
 *   CONFIG_CF21BL_STABILIZER=y            # rate loop
 *   CONFIG_CF21BL_ANGLE_MODE=y            # + angle loop (self-leveling)
 *
 * Add to CMakeLists.txt inside the if(CONFIG_PWM) block:
 *   if(CONFIG_CF21BL_STABILIZER)
 *     target_sources(app PRIVATE
 *       ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_stabilizer.c
 *       ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_imu.c)
 *   endif()
 *
 * Substrate setpoint convention  (substrate_twist_t, normalized [-1, +1])
 * -------------------------------------------------------------------------
 * Rate mode (CONFIG_CF21BL_ANGLE_MODE not set):
 *   angular.x  roll  rate setpoint → scaled to ±CF21BL_MAX_RATE_RPS
 *   angular.y  pitch rate setpoint → scaled to ±CF21BL_MAX_RATE_RPS
 *   angular.z  yaw   rate setpoint → scaled to ±CF21BL_MAX_YAW_RATE_RPS
 *
 * Angle mode (CONFIG_CF21BL_ANGLE_MODE=y):
 *   angular.x  desired roll  angle → scaled to ±CF21BL_MAX_ANGLE_DEG (degrees)
 *   angular.y  desired pitch angle → scaled to ±CF21BL_MAX_ANGLE_DEG (degrees)
 *   angular.z  yaw   rate setpoint → scaled to ±CF21BL_MAX_YAW_RATE_RPS
 *
 * Both modes (CONFIG_CF21BL_ALTITUDE_HOLD not set):
 *   linear.z   collective thrust passed through directly to cf21bl_mix()
 *   linear.x/y velocity feedforward (angle mode) or ignored (rate mode)
 *
 * Altitude hold (CONFIG_CF21BL_ALTITUDE_HOLD=y):
 *   linear.z < -0.9  → idle: motors at minimum, altitude PID inactive,
 *                       integrators reset (safe to sit on ground)
 *   linear.z ∈ [-1, +1] → altitude setpoint: -1→0 m, 0→1 m, +1→2 m above home.
 *                       Home is averaged over the first 50 BMP388 readings (~1 s)
 *                       at boot; CF21BL_HOVER_T (65%) is the collective baseline.
 *   linear.x/y see position hold below (or velocity feedforward without LH)
 *
 * Position hold (CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD=y, requires ANGLE_MODE):
 *   When the lighthouse has a valid fix and linear.z > -0.9 (flying):
 *     linear.x ∈ [-1, +1] → X position setpoint ±CONFIG_CF21BL_POS_MAX_M meters
 *     linear.y ∈ [-1, +1] → Y position setpoint ±CONFIG_CF21BL_POS_MAX_M meters
 *     Both relative to the home position captured at first valid LH fix.
 *     Implemented as a P controller (position error → angle correction) feeding
 *     into the existing angle loop; no separate I/D to avoid cascade windup.
 *   When the fix is invalid: falls back to standard angle-mode feedforward.
 *   cf21bl_lighthouse_init() must be called before the stabilizer starts.
 *
 * cf21bl_stabilizer_start() calls cf21bl_imu_init() and cf21bl_imu_filter_init()
 * internally; the caller does not need to initialize the IMU separately.
 * It is called from cf21bl_init() after ESC arming completes.
 */

#ifndef TAPESTRY_CF21BL_STABILIZER_H
#define TAPESTRY_CF21BL_STABILIZER_H

#include <stdbool.h>
#include <tapestry/substrate.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cf21bl_stabilizer_start — Initialize IMU, reset PIDs, and launch the
 * stabilizer thread. Returns 0 on success, negative errno on failure.
 * Called once from cf21bl_init().
 */
int cf21bl_stabilizer_start(void);

/*
 * cf21bl_stabilizer_set_setpoint — Store a new motion setpoint (thread-safe).
 * Called from substrate_move(); the stabilizer thread reads it each cycle.
 */
void cf21bl_stabilizer_set_setpoint(const substrate_twist_t *sp);

/*
 * cf21bl_stabilizer_get_pos_home — Copy out the lighthouse home position
 * (world-frame meters, captured at first valid fix — see "Position hold"
 * above) that linear.x/y=0 currently resolves to.  Needed by callers that
 * want to command an ABSOLUTE world-frame target rather than a home-relative
 * one (e.g. a multi-drone formation converging on shared world coordinates):
 *   sp.linear.x = (target_x - home_x) / CONFIG_CF21BL_POS_MAX_M, clamped.
 * Returns false (leaves *x and *y untouched) when CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
 * is not built in, or no home has been captured yet (no fix seen since the
 * drone last left idle).  Home is HELD across a mid-flight fix dropout —
 * the lighthouse world frame does not move when the fix drops, and
 * re-capturing on re-acquisition silently re-origined the whole
 * home-relative position loop; it is released on the return to idle, so
 * the next takeoff captures a fresh one.
 */
bool cf21bl_stabilizer_get_pos_home(float *x, float *y);

/*
 * cf21bl_stabilizer_request_land — hand the vertical profile to the
 * stabilizer's closed-loop descent (the same one the stale-setpoint and
 * critical-battery paths use): walk the altitude target down from the
 * MEASURED altitude, hold it at ground level until the airframe has
 * settled, and only then cut the motors.
 *
 * Use this instead of walking linear.z down yourself.  A self-managed ramp
 * has to cross the idle sentinel to reach zero, which cuts thrust at a
 * commanded 0.02 m while the airframe still lags above it; this path cuts
 * on settle, not on the target reaching a number.  Keep sending a normal
 * (non-idle) linear.z while it runs — the descent overrides the altitude
 * target internally — and keep commanding linear.x/y if you want to land
 * on a particular spot, since this deliberately does not touch them.
 *
 * Clearing the request before touchdown abandons the descent and returns
 * control to the application, matching the watchdog paths' semantics.
 * No-op without CONFIG_CF21BL_ALTITUDE_HOLD (there is no closed-loop
 * altitude to walk down); callers must keep their own fallback for that
 * configuration.
 */
void cf21bl_stabilizer_request_land(bool active);

/*
 * cf21bl_stabilizer_is_landed — true once a requested (or forced) descent
 * has settled on the ground and latched the motors off.  Always false
 * without CONFIG_CF21BL_ALTITUDE_HOLD.
 */
bool cf21bl_stabilizer_is_landed(void);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CF21BL_STABILIZER_H */
