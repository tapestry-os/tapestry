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
 *   linear.x/y zeroed (velocity feedforward not yet implemented)
 *
 * Altitude hold (CONFIG_CF21BL_ALTITUDE_HOLD=y):
 *   linear.z < -0.9  → idle: motors at minimum, altitude PID inactive,
 *                       integrators reset (safe to sit on ground)
 *   linear.z ∈ [-1, +1] → altitude setpoint: -1→0 m, 0→1 m, +1→2 m above home.
 *                       Home is averaged over the first 50 BMP388 readings (~1 s)
 *                       at boot; CF21BL_HOVER_T (65%) is the collective baseline.
 *   linear.x/y zeroed (velocity feedforward not yet implemented)
 *
 * cf21bl_stabilizer_start() calls cf21bl_imu_init() and cf21bl_imu_filter_init()
 * internally; the caller does not need to initialize the IMU separately.
 * It is called from cf21bl_init() after ESC arming completes.
 */

#ifndef TAPESTRY_CF21BL_STABILIZER_H
#define TAPESTRY_CF21BL_STABILIZER_H

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

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CF21BL_STABILIZER_H */
