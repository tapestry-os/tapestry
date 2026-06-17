/*
 * cf21_stabilizer.h — Cascaded attitude controller for the Crazyflie 2.1 brushless
 *
 * Two-loop architecture:
 *
 *   Inner rate loop (~1 kHz, always active):
 *     Three independent PID controllers on gyro_rps[0/1/2] (rad/s).
 *     Slaved to the BMI088 INT3 data-ready interrupt via cf21_imu_read().
 *
 *   Outer angle loop (CONFIG_CF21_ANGLE_MODE=y):
 *     P controller on roll/pitch error from the complementary filter.
 *     Produces inner-loop rate setpoints; yaw remains rate-controlled.
 *
 * Enabling the stabilizer
 * -----------------------
 * Add to your application's boards/crazyflie21br.conf:
 *   CONFIG_CF21_STABILIZER=y            # rate loop
 *   CONFIG_CF21_ANGLE_MODE=y            # + angle loop (self-leveling)
 *
 * Add to CMakeLists.txt inside the if(CONFIG_PWM) block:
 *   if(CONFIG_CF21_STABILIZER)
 *     target_sources(app PRIVATE
 *       ${TAPESTRY_OS_BOARDS}/crazyflie21br/cf21_stabilizer.c
 *       ${TAPESTRY_OS_BOARDS}/crazyflie21br/cf21_imu.c)
 *   endif()
 *
 * Substrate setpoint convention  (substrate_twist_t, normalized [-1, +1])
 * -------------------------------------------------------------------------
 * Rate mode (CONFIG_CF21_ANGLE_MODE not set):
 *   angular.x  roll  rate setpoint → scaled to ±CF21_MAX_RATE_RPS
 *   angular.y  pitch rate setpoint → scaled to ±CF21_MAX_RATE_RPS
 *   angular.z  yaw   rate setpoint → scaled to ±CF21_MAX_YAW_RATE_RPS
 *
 * Angle mode (CONFIG_CF21_ANGLE_MODE=y):
 *   angular.x  desired roll  angle → scaled to ±CF21_MAX_ANGLE_DEG (degrees)
 *   angular.y  desired pitch angle → scaled to ±CF21_MAX_ANGLE_DEG (degrees)
 *   angular.z  yaw   rate setpoint → scaled to ±CF21_MAX_YAW_RATE_RPS
 *
 * Both modes:
 *   linear.z   collective thrust passed through directly to cf21_mix()
 *   linear.x/y zeroed (velocity feedforward not yet implemented)
 *
 * cf21_stabilizer_start() calls cf21_imu_init() and cf21_imu_filter_init()
 * internally; the caller does not need to initialize the IMU separately.
 * It is called from cf21_init() after ESC arming completes.
 */

#ifndef TAPESTRY_CF21_STABILIZER_H
#define TAPESTRY_CF21_STABILIZER_H

#include <tapestry/substrate.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cf21_stabilizer_start — Initialize IMU, reset PIDs, and launch the
 * stabilizer thread. Returns 0 on success, negative errno on failure.
 * Called once from cf21_init().
 */
int cf21_stabilizer_start(void);

/*
 * cf21_stabilizer_set_setpoint — Store a new motion setpoint (thread-safe).
 * Called from substrate_move(); the stabilizer thread reads it each cycle.
 */
void cf21_stabilizer_set_setpoint(const substrate_twist_t *sp);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CF21_STABILIZER_H */
