/*
 * cf21bl_imu.h — BMI088 IMU driver wrapper for the Crazyflie 2.1 brushless
 *
 * Thin wrapper around Zephyr's in-tree bmi08x sensor driver
 * (bosch,bmi08x-accel @0x18 / bosch,bmi08x-gyro @0x69 on I2C3, see
 * crazyflie21bl.overlay). The gyro's INT3 data-ready interrupt (PC14,
 * ~1 kHz) drives sampling — cf21bl_imu_read() blocks until the next sample
 * is ready.
 *
 * Units:
 *   gyro  — rad/s  (Zephyr SENSOR_CHAN_GYRO_XYZ convention)
 *   accel — g      (SENSOR_CHAN_ACCEL_XYZ m/s^2, converted by SENSOR_G)
 *
 * Axis convention: raw BMI088 sensor axes map directly onto the
 * <tapestry/substrate.h> body frame (X=forward/nose, Y=left, Z=up), no
 * permutation or sign flip needed — confirmed by hand-tilt test, see
 * cf21bl_imu_filter_update() in cf21bl_imu.c.
 */

#ifndef TAPESTRY_CF21BL_IMU_H
#define TAPESTRY_CF21BL_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro_rps[3];   /* raw BMI088 axes, rad/s */
    float accel_g[3];    /* raw BMI088 axes, g */
} cf21bl_imu_sample_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;   /* available from Mahony quaternion; not used by stabilizer yet */
    float accel_up_g;
    /* Raw (filtered) body accel dotted with the Mahony-estimated world-up
     * direction, in g. Reads +1 g when level and stationary regardless of
     * tilt; subtract 1 g and convert to m/s^2 to get net vertical
     * acceleration in the world frame, e.g. for altitude-hold velocity
     * estimation. */
} cf21bl_imu_attitude_t;

/*
 * cf21bl_imu_init — verify the BMI088 accel+gyro devices are ready and arm
 * the gyro's INT3 data-ready trigger. Returns 0 on success, negative errno
 * otherwise.
 */
int cf21bl_imu_init(void);

/*
 * cf21bl_imu_read — wait for the next gyro data-ready interrupt, then fetch
 * and return the latest accel+gyro sample. Blocks the calling thread.
 * Returns 0 on success, negative errno otherwise.
 */
int cf21bl_imu_read(cf21bl_imu_sample_t *out);

/*
 * cf21bl_imu_get_drdy_count — running count of gyro INT3 data-ready
 * interrupts seen since boot. Used by imu-test to measure the interrupt
 * rate (sample the delta over a known time window).
 */
uint32_t cf21bl_imu_get_drdy_count(void);

/*
 * cf21bl_imu_calibrate_gyro — average n_samples gyro readings with motors off
 * to measure and subtract the static bias.  Call once from
 * cf21bl_stabilizer_start() after cf21bl_imu_init(), before the main loop.
 * Drone must be stationary on the ground.  Discards 100 warm-up samples first.
 * Typical duration: ~1.1 s for n_samples=1000 at 1 kHz.
 */
void cf21bl_imu_calibrate_gyro(int n_samples);

/*
 * cf21bl_imu_filter_init — (re)initialize the Mahony quaternion filter.
 * Gains: twoKi=0.002; twoKp is two-stage (0.8 ground / 0.15 airborne,
 * see cf21bl_imu_set_airborne).  Init resets to the ground gain.
 */
void cf21bl_imu_filter_init(void);

/*
 * cf21bl_imu_set_airborne — tell the Mahony filter whether the drone is
 * actually flying.  On the ground (false, the init state) the accel
 * correction runs at full gain so spin-up vibration and handling
 * disturbances re-level quickly; airborne (true) it drops to a slow gain
 * so sustained lateral accelerations don't tilt the level reference
 * ("toilet bowl").  Called by the stabilizer: set once liftoff is
 * detected, cleared at the idle sentinel.  Thread-safe (single bool).
 */
void cf21bl_imu_set_airborne(bool airborne);

/*
 * cf21bl_imu_filter_update — fuse one IMU sample into the Mahony quaternion
 * filter and return the updated roll/pitch/yaw estimate in degrees.
 * dt_s is the time since the previous call, in seconds.
 */
void cf21bl_imu_filter_update(const cf21bl_imu_sample_t *sample, float dt_s,
                             cf21bl_imu_attitude_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CF21BL_IMU_H */
