/*
 * cf21_imu.c — BMI088 IMU driver wrapper for the Crazyflie 2.1 brushless
 *
 * See cf21_imu.h for the public API and unit conventions.
 *
 * Sampling is slaved to the gyro's INT3 data-ready interrupt (PC14,
 * ~1 kHz, see crazyflie21br.overlay). The bmi08x driver runs its own
 * trigger thread (CONFIG_BMI08X_GYRO_TRIGGER_OWN_THREAD) which calls
 * gyro_drdy_handler() below; that handler bumps a counter (for rate
 * measurement) and posts a semaphore that cf21_imu_read() waits on.
 */

#include "cf21_imu.h"

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cf21_imu, LOG_LEVEL_INF);

#define RAD_TO_DEG 57.29577951308232f   /* 180 / pi */

static const struct device *const accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi088_accel));
static const struct device *const gyro_dev  = DEVICE_DT_GET(DT_NODELABEL(bmi088_gyro));

static K_SEM_DEFINE(drdy_sem, 0, K_SEM_MAX_LIMIT);
static uint32_t drdy_count;

/* ── Complementary filter state ─────────────────────────────────────────── */

static float filter_alpha = 0.98f;
static float filter_roll_deg;
static float filter_pitch_deg;

/* ── Gyro data-ready trigger ────────────────────────────────────────────── */

static void gyro_drdy_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(trig);

    drdy_count++;
    k_sem_give(&drdy_sem);
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int cf21_imu_init(void)
{
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("BMI088 accel device not ready");
        return -ENODEV;
    }
    if (!device_is_ready(gyro_dev)) {
        LOG_ERR("BMI088 gyro device not ready");
        return -ENODEV;
    }

    const struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_GYRO_XYZ,
    };
    int ret = sensor_trigger_set(gyro_dev, &trig, gyro_drdy_handler);
    if (ret) {
        LOG_ERR("Failed to set gyro data-ready trigger: %d", ret);
        return ret;
    }

    LOG_INF("BMI088 accel+gyro ready, INT3 trigger armed");
    return 0;
}

int cf21_imu_read(cf21_imu_sample_t *out)
{
    k_sem_take(&drdy_sem, K_FOREVER);

    int ret = sensor_sample_fetch(gyro_dev);
    if (ret) {
        return ret;
    }
    ret = sensor_sample_fetch(accel_dev);
    if (ret) {
        return ret;
    }

    struct sensor_value gyro[3];
    struct sensor_value accel[3];

    sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    for (int i = 0; i < 3; i++) {
        out->gyro_rps[i] = sensor_value_to_float(&gyro[i]);
        out->accel_g[i] = sensor_value_to_float(&accel[i]) / 9.80665f;
    }

    return 0;
}

uint32_t cf21_imu_get_drdy_count(void)
{
    return drdy_count;
}

/* ── Complementary filter ───────────────────────────────────────────────── */

void cf21_imu_filter_init(float alpha)
{
    filter_alpha = alpha;
    filter_roll_deg = 0.0f;
    filter_pitch_deg = 0.0f;
}

/*
 * Axis mapping vs. substrate.h body frame: confirmed by hand-tilt test
 * (2026-06-12, see project_crazyflie21br.md) — raw BMI088 axes map directly
 * onto the substrate body frame with NO permutation or sign flip:
 *   X = forward/nose, Y = left, Z = up
 * Verified via gravity vector readings:
 *   level:            accel ~= ( 0,  0, +1)
 *   nose-down  ~90deg: accel ~= (-1,  0,  0)
 *   right-down ~90deg: accel ~= ( 0, +1,  0)
 * roll  ~ rotation about the X axis (gyro_rps[0], accel Y/Z plane);
 *         positive = right-side-down, matches atan2(accel_y, accel_z).
 * pitch ~ rotation about the Y axis (gyro_rps[1], accel X vs. Y/Z plane);
 *         positive = nose-up, matches atan2(accel_x, |accel_yz|) (note:
 *         NOT atan2(-accel_x, ...) — that sign gave nose-down -> +90deg,
 *         contradicting substrate's pitch-positive-is-nose-up convention).
 */
void cf21_imu_filter_update(const cf21_imu_sample_t *sample, float dt_s,
                             cf21_imu_attitude_t *out)
{
    float roll_acc_deg = atan2f(sample->accel_g[1], sample->accel_g[2]) * RAD_TO_DEG;
    float pitch_acc_deg = atan2f(sample->accel_g[0],
                                  sqrtf(sample->accel_g[1] * sample->accel_g[1] +
                                        sample->accel_g[2] * sample->accel_g[2])) *
                           RAD_TO_DEG;

    float roll_gyro_deg = filter_roll_deg + sample->gyro_rps[0] * RAD_TO_DEG * dt_s;
    float pitch_gyro_deg = filter_pitch_deg + sample->gyro_rps[1] * RAD_TO_DEG * dt_s;

    filter_roll_deg = filter_alpha * roll_gyro_deg + (1.0f - filter_alpha) * roll_acc_deg;
    filter_pitch_deg = filter_alpha * pitch_gyro_deg + (1.0f - filter_alpha) * pitch_acc_deg;

    out->roll_deg = filter_roll_deg;
    out->pitch_deg = filter_pitch_deg;
}
