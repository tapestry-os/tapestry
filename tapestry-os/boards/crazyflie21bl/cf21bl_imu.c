/*
 * cf21bl_imu.c — BMI088 IMU driver wrapper for the Crazyflie 2.1 brushless
 *
 * See cf21bl_imu.h for the public API and unit conventions.
 *
 * Sampling is slaved to the gyro's INT3 data-ready interrupt (PC14,
 * ~1 kHz, see crazyflie21bl.overlay). The bmi08x driver runs its own
 * trigger thread (CONFIG_BMI08X_GYRO_TRIGGER_OWN_THREAD) which calls
 * gyro_drdy_handler() below; that handler bumps a counter (for rate
 * measurement) and posts a semaphore that cf21bl_imu_read() waits on.
 *
 * Sensor filtering matches CF21BL stock firmware (sensors_bmi088_bmp3xx.c):
 *   - 2nd-order Butterworth LPF: gyro 80 Hz cutoff, accel 30 Hz cutoff
 *   - Attitude: Mahony quaternion filter, twoKp=0.8, twoKi=0.002
 */

#include "cf21bl_imu.h"

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cf21bl_imu, LOG_LEVEL_INF);

#define RAD_TO_DEG  57.29577951308232f   /* 180 / pi */
#define CF21BL_PI      3.14159265358979f

/* Low-pass filter cutoff frequencies — from CF21BL stock firmware */
#define GYRO_LPF_CUTOFF_HZ   80.0f
#define ACCEL_LPF_CUTOFF_HZ  30.0f
#define SENSOR_SAMPLE_HZ   1000.0f   /* gyro INT3 rate drives both reads */

/* Mahony filter gains — from CF21BL build/.config (twoKp=2*0.4, twoKi=2*0.001) */
#define MAHONY_TWO_KP  0.8f
#define MAHONY_TWO_KI  0.002f

static const struct device *const accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi088_accel));
static const struct device *const gyro_dev  = DEVICE_DT_GET(DT_NODELABEL(bmi088_gyro));

static K_SEM_DEFINE(drdy_sem, 0, K_SEM_MAX_LIMIT);
static uint32_t drdy_count;

/* ── 2nd-order Butterworth low-pass filter (CF lpf2p) ──────────────────── */

typedef struct {
    float a1, a2, b0, b1, b2;
    float d1, d2;
} lpf2p_t;

static lpf2p_t g_gyro_lpf[3];
static lpf2p_t g_accel_lpf[3];

/* Static gyro bias (rad/s), measured at startup with motors off */
static float g_gyro_bias[3];

static void lpf2p_init(lpf2p_t *f, float sample_hz, float cutoff_hz)
{
    const float fr  = sample_hz / cutoff_hz;
    const float ohm = tanf(CF21BL_PI / fr);
    const float c   = 1.0f + 2.0f * cosf(CF21BL_PI / 4.0f) * ohm + ohm * ohm;
    f->b0 = ohm * ohm / c;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    f->a2 = (1.0f - 2.0f * cosf(CF21BL_PI / 4.0f) * ohm + ohm * ohm) / c;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

static float lpf2p_apply(lpf2p_t *f, float x)
{
    float d0  = x - f->a1 * f->d1 - f->a2 * f->d2;
    float out = f->b0 * d0 + f->b1 * f->d1 + f->b2 * f->d2;
    f->d2 = f->d1;
    f->d1 = d0;
    return out;
}

/* ── Gyro bias + accel scale calibration ────────────────────────────────── */

/*
 * Variance threshold for accepting a gyro bias measurement.
 * Matched to stock sensors_bmi088_bmp3xx.c GYRO_VARIANCE_BASE = 100 (raw
 * LSB²), scaled to (rad/s)²:
 *   BMI088 @ 2000 dps: 1 LSB = 0.0610 deg/s = 0.001064 rad/s
 *   100 LSB² × (0.001064 rad/s/LSB)² ≈ 1.13 × 10⁻⁴ (rad/s)²
 * In practice the LPF attenuates noise below this, so the threshold is loose
 * enough to accept any stationary drone and reject any moving one.
 */
#define GYRO_VAR_THRESHOLD  1.13e-4f   /* (rad/s)² per axis */
#define GYRO_BIAS_SAMPLES   512        /* circular buffer depth — matches stock */
#define GYRO_WARMUP_SAMPLES 100        /* discarded to let LPF settle */
#define GYRO_MAX_RETRIES    10         /* attempts before giving up */

/* Static accel scale factor (set by cf21bl_imu_calibrate_gyro, used in read) */
static float g_accel_scale = 1.0f;

/*
 * cf21bl_imu_calibrate_gyro — variance-gated gyro bias + accel scale.
 *
 * Matches the stock CF firmware approach (sensors_bmi088_bmp3xx.c):
 *
 *   Gyro bias (processGyroBias / processGyroBiasNoBuffer):
 *     Collect GYRO_BIAS_SAMPLES into a circular buffer, compute per-axis
 *     variance.  Only accept the mean as bias when variance < GYRO_VAR_THRESHOLD
 *     on all three axes — ensures the drone was stationary.  Retries up to
 *     GYRO_MAX_RETRIES times (one buffer per retry).  Logs a warning if
 *     the variance never passes (e.g. drone was not placed down in time).
 *
 *   Accel scale (processAccScale):
 *     Average |a_vec| over 200 samples to measure the absolute scale factor.
 *     Divides subsequent accel readings by this value so 1 g → 1.0 exactly.
 *     (The Mahony filter normalises the accel vector, so this only matters
 *     for altitude-from-accel which we don't currently use — but it matches
 *     stock and costs nothing.)
 *
 * Must be called from cf21bl_stabilizer_start() AFTER cf21bl_imu_init()
 * and BEFORE the main stabilizer loop.  Drone must be stationary on ground.
 * Motors are silent at this point: idle PWM (1000 µs) < spin threshold (1180 µs).
 */
void cf21bl_imu_calibrate_gyro(int n_samples)
{
    cf21bl_imu_sample_t s;

    /* Warm-up: discard initial samples so LPF has fully settled */
    g_gyro_bias[0] = g_gyro_bias[1] = g_gyro_bias[2] = 0.0f;
    for (int i = 0; i < GYRO_WARMUP_SAMPLES; i++) {
        cf21bl_imu_read(&s);
    }

    /* ── Variance-gated gyro bias ─────────────────────────────────────── */
    bool bias_found = false;

    for (int attempt = 0; attempt < GYRO_MAX_RETRIES && !bias_found; attempt++) {
        float buf[GYRO_BIAS_SAMPLES][3];
        float sum[3]   = {0};
        float sumsq[3] = {0};

        /* Collect one buffer of GYRO_BIAS_SAMPLES samples */
        for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
            cf21bl_imu_read(&s);
            for (int ax = 0; ax < 3; ax++) {
                buf[i][ax] = s.gyro_rps[ax];
                sum[ax]   += s.gyro_rps[ax];
                sumsq[ax] += s.gyro_rps[ax] * s.gyro_rps[ax];
            }
        }

        /* Compute mean and variance for each axis */
        float mean[3], var[3];
        bool all_stable = true;
        for (int ax = 0; ax < 3; ax++) {
            mean[ax] = sum[ax] / GYRO_BIAS_SAMPLES;
            var[ax]  = sumsq[ax] / GYRO_BIAS_SAMPLES - mean[ax] * mean[ax];
            if (var[ax] >= GYRO_VAR_THRESHOLD) {
                all_stable = false;
            }
        }

        if (all_stable) {
            g_gyro_bias[0] = mean[0];
            g_gyro_bias[1] = mean[1];
            g_gyro_bias[2] = mean[2];
            bias_found = true;
            LOG_INF("Gyro bias (attempt %d): "
                    "gx=%+.5f  gy=%+.5f  gz=%+.5f  rad/s  "
                    "var=%.2e/%.2e/%.2e",
                    attempt + 1,
                    (double)g_gyro_bias[0],
                    (double)g_gyro_bias[1],
                    (double)g_gyro_bias[2],
                    (double)var[0], (double)var[1], (double)var[2]);
        } else {
            LOG_WRN("Gyro variance too high (attempt %d) — "
                    "var=%.2e/%.2e/%.2e (threshold %.2e) — "
                    "keep drone still",
                    attempt + 1,
                    (double)var[0], (double)var[1], (double)var[2],
                    (double)GYRO_VAR_THRESHOLD);
        }
    }

    if (!bias_found) {
        LOG_ERR("Gyro bias calibration failed after %d attempts — "
                "using zero bias (drift expected)", GYRO_MAX_RETRIES);
    }

    /* ── Accel scale (stock: processAccScale, 200 samples) ───────────── */
    /* g_gyro_bias is now set; subsequent cf21bl_imu_read() calls subtract it */
    float scale_sum = 0.0f;
    int   scale_n   = 200;
    for (int i = 0; i < scale_n; i++) {
        cf21bl_imu_read(&s);
        float mag = sqrtf(s.accel_g[0] * s.accel_g[0] +
                          s.accel_g[1] * s.accel_g[1] +
                          s.accel_g[2] * s.accel_g[2]);
        scale_sum += mag;
    }
    g_accel_scale = scale_sum / (float)scale_n;
    LOG_INF("Accel scale: %.5f g  (ideal=1.0)", (double)g_accel_scale);
}

/* ── Mahony quaternion filter state ────────────────────────────────────── */

static float mq0 = 1.0f, mq1, mq2, mq3;  /* quaternion (w, x, y, z) */
static float mifbx, mifby, mifbz;         /* integral feedback */

/* ── Gyro data-ready trigger ────────────────────────────────────────────── */

static void gyro_drdy_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(trig);

    drdy_count++;
    k_sem_give(&drdy_sem);
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int cf21bl_imu_init(void)
{
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("BMI088 accel device not ready");
        return -ENODEV;
    }
    if (!device_is_ready(gyro_dev)) {
        LOG_ERR("BMI088 gyro device not ready");
        return -ENODEV;
    }

    for (int i = 0; i < 3; i++) {
        lpf2p_init(&g_gyro_lpf[i],  SENSOR_SAMPLE_HZ, GYRO_LPF_CUTOFF_HZ);
        lpf2p_init(&g_accel_lpf[i], SENSOR_SAMPLE_HZ, ACCEL_LPF_CUTOFF_HZ);
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

int cf21bl_imu_read(cf21bl_imu_sample_t *out)
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

    /* BMI088 gyro Y axis: positive ω_y takes the nose toward −Z (nose DOWN) per
     * right-hand rule on the body frame (X=fwd, Y=left, Z=up).  Negate so that
     * gyro_rps[1] > 0 means nose-up, consistent with substrate.h pitch convention.
     * CF firmware applies the same negation: controller_pid.c uses -sensors->gyro.y
     * in the rate controller call. X (roll) and Z (yaw) have correct signs natively. */
    float raw_gyro[3] = {
         sensor_value_to_float(&gyro[0]),
        -sensor_value_to_float(&gyro[1]),
         sensor_value_to_float(&gyro[2]),
    };
    float raw_accel[3];
    for (int i = 0; i < 3; i++) {
        raw_accel[i] = sensor_value_to_float(&accel[i]) / 9.80665f;
    }

    for (int i = 0; i < 3; i++) {
        out->gyro_rps[i] = lpf2p_apply(&g_gyro_lpf[i],  raw_gyro[i]) - g_gyro_bias[i];
        out->accel_g[i]  = lpf2p_apply(&g_accel_lpf[i], raw_accel[i]) / g_accel_scale;
    }

    return 0;
}

uint32_t cf21bl_imu_get_drdy_count(void)
{
    return drdy_count;
}

/* ── Mahony quaternion filter ───────────────────────────────────────────── */

/*
 * Axis mapping vs. substrate.h body frame: confirmed by hand-tilt test
 * (2026-06-12): X=forward, Y=left, Z=up; gravity reads (0,0,+1g) when level.
 *
 *   level:             accel ~= ( 0,  0, +1)
 *   nose-down  ~90deg: accel ~= (-1,  0,  0)
 *   right-down ~90deg: accel ~= ( 0, +1,  0)
 *
 * The CF21BL Mahony halfvx/vy/vz gravity estimation and Euler extraction
 * formulas work for our body frame with ONE gyro sign adjustment: CF feeds
 * raw gyro Y to the Mahony (positive = nose-DOWN) and only negates it for the
 * rate controller.  We negate gyro Y at read time for the rate PID, so we must
 * re-negate before passing to Mahony.  See the gy assignment below.
 * Euler sign conventions (substrate.h):
 *   roll  > 0 = right-side-down
 *   pitch > 0 = nose-up
 *   yaw   > 0 = counterclockwise (turn left)
 */
void cf21bl_imu_filter_init(void)
{
    mq0 = 1.0f; mq1 = 0.0f; mq2 = 0.0f; mq3 = 0.0f;
    mifbx = 0.0f; mifby = 0.0f; mifbz = 0.0f;
}

void cf21bl_imu_filter_update(const cf21bl_imu_sample_t *sample, float dt_s,
                             cf21bl_imu_attitude_t *out)
{
    float gx = sample->gyro_rps[0];
    /* Mahony algorithm uses CF raw gyro convention: gy > 0 = nose-DOWN.
     * gyro_rps[1] was negated in cf21bl_imu_read() (nose-UP positive) for the
     * rate PID, but the Mahony quaternion integration requires the un-negated
     * value.  CF firmware feeds raw (un-negated) gyro.y to sensfusion6UpdateQ
     * and only negates it in controller_pid.c for the rate controller. */
    float gy = -sample->gyro_rps[1];
    float gz = sample->gyro_rps[2];
    float ax = sample->accel_g[0];
    float ay = sample->accel_g[1];
    float az = sample->accel_g[2];

    float acc_norm_sq = ax*ax + ay*ay + az*az;
    if (acc_norm_sq > 0.0f) {
        float recip = 1.0f / sqrtf(acc_norm_sq);
        ax *= recip; ay *= recip; az *= recip;

        /* Estimated direction of gravity in body frame (half-magnitude) */
        float halfvx = mq1*mq3 - mq0*mq2;
        float halfvy = mq0*mq1 + mq2*mq3;
        float halfvz = mq0*mq0 - 0.5f + mq3*mq3;

        /* Error: cross product of measured vs. estimated gravity */
        float halfex = ay*halfvz - az*halfvy;
        float halfey = az*halfvx - ax*halfvz;
        float halfez = ax*halfvy - ay*halfvx;

        mifbx += MAHONY_TWO_KI * halfex * dt_s;
        mifby += MAHONY_TWO_KI * halfey * dt_s;
        mifbz += MAHONY_TWO_KI * halfez * dt_s;

        gx += mifbx + MAHONY_TWO_KP * halfex;
        gy += mifby + MAHONY_TWO_KP * halfey;
        gz += mifbz + MAHONY_TWO_KP * halfez;
    } else {
        mifbx = 0.0f; mifby = 0.0f; mifbz = 0.0f;
    }

    /* Integrate rate of change of quaternion */
    float hdt = 0.5f * dt_s;
    float qa = mq0, qb = mq1, qc = mq2;
    mq0 += (-qb*gx - qc*gy - mq3*gz) * hdt;
    mq1 += ( qa*gx + qc*gz - mq3*gy) * hdt;
    mq2 += ( qa*gy - qb*gz + mq3*gx) * hdt;
    mq3 += ( qa*gz + qb*gy - qc*gx) * hdt;

    /* Normalize */
    float recip = 1.0f / sqrtf(mq0*mq0 + mq1*mq1 + mq2*mq2 + mq3*mq3);
    mq0 *= recip; mq1 *= recip; mq2 *= recip; mq3 *= recip;

    /* Extract Euler angles — same formulas as CF sensfusion6GetEulerRPY */
    float grav_x = 2.0f * (mq1*mq3 - mq0*mq2);
    if      (grav_x >  1.0f) grav_x =  1.0f;
    else if (grav_x < -1.0f) grav_x = -1.0f;

    float grav_y = 2.0f * (mq0*mq1 + mq2*mq3);
    float grav_z = mq0*mq0 - mq1*mq1 - mq2*mq2 + mq3*mq3;

    out->pitch_deg = asinf(grav_x) * RAD_TO_DEG;
    out->roll_deg  = atan2f(grav_y, grav_z) * RAD_TO_DEG;
    out->yaw_deg   = atan2f(2.0f*(mq1*mq2 + mq0*mq3),
                            mq0*mq0 + mq1*mq1 - mq2*mq2 - mq3*mq3) * RAD_TO_DEG;
}
