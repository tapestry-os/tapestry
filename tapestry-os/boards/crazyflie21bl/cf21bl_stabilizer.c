/*
 * cf21bl_stabilizer.c — Cascaded attitude controller for the Crazyflie 2.1 brushless
 *
 * See cf21bl_stabilizer.h for architecture, enabling instructions, and setpoint
 * conventions.
 *
 * PID gain derivation
 * -------------------
 * Starting values are converted from the Crazyflie stock firmware (crazyflie-
 * firmware config.h), which expresses rate-loop gains as:
 *
 *   Kp/Ki/Kd targeting:  error in deg/s  →  output in ~INT16 motor units
 *   CF stock roll/pitch:  Kp=250, Ki=500,   Kd=2.5
 *   CF stock yaw:         Kp=120, Ki=16.67, Kd=0
 *
 * Our system:  error in rad/s  →  output normalized [-1, +1]
 *   Conversion: K_ours = K_cf * (180/π) / INT16_MAX
 *   where (180/π) converts rad/s error to deg/s, INT16_MAX=32767 normalizes.
 *
 * Outer angle loop: CF stock Kp_angle=6.0 (deg/s per deg) converted to
 *   (rad/s per deg): Kp_angle_ours = 6.0 * (π/180) ≈ 0.105.
 *
 * All values are starting points; tune on hardware.
 */

#include "cf21bl_stabilizer.h"
#include "cf21bl_imu.h"
#include "crazyflie21bl.h"      /* cf21bl_set_motors() */
#include "crazyflie21bl_mix.h"  /* cf21bl_mix(), cf21bl_motors_t */
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
#include "cf21bl_lighthouse.h"
#endif

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
#include <zephyr/drivers/sensor.h>
#endif

LOG_MODULE_REGISTER(cf21bl_stabilizer, LOG_LEVEL_INF);

/* ── Configuration ─────────────────────────────────────────────────────────── */

#define CF21BL_STAB_STACK_SIZE    2048
#define CF21BL_STAB_PRIO          K_PRIO_PREEMPT(0)

/* Fixed dt matching the BMI088 gyro INT3 rate (~1 kHz).
 * A constant dt avoids hardware-cycle-counter reads each iteration and is
 * accurate enough for a sensor-slaved loop with a stable interrupt source. */
#define CF21BL_LOOP_DT            (1.0f / 1000.0f)

/* Physical rate and angle limits */
#define CF21BL_MAX_RATE_RPS       3.49f   /* ±200 deg/s */
#define CF21BL_MAX_YAW_RATE_RPS   1.75f   /* ±100 deg/s */
#define CF21BL_MAX_ANGLE_DEG      30.0f   /* ±30° for angle mode full-stick */
#define CF21BL_MAX_ANGLE_RATE_RPS CF21BL_MAX_RATE_RPS

/* Rate PID gains — roll and pitch (symmetric)
 * CF21BL stock converted: Kp=200×(180/π)/32767=0.350, Ki=0.699, Kd=0.00437.
 * PWM actuator latency (~5-10ms) + LPF phase delay (~2ms gyro, ~5ms accel)
 * requires lower gains than DSHOT-based CF firmware; start at 0.11 and tune up. */
#define CF21BL_RP_KP      0.11f
#define CF21BL_RP_KI      0.22f
#define CF21BL_RP_KD      0.0011f
#define CF21BL_RP_ILIM    0.3f    /* max ki·integral contribution, output units */
#define CF21BL_RP_OLIM    0.5f    /* output clamp, [-0.5, +0.5] */

/* Rate PID gains — yaw */
#define CF21BL_YAW_KP     0.21f
#define CF21BL_YAW_KI     0.029f
#define CF21BL_YAW_KD     0.0f
#define CF21BL_YAW_ILIM   0.1f
#define CF21BL_YAW_OLIM   0.3f

/* Outer angle loop (PD_ROLL_KP=6.0, KI=3.0, deg/s per deg → rad/s per deg via × π/180) */
#define CF21BL_ANGLE_KP   0.105f  /* rad/s per degree error */
#define CF21BL_ANGLE_KI   0.052f  /* rad/s per degree·s — corrects steady trim offsets */
#define CF21BL_ANGLE_ILIM 0.35f   /* ±20 deg/s equivalent (CF21BL integration limit) */

/* Position hold (CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD) */
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
/* linear.x/y ∈ [-1,+1] → setpoint in [−POS_MAX, +POS_MAX] metres from origin */
#define CF21BL_POS_MAX_M       ((float)CONFIG_CF21BL_POS_MAX_M)
/* Position P gain: metres error → angle correction in degrees.
 * 0.05 → at 1 m error the correction is 2.9° (within the 10° safe limit). */
#define CF21BL_POS_KP          0.05f   /* rad/m  — converts metres error to rad */
#define CF21BL_POS_OLIM_DEG    10.0f   /* max angle correction from position loop */
#define CF21BL_POS_OLIM_RAD    (CF21BL_POS_OLIM_DEG * (float)M_PI / 180.0f)
#endif /* CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD */

/* Altitude hold (CONFIG_CF21BL_ALTITUDE_HOLD) */
#define CF21BL_BARO_POLL_DIV   20      /* poll every 20 loop iters ≈ 50 Hz           */
#define CF21BL_BARO_LP_ALPHA   0.15f   /* IIR low-pass α, τ ≈ 120 ms at 50 Hz       */
#define CF21BL_PA_PER_M        11.77f  /* Pa per metre, standard atmosphere          */
#define CF21BL_ALT_SP_OFFSET   1.0f    /* linear.z=0 → 1 m above home               */
#define CF21BL_HOVER_T         0.65f   /* measured tethered hover throttle (65%)     */
#define CF21BL_T_FLOOR         0.10f   /* minimum collective mid-flight              */
/* Z-PID gains — conservative for first autonomous hover tests.
 * At 0.5m error: pterm = 0.05×0.5 = 0.025 → T_cmd = 0.675 (gentle liftoff).
 * Tune up incrementally once autonomous takeoff is confirmed stable. */
#define CF21BL_Z_KP            0.05f
#define CF21BL_Z_KI            0.01f
#define CF21BL_Z_ILIM          0.10f   /* max I-term collective contribution [0..1]  */
#define CF21BL_Z_OLIM          0.15f   /* ±15% collective correction range           */

/* ── PID ────────────────────────────────────────────────────────────────────── */

typedef struct {
    float kp, ki, kd;
    float integral;
    float e_prev;
    float i_limit;
    float out_limit;
} cf21bl_pid_t;

static void pid_init(cf21bl_pid_t *p, float kp, float ki, float kd,
                     float i_limit, float out_limit)
{
    p->kp = kp;  p->ki = ki;  p->kd = kd;
    p->integral  = 0.0f;
    p->e_prev    = 0.0f;
    p->i_limit   = i_limit;
    p->out_limit = out_limit;
}

/*
 * Discrete PID:  u = Kp·e + Ki·∫e·dt + Kd·(Δe/dt)
 *
 * Anti-windup: the integral is clamped so that ki·integral stays within
 * ±i_limit (same units as the output).  This prevents the integral from
 * saturating the output during extended disturbances (e.g. constrained
 * tethered hover) and ensures the I-term contribution remains bounded
 * even when Kd is zero and rate errors persist.
 */
static float pid_update(cf21bl_pid_t *p, float error, float dt)
{
    float pterm = p->kp * error;

    p->integral += error * dt;
    if (p->ki != 0.0f) {
        float i_max = p->i_limit / p->ki;
        if      (p->integral >  i_max) p->integral =  i_max;
        else if (p->integral < -i_max) p->integral = -i_max;
    } else {
        p->integral = 0.0f;
    }
    float iterm = p->ki * p->integral;

    float dterm = p->kd * (error - p->e_prev) / dt;
    p->e_prev = error;

    float out = pterm + iterm + dterm;
    if      (out >  p->out_limit) out =  p->out_limit;
    else if (out < -p->out_limit) out = -p->out_limit;
    return out;
}

/* ── Shared setpoint (spinlock-protected) ──────────────────────────────────── */

static struct k_spinlock g_sp_lock;

/* Default: linear.z=-1 → T=0 (idle throttle), all angular zero.
 * Keeps motors at minimum until the application explicitly commands thrust. */
static substrate_twist_t g_setpoint = { .linear = { .z = -1.0f } };

/* ── PID instances ──────────────────────────────────────────────────────────── */

static cf21bl_pid_t g_pid_roll;
static cf21bl_pid_t g_pid_pitch;
static cf21bl_pid_t g_pid_yaw;

#ifdef CONFIG_CF21BL_ANGLE_MODE
static cf21bl_pid_t g_pid_roll_angle;
static cf21bl_pid_t g_pid_pitch_angle;
#endif

#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
/* Home position captured at first valid lighthouse fix (metres, world frame).
 * Position setpoints are offsets from this origin. */
static float    g_pos_home_x;
static float    g_pos_home_y;
static bool     g_pos_home_set;
#endif

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
static const struct device *const baro_dev =
    DEVICE_DT_GET(DT_NODELABEL(bmp388_baro));
static cf21bl_pid_t g_pid_z;
#endif

/* ── Stabilizer thread ──────────────────────────────────────────────────────── */

K_THREAD_STACK_DEFINE(g_stab_stack, CF21BL_STAB_STACK_SIZE);
static struct k_thread g_stab_thread;

static void stabilizer_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /* Consume the first sample; the IMU may hold stale data from before
     * the trigger was armed, which would produce a spurious derivative spike. */
    cf21bl_imu_sample_t sample;
    cf21bl_imu_read(&sample);

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
    /* Record home altitude: average 50 BMP388 readings (~1 s at 50 Hz).
     * IMU interrupt fires normally during this period; semaphore accumulates
     * and is drained on the first main-loop iteration. */
    float p_home = 0.0f;
    for (int n = 0; n < 50; n++) {
        sensor_sample_fetch(baro_dev);
        struct sensor_value sv;
        sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &sv);
        p_home += sensor_value_to_float(&sv) * 1000.0f;  /* kPa → Pa */
        k_msleep(20);
    }
    p_home /= 50.0f;
    LOG_INF("Baro home: %.1f Pa", (double)p_home);
    float alt_filt = 0.0f;   /* LP-filtered altitude above home (metres) */
    int   baro_cnt = 0;
#endif

    while (true) {
        if (cf21bl_imu_read(&sample) != 0) {
            continue;
        }

        substrate_twist_t sp;
        {
            k_spinlock_key_t key = k_spin_lock(&g_sp_lock);
            sp = g_setpoint;
            k_spin_unlock(&g_sp_lock, key);
        }

        float roll_rate_sp, pitch_rate_sp;
        float yaw_rate_sp = sp.angular.z * CF21BL_MAX_YAW_RATE_RPS;

#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
        /*
         * ── Position hold (outermost loop) ────────────────────────────────
         *
         * When a lighthouse fix is available, replace linear.x/y (which the
         * caller treats as velocity feedforward in angle mode) with an angle
         * correction derived from position error in the world frame.
         *
         * linear.x/y ∈ [-1,+1] → XY position setpoint ∈ ±CF21BL_POS_MAX_M
         * relative to the home position captured at first valid fix.
         *
         * The position loop is a P controller: position error [m] → angle
         * correction [rad].  The correction is added to the angular setpoint
         * so the angle loop drives the error to zero.
         *
         * Axis sign convention (matching mix.h):
         *   forward motion (+X world) requires negative pitch → subtract from
         *   pitch_sp_deg below.
         *   left motion    (+Y world) requires negative roll  → subtract from
         *   roll_sp_deg below.
         * So we negate the position correction before adding to the angle sp.
         *
         * No I or D term here: the angle/rate loops below already integrate
         * the position error implicitly (cascaded loops).  Adding integral at
         * the position level causes windup during the initial transient.
         */
        float pos_pitch_correction_deg = 0.0f;
        float pos_roll_correction_deg  = 0.0f;

        if (cf21bl_lighthouse_is_valid() && sp.linear.z > -0.9f) {
            lh2_position_t lhpos;
            cf21bl_lighthouse_get_position(&lhpos);

            if (!g_pos_home_set) {
                g_pos_home_x   = lhpos.x;
                g_pos_home_y   = lhpos.y;
                g_pos_home_set = true;
            }

            float sp_x = sp.linear.x * CF21BL_POS_MAX_M;
            float sp_y = sp.linear.y * CF21BL_POS_MAX_M;

            float ex = (g_pos_home_x + sp_x) - lhpos.x;
            float ey = (g_pos_home_y + sp_y) - lhpos.y;

            /* P correction in radians, converted to degrees for the angle loop */
            float cx_rad = CF21BL_POS_KP * ex;
            float cy_rad = CF21BL_POS_KP * ey;

            if (cx_rad >  CF21BL_POS_OLIM_RAD) { cx_rad =  CF21BL_POS_OLIM_RAD; }
            if (cx_rad < -CF21BL_POS_OLIM_RAD) { cx_rad = -CF21BL_POS_OLIM_RAD; }
            if (cy_rad >  CF21BL_POS_OLIM_RAD) { cy_rad =  CF21BL_POS_OLIM_RAD; }
            if (cy_rad < -CF21BL_POS_OLIM_RAD) { cy_rad = -CF21BL_POS_OLIM_RAD; }

            pos_pitch_correction_deg = cx_rad * (180.0f / (float)M_PI);
            pos_roll_correction_deg  = cy_rad * (180.0f / (float)M_PI);

            static int pos_log_div;
            if (++pos_log_div >= 500) {
                pos_log_div = 0;
                LOG_INF("pos x=%.3f y=%.3f z=%.3f ex=%.3f ey=%.3f",
                        (double)lhpos.x, (double)lhpos.y, (double)lhpos.z,
                        (double)ex, (double)ey);
            }
        } else if (g_pos_home_set && !cf21bl_lighthouse_is_valid()) {
            /* Fix lost mid-flight: log once and allow angle-mode fallback */
            LOG_WRN("LH2 fix lost, falling back to angle mode feedforward");
            g_pos_home_set = false;
        }
#endif /* CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD */

#ifdef CONFIG_CF21BL_ANGLE_MODE
        /* ── Outer angle loop ─────────────────────────────────────────────── */
        cf21bl_imu_attitude_t att;
        cf21bl_imu_filter_update(&sample, CF21BL_LOOP_DT, &att);

        /* Velocity feedforward or position correction (see position hold above).
         * Sign convention from mix.h:
         *   P = angular.y - linear.x → forward (linear.x>0) → pitch negative
         *   R = angular.x - linear.y → left    (linear.y>0) → roll  negative
         *
         * When position hold is active, pos_*_correction_deg supersedes the
         * feedforward tilt so we do not double-count linear.x/y. */
#ifdef CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD
#define CF21BL_MAX_FWD_TILT_DEG  0.0f   /* position loop handles lateral motion */
        float roll_sp_deg  = sp.angular.x * CF21BL_MAX_ANGLE_DEG
                           + pos_roll_correction_deg;
        float pitch_sp_deg = sp.angular.y * CF21BL_MAX_ANGLE_DEG
                           + pos_pitch_correction_deg;
#else
#define CF21BL_MAX_FWD_TILT_DEG  10.0f
        float roll_sp_deg  = sp.angular.x * CF21BL_MAX_ANGLE_DEG
                           - sp.linear.y  * CF21BL_MAX_FWD_TILT_DEG;
        float pitch_sp_deg = sp.angular.y * CF21BL_MAX_ANGLE_DEG
                           - sp.linear.x  * CF21BL_MAX_FWD_TILT_DEG;
#endif /* CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD */

        roll_rate_sp  = pid_update(&g_pid_roll_angle,
                                   roll_sp_deg  - att.roll_deg,  CF21BL_LOOP_DT);
        pitch_rate_sp = pid_update(&g_pid_pitch_angle,
                                   pitch_sp_deg - att.pitch_deg, CF21BL_LOOP_DT);
#else
        /* ── Rate mode: direct rate setpoint ─────────────────────────────── */
        roll_rate_sp  = sp.angular.x * CF21BL_MAX_RATE_RPS;
        pitch_rate_sp = sp.angular.y * CF21BL_MAX_RATE_RPS;
#endif

        /* ── Inner rate loop ──────────────────────────────────────────────── */
        float u_roll  = pid_update(&g_pid_roll,
                                   roll_rate_sp  - sample.gyro_rps[0],
                                   CF21BL_LOOP_DT);
        float u_pitch = pid_update(&g_pid_pitch,
                                   pitch_rate_sp - sample.gyro_rps[1],
                                   CF21BL_LOOP_DT);
        float u_yaw   = pid_update(&g_pid_yaw,
                                   yaw_rate_sp   - sample.gyro_rps[2],
                                   CF21BL_LOOP_DT);

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
        float linear_z_out;
        if (sp.linear.z < -0.9f) {
            /* Idle sentinel: disarm altitude PID and reset integrators so
             * there is no windup while the drone sits on the ground. */
            g_pid_z.integral = 0.0f;
            g_pid_z.e_prev   = 0.0f;
            linear_z_out = -1.0f;
        } else {
            /* Poll BMP388 every CF21BL_BARO_POLL_DIV iterations (~50 Hz).
             * sensor_sample_fetch blocks ~100 µs for the I2C transaction;
             * I2C3 bus is idle at this point (BMI088 RTIO completes before
             * the drdy semaphore is posted, so no bus contention). */
            if (++baro_cnt >= CF21BL_BARO_POLL_DIV) {
                baro_cnt = 0;
                sensor_sample_fetch(baro_dev);
                struct sensor_value sv;
                sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &sv);
                float p_Pa    = sensor_value_to_float(&sv) * 1000.0f;
                float alt_raw = (p_home - p_Pa) / CF21BL_PA_PER_M;
                alt_filt = (1.0f - CF21BL_BARO_LP_ALPHA) * alt_filt
                         + CF21BL_BARO_LP_ALPHA * alt_raw;
            }
            float target_alt = sp.linear.z + CF21BL_ALT_SP_OFFSET;
            float delta_T    = pid_update(&g_pid_z, target_alt - alt_filt,
                                          CF21BL_LOOP_DT);
            float T_cmd = CF21BL_HOVER_T + delta_T;
            if (T_cmd < CF21BL_T_FLOOR) { T_cmd = CF21BL_T_FLOOR; }
            if (T_cmd > 1.0f)           { T_cmd = 1.0f; }
            linear_z_out = T_cmd * 2.0f - 1.0f;   /* [0,1] → [-1,+1] for mix */

            /* Log at ~2 Hz so progress is visible on console. */
            static int alt_log_div;
            if (++alt_log_div >= 500) {
                alt_log_div = 0;
                LOG_INF("alt=%.3f m  target=%.3f m  T=%.2f",
                        (double)alt_filt, (double)target_alt, (double)T_cmd);
            }
        }
#else
        float linear_z_out = sp.linear.z;
#endif

        substrate_twist_t out = {
            .linear  = { .x = 0.0f, .y = 0.0f, .z = linear_z_out },
            .angular = { .x = u_roll, .y = u_pitch, .z = u_yaw },
        };

        cf21bl_motors_t motors;
        cf21bl_mix(&out, &motors);
        cf21bl_set_motors(&motors);
    }
}

/* ── API ────────────────────────────────────────────────────────────────────── */

int cf21bl_stabilizer_start(void)
{
    int ret = cf21bl_imu_init();
    if (ret) {
        LOG_ERR("cf21bl_imu_init failed: %d", ret);
        return ret;
    }

    /* Measure static gyro bias (~1.1 s, motors at idle below spin threshold).
     * This must be done before filter init so the bias is zeroed for the
     * Mahony integral feedback, which then only needs to correct residual
     * temperature drift rather than the full static offset. */
    LOG_INF("Calibrating gyro bias — keep drone stationary on ground ...");
    cf21bl_imu_calibrate_gyro(1000);

    cf21bl_imu_filter_init();

    pid_init(&g_pid_roll,  CF21BL_RP_KP,  CF21BL_RP_KI,  CF21BL_RP_KD,
             CF21BL_RP_ILIM,  CF21BL_RP_OLIM);
    pid_init(&g_pid_pitch, CF21BL_RP_KP,  CF21BL_RP_KI,  CF21BL_RP_KD,
             CF21BL_RP_ILIM,  CF21BL_RP_OLIM);
    pid_init(&g_pid_yaw,   CF21BL_YAW_KP, CF21BL_YAW_KI, CF21BL_YAW_KD,
             CF21BL_YAW_ILIM, CF21BL_YAW_OLIM);

#ifdef CONFIG_CF21BL_ANGLE_MODE
    pid_init(&g_pid_roll_angle,  CF21BL_ANGLE_KP, CF21BL_ANGLE_KI, 0.0f,
             CF21BL_ANGLE_ILIM, CF21BL_MAX_ANGLE_RATE_RPS);
    pid_init(&g_pid_pitch_angle, CF21BL_ANGLE_KP, CF21BL_ANGLE_KI, 0.0f,
             CF21BL_ANGLE_ILIM, CF21BL_MAX_ANGLE_RATE_RPS);
#endif

#ifdef CONFIG_CF21BL_ALTITUDE_HOLD
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("BMP388 not ready");
        return -ENODEV;
    }
    pid_init(&g_pid_z, CF21BL_Z_KP, CF21BL_Z_KI, 0.0f,
             CF21BL_Z_ILIM, CF21BL_Z_OLIM);
#endif

    k_thread_create(&g_stab_thread, g_stab_stack,
                    K_THREAD_STACK_SIZEOF(g_stab_stack),
                    stabilizer_fn, NULL, NULL, NULL,
                    CF21BL_STAB_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&g_stab_thread, "cf21bl_stab");

    LOG_INF("CF21 stabilizer started (%s mode)",
            IS_ENABLED(CONFIG_CF21BL_ANGLE_MODE) ? "angle" : "rate");
    return 0;
}

void cf21bl_stabilizer_set_setpoint(const substrate_twist_t *sp)
{
    k_spinlock_key_t key = k_spin_lock(&g_sp_lock);
    g_setpoint = *sp;
    k_spin_unlock(&g_sp_lock, key);
}
