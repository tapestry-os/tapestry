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

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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

#ifdef CONFIG_CF21BL_ANGLE_MODE
        /* ── Outer angle loop ─────────────────────────────────────────────── */
        cf21bl_imu_attitude_t att;
        cf21bl_imu_filter_update(&sample, CF21BL_LOOP_DT, &att);

        float roll_sp_deg  = sp.angular.x * CF21BL_MAX_ANGLE_DEG;
        float pitch_sp_deg = sp.angular.y * CF21BL_MAX_ANGLE_DEG;

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

        substrate_twist_t out = {
            .linear  = { .x = 0.0f, .y = 0.0f, .z = sp.linear.z },
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
