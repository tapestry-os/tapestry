/*
 * substrate_webots.c — see substrate_webots.h
 */

#include <tapestry/substrate.h>
#include "substrate_webots.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/led.h>

/* Physical constants — must match ../../protos/WarehouseAMR.proto. There is
 * no way to read wheelRadius/trackWidth back out of a PROTO through the
 * controller API, so these are duplicated by necessity; the PROTO carries a
 * matching comment pointing here. */
#define WHEEL_RADIUS_M   0.08
#define TRACK_WIDTH_M    0.36

/* Command scaling — twist components are normalized [-1, 1] per
 * substrate.h. 0.9 m/s is a deliberately conservative warehouse AMR cruise
 * (real ones do 1.5-2.0 m/s); it is well above the 0.7 m/s rate at which
 * tracker.c advances the commanded setpoint (DEMO_MAX_SPEED_MPS, set in
 * sources.mk), so the setpoint — not the motor — is what limits travel
 * speed, which is the same ordering the cf21bl substrate has. */
#define MAX_SPEED_MPS      0.9
#define MAX_YAW_RATE_RADPS 2.0

/* Motor velocity ceiling, rad/s — matches maxVelocity in the PROTO's
 * RotationalMotor nodes. Exceeding it is silently clamped BY WEBOTS, per
 * wheel, which would distort a turn (one wheel clipped, the other not); the
 * proportional rescale in apply_wheels() below exists to keep the commanded
 * v/omega ratio intact instead. */
#define MAX_WHEEL_RADPS   14.0

static WbDeviceTag g_left, g_right;
static WbDeviceTag g_gps, g_imu;
static WbDeviceTag g_led;
static bool        g_has_led;

static substrate_twist_t g_latched_twist;
static bool              g_halted;

static float g_last_x, g_last_y, g_last_z;
static float g_last_roll, g_last_pitch, g_last_yaw;

int substrate_init(void)
{
    wb_robot_init();

    g_left  = wb_robot_get_device("left wheel motor");
    g_right = wb_robot_get_device("right wheel motor");

    /* Velocity control: position INFINITY frees the joint from position
     * control so wb_motor_set_velocity() is what actually drives it —
     * the same idiom the cf21bl substrate uses for its rotors. */
    wb_motor_set_position(g_left,  INFINITY);
    wb_motor_set_position(g_right, INFINITY);
    wb_motor_set_velocity(g_left,  0.0);
    wb_motor_set_velocity(g_right, 0.0);

    int timestep = (int)wb_robot_get_basic_time_step();

    g_gps = wb_robot_get_device("gps");
    wb_gps_enable(g_gps, timestep);
    g_imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(g_imu, timestep);

    /* The status LED is how substrate_set_signal() becomes visible in the
     * 3D view at all. Optional: a world that omits it still runs. */
    g_led     = wb_robot_get_device("status led");
    g_has_led = (g_led != 0);

    return 0;
}

void substrate_move(const substrate_twist_t *twist)
{
    if (twist == NULL || g_halted) {
        return;
    }
    g_latched_twist = *twist;
}

/* Convert a (forward m/s, yaw rad/s) pair into the two wheel angular
 * velocities of a differential drive, then scale BOTH down proportionally
 * if either exceeds the motor ceiling.
 *
 * The proportional rescale is the whole point: clamping each wheel
 * independently changes the ratio between them, which changes the turn
 * radius the caller asked for. Scaling both preserves the commanded path
 * and only slows it down. */
static void apply_wheels(double v_mps, double omega_radps)
{
    double half_track = 0.5 * TRACK_WIDTH_M;
    double wl = (v_mps - omega_radps * half_track) / WHEEL_RADIUS_M;
    double wr = (v_mps + omega_radps * half_track) / WHEEL_RADIUS_M;

    double peak = fmax(fabs(wl), fabs(wr));
    if (peak > MAX_WHEEL_RADPS) {
        double scale = MAX_WHEEL_RADPS / peak;
        wl *= scale;
        wr *= scale;
    }

    wb_motor_set_velocity(g_left,  wl);
    wb_motor_set_velocity(g_right, wr);
}

void substrate_webots_step(double dt)
{
    (void)dt;   /* no integrating state on this platform — unlike the
                 * cf21bl backend, whose altitude setpoint integrates */

    const double *p = wb_gps_get_values(g_gps);
    if (p != NULL) {
        g_last_x = (float)p[0];
        g_last_y = (float)p[1];
        g_last_z = (float)p[2];
    }

    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(g_imu);
    if (rpy != NULL) {
        g_last_roll  = (float)rpy[0];
        g_last_pitch = (float)rpy[1];
        g_last_yaw   = (float)rpy[2];
    }

    if (g_halted) {
        wb_motor_set_velocity(g_left,  0.0);
        wb_motor_set_velocity(g_right, 0.0);
        return;
    }

    /* linear.y / linear.z are deliberately not read — see the twist
     * convention note in substrate_webots.h. */
    apply_wheels((double)g_latched_twist.linear.x  * MAX_SPEED_MPS,
                 (double)g_latched_twist.angular.z * MAX_YAW_RATE_RADPS);
}

void substrate_webots_halt(void)
{
    g_halted = true;
    g_latched_twist = (substrate_twist_t){0};
    wb_motor_set_velocity(g_left,  0.0);
    wb_motor_set_velocity(g_right, 0.0);
}

void substrate_webots_get_position(float *x, float *y, float *z)
{
    if (x) { *x = g_last_x; }
    if (y) { *y = g_last_y; }
    if (z) { *z = g_last_z; }
}

float substrate_webots_get_yaw(void)
{
    return g_last_yaw;
}

void substrate_webots_get_orientation(substrate_quat_t *q)
{
    if (q == NULL) {
        return;
    }
    /* ZYX intrinsic (aerospace) roll/pitch/yaw -> unit quaternion, same
     * conversion and convention as the cf21bl substrate's. */
    float cr = cosf(g_last_roll  * 0.5f), sr = sinf(g_last_roll  * 0.5f);
    float cp = cosf(g_last_pitch * 0.5f), sp = sinf(g_last_pitch * 0.5f);
    float cy = cosf(g_last_yaw   * 0.5f), sy = sinf(g_last_yaw   * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

/* ── Remaining substrate.h surface ───────────────────────────────────────── */

void substrate_set_signal(substrate_signal_t signal)
{
    if (!g_has_led) {
        return;
    }
    /* LED colour indices are defined by the PROTO's LED node colour array —
     * see ../../protos/WarehouseAMR.proto. Order matches substrate_signal_t
     * so this is an index, not a lookup table. */
    wb_led_set(g_led, (int)signal);
}

void substrate_set_power(substrate_power_state_t state)
{
    (void)state;   /* no modeled power states on this platform */
}

int substrate_sense(substrate_sensor_t type, float *out)
{
    if (type == SUBSTRATE_SENSOR_BATTERY) {
        *out = 1.0f;   /* no modeled battery drain in this sim, same as the
                        * cf21bl Webots backend */
        return 0;
    }
    return -1;         /* SUBSTRATE_SENSOR_PROXIMITY: unsupported here */
}

void substrate_bond(void)    { }
void substrate_release(void) { }
void substrate_emit(void)    { }
