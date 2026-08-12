/*
 * substrate_webots.c — see substrate_webots.h
 */

#include <tapestry/substrate.h>
#include "substrate_webots.h"
#include "pid_controller.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>

/* Command scaling — twist components are normalized [-1.0, 1.0] per
 * substrate.h; these map that range onto real units for pid_controller.c. */
#define MAX_SPEED_MPS      0.3
#define MAX_YAW_RATE_RADPS 1.0
#define ALT_RATE_MPS       0.3
#define ALT_MIN_M          0.03
#define ALT_MAX_M          3.0

static WbDeviceTag g_m1, g_m2, g_m3, g_m4;
static WbDeviceTag g_gps, g_gyro, g_imu;

static gains_pid_t      g_gains;
static substrate_twist_t g_latched_twist;
static double            g_height_desired = ALT_MIN_M;

static bool   g_have_prev_xy;
static double g_prev_x, g_prev_y;

static float g_last_x, g_last_y, g_last_z, g_last_yaw;

int substrate_init(void)
{
    wb_robot_init();

    g_m1 = wb_robot_get_device("m1_motor");
    g_m2 = wb_robot_get_device("m2_motor");
    g_m3 = wb_robot_get_device("m3_motor");
    g_m4 = wb_robot_get_device("m4_motor");
    wb_motor_set_position(g_m1, INFINITY);
    wb_motor_set_position(g_m2, INFINITY);
    wb_motor_set_position(g_m3, INFINITY);
    wb_motor_set_position(g_m4, INFINITY);
    wb_motor_set_velocity(g_m1, 0.0);
    wb_motor_set_velocity(g_m2, 0.0);
    wb_motor_set_velocity(g_m3, 0.0);
    wb_motor_set_velocity(g_m4, 0.0);

    int timestep = (int)wb_robot_get_basic_time_step();

    g_gps = wb_robot_get_device("gps");
    wb_gps_enable(g_gps, timestep);
    g_gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(g_gyro, timestep);
    g_imu = wb_robot_get_device("inertial_unit");
    wb_inertial_unit_enable(g_imu, timestep);

    /* Same gains as Webots' bundled crazyflie.c reference controller. */
    g_gains.kp_att_y   = 1;
    g_gains.kd_att_y   = 0.5;
    g_gains.kp_att_rp  = 0.5;
    g_gains.kd_att_rp  = 0.1;
    g_gains.kp_vel_xy  = 2;
    g_gains.kd_vel_xy  = 0.5;
    g_gains.kp_z       = 10;
    g_gains.ki_z       = 5;
    g_gains.kd_z       = 5;
    init_pid_attitude_fixed_height_controller();

    return 0;
}

void substrate_move(const substrate_twist_t *twist)
{
    g_latched_twist = *twist;
}

void substrate_webots_step(double dt)
{
    if (dt <= 0.0) {
        return;
    }

    actual_state_t actual = {0};
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(g_imu);
    actual.roll  = rpy[0];
    actual.pitch = rpy[1];
    actual.yaw_rate = wb_gyro_get_values(g_gyro)[2];

    const double *pos = wb_gps_get_values(g_gps);
    actual.altitude = pos[2];

    double vx_global = 0.0, vy_global = 0.0;
    if (g_have_prev_xy) {
        vx_global = (pos[0] - g_prev_x) / dt;
        vy_global = (pos[1] - g_prev_y) / dt;
    }
    g_prev_x = pos[0];
    g_prev_y = pos[1];
    g_have_prev_xy = true;

    double yaw = rpy[2];
    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);
    actual.vx =  vx_global * cosyaw + vy_global * sinyaw;
    actual.vy = -vx_global * sinyaw + vy_global * cosyaw;

    g_last_x = (float)pos[0];
    g_last_y = (float)pos[1];
    g_last_z = (float)pos[2];
    g_last_yaw = (float)yaw;

    g_height_desired += g_latched_twist.linear.z * ALT_RATE_MPS * dt;
    if (g_height_desired < ALT_MIN_M) { g_height_desired = ALT_MIN_M; }
    if (g_height_desired > ALT_MAX_M) { g_height_desired = ALT_MAX_M; }

    desired_state_t desired = {0};
    desired.roll     = 0.0;
    desired.pitch    = 0.0;
    desired.yaw_rate = g_latched_twist.angular.z * MAX_YAW_RATE_RADPS;
    desired.vx       = g_latched_twist.linear.x * MAX_SPEED_MPS;
    desired.vy       = g_latched_twist.linear.y * MAX_SPEED_MPS;
    desired.altitude = g_height_desired;

    motor_power_t mp = {0};
    pid_velocity_fixed_height_controller(actual, &desired, g_gains, dt, &mp);

    /* Alternating sign convention matches Webots' reference crazyflie.c —
     * adjacent rotors spin opposite directions. */
    wb_motor_set_velocity(g_m1, -mp.m1);
    wb_motor_set_velocity(g_m2,  mp.m2);
    wb_motor_set_velocity(g_m3, -mp.m3);
    wb_motor_set_velocity(g_m4,  mp.m4);
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

void substrate_set_signal(substrate_signal_t signal)
{
    (void)signal;   /* no LED/tone/marker equivalent in this sim — no-op */
}

void substrate_set_power(substrate_power_state_t state)
{
    (void)state;    /* no-op — this backend has no power domain to manage */
}

int substrate_sense(substrate_sensor_t type, float *out)
{
    if (type == SUBSTRATE_SENSOR_BATTERY) {
        *out = 1.0f;   /* no modeled battery drain in this sim (M1 scope) */
        return 0;
    }
    return -1;          /* SUBSTRATE_SENSOR_PROXIMITY: unsupported here */
}

void substrate_bond(void)    {}
void substrate_release(void) {}
void substrate_emit(void)    {}
