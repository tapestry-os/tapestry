/*
 * substrate_webots_cutebot.c — see substrate_webots_cutebot.h
 */

#include <tapestry/substrate.h>
#include "substrate_webots_cutebot.h"

#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>

/* Physical calibration — approximate, not a transplant of hardware's
 * exact non-linear motor curve (see cutebot-formation/src/formation.h's
 * DEMO_MAX_SPEED comment for that curve).
 *
 * WHEEL_RADIUS_M / WHEEL_TRACK_M are THIS PROTO's actual simulated wheel
 * geometry (CutebotRover.proto's wheel Cylinder radius / 2x its anchor Y)
 * — substrate_move() uses them only to convert a commanded m/s and rad/s
 * into the correct motor rad/s for whatever size this model happens to be
 * drawn at. They are NOT the real hardware's dimensions and must NOT feed
 * MAX_YAW_RATE_RADPS below (see HARDWARE_TRACK_M).
 */
#define WHEEL_RADIUS_M      0.0462f
#define WHEEL_TRACK_M       0.187f

/* HARDWARE_TRACK_M is the measured real Cutebot Mini's track width
 * (85 mm, same as hardware's DEMO_WHEEL_TRACK) — fixed regardless of how
 * this PROTO is visually scaled. MAX_SPEED_MPS approximates hardware's
 * measured ~1.08 m/s linearized top speed. MAX_YAW_RATE_RADPS is the
 * differential-drive-consistent value derived from both (2 * MAX_SPEED_MPS
 * / HARDWARE_TRACK_M) rather than an independent guess — same ratio
 * hardware implies (DEMO_MAX_OMEGA), since it cancels the abstract-unit-
 * vs-meters choice entirely (speed / track-length is scale-invariant) —
 * scale-invariant with respect to the CHOREO's abstract units, not with
 * respect to this PROTO's own visual size, which is what made it easy to
 * miscompute this from WHEEL_TRACK_M above instead. */
#define HARDWARE_TRACK_M    0.085f
#define MAX_SPEED_MPS       1.08f
#define MAX_YAW_RATE_RADPS  (2.0f * MAX_SPEED_MPS / HARDWARE_TRACK_M)

static WbDeviceTag g_left_motor, g_right_motor;
static WbDeviceTag g_gps, g_imu;

int substrate_init(void)
{
    wb_robot_init();

    g_left_motor  = wb_robot_get_device("left_wheel_motor");
    g_right_motor = wb_robot_get_device("right_wheel_motor");
    wb_motor_set_position(g_left_motor, INFINITY);
    wb_motor_set_position(g_right_motor, INFINITY);
    wb_motor_set_velocity(g_left_motor, 0.0);
    wb_motor_set_velocity(g_right_motor, 0.0);

    int timestep = (int)wb_robot_get_basic_time_step();

    g_gps = wb_robot_get_device("gps");
    wb_gps_enable(g_gps, timestep);
    g_imu = wb_robot_get_device("inertial_unit");
    wb_inertial_unit_enable(g_imu, timestep);

    return 0;
}

void substrate_move(const substrate_twist_t *twist)
{
    float linear_mps  = twist->linear.x  * MAX_SPEED_MPS;
    float angular_rps = twist->angular.z * MAX_YAW_RATE_RADPS;

    float v_left_mps  = linear_mps - angular_rps * (WHEEL_TRACK_M * 0.5f);
    float v_right_mps = linear_mps + angular_rps * (WHEEL_TRACK_M * 0.5f);

    wb_motor_set_velocity(g_left_motor,  (double)(v_left_mps  / WHEEL_RADIUS_M));
    wb_motor_set_velocity(g_right_motor, (double)(v_right_mps / WHEEL_RADIUS_M));
}

void substrate_webots_cutebot_get_position(float *x, float *y, float *z)
{
    const double *pos = wb_gps_get_values(g_gps);
    if (x) { *x = (float)pos[0]; }
    if (y) { *y = (float)pos[1]; }
    if (z) { *z = (float)pos[2]; }
}

float substrate_webots_cutebot_get_yaw(void)
{
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(g_imu);
    return (float)rpy[2];
}

void substrate_set_signal(substrate_signal_t signal)
{
    (void)signal;   /* no LED equivalent in this sim — no-op, same as cf21bl */
}

void substrate_set_power(substrate_power_state_t state)
{
    (void)state;    /* no-op — this backend has no power domain to manage */
}

int substrate_sense(substrate_sensor_t type, float *out)
{
    if (type == SUBSTRATE_SENSOR_BATTERY) {
        *out = 1.0f;   /* no modeled battery drain in this sim */
        return 0;
    }
    return -1;          /* SUBSTRATE_SENSOR_PROXIMITY: unsupported here */
}

void substrate_bond(void)    {}
void substrate_release(void) {}
void substrate_emit(void)    {}

void substrate_identify(uint8_t ordinal)
{
    (void)ordinal;   /* no viewport to look at in this sim — no-op, same
                       * as cf21bl; moot here anyway since element_id
                       * comes from controllerArgs, not negotiation. */
}
