/*
 * webots_stub.c — trivial definitions for the CI-only headers in
 * ./webots/ (the AMR's device set: motor, gps, inertial_unit, led) plus
 * the substrate-agnostic wb_robot_* declarations reused from
 * ../../webots-formation/controllers/common/webots_stub/.
 *
 * Just enough to let ../Makefile LINK a binary — which is what turns this
 * from "syntax-checks against fabricated prototypes" into "also catches
 * signature mismatches". Nothing here is ever run: wb_robot_step() returns
 * -1 immediately so that main()'s loop cannot spin if it ever were.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/led.h>
#include <webots/supervisor.h>

void wb_robot_init(void) {}
void wb_robot_cleanup(void) {}
int wb_robot_step(int duration) { (void)duration; return -1; }
double wb_robot_get_basic_time_step(void) { return 16.0; }
WbDeviceTag wb_robot_get_device(const char *name) { (void)name; return 0; }

void wb_motor_set_position(WbDeviceTag tag, double position) { (void)tag; (void)position; }
void wb_motor_set_velocity(WbDeviceTag tag, double velocity) { (void)tag; (void)velocity; }

void wb_gps_enable(WbDeviceTag tag, int sampling_period) { (void)tag; (void)sampling_period; }
const double *wb_gps_get_values(WbDeviceTag tag)
{
    (void)tag;
    static const double v[3];
    return v;
}

void wb_inertial_unit_enable(WbDeviceTag tag, int sampling_period) { (void)tag; (void)sampling_period; }
const double *wb_inertial_unit_get_roll_pitch_yaw(WbDeviceTag tag)
{
    (void)tag;
    static const double v[3];
    return v;
}

void wb_led_set(WbDeviceTag tag, int value) { (void)tag; (void)value; }

void wb_supervisor_set_label(int id, const char *text, double x, double y,
                             double size, int color, double transparency,
                             const char *font)
{
    (void)id; (void)text; (void)x; (void)y;
    (void)size; (void)color; (void)transparency; (void)font;
}
