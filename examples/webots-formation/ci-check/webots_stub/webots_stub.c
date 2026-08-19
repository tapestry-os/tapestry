/*
 * webots_stub.c — trivial definitions for the CI-only headers declared
 * across ../../controllers/common/webots_stub/webots/ (robot.h, shared
 * with any future substrate) and ./webots/ (motor/gps/gyro/inertial_unit,
 * cf21bl's own device set) — just enough to let ../Makefile link a
 * binary. This is what turns the stub build from "syntax-checks against
 * fabricated prototypes" into "also catches signature mismatches the
 * linker would catch" — cf21bl's main.c/substrate_webots.c call these
 * exactly as they'd call the real Webots libcontroller. Nothing here is
 * ever run: CI builds this binary and throws it away, never executes it
 * (wb_robot_step() below returns -1 immediately for exactly that reason —
 * if it ever were invoked, main()'s loop must not spin).
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>

void wb_robot_init(void) {}
void wb_robot_cleanup(void) {}
int wb_robot_step(int duration) { (void)duration; return -1; }
double wb_robot_get_basic_time_step(void) { return 8.0; }
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

void wb_gyro_enable(WbDeviceTag tag, int sampling_period) { (void)tag; (void)sampling_period; }
const double *wb_gyro_get_values(WbDeviceTag tag)
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
