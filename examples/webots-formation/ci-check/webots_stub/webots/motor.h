/* CI-only stub — cf21bl's device set specifically, not shared with any
 * future substrate. See ../../../controllers/common/webots_stub/webots/
 * types.h for the substrate-agnostic half of this stub tree, and
 * ../../Makefile for why this one lives here instead. */

#ifndef WEBOTS_CI_STUB_MOTOR_H
#define WEBOTS_CI_STUB_MOTOR_H

#include <webots/types.h>

void wb_motor_set_position(WbDeviceTag tag, double position);
void wb_motor_set_velocity(WbDeviceTag tag, double velocity);

#endif
