/* See ../../../controllers/common/webots_stub/webots/types.h. */

#ifndef WEBOTS_CI_STUB_INERTIAL_UNIT_H
#define WEBOTS_CI_STUB_INERTIAL_UNIT_H

#include <webots/types.h>

void wb_inertial_unit_enable(WbDeviceTag tag, int sampling_period);
const double *wb_inertial_unit_get_roll_pitch_yaw(WbDeviceTag tag);

#endif
