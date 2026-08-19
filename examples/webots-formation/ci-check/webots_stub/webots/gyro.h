/* See ../../../controllers/common/webots_stub/webots/types.h. */

#ifndef WEBOTS_CI_STUB_GYRO_H
#define WEBOTS_CI_STUB_GYRO_H

#include <webots/types.h>

void wb_gyro_enable(WbDeviceTag tag, int sampling_period);
const double *wb_gyro_get_values(WbDeviceTag tag);

#endif
