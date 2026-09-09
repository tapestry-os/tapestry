/* See ../../../controllers/common/webots_stub/webots/types.h. */

#ifndef WEBOTS_CI_STUB_GPS_H
#define WEBOTS_CI_STUB_GPS_H

#include <webots/types.h>

void wb_gps_enable(WbDeviceTag tag, int sampling_period);
const double *wb_gps_get_values(WbDeviceTag tag);

#endif
