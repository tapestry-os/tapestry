/* See types.h for what this stub tree is and isn't. */

#ifndef WEBOTS_CI_STUB_ROBOT_H
#define WEBOTS_CI_STUB_ROBOT_H

#include "types.h"

void wb_robot_init(void);
void wb_robot_cleanup(void);
int wb_robot_step(int duration);
double wb_robot_get_basic_time_step(void);
WbDeviceTag wb_robot_get_device(const char *name);

#endif
