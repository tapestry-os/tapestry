/* CI-only stub — see webots/motor.h. The LED is what makes
 * substrate_set_signal() visible in the 3D view; the cf21bl substrate has
 * no equivalent, which is why this header has no counterpart in that
 * example's stub tree. */
#ifndef WEBOTS_CI_STUB_LED_H
#define WEBOTS_CI_STUB_LED_H
#include <webots/types.h>
void wb_led_set(WbDeviceTag tag, int value);
#endif
