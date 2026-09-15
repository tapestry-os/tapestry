/* CI-only stub — the Supervisor API surface the warehouse_supervisor
 * controller uses. See webots/motor.h. */
#ifndef WEBOTS_CI_STUB_SUPERVISOR_H
#define WEBOTS_CI_STUB_SUPERVISOR_H
#include <webots/types.h>
void wb_supervisor_set_label(int id, const char *text, double x, double y,
                             double size, int color, double transparency,
                             const char *font);
#endif
