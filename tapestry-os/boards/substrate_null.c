/*
 * tapestry-os/boards/substrate_null.c
 * No-op substrate implementation for simulation and boards without actuators.
 *
 * Implements <tapestry/substrate.h> as stubs.  Selected at build time when no
 * hardware-specific substrate source is compiled in.
 */

#include <tapestry/substrate.h>

int  substrate_init(void)                              { return 0; }
void substrate_move(const substrate_twist_t *twist)    { (void)twist; }
void substrate_set_signal(substrate_signal_t signal)   { (void)signal; }
void substrate_set_power(substrate_power_state_t state){ (void)state; }
int  substrate_sense(substrate_sensor_t type, float *out)
{
    (void)type;
    (void)out;
    return -1;
}
void substrate_bond(void)    {}
void substrate_release(void) {}
void substrate_emit(void)    {}
