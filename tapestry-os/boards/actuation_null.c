/*
 * tapestry-os/boards/actuation_null.c
 * No-op actuation implementation
 *
 * Implements <tapestry/actuation.h> as a stub for boards without physical
 * actuators and for simulation targets.  Selected at build time when no
 * hardware-specific actuation source is compiled in.
 */

#include <tapestry/actuation.h>

int actuation_init(void)                                       { return 0; }
void actuation_drive(int left_pct, int right_pct)              { (void)left_pct; (void)right_pct; }
void actuation_set_leds(uint8_t r, uint8_t g, uint8_t b)      { (void)r; (void)g; (void)b; }
void actuation_update(scr_role_t role, scr_quorum_state_t q)   { (void)role; (void)q; }
