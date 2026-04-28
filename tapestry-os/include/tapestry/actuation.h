/*
 * tapestry/actuation.h — Tapestry Board Actuation API
 *
 * Stable interface between application logic and the physical hardware that
 * a Tapestry element drives: motors, LEDs, or any other output.
 *
 * Applications include this header and call these functions directly.  The
 * build system selects the implementation. Add support for a new element:
 * 
 *   1. Create tapestry-os/boards/<your_board>/actuation_<your_element>.c
 *   2. Implement the four functions below.
 *   3. In your app's CMakeLists.txt, compile that file when your board
 *      config is active (see examples/collective-formation/CMakeLists.txt). 
 *
 * Or use the following no-op stub for boards without physical actuators or 
 * for simulation:
 *   tapestry-os/boards/actuation_null.c
 */

#ifndef TAPESTRY_ACTUATION_H
#define TAPESTRY_ACTUATION_H

#include <stdint.h>
#include <tapestry/scr.h>   /* scr_role_t, scr_quorum_state_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the actuation hardware.
 * Returns 0 on success, negative errno if the hardware is unreachable.
 * Callers should log a warning and continue if this returns non-zero.
 * The remaining actuation calls are no-ops when hardware is absent. */
int actuation_init(void);

/* Assuming two motors for now, set left and right motor speeds in percent [-100, 100].
 *   > 0  = forward,  < 0  = backward,  0 = stop (coast). */
void actuation_drive(int left_pct, int right_pct);

/* Assuming indicator LEDs can be set to an explicit RGB colour.
 * Useful for application-defined peer-count or status feedback. */
void actuation_set_leds(uint8_t r, uint8_t g, uint8_t b);

/* Update motors and LEDs from the current L5 SCR role and quorum state.
 * Intended to be called once per main-loop cycle on elements running L5.
 * The mapping from (role, quorum) to physical behaviour is
 * implementation-defined (see ../boards/ for reference implementations). */
void actuation_update(scr_role_t role, scr_quorum_state_t quorum);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_ACTUATION_H */
