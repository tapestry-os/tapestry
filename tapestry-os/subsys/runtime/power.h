/*
 * power.h — Tapestry L2 power state machine (internal)
 *
 * Private interface between runtime.c and power.c.
 * Consumers of the L2 layer use <tapestry/runtime.h> instead.
 */

#ifndef TAPESTRY_POWER_H
#define TAPESTRY_POWER_H

#include <tapestry/scr.h>      /* scr_quorum_state_t */
#include <tapestry/substrate.h>

/*
 * tapestry_power_init — Reset state machine to ACTIVE.
 * Called once from tapestry_runtime_init().
 */
void tapestry_power_init(void);

/*
 * tapestry_power_tick — Run one cycle of the auto-stepping policy.
 *
 * When CONFIG_TAPESTRY_POWER_AUTO_POLICY=y:
 *   - SCR_QUORUM_LOST held for >= CONFIG_TAPESTRY_POWER_SLEEP_GRACE_CYCLES
 *     consecutive ticks steps the element from ACTIVE to IDLE.
 *   - Any recovery (quorum >= DEGRADED) immediately steps back to ACTIVE.
 * When the option is disabled, this function is a no-op.
 *
 * Called from tapestry_runtime_tick() after scr_tick().
 */
void tapestry_power_tick(scr_quorum_state_t quorum_state);

#endif /* TAPESTRY_POWER_H */
