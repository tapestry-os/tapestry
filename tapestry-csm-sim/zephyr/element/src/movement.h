/*
 * movement.h — Tapestry element position update
 *
 * Computes each element's next position every cycle using two forces:
 *
 *   Random walk — a small random displacement in any 2-D direction,
 *                 scaled to at most WALK_STEP_MAX per cycle.
 *
 *   Repulsion   — a push away from every active peer within
 *                 REPULSION_RADIUS, read from the local world model.
 *                 Force is proportional to (REPULSION_RADIUS − dist).
 *
 * The net displacement vector is clamped to WALK_STEP_MAX before being
 * applied, then the resulting position is clamped within [0, WORLD_SIZE].
 */

#ifndef TAPESTRY_MOVEMENT_H
#define TAPESTRY_MOVEMENT_H

#include "state.h"
#include "world_model.h"

/*
 * movement_tick — Apply one position update to own_state.
 *
 * Reads peer positions from wm (read-only).
 * Increments own_state->update_seq.
 * Does NOT call wm_update_self — the caller must do that afterwards.
 */
void movement_tick(element_state_t *own_state, const world_model_t *wm);

#endif /* TAPESTRY_MOVEMENT_H */
