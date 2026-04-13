/*
 * scr.h — Tapestry L5 Swarm Coordination Runtime
 *
 * The SCR sits above the CSM (L4) and provides two collective services:
 *
 *   Quorum management — tracks whether the swarm has enough reachable,
 *     fresh peers to operate with confidence.  Three levels: HEALTHY,
 *     DEGRADED, and LOST, parameterised by quorum_min and quorum_target
 *     (both expressed as peer counts, not fractions).
 *
 *   Role election — deterministic, message-free leader election.  The
 *     fresh peer with the lowest element_id is elected leader.  Every
 *     element computes this independently from its current world model
 *     snapshot; no additional messaging is required.  Convergence time
 *     is bounded by WM_STALE_THRESHOLD_MS (1500 ms).
 *
 * Relationship to L4:
 *   scr_tick() reads the world model (read-only) and derives role and
 *   quorum state from the current set of active, non-stale entries.
 *   It does not write to the world model.
 *
 * Design invariant (shared with L4):
 *   No OS-specific types, no dynamic allocation.  Pure C99.
 *   Compiles cleanly against any C99 toolchain with libm.
 */

#ifndef TAPESTRY_SCR_H
#define TAPESTRY_SCR_H

#include <stdint.h>
#include <stdbool.h>
#include "../csm/state.h"
#include "../csm/world_model.h"

/* ── Quorum health ────────────────────────────────────────────────────────── */
/*
 * SCR_QUORUM_HEALTHY   >= quorum_target fresh non-self peers visible.
 *                       Normal operation: full consensus available.
 *
 * SCR_QUORUM_DEGRADED  >= quorum_min but < quorum_target fresh peers.
 *                       Reduced confidence: proceed with caution.
 *                       L6 may choose to narrow the action envelope.
 *
 * SCR_QUORUM_LOST      < quorum_min fresh peers.
 *                       Cannot form reliable consensus.  Leader election
 *                       is suspended; role reverts to SCR_ROLE_NONE.
 */

typedef enum {
    SCR_QUORUM_LOST     = 0,
    SCR_QUORUM_DEGRADED = 1,
    SCR_QUORUM_HEALTHY  = 2,
} scr_quorum_state_t;

/* ── Swarm roles ──────────────────────────────────────────────────────────── */
/*
 * SCR_ROLE_NONE      Role undetermined.  Set when quorum is LOST.
 *
 * SCR_ROLE_FOLLOWER  Participating peer.  Executes coordinated tasks but
 *                    does not initiate collective decisions.
 *
 * SCR_ROLE_LEADER    Elected coordinator.  The fresh peer (including self)
 *                    with the lowest element_id.  Only one leader exists at
 *                    any point in time — all peers compute the same result
 *                    from their (converged) world model snapshot.
 */

typedef enum {
    SCR_ROLE_NONE     = 0,
    SCR_ROLE_FOLLOWER = 1,
    SCR_ROLE_LEADER   = 2,
} scr_role_t;

/* ── Runtime state ────────────────────────────────────────────────────────── */

typedef struct {
    element_id_t       own_id;        /* This element's ID (immutable)          */
    uint8_t            quorum_min;    /* Minimum fresh peers for DEGRADED (>=)  */
    uint8_t            quorum_target; /* Minimum fresh peers for HEALTHY (>=)   */

    /* Computed fields — updated by scr_tick() */
    scr_role_t         role;          /* Own role this tick                      */
    element_id_t       leader_id;     /* Elected leader this tick                */
    bool               leader_valid;  /* false when quorum is LOST               */
    scr_quorum_state_t quorum_state;  /* Current quorum health level             */
    uint8_t            fresh_count;   /* # non-self fresh peers this tick        */
} scr_state_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * scr_init — Initialise the SCR runtime.
 *
 * @param scr           State to initialise (caller-allocated).
 * @param own_id        This element's unique ID.
 * @param quorum_min    Minimum fresh non-self peers for DEGRADED state.
 *                      Typical value: 1.
 * @param quorum_target Minimum fresh non-self peers for HEALTHY state.
 *                      Typical value: floor(expected_swarm_size / 2).
 */
void scr_init(scr_state_t *scr,
              element_id_t own_id,
              uint8_t quorum_min,
              uint8_t quorum_target);

/*
 * scr_tick — Recompute role and quorum state from the current world model.
 *
 * Must be called after wm_tick() each cycle.  Reads the world model
 * snapshot; does not write to it.
 *
 * Algorithm:
 *   1. Count fresh (active and non-stale) non-self peers from wm.
 *   2. Classify quorum: HEALTHY / DEGRADED / LOST.
 *   3. If quorum >= DEGRADED: elect leader as min(own_id, fresh_peer_ids).
 *   4. Set role: LEADER if own_id == leader_id, else FOLLOWER.
 *      Role is NONE if quorum is LOST.
 */
void scr_tick(scr_state_t *scr, const world_model_t *wm);

/*
 * scr_get_role — Return own role as of the last scr_tick().
 */
scr_role_t scr_get_role(const scr_state_t *scr);

/*
 * scr_get_leader — Return the elected leader ID.
 * Returns ELEMENT_ID_INVALID when quorum is LOST (leader_valid == false).
 */
element_id_t scr_get_leader(const scr_state_t *scr);

/*
 * scr_get_quorum — Return the current quorum health level.
 */
scr_quorum_state_t scr_get_quorum(const scr_state_t *scr);

#endif /* TAPESTRY_SCR_H */
