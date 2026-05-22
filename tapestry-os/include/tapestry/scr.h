/*
 * tapestry/scr.h — Tapestry Swarm Coordination Runtime (L5) public API
 *
 * Includes <tapestry/csm.h> (L4), so including this header gives access
 * to the complete L4–L5 public surface.
 *
 * The SCR sits above the CSM and provides collective services:
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
 *   Extended roles — non-leaders self-assign a functional role from
 *     their capability flags (relay, sensor, actuator).  All peers
 *     independently compute the same leader; each follower independently
 *     derives its own extended role.  Message-free invariant is preserved.
 *
 *   Task slot — each element's ordinal index in the sorted fresh peer
 *     list (0 = leader).  Gives L6 BSE a deterministic, collision-free
 *     vertex-assignment index without L5 performing task decomposition.
 *
 *   Abort protocol — detects quorum-loss and quorum-recovery transitions
 *     as one-tick signals (SCR_ABORT_TRIGGERED / SCR_ABORT_CLEARED) that
 *     L6 can consume each cycle without polling quorum state history.
 *
 *   Lightweight BFT — peer whitelist (deployment-time trust restriction)
 *     and anomaly exclusion (reactive fault detection) mitigate the most
 *     common Byzantine vector (rogue ID injection) for closed deployments.
 *     Full PBFT consensus is not implemented; this is a best-effort
 *     mitigation, not a formal BFT guarantee.
 *
 * Design invariant (shared with L4):
 *   No OS-specific types, no dynamic allocation.  Pure C99.
 *   Compiles cleanly against any C99 toolchain with libm.
 *   Peer IDs must be in [0, 31]; BFT masks are uint32_t (MAX_ELEMENTS = 32).
 */

#ifndef TAPESTRY_SCR_PUBLIC_H
#define TAPESTRY_SCR_PUBLIC_H

#include <stdint.h>
#include <stdbool.h>
#include <tapestry/csm.h>

/* ── Quorum health ────────────────────────────────────────────────────────── */
/*
 * SCR_QUORUM_HEALTHY   >= quorum_target fresh non-self peers visible.
 *                       Normal operation; full quorum consensus available.
 *
 * SCR_QUORUM_DEGRADED  >= quorum_min but < quorum_target fresh peers.
 *                       Reduced confidence: proceed with caution.
 *                       L6 may choose to narrow the action envelope.
 *
 * SCR_QUORUM_LOST      < quorum_min fresh peers.
 *                       Cannot form reliable consensus.  Leader election
 *                       suspended; role reverts to SCR_ROLE_NONE.
 *                       Abort protocol fires (SCR_ABORT_TRIGGERED).
 */

typedef enum {
    SCR_QUORUM_LOST     = 0,
    SCR_QUORUM_DEGRADED = 1,
    SCR_QUORUM_HEALTHY  = 2,
} scr_quorum_state_t;

/* ── Capability flags ─────────────────────────────────────────────────────── */
/*
 * Set once at scr_init() from firmware or hardware configuration.
 * Multiple bits may be set; the highest-priority set bit determines
 * the extended follower role (RELAY > SENSOR > ACTUATOR > FOLLOWER).
 */

typedef uint8_t scr_capability_t;

#define SCR_CAP_NONE     ((scr_capability_t)0x00)  /* unspecialised follower    */
#define SCR_CAP_RELAY    ((scr_capability_t)0x01)  /* message-forwarding node   */
#define SCR_CAP_SENSOR   ((scr_capability_t)0x02)  /* sensing / observation node */
#define SCR_CAP_ACTUATOR ((scr_capability_t)0x04)  /* physical actuation node   */

/* ── Swarm roles ──────────────────────────────────────────────────────────── */
/*
 * Wire-format note: NONE=0, FOLLOWER=1, LEADER=2 are stable across versions.
 * Extended roles 3–5 are additive; older orchestrators may see them as unknown.
 */

typedef enum {
    SCR_ROLE_NONE     = 0,
    SCR_ROLE_FOLLOWER = 1,
    SCR_ROLE_LEADER   = 2,
    SCR_ROLE_RELAY    = 3,
    SCR_ROLE_SENSOR   = 4,
    SCR_ROLE_ACTUATOR = 5,
} scr_role_t;

/* ── Abort state ─────────────────────────────────────────────────────────── */
/*
 * SCR_ABORT_NONE       Normal operation or startup; no quorum transition.
 *
 * SCR_ABORT_TRIGGERED  Quorum just dropped from >= DEGRADED to LOST.
 *                      Held until quorum recovers.  L6 should halt motion.
 *
 * SCR_ABORT_CLEARED    Quorum just recovered from LOST to >= DEGRADED.
 *                      Held for exactly one tick, then reset to NONE.
 *                      L6 may resume normal operation.
 */

typedef enum {
    SCR_ABORT_NONE      = 0,
    SCR_ABORT_TRIGGERED = 1,
    SCR_ABORT_CLEARED   = 2,
} scr_abort_state_t;

/* ── Post-tick hook ───────────────────────────────────────────────────────── */
/*
 * Callback invoked by scr_tick() after all L5 outputs are stable.
 * Set scr_state_t::on_tick once at init to wire L6 into the L5 tick.
 * NULL disables.  Signature matches choreo_tick() for direct assignment.
 * Must not call scr_tick() re-entrantly.
 */
struct scr_state;
typedef void (*scr_post_tick_fn)(const world_model_t *wm,
                                 const struct scr_state *scr);

/* ── Runtime state ────────────────────────────────────────────────────────── */

typedef struct scr_state {
    /* Configuration — set at scr_init(), immutable thereafter */
    element_id_t       own_id;
    uint8_t            quorum_min;
    uint8_t            quorum_target;
    scr_capability_t   capabilities;
    scr_post_tick_fn   on_tick;

    /* Computed fields — updated by scr_tick() */
    scr_role_t         role;
    element_id_t       leader_id;
    bool               leader_valid;
    scr_quorum_state_t quorum_state;
    uint8_t            fresh_count;
    uint8_t            task_slot;      /* ordinal in sorted peer list (0=leader);
                                          valid when quorum >= DEGRADED          */
    uint8_t            swarm_size;     /* self + fresh peers;
                                          valid when quorum >= DEGRADED          */
    scr_abort_state_t  abort_state;

    /* Internal — use scr_get_abort_state() */
    scr_quorum_state_t _prev_quorum_state;

    /* BFT peer filtering — configure via scr_set_peer_whitelist() /
     * scr_report_anomaly().  Requires peer_id < 32.                */
    uint32_t           peer_whitelist_mask;
    uint32_t           anomaly_mask;
} scr_state_t;

/* ── Core API ─────────────────────────────────────────────────────────────── */

void scr_init(scr_state_t *scr,
              element_id_t own_id,
              uint8_t quorum_min,
              uint8_t quorum_target,
              scr_capability_t capabilities);

/*
 * Recompute role and quorum from the current world model.  Call after
 * wm_tick() each cycle.  Reads the world model; does not write to it.
 */
void scr_tick(scr_state_t *scr, const world_model_t *wm);

/* ── Accessors ───────────────────────────────────────────────────────────── */

scr_role_t         scr_get_role(const scr_state_t *scr);
element_id_t       scr_get_leader(const scr_state_t *scr);
scr_quorum_state_t scr_get_quorum(const scr_state_t *scr);
uint8_t            scr_get_task_slot(const scr_state_t *scr);
uint8_t            scr_get_swarm_size(const scr_state_t *scr);
scr_abort_state_t  scr_get_abort_state(const scr_state_t *scr);

/* ── Lightweight BFT — peer filtering ───────────────────────────────────── */

/* Restrict election candidates to a trusted set.  mask=0 allows all (default). */
void scr_set_peer_whitelist(scr_state_t *scr, uint32_t mask);

/* Exclude peer_id from election until cleared (e.g. on auth failure). */
void scr_report_anomaly(scr_state_t *scr, element_id_t peer_id);

void scr_clear_anomaly(scr_state_t *scr, element_id_t peer_id);
void scr_clear_all_anomalies(scr_state_t *scr);

#endif /* TAPESTRY_SCR_PUBLIC_H */
