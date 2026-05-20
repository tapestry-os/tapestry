/*
 * scr.h — Tapestry L5 Swarm Coordination Runtime
 *
 * The SCR sits above the CSM (L4) and provides collective services:
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
 *   Abort protocol — detects quorum-loss transitions and quorum-recovery
 *     transitions as one-tick signals (SCR_ABORT_TRIGGERED / SCR_ABORT_CLEARED)
 *     that L6 can consume each cycle without polling quorum state history.
 *
 *   Lightweight BFT — peer whitelist (deployment-time trust restriction)
 *     and anomaly exclusion (reactive fault detection) mitigate the most
 *     common Byzantine vector (rogue ID injection) for closed deployments.
 *     Full PBFT consensus is not implemented; this is a best-effort
 *     mitigation, not a formal BFT guarantee.
 *
 * Relationship to L4:
 *   scr_tick() reads the world model (read-only) and derives all state
 *   from the current set of active, non-stale entries.  It does not write
 *   to the world model.
 *
 * Design invariant (shared with L4):
 *   No OS-specific types, no dynamic allocation.  Pure C99.
 *   Compiles cleanly against any C99 toolchain with libm.
 *   Peer IDs must be in [0, 31]; BFT masks are uint32_t (MAX_ELEMENTS = 32).
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
 *                       Normal operation: full quorum consensus available
 *                       (quorum-based coordination; not formal BFT).
 *
 * SCR_QUORUM_DEGRADED  >= quorum_min but < quorum_target fresh peers.
 *                       Reduced confidence: proceed with caution.
 *                       L6 may choose to narrow the action envelope.
 *
 * SCR_QUORUM_LOST      < quorum_min fresh peers.
 *                       Cannot form reliable consensus.  Leader election
 *                       is suspended; role reverts to SCR_ROLE_NONE.
 *                       Abort protocol fires (SCR_ABORT_TRIGGERED).
 */

typedef enum {
    SCR_QUORUM_LOST     = 0,
    SCR_QUORUM_DEGRADED = 1,
    SCR_QUORUM_HEALTHY  = 2,
} scr_quorum_state_t;

/* ── Capability flags ─────────────────────────────────────────────────────── */
/*
 * Capability bits describe this element's physical role in the swarm.
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
 * SCR_ROLE_NONE      Role undetermined.  Set when quorum is LOST.
 *
 * SCR_ROLE_LEADER    Elected coordinator.  The fresh peer (including self)
 *                    with the lowest element_id.  Only one leader exists at
 *                    any point in time — all peers compute the same result
 *                    from their (converged) world model snapshot.
 *                    The leader retains this role regardless of capability flags.
 *
 * SCR_ROLE_FOLLOWER  Generic participating peer.  Set when no capability
 *                    flag maps to an extended role.
 *
 * SCR_ROLE_RELAY     Follower with SCR_CAP_RELAY capability.
 *
 * SCR_ROLE_SENSOR    Follower with SCR_CAP_SENSOR capability (no RELAY).
 *
 * SCR_ROLE_ACTUATOR  Follower with SCR_CAP_ACTUATOR capability (no RELAY or
 *                    SENSOR).
 *
 * Extended roles are self-assigned: each element independently maps its own
 * capability flags to a role.  No messaging required.
 *
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
 *                      Held until quorum recovers.  L6 should halt motion
 *                      and await the CLEARED signal.
 *
 * SCR_ABORT_CLEARED    Quorum just recovered from LOST to >= DEGRADED,
 *                      following a prior TRIGGERED event.
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
 * scr_post_tick_fn — callback invoked by scr_tick() after all L5 outputs
 * (role, quorum, task_slot, abort_state) are stable.
 *
 * Set scr_state_t::on_tick once at L2 init time to wire L6 into the L5
 * tick.  NULL disables.  Signature matches choreo_tick() so that function
 * can be assigned directly without a wrapper.
 *
 * The callback runs with the world model in read-only mode (same constraint
 * as scr_tick itself).  It must not call scr_tick() re-entrantly.
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
    scr_post_tick_fn   on_tick;       /* L6 entry point; NULL = disabled         */

    /* Computed fields — updated by scr_tick() */
    scr_role_t         role;          /* Own role this tick                      */
    element_id_t       leader_id;     /* Elected leader this tick                */
    bool               leader_valid;  /* false when quorum is LOST               */
    scr_quorum_state_t quorum_state;  /* Current quorum health level             */
    uint8_t            fresh_count;   /* # trusted non-self fresh peers this tick */
    uint8_t            task_slot;     /* Ordinal in sorted peer list (0 = leader);
                                         valid when quorum >= DEGRADED            */
    uint8_t            swarm_size;    /* Total candidates (self + fresh peers);
                                         valid when quorum >= DEGRADED            */
    scr_abort_state_t  abort_state;   /* Quorum-transition signal; see above     */

    /* Internal — do not read directly; use scr_get_abort_state() */
    scr_quorum_state_t _prev_quorum_state;

    /* BFT peer filtering — configure via scr_set_peer_whitelist() /
     * scr_report_anomaly().  Requires peer_id < 32.                */
    uint32_t           peer_whitelist_mask;  /* 0 = allow all; bit N = peer N trusted */
    uint32_t           anomaly_mask;         /* bit N set = peer N excluded from election */
} scr_state_t;

/* ── Core API ─────────────────────────────────────────────────────────────── */

/*
 * scr_init — Initialise the SCR runtime.
 *
 * @param scr           State to initialise (caller-allocated).
 * @param own_id        This element's unique ID.
 * @param quorum_min    Minimum fresh non-self peers for DEGRADED state.
 *                      Typical value: 1.
 * @param quorum_target Minimum fresh non-self peers for HEALTHY state.
 *                      Typical value: floor(expected_swarm_size / 2).
 * @param capabilities  SCR_CAP_* bitmask for this element's physical role.
 *                      Pass SCR_CAP_NONE for an unspecialised follower.
 */
void scr_init(scr_state_t *scr,
              element_id_t own_id,
              uint8_t quorum_min,
              uint8_t quorum_target,
              scr_capability_t capabilities);

/*
 * scr_tick — Recompute role and quorum state from the current world model.
 *
 * Must be called after wm_tick() each cycle.  Reads the world model
 * snapshot; does not write to it.
 *
 * Algorithm:
 *   1. Collect trusted (whitelist + anomaly-free) fresh non-self peers.
 *   2. Classify quorum: HEALTHY / DEGRADED / LOST.
 *   3. Run abort state machine: detect DEGRADED/HEALTHY→LOST and
 *      LOST→DEGRADED/HEALTHY transitions.
 *   4. If quorum >= DEGRADED: sort candidate set (self + fresh peers);
 *      task_slot = own ordinal in sorted list; leader = candidates[0].
 *   5. Set role: LEADER if own_id == leader_id, else assign extended role
 *      from capability flags (RELAY > SENSOR > ACTUATOR > FOLLOWER).
 *      Role is NONE if quorum is LOST.
 */
void scr_tick(scr_state_t *scr, const world_model_t *wm);

/* ── Role / quorum / election accessors ──────────────────────────────────── */

scr_role_t         scr_get_role(const scr_state_t *scr);
element_id_t       scr_get_leader(const scr_state_t *scr);
scr_quorum_state_t scr_get_quorum(const scr_state_t *scr);

/*
 * scr_get_task_slot — Own ordinal in the sorted fresh peer list (0 = leader).
 * Valid only when quorum >= DEGRADED.
 */
uint8_t scr_get_task_slot(const scr_state_t *scr);

/*
 * scr_get_swarm_size — Total fresh candidates (self + fresh peers).
 * Valid only when quorum >= DEGRADED.
 */
uint8_t scr_get_swarm_size(const scr_state_t *scr);

/*
 * scr_get_abort_state — Return the current abort protocol signal.
 *
 * SCR_ABORT_TRIGGERED: quorum was just lost; L6 should halt motion.
 * SCR_ABORT_CLEARED:   quorum just recovered; held for one tick only.
 * SCR_ABORT_NONE:      steady state; no transition detected.
 */
scr_abort_state_t scr_get_abort_state(const scr_state_t *scr);

/* ── Lightweight BFT — peer filtering ───────────────────────────────────── */
/*
 * These functions modify the candidate whitelist and anomaly mask.
 * Changes take effect on the next scr_tick() call.
 *
 * Peer IDs must be < 32 (uint32_t mask limit matches MAX_ELEMENTS = 32).
 *
 * Full PBFT is not implemented.  This provides closed-deployment access
 * control (whitelist) and reactive fault detection (anomaly reporting)
 * as a best-effort BFT mitigation.
 */

/*
 * scr_set_peer_whitelist — Restrict election candidates to a trusted set.
 * mask = 0 disables whitelist filtering (open deployment; default).
 * Bit N set means peer ID N is trusted.  Self is always included
 * regardless of mask.
 */
void scr_set_peer_whitelist(scr_state_t *scr, uint32_t mask);

/*
 * scr_report_anomaly — Exclude peer_id from election until cleared.
 * Call when the transport layer detects inconsistent state from a peer
 * (e.g. authentication failure, duplicate ID from an unexpected source).
 */
void scr_report_anomaly(scr_state_t *scr, element_id_t peer_id);

/*
 * scr_clear_anomaly — Re-admit a previously excluded peer.
 */
void scr_clear_anomaly(scr_state_t *scr, element_id_t peer_id);

/*
 * scr_clear_all_anomalies — Re-admit all excluded peers.
 */
void scr_clear_all_anomalies(scr_state_t *scr);

#endif /* TAPESTRY_SCR_H */
