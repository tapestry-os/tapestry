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
/*
 * BONDING and ABS_POSITION (below) are physical/positioning capabilities,
 * not follower roles — scr_tick()'s role assignment (§ "Role assignment"
 * in scr.c) only ever inspects RELAY/SENSOR/ACTUATOR by name, so these two
 * bits are invisible to role election by construction, not by omission.
 * Never gossiped (same as the three above) — set once locally at
 * scr_init() from firmware/hardware configuration.
 */
#define SCR_CAP_BONDING      ((scr_capability_t)0x08)  /* physical bonding/docking */
#define SCR_CAP_ABS_POSITION ((scr_capability_t)0x10)  /* lighthouse/GPS/mocap etc */

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
    /* Configuration — set at scr_init(), immutable thereafter (except
     * quorum_hold_ms, below, which has its own setter for the same
     * reason peer_whitelist_mask does: adding a scr_init() parameter
     * would break all 6 call sites; see scr_set_quorum_hold_ms()) */
    element_id_t       own_id;
    uint8_t            quorum_min;
    uint8_t            quorum_target;
    scr_capability_t   capabilities;
    scr_post_tick_fn   on_tick;

    /* Quorum-recovery hold — configure via scr_set_quorum_hold_ms().
     * 0 (scr_init()'s default) disables it: quorum_state tracks the
     * instantaneous classification exactly as before this field existed. */
    uint32_t           quorum_hold_ms;

    /* Internal — accumulates while a LOST -> >=DEGRADED recovery is
     * pending confirmation; see scr_set_quorum_hold_ms(). */
    uint32_t           _quorum_recovery_ms;

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

/* ── Quorum-recovery hold ─────────────────────────────────────────────────── */

/*
 * scr_set_quorum_hold_ms — Require a LOST -> >=DEGRADED recovery to be
 * SUSTAINED for hold_ms before scr_tick() reports it, instead of reporting
 * the very first tick a peer looks fresh again.
 *
 * Rationale: a single lucky gossip frame keeps a peer entry "fresh" for
 * WM_STALE_THRESHOLD_MS even under otherwise-poor link quality, so gating
 * quorum recovery on instantaneous freshness can flicker quorum_state (and
 * therefore SCR_ABORT_CLEARED) up for one tick on a single packet — not
 * real, sustained contact. Requiring hold_ms of continuous >=DEGRADED
 * classification before reporting recovery filters that out; a genuine
 * partition heal is unaffected once the network is actually reconnected
 * for that long.
 *
 * Scope — deliberately narrow, matching the flight-tested behavior this
 * generalizes (see CHANGELOG for the app-level filter this replaced):
 *   - Only the recovery edge (LOST -> >=DEGRADED) is held. Quorum LOSS is
 *     always immediate and unaffected by this setting — SCR_ABORT_TRIGGERED
 *     still fires on the very tick quorum drops.
 *   - Fluctuation entirely within the recovered zone (DEGRADED <-> HEALTHY,
 *     never touching LOST) is never held — quorum_state tracks it live.
 *   - While a recovery is held, scr_get_quorum() reports SCR_QUORUM_LOST
 *     (not the true, better instantaneous classification) and
 *     scr_get_abort_state() stays at whatever it already was — this is the
 *     whole point: L6/L7 must see a single, self-consistent view, so
 *     nothing downstream of scr_tick() (role, leader, task_slot,
 *     SCR_ABORT_CLEARED — everything Step 4 onward computes) ever observes
 *     a "recovered but still LOST" state that the L5 contract says cannot
 *     happen. SCR_ABORT_CLEARED is delayed by hold_ms as a direct
 *     consequence — this is intentional, not a side effect to work around.
 *   - The recovery timer resets to 0 immediately if quorum drops back to
 *     LOST before hold_ms elapses (no partial credit toward the next
 *     recovery attempt).
 *
 * Cold-boot acquisition is held too, not just recovery from a real prior
 * partition: scr_init() starts quorum_state at SCR_QUORUM_LOST, so the
 * very first tick(s) after boot look identical to "recovering from LOST"
 * to the logic above, and are held for the same hold_ms. This is
 * intentional and faithful to the app-level filter this generalizes
 * (its own hold counter also started at 0 on boot) — call
 * scr_set_quorum_hold_ms() once, right after scr_init(), and expect
 * every element's very first quorum acquisition to take at least
 * hold_ms, not just its recovery from a later partition.
 *
 * hold_ms=0 (scr_init()'s default) disables this entirely: quorum_state
 * tracks the instantaneous classification on every tick, exactly as before
 * this feature existed. Assumes scr_tick() is called once per WM_CYCLE_MS,
 * the same cadence every other per-cycle debounce in this codebase (BSE's
 * anchor hold, Choreo's membership hold) already assumes.
 */
void scr_set_quorum_hold_ms(scr_state_t *scr, uint32_t hold_ms);

/* ── Lightweight BFT — peer filtering ───────────────────────────────────── */

/* Restrict election candidates to a trusted set.  mask=0 allows all (default). */
void scr_set_peer_whitelist(scr_state_t *scr, uint32_t mask);

/*
 * scr_peer_is_trusted — whitelist + anomaly check for a single peer_id, the
 * same test scr_tick() applies when building its candidate set.  Exposed so
 * callers building their own peer set from the world model (L6's FORM/
 * EXCHANGE/MOVE participant collection) can apply identical filtering —
 * otherwise an anomaly-excluded or non-whitelisted peer counted by L6 but
 * not L5 diverges the two layers' rank/count and lets that peer still
 * influence geometry despite being excluded from quorum and leader election.
 * Self should never be passed here (always trusted).
 */
bool scr_peer_is_trusted(const scr_state_t *scr, element_id_t id);

/* Exclude peer_id from election until cleared (e.g. on auth failure). */
void scr_report_anomaly(scr_state_t *scr, element_id_t peer_id);

void scr_clear_anomaly(scr_state_t *scr, element_id_t peer_id);
void scr_clear_all_anomalies(scr_state_t *scr);

#endif /* TAPESTRY_SCR_PUBLIC_H */
