/*
 * world_model.h — Tapestry L4 Simulation
 *
 * The Collective State Manager (CSM) — L4 of the Tapestry stack.
 *
 * Each element maintains one instance of world_model_t: its local
 * understanding of where every other element is. No element holds
 * the complete authoritative world model — the complete picture exists
 * only as the union of all elements' local models.
 *
 * Data model:
 *   - The world model is a fixed-size table of wm_entry_t, indexed by
 *     element ID. This is Option A (flat table) as discussed — correct
 *     for 20 elements; will be refactored to quadtree for Option B.
 *   - Each entry holds the last known state of that element plus
 *     metadata: when we last heard from it, how stale it is, and
 *     whether we consider it active.
 *   - The owner element's own entry is always authoritative and never
 *     expires. All other entries are gossip-propagated replicas.
 *
 * Consistency modes — a continuous dial:
 *   consistency_bias = 0.0  — Pure AP (Available/Partition-tolerant).
 *     Quorum threshold is 0: the element never freezes.  World models
 *     diverge freely during a partition; reconciliation converges after heal.
 *
 *   consistency_bias = 1.0  — Pure CP (Consistent/Partition-tolerant).
 *     Quorum threshold is WM_QUORUM_FRACTION (0.5): the element freezes when
 *     fewer than 50% of known peers are fresh.  No divergence in own-state
 *     during partition, at the cost of availability.
 *
 *   0.0 < consistency_bias < 1.0  — Hybrid.
 *     Quorum threshold scales linearly: threshold = bias * WM_QUORUM_FRACTION.
 *     The element degrades gracefully — it enters a reduced-confidence state
 *     before fully freezing, rather than switching abruptly.  The confidence
 *     metric reports how far from the freeze point the element currently is.
 *
 * Staleness:
 *   Each entry has an age_ms counter incremented each cycle. Entries
 *   older than WM_STALE_THRESHOLD_MS are flagged stale. Entries older
 *   than WM_EXPIRE_THRESHOLD_MS are marked inactive (element presumed
 *   dead or unreachable). Staleness directly drives the consistency
 *   metric reported in telemetry.
 */

#ifndef TAPESTRY_WORLD_MODEL_H
#define TAPESTRY_WORLD_MODEL_H

#include <stdint.h>
#include <stdbool.h>
#include "state.h"

/* ── Timing constants ────────────────────────────────────────────────────── */

#define GOSSIP_INTERVAL_MS        500   /* How often each element gossips     */
#define WM_STALE_THRESHOLD_MS    1500   /* Entry flagged stale after this     */
#define WM_EXPIRE_THRESHOLD_MS   5000   /* Entry marked inactive after this   */
#define WM_CYCLE_MS               100   /* World model update tick rate       */

/* ── Quorum fraction — maximum threshold, reached at consistency_bias=1.0 ── */

#define WM_QUORUM_FRACTION       0.5f   /* Effective threshold = bias * 0.5   */

/* ── World model entry ───────────────────────────────────────────────────── */
/*
 * One entry per known element. The owner's own entry has is_self = true
 * and is always fresh. All other entries are gossip replicas.
 */

typedef struct {
    element_state_t state;          /* Last known state of this element       */
    uint32_t        age_ms;         /* Milliseconds since last update received*/
    bool            is_active;      /* False if expired (presumed dead)       */
    bool            is_stale;       /* True if age_ms > WM_STALE_THRESHOLD_MS */
    bool            is_self;        /* True if this entry belongs to owner    */
    uint32_t        update_count;   /* How many gossip updates received       */
} wm_entry_t;

/* ── Consistency metric ──────────────────────────────────────────────────── */
/*
 * Computed each cycle. Reported in telemetry.
 *
 * fresh_ratio  = active_fresh / active_total  [0.0, 1.0]
 * quorum_held  = fresh_ratio >= (bias * WM_QUORUM_FRACTION)
 * confidence   = fresh_ratio / quorum_threshold, clamped to [0.0, 1.0].
 *                1.0 means the element is at or above quorum.
 *                0.0 means no fresh peers at all (full partition).
 *                Values in between indicate partial degradation.
 *                Always 1.0 when consistency_bias == 0.0 (pure AP).
 * degraded     = !quorum_held — element is below its quorum threshold.
 */

typedef struct {
    uint8_t  active_total;      /* Elements currently marked active           */
    uint8_t  active_fresh;      /* Active elements with non-stale entries     */
    uint8_t  active_stale;      /* Active elements with stale entries         */
    uint8_t  inactive_total;    /* Elements marked inactive (expired)         */
    uint8_t  collision_count;   /* Collisions detected this cycle             */
    float    fresh_ratio;       /* active_fresh / active_total [0.0, 1.0]    */
    bool     quorum_held;       /* True if fresh_ratio >= effective threshold */
    bool     degraded;          /* True when quorum lost (any bias > 0)       */
    float    confidence;        /* Proximity to quorum [0.0, 1.0]            */
} wm_consistency_metric_t;

/* ── World model ─────────────────────────────────────────────────────────── */

typedef struct {
    element_id_t            owner_id;              /* This element's own ID   */
    wm_entry_t              entries[MAX_ELEMENTS]; /* One slot per possible ID*/
    uint8_t                 known_count;           /* How many IDs ever seen  */
    float                   consistency_bias;      /* 0.0=AP .. 1.0=CP        */
    wm_consistency_metric_t metric;                /* Current consistency state*/
    uint32_t                cycle_count;           /* Total update cycles run */

    /* Reconciliation state — populated after partition heal */
    uint32_t                last_reconcile_cycle;  /* Cycle when last reconcile*/
    uint32_t                reconcile_duration_ms; /* How long reconcile took  */
} world_model_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * wm_init — Initialize world model for an element.
 * Call once at startup with the element's own initial state.
 */
void wm_init(world_model_t *wm,
             element_id_t owner_id,
             const element_state_t *own_state,
             float consistency_bias);

/*
 * wm_update_self — Update the owner's own entry.
 * Called after each local state change (position update, power change).
 * Increments the logical clock and writes the new value back to own_state
 * so the caller's copy stays in sync with the world model entry.
 * Never marks own entry stale or expired.
 */
void wm_update_self(world_model_t *wm, element_state_t *own_state);

/*
 * wm_receive_gossip — Process an incoming gossip message.
 * Applies Lamport clock merge: entry.logical_clock = max(local, received)+1.
 * Resets age_ms for the received entry. Marks entry active and fresh.
 * Returns true if this gossip updated our knowledge (clock was newer).
 */
bool wm_receive_gossip(world_model_t *wm, const element_state_t *received);

/*
 * wm_tick — Advance world model by elapsed_ms milliseconds.
 * Ages all non-self entries. Flags stale and expired entries.
 * Recomputes consistency metric. Applies CP freeze if quorum lost.
 * Call at WM_CYCLE_MS intervals.
 */
void wm_tick(world_model_t *wm, uint32_t elapsed_ms);

/*
 * wm_get_entry — Get current world model entry for a given element ID.
 * Returns NULL if ID has never been seen.
 * The returned pointer is valid until the next wm_tick or wm_receive_gossip.
 */
const wm_entry_t *wm_get_entry(const world_model_t *wm, element_id_t id);

/*
 * wm_get_metric — Get current consistency metric.
 * Valid after most recent wm_tick call.
 */
const wm_consistency_metric_t *wm_get_metric(const world_model_t *wm);

/*
 * wm_nearest_elements — Fill out_ids with up to max_count element IDs
 * whose last known positions are within radius of the given position,
 * ordered by distance (closest first). Returns actual count found.
 * Used by repulsion and gossip radius calculations.
 * Only includes active (non-expired) entries.
 */
uint8_t wm_nearest_elements(const world_model_t *wm,
                             const position_t *pos,
                             float radius,
                             element_id_t *out_ids,
                             uint8_t max_count);

/*
 * wm_check_collisions — Check own position against all active world model
 * entries. Fills out_events with any elements within MIN_SEPARATION.
 * Returns count of collision events found.
 */
uint8_t wm_check_collisions(const world_model_t *wm,
                             const position_t *own_pos,
                             collision_event_t *out_events,
                             uint8_t max_events);

/*
 * wm_reconcile — Merge two world model snapshots after partition heal.
 * For each entry, keeps the version with the higher logical clock.
 * Records reconciliation timing for telemetry.
 * Call when partition island changes from isolated to connected.
 */
void wm_reconcile(world_model_t *wm,
                  const wm_entry_t *received_entries,
                  uint8_t entry_count,
                  uint32_t now_ms);

/*
 * wm_snapshot — Copy all active entries into out_entries for gossip
 * transmission or telemetry emission. Returns count of entries copied.
 */
uint8_t wm_snapshot(const world_model_t *wm,
                    element_state_t *out_states,
                    uint8_t max_count);

#endif /* TAPESTRY_WORLD_MODEL_H */
