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
 * Consistency modes:
 *   WM_MODE_AP — Available/Partition-tolerant.
 *     The element continues updating its own position and gossiping
 *     during a partition. World models in each partition island diverge.
 *     After reconnection, reconciliation uses logical clocks to merge.
 *     Convergence is fast but divergence during partition is high.
 *
 *   WM_MODE_CP — Consistent/Partition-tolerant.
 *     The element freezes its own position updates when it detects
 *     a partition (quorum loss). It continues gossiping within its
 *     island but does not move. After reconnection, no divergence
 *     in own-state exists, but coordination cost is higher.
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

/* ── Quorum: fraction of known elements required to avoid CP freeze ───────── */

#define WM_QUORUM_FRACTION       0.5f   /* Must hear from >50% of last-known  */
                                        /* active elements to maintain quorum  */

/* ── Consistency mode ────────────────────────────────────────────────────── */

typedef enum {
    WM_MODE_AP = 0,   /* Available + Partition tolerant — keep moving        */
    WM_MODE_CP = 1,   /* Consistent + Partition tolerant — freeze on partition*/
} wm_consistency_mode_t;

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
 * Computed each cycle. Reported in telemetry. Drives CP quorum check.
 * fresh_ratio = active_fresh / active_total
 * A fresh_ratio of 1.0 means perfect world model accuracy.
 * A fresh_ratio below WM_QUORUM_FRACTION triggers CP freeze.
 */

typedef struct {
    uint8_t  active_total;      /* Elements currently marked active           */
    uint8_t  active_fresh;      /* Active elements with non-stale entries     */
    uint8_t  active_stale;      /* Active elements with stale entries         */
    uint8_t  inactive_total;    /* Elements marked inactive (expired)         */
    uint8_t  collision_count;   /* Collisions detected this cycle             */
    float    fresh_ratio;       /* active_fresh / active_total [0.0, 1.0]    */
    bool     quorum_held;       /* True if fresh_ratio >= WM_QUORUM_FRACTION  */
    bool     cp_frozen;         /* True if CP mode and quorum lost            */
} wm_consistency_metric_t;

/* ── World model ─────────────────────────────────────────────────────────── */

typedef struct {
    element_id_t           owner_id;              /* This element's own ID   */
    wm_entry_t             entries[MAX_ELEMENTS]; /* One slot per possible ID*/
    uint8_t                known_count;           /* How many IDs ever seen  */
    wm_consistency_mode_t  mode;                  /* AP or CP                */
    wm_consistency_metric_t metric;               /* Current consistency state*/
    uint32_t               cycle_count;           /* Total update cycles run */

    /* Reconciliation state — populated after partition heal */
    uint32_t               last_reconcile_cycle;  /* Cycle when last reconcile*/
    uint32_t               reconcile_duration_ms; /* How long reconcile took  */
} world_model_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * wm_init — Initialize world model for an element.
 * Call once at startup with the element's own initial state.
 */
void wm_init(world_model_t *wm,
             element_id_t owner_id,
             const element_state_t *own_state,
             wm_consistency_mode_t mode);

/*
 * wm_update_self — Update the owner's own entry.
 * Called after each local state change (position update, power change).
 * Increments logical clock. Never marks own entry stale or expired.
 */
void wm_update_self(world_model_t *wm, const element_state_t *own_state);

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
