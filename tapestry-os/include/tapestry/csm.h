/*
 * tapestry/csm.h — Tapestry Collective State Manager (L4) public API
 *
 * Defines all types, constants, and function declarations for L4 CSM.
 * Application code, simulation harnesses, and platform adaptation layers
 * include only this header.
 *
 * No OS-specific or Zephyr types appear anywhere in this interface.
 * The CSM is portable to any platform that provides a C99 toolchain
 * and a libm (for sqrtf used by position_distance).
 *
 * Design notes — world model:
 *   Each element maintains one world_model_t: its local understanding of where
 *   every other element is.  No element holds the complete authoritative world
 *   model — the complete picture exists only as the union of all elements'
 *   local models.
 *
 *   The world model is a fixed-size flat table of wm_entry_t indexed by
 *   element ID.  Each entry holds the last known state of that element plus
 *   metadata: age, staleness, and liveness.  The owner's own entry is always
 *   authoritative and never expires.
 *
 * Consistency modes — a continuous dial from AP to CP:
 *   consistency_bias = 0.0  Pure AP.  Quorum threshold is 0; element never
 *                           freezes.  World models diverge freely during a
 *                           partition and converge after heal.
 *   consistency_bias = 1.0  Pure CP.  Quorum threshold is WM_QUORUM_FRACTION
 *                           (0.5): element freezes when fewer than 50% of known
 *                           peers are fresh.  No divergence during partition,
 *                           at the cost of availability.
 *   0 < bias < 1            Hybrid.  Threshold scales linearly; element
 *                           degrades gracefully before fully freezing.
 */

#ifndef TAPESTRY_CSM_H
#define TAPESTRY_CSM_H

#include <stdint.h>
#include <stdbool.h>

/* ── World space ─────────────────────────────────────────────────────────── */

#define WORLD_SIZE          100.0f   /* World is a 100×100 unit 2D space      */
#define MIN_SEPARATION      3.0f     /* Minimum safe distance between elements */
#define WALK_STEP_MAX       1.0f     /* Max position delta per update cycle    */
#define REPULSION_RADIUS    6.0f     /* Repulsion kicks in within this radius  */
#define GOSSIP_RADIUS       25.0f    /* Elements gossip within this radius     */

/* ── Element identity ────────────────────────────────────────────────────── */

#define MAX_ELEMENTS        32       /* Maximum elements in the swarm          */
#define ELEMENT_ID_INVALID  0xFF

typedef uint8_t element_id_t;

/* ── Position ────────────────────────────────────────────────────────────── */

typedef struct {
    float x;
    float y;
} position_t;

/* ── Element health flags (health_flags bitmask) ─────────────────────────── */

#define ELEMENT_HEALTH_OK           0x00u  /* All subsystems nominal             */
#define ELEMENT_HEALTH_LOW_BATTERY  0x01u  /* energy_level below 20%             */
#define ELEMENT_HEALTH_SENSOR_FAULT 0x02u  /* On-board sensor reporting failure  */
#define ELEMENT_HEALTH_DEGRADED     0x04u  /* Reduced capability (hot, throttled)*/

/* ── Element state ───────────────────────────────────────────────────────── */
/*
 * Authoritative local state owned exclusively by one element.  All other
 * elements hold gossip-propagated replicas — never the authoritative copy.
 * Positions use a Lamport clock for causal ordering without wall-clock sync.
 */

typedef struct {
    element_id_t  id;               /* Unique identifier [0, MAX_ELEMENTS)    */
    position_t    position;         /* Current 2D position in world space      */
    uint32_t      logical_clock;    /* Lamport clock — incremented each update */
    uint8_t       partition_island; /* Orchestrator-assigned partition group;  */
                                    /* elements in different islands cannot    */
                                    /* exchange gossip.  0 = no partition.     */
    uint32_t      update_seq;       /* Monotonic update counter (debug/log)   */
    uint8_t       energy_level;     /* Battery/power [0=empty, 100=full]      */
    uint8_t       health_flags;     /* ELEMENT_HEALTH_* bitmask               */
} element_state_t;

/* ── Collision event ─────────────────────────────────────────────────────── */
/*
 * Recorded when two elements' positions are within MIN_SEPARATION.  A direct
 * L4 health indicator: collisions occur when world model staleness prevents
 * repulsion from working correctly.
 */

typedef struct {
    element_id_t  element_a;
    element_id_t  element_b;
    float         distance;         /* Actual distance at collision time       */
    uint32_t      logical_clock;    /* Local clock of detecting element        */
} collision_event_t;

/* ── Position utilities ──────────────────────────────────────────────────── */

static inline float position_distance(const position_t *a, const position_t *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    /* sqrtf requires <math.h> in the including .c file */
    extern float sqrtf(float);
    return sqrtf(dx * dx + dy * dy);
}

static inline void position_clamp(position_t *p)
{
    if (p->x < 0.0f) p->x = 0.0f;
    if (p->x > WORLD_SIZE) p->x = WORLD_SIZE;
    if (p->y < 0.0f) p->y = 0.0f;
    if (p->y > WORLD_SIZE) p->y = WORLD_SIZE;
}

/* ── Timing constants ────────────────────────────────────────────────────── */

#define GOSSIP_INTERVAL_MS        500   /* How often each element gossips      */
#define WM_STALE_THRESHOLD_MS    1500   /* Entry flagged stale after this      */
#define WM_EXPIRE_THRESHOLD_MS   5000   /* Entry marked inactive after this    */
#define WM_CYCLE_MS               100   /* World model update tick rate        */

/* ── Quorum fraction ─────────────────────────────────────────────────────── */

#define WM_QUORUM_FRACTION       0.5f   /* Max threshold; scales with bias     */

/* ── World model entry ───────────────────────────────────────────────────── */

typedef struct {
    element_state_t state;          /* Last known state of this element        */
    uint32_t        age_ms;         /* Milliseconds since last update received */
    bool            is_active;      /* False if expired (presumed dead)        */
    bool            is_stale;       /* True if age_ms > WM_STALE_THRESHOLD_MS  */
    bool            is_self;        /* True if this entry belongs to owner     */
    uint32_t        update_count;   /* How many gossip updates received        */
} wm_entry_t;

/* ── Consistency metric ──────────────────────────────────────────────────── */
/*
 * Computed each cycle by wm_tick.  Reported in telemetry.
 *
 * fresh_ratio  = active_fresh / active_total  [0.0, 1.0]
 * quorum_held  = fresh_ratio >= (bias * WM_QUORUM_FRACTION)
 * confidence   = fresh_ratio / quorum_threshold, clamped to [0.0, 1.0].
 *                Always 1.0 when consistency_bias == 0.0 (pure AP).
 */

typedef struct {
    uint8_t  active_total;       /* Elements currently marked active           */
    uint8_t  active_fresh;       /* Active elements with non-stale entries     */
    uint8_t  active_stale;       /* Active elements with stale entries         */
    uint8_t  inactive_total;     /* Elements marked inactive (expired)         */
    uint8_t  collision_count;    /* Collisions detected this cycle             */
    float    fresh_ratio;        /* active_fresh / active_total [0.0, 1.0]    */
    bool     quorum_held;        /* True if fresh_ratio >= effective threshold */
    bool     degraded;           /* True when quorum lost (any bias > 0)       */
    float    confidence;         /* Proximity to quorum [0.0, 1.0]            */
} wm_consistency_metric_t;

/* ── World model ─────────────────────────────────────────────────────────── */

typedef struct {
    element_id_t            owner_id;              /* This element's own ID    */
    wm_entry_t              entries[MAX_ELEMENTS]; /* One slot per possible ID */
    uint8_t                 known_count;           /* How many IDs ever seen   */
    float                   consistency_bias;      /* 0.0=AP .. 1.0=CP         */
    wm_consistency_metric_t metric;                /* Current consistency state */
    uint32_t                cycle_count;           /* Total update cycles run  */
    uint32_t                last_reconcile_cycle;  /* Cycle of last reconcile  */
    uint32_t                reconcile_duration_ms; /* How long reconcile took  */
} world_model_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/* Initialize world model for an element.  Call once at startup. */
void wm_init(world_model_t *wm,
             element_id_t owner_id,
             const element_state_t *own_state,
             float consistency_bias);

/* Update the owner's own entry and increment the logical clock.
 * Writes the new clock value back to own_state so the caller stays in sync. */
void wm_update_self(world_model_t *wm, element_state_t *own_state);

/* Process an incoming gossip message.  Applies Lamport clock merge.
 * Returns true if this update advanced our knowledge of the sender. */
bool wm_receive_gossip(world_model_t *wm, const element_state_t *received);

/* Advance world model by elapsed_ms.  Ages entries, recomputes metric.
 * Call at WM_CYCLE_MS intervals. */
void wm_tick(world_model_t *wm, uint32_t elapsed_ms);

/* Get current world model entry for a given ID.  Returns NULL if never seen.
 * Pointer is valid until the next wm_tick or wm_receive_gossip. */
const wm_entry_t *wm_get_entry(const world_model_t *wm, element_id_t id);

/* Get current consistency metric.  Valid after the most recent wm_tick. */
const wm_consistency_metric_t *wm_get_metric(const world_model_t *wm);

/* Fill out_ids with up to max_count active element IDs within radius of pos,
 * ordered by distance (closest first).  Returns actual count found. */
uint8_t wm_nearest_elements(const world_model_t *wm,
                             const position_t *pos,
                             float radius,
                             element_id_t *out_ids,
                             uint8_t max_count);

/* Check own position against all active entries.  Fills out_events with any
 * elements within MIN_SEPARATION.  Returns collision count. */
uint8_t wm_check_collisions(const world_model_t *wm,
                             const position_t *own_pos,
                             collision_event_t *out_events,
                             uint8_t max_events);

/* Merge received world model entries after partition heal.  Keeps the entry
 * with the higher logical clock for each peer. */
void wm_reconcile(world_model_t *wm,
                  const wm_entry_t *received_entries,
                  uint8_t entry_count,
                  uint32_t now_ms);

/* Copy all active entries into out_states for gossip or telemetry.
 * Returns count of entries copied. */
uint8_t wm_snapshot(const world_model_t *wm,
                    element_state_t *out_states,
                    uint8_t max_count);

#endif /* TAPESTRY_CSM_H */
