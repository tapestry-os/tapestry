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
    float z;
} position_t;

/* ── Orientation ─────────────────────────────────────────────────────────── */
/*
 * Unit quaternion, w-first — same field order/convention as
 * substrate_quat_t (substrate.h), which this mirrors rather than reuses:
 * CSM (L4) does not depend on substrate (L1) headers, the same way
 * position_t has never reused an L1 position type. Application code
 * converts between the two at the point where sensor output becomes
 * gossiped state.
 *
 * Identity ({1,0,0,0} — see orientation_identity()) means "no orientation
 * data available" as much as it means "no rotation" — elements with no
 * attitude sensing gossip identity rather than a fabricated estimate.
 *
 * Reference frame — REQUIRED for orientation to be meaningful across
 * elements, not just loggable per-element:
 *
 *   orientation_t is expressed in the SAME world frame as position_t on
 *   that platform — never a local/body frame, and never "wherever this
 *   element happened to be pointed at boot". Identity means "aligned
 *   with this platform's world +X/+Y/+Z axes", the same axes position.x/
 *   y/z are already measured against (Webots' world frame; the shared
 *   lighthouse world frame on cf21bl hardware — see
 *   examples/cf21bl-formation/formation.h's units note). Tying the two
 *   together this way means no new convention has to be invented per
 *   platform: whatever already makes two elements' POSITIONS comparable
 *   makes their ORIENTATIONS comparable too.
 *
 *   If a platform's world frame is genuinely geographically anchored
 *   (a real compass/magnetometer reference, not just an arbitrary
 *   simulation or lab-calibrated frame) then that frame's axes MUST be
 *   ENU: +X = East, +Y = North, +Z = Up. No current platform in this
 *   repo has that (Webots' world frame is simulation-arbitrary; cf21bl
 *   hardware has no magnetometer — BMI088 is accel+gyro only), so this
 *   is not yet exercised by any code path, only documented for when it
 *   is — do not assume ENU for a lighthouse- or Webots-referenced
 *   platform.
 *
 *   A platform wiring up a real attitude estimate for the first time
 *   must calibrate its zero-reference to this world frame, not just
 *   report raw sensor output — see
 *   examples/cf21bl-formation/README.md's "Known limitations" for what
 *   that means concretely on that platform (the existing "nose along
 *   lighthouse world +X" boot placement is the calibration hook that
 *   would make this correct, IF the eventual accessor takes its zero
 *   from that same moment).
 */

typedef struct {
    float w;
    float x;
    float y;
    float z;
} orientation_t;

static inline orientation_t orientation_identity(void)
{
    orientation_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

/* ── Element health flags (health_flags bitmask) ─────────────────────────── */

#define ELEMENT_HEALTH_OK           0x00u  /* All subsystems nominal             */
#define ELEMENT_HEALTH_LOW_BATTERY  0x01u  /* energy_level below 20%             */
#define ELEMENT_HEALTH_SENSOR_FAULT 0x02u  /* On-board sensor reporting failure  */
#define ELEMENT_HEALTH_DEGRADED     0x04u  /* Reduced capability (hot, throttled)*/
/* No position fix yet: this element's gossiped position is the zero-init
 * PLACEHOLDER, not a measurement.  Receivers must exclude such an element
 * from anything that reasons about where it physically is — separation
 * checks, repulsion, station snapshots.  (2026-08-31 flight 41: a drone
 * still waiting for its lighthouse fix gossiped (0,0,0); its partner,
 * sitting 1.1 m away, measured a 0.31 m "separation violation" against
 * the phantom at the origin.)  Cleared for good on the first real fix.
 * Additive bit — health_flags is already on the wire (gossip.c), so this
 * needs no TAPESTRY_WIRE_VERSION bump and older receivers just ignore it. */
#define ELEMENT_HEALTH_NO_POSITION  0x08u  /* Position is a placeholder      */
/* This element HAD a fix and lost it mid-flight: the gossiped position is
 * its last real measurement, held (not zeroed) while it waits out a
 * lighthouse dropout, not a live one.  Distinct from NO_POSITION (never
 * measured, currently (0,0,0)) — a NO_POSITION entry is excluded from
 * separation and repulsion outright; a POSITION_STALE entry is a genuine
 * airframe at a real last-known point and stays visible to both, the same
 * as any other stale-but-active peer.  Cleared on the next fix. */
#define ELEMENT_HEALTH_POSITION_STALE 0x10u  /* Held position, fix down now */
/* This element has self-declared that it has stopped participating in the
 * collective's Choreo script (landed to completion, hit a safety backstop,
 * lost its position fix, or breached the geofence) — see
 * element_is_participating() below.  Departure must be DECLARED, never
 * inferred from silence: an element that goes quiet without setting this
 * bit is presumed LOST only after WM_EXPIRE_THRESHOLD_MS, which is exactly
 * why cf21bl-formation's main.c keeps a landed element gossiping in choreo
 * mode instead of going silent (the flight-12 deadlock this bit's producer
 * comment references).  Reason lives in bits [7:6]
 * (tapestry_departure_reason_t) — a narrower vocabulary than the
 * platform's own land_reason_t: LOST has no wire representation, because a
 * receiver already infers it locally from expiry.  Additive bit, same
 * precedent as NO_POSITION/POSITION_STALE above — health_flags is already
 * on the wire verbatim (gossip.c), so this needs no TAPESTRY_WIRE_VERSION
 * bump; older receivers just ignore it and keep today's ghost-vote
 * behavior. Excluded from collective predicates (scope="all" achievement,
 * swap-partner selection, quorum) but NOT from anything physical
 * (separation/repulsion) — a departed element is still a real obstacle. */
#define ELEMENT_HEALTH_DEPARTED       0x20u

#define ELEMENT_DEPARTED_REASON_SHIFT 6u
#define ELEMENT_DEPARTED_REASON_MASK  0xC0u

typedef enum {
    ELEMENT_DEPARTED_COMPLETE = 0,  /* script finished — quiescence -> land */
    ELEMENT_DEPARTED_BACKSTOP = 1,  /* mission-duration backstop elapsed    */
    ELEMENT_DEPARTED_FIXLOSS  = 2,  /* position fix lost past grace period  */
    ELEMENT_DEPARTED_GEOFENCE = 3,  /* strayed past the geofence radius     */
} tapestry_departure_reason_t;

/* Sets ELEMENT_HEALTH_DEPARTED and packs reason into bits [7:6], leaving
 * every other health_flags bit untouched. */
static inline uint8_t element_health_set_departed(uint8_t health_flags,
                                                    tapestry_departure_reason_t reason)
{
    health_flags &= (uint8_t)~ELEMENT_DEPARTED_REASON_MASK;
    health_flags |= ELEMENT_HEALTH_DEPARTED;
    health_flags |= (uint8_t)(((uint8_t)reason << ELEMENT_DEPARTED_REASON_SHIFT) &
                               ELEMENT_DEPARTED_REASON_MASK);
    return health_flags;
}

static inline tapestry_departure_reason_t
element_health_departed_reason(uint8_t health_flags)
{
    return (tapestry_departure_reason_t)
        ((health_flags & ELEMENT_DEPARTED_REASON_MASK) >> ELEMENT_DEPARTED_REASON_SHIFT);
}

/* ── Element state ───────────────────────────────────────────────────────── */
/*
 * Authoritative local state owned exclusively by one element.  All other
 * elements hold gossip-propagated replicas — never the authoritative copy.
 * Positions use a Lamport clock for causal ordering without wall-clock sync.
 */

typedef struct {
    element_id_t  id;               /* Unique identifier [0, MAX_ELEMENTS)    */
    position_t    position;         /* Current 3D position in world space      */
    orientation_t orientation;      /* Current attitude (unit quaternion) —   */
                                    /* orientation_identity() when unsensed    */
    uint32_t      logical_clock;    /* Lamport clock — incremented each update */
    uint8_t       partition_island; /* Orchestrator-assigned partition group;  */
                                    /* elements in different islands cannot    */
                                    /* exchange gossip.  0 = no partition.     */
    uint32_t      update_seq;       /* Monotonic update counter (debug/log)   */
    uint8_t       energy_level;     /* Battery/power [0=empty, 100=full]      */
    uint8_t       health_flags;     /* ELEMENT_HEALTH_* bitmask               */
    bool          goal_achieved;    /* L6/L7 own-goal achievement predicate — */
                                    /* set by the application from            */
                                    /* choreo_goal_achieved() before gossip;  */
                                    /* gossiped so peers can aggregate a      */
                                    /* collective ("scope=all") predicate     */
                                    /* (see choreo_collective_achieved()).    */
    uint8_t       current_track;    /* L7 active track index (wire v4) — see  */
                                    /* choreo_current_track() and choreo.h §7 */
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

/*
 * 3D Euclidean distance. Every caller (formation.c's spring field and
 * emergency repulsion, wm_check_collisions, wm_nearest_elements) gets z
 * folded in uniformly — including the MIN_SEPARATION-class safety checks.
 * That is a deliberate, explicit choice, not an oversight: on platforms
 * where altitude is independently held constant per element (e.g.
 * cf21bl-formation's per-ID cruise-altitude staggering), folding a fixed
 * altitude difference into the distance metric measurably weakens
 * horizontal-proximity detection versus the 2D-only math it replaced.
 * A platform that staggers altitude per element should size its
 * MIN_SEPARATION with that weakening in mind — see that example's README.
 */
static inline float position_distance(const position_t *a, const position_t *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    /* sqrtf requires <math.h> in the including .c file */
    extern float sqrtf(float);
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* Clamps x/y to the abstract [0, WORLD_SIZE] sim world only — z (altitude)
 * has no equivalent bound here; callers that need one define their own
 * (e.g. cf21bl-formation's DEMO_ARENA_LIMIT_M is a different, meters-scale
 * space entirely — see formation.h). */
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

/* ── Participation predicate ─────────────────────────────────────────────── */
/*
 * False if the peer has self-declared departure (ELEMENT_HEALTH_DEPARTED)
 * OR gone silent past WM_EXPIRE_THRESHOLD_MS (is_active == false — the only
 * *inferred* case, implicitly reason LOST).  Collective predicates —
 * choreo_collective_achieved()'s scope="all" vote, bse.c's
 * collect_participants() swap-partner/centroid selection, scr.c's quorum
 * denominator — must use this instead of checking is_active alone, or a
 * departed element's frozen gossiped state keeps voting forever (the
 * ghost-vote bug: a landed peer's frozen `achieved` bit either blocks or
 * spuriously passes a scope="all" step it has already left).
 *
 * formation.c's separation/repulsion math must NOT use this predicate — a
 * departed element (landed, or holding position after a fix loss) is
 * still a genuine physical obstacle regardless of whether it is still
 * "participating" in the script.
 */
static inline bool element_is_participating(const wm_entry_t *e)
{
    if (!e->is_active) {
        return false;
    }
    return (e->state.health_flags & ELEMENT_HEALTH_DEPARTED) == 0;
}

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
