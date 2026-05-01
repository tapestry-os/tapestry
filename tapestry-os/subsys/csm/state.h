/*
 * state.h — Tapestry L4 Simulation
 *
 * Defines the authoritative local state of a single Tapestry element.
 * This is the data each element owns exclusively. No other element
 * holds an authoritative copy — only gossip-propagated replicas.
 *
 * Design notes:
 *   - Positions are floats in a normalized 2D space [0.0, WORLD_SIZE].
 *   - Logical clock is a Lamport clock: incremented on every local
 *     state change, and set to max(local, received)+1 on gossip receive.
 *     This gives us a partial causal ordering without wall-clock sync.
 *   - partition_island is set by the orchestrator via control socket
 *     to simulate physical separation. Elements in different islands
 *     cannot exchange gossip messages.
 */

#ifndef TAPESTRY_STATE_H
#define TAPESTRY_STATE_H

#include <stdint.h>
#include <stdbool.h>

/* ── World space ─────────────────────────────────────────────────────────── */

#define WORLD_SIZE          100.0f   /* World is a 100x100 unit 2D space     */
#define MIN_SEPARATION      3.0f     /* Minimum safe distance between elements*/
#define WALK_STEP_MAX       1.0f     /* Max position delta per update cycle  */
#define REPULSION_RADIUS    6.0f     /* Repulsion kicks in within this radius */
#define GOSSIP_RADIUS       25.0f    /* Elements gossip within this radius   */

/* ── Element identity ────────────────────────────────────────────────────── */

#define MAX_ELEMENTS        32       /* Maximum elements in the simulation   */
#define ELEMENT_ID_INVALID  0xFF

typedef uint8_t element_id_t;

/* ── Position ─────────────────────────────────────────────────────────────── */

typedef struct {
    float x;
    float y;
} position_t;

/* ── Authoritative element state ─────────────────────────────────────────── */
/*
 * This struct is what each element owns. It is the single source of truth
 * for this element's position and status. All other elements hold stale
 * copies of this in their world model — never the authoritative version.
 */

typedef struct {
    element_id_t  id;              /* Unique identifier [0, MAX_ELEMENTS)    */
    position_t    position;        /* Current 2D position in world space      */
    uint32_t      logical_clock;   /* Lamport clock — incremented each update */
    uint8_t       partition_island;/* Orchestrator-assigned partition group   */
                                   /* Elements in different islands cannot    */
                                   /* exchange gossip. 0 = no partition.      */
    uint32_t      update_seq;      /* Monotonic update counter (debug/log)   */
} element_state_t;

/* ── Collision event ─────────────────────────────────────────────────────── */
/*
 * A collision is recorded when two elements' positions are within
 * MIN_SEPARATION of each other. This is a direct L4 health indicator:
 * collisions occur when world model staleness prevents repulsion from
 * working correctly.
 */

typedef struct {
    element_id_t  element_a;
    element_id_t  element_b;
    float         distance;        /* Actual distance at collision time        */
    uint32_t      logical_clock;   /* Local clock of detecting element         */
} collision_event_t;

/* ── Utility: Euclidean distance between two positions ───────────────────── */

static inline float position_distance(const position_t *a, const position_t *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    /* Note: sqrtf requires <math.h> in the including .c file               */
    extern float sqrtf(float);
    return sqrtf(dx * dx + dy * dy);
}

/* ── Utility: clamp position within world bounds ─────────────────────────── */

static inline void position_clamp(position_t *p)
{
    if (p->x < 0.0f) p->x = 0.0f;
    if (p->x > WORLD_SIZE) p->x = WORLD_SIZE;
    if (p->y < 0.0f) p->y = 0.0f;
    if (p->y > WORLD_SIZE) p->y = WORLD_SIZE;
}

#endif /* TAPESTRY_STATE_H */
