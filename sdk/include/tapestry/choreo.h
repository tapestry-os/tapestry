/*
 * tapestry/choreo.h — Tapestry Choreography SDK, L7
 *
 * This header is the stable interface between user applications and the
 * Tapestry stack.  Application developers code against this API exclusively;
 * they do not call into L4 (CSM), L5 (SCR), or L6 (BSE) directly.
 *
 * ┌─────────────────────────────────────────┐
 * │  L7  Choreography (your code)           │  ← codes against choreo.h
 * │  L6  BSE — Behavior Synthesis Engine    │  ← bse.h (tapestry-os)
 * │  L5  SCR — Swarm Coordination Runtime   │
 * │  L4  CSM — Coherent Swarm Memory        │
 * │  L3  Transport (UDP / BLE gossip)       │
 * │  L2  Element Runtime  (Zephyr)          │
 * │  L1  Physical Substrate Interface       │
 * └─────────────────────────────────────────┘
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  STUB IMPLEMENTATION — NOT FOR PRODUCTION USE                            ║
 * ║                                                                          ║
 * ║  The stub backing (sdk/src/choreo_stub.c) delegates to bse_stub.c.       ║
 * ║  Achievement detection, multi-goal sequencing, and the feedback          ║
 * ║  controller are absent; goal status is ACTIVE until explicitly           ║
 * ║  cancelled or quorum is lost (→ FAILED).                                 ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#ifndef TAPESTRY_CHOREO_H
#define TAPESTRY_CHOREO_H

#include <tapestry/bse.h>   /* tapestry_bse_directive_t, tapestry_bse_shape_t */

/* ── Goal ─────────────────────────────────────────────────────────────────── */
/*
 * A goal is a declarative desired world state submitted by the application.
 * The BSE decomposes it into per-element directives each cycle.
 *
 * The stub supports one active goal at a time.  The commercial BSE supports
 * a prioritised goal queue with preemption and sequencing.
 */

typedef enum {
    CHOREO_GOAL_NONE     = 0,
    CHOREO_GOAL_FORM     = 1,   /* arrange elements into a geometric shape */
    CHOREO_GOAL_MOVE     = 2,   /* translate formation to target point     */
    CHOREO_GOAL_DISPERSE = 3,   /* spread elements across the arena        */
    CHOREO_GOAL_CONVERGE = 4,   /* gather elements at a point              */
} choreo_goal_type_t;

typedef struct {
    choreo_goal_type_t    type;
    tapestry_position_t   target;   /* MOVE / CONVERGE destination           */
    float                 radius;   /* FORM radius; DISPERSE minimum spacing */
    tapestry_bse_shape_t  shape;    /* FORM shape (circle / line / grid)     */
} choreo_goal_t;

typedef enum {
    CHOREO_STATUS_IDLE     = 0,   /* no goal submitted                     */
    CHOREO_STATUS_ACTIVE   = 1,   /* goal in progress                      */
    CHOREO_STATUS_ACHIEVED = 2,   /* goal satisfied (commercial BSE only)  */
    CHOREO_STATUS_FAILED   = 3,   /* quorum lost or goal unachievable      */
} choreo_status_t;

/* ── SDK API ──────────────────────────────────────────────────────────────── */

/*
 * choreo_init — Initialise the choreography layer for this element.
 * Must be called once, before any other choreo_* function.
 */
void choreo_init(element_id_t self_id);

/*
 * choreo_submit_goal — Submit a new goal, replacing any current one.
 * The BSE begins decomposing it on the next choreo_tick().
 * Returns 0 on success, -1 if goal is NULL or type is unrecognised.
 */
int choreo_submit_goal(const choreo_goal_t *goal);

/*
 * choreo_cancel_goal — Cancel the current goal and return to IDLE.
 */
void choreo_cancel_goal(void);

/*
 * choreo_goal_status — Return the status of the current goal.
 */
choreo_status_t choreo_goal_status(void);

/*
 * choreo_tick — Drive L6 decomposition for this cycle.
 *
 * Call once per main-loop cycle, after wm_tick() and scr_tick().
 * Updates the directive returned by choreo_get_directive().
 */
void choreo_tick(const world_model_t *wm, const scr_state_t *scr);

/*
 * choreo_get_directive — Return the current per-element behavioral directive.
 *
 * Valid after the first choreo_tick().  Never returns NULL.
 * The directive is recomputed each tick; do not cache across cycles.
 */
const tapestry_bse_directive_t *choreo_get_directive(void);

#endif /* TAPESTRY_CHOREO_H */
