/*
 * app.h — Tapestry Application SDK, L7
 *
 * This header is the stable interface between user applications and the
 * Tapestry stack.  Application developers code against this API exclusively;
 * they do not call into L4 (CSM), L5 (SCR), or L6 (BSE) directly.
 *
 * ┌─────────────────────────────────────────┐
 * │  L7  Application (your code)            │  ← codes against app.h
 * │  L6  BSE — Behavior Synthesis Engine    │  ← bse.h (tapestry-os)
 * │  L5  SCR — Swarm Coordination Runtime   │
 * │  L4  CSM — Coherent Swarm Memory        │
 * │  L3  Transport (UDP / BLE gossip)       │
 * │  L2  Element Runtime  (Zephyr)          │
 * │  L1  Physical Substrate Interface       │
 * └─────────────────────────────────────────┘
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  STUB IMPLEMENTATION — NOT FOR PRODUCTION USE                           ║
 * ║                                                                          ║
 * ║  The stub backing (sdk/src/app_stub.c) delegates to bse_stub.c.        ║
 * ║  Achievement detection, multi-goal sequencing, and the feedback         ║
 * ║  controller are absent; goal status is ACTIVE until explicitly          ║
 * ║  cancelled or quorum is lost (→ FAILED).                               ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#ifndef TAPESTRY_APP_H
#define TAPESTRY_APP_H

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
    TAPESTRY_GOAL_NONE     = 0,
    TAPESTRY_GOAL_FORM     = 1,   /* arrange elements into a geometric shape */
    TAPESTRY_GOAL_MOVE     = 2,   /* translate formation to target point     */
    TAPESTRY_GOAL_DISPERSE = 3,   /* spread elements across the arena        */
    TAPESTRY_GOAL_CONVERGE = 4,   /* gather elements at a point              */
} tapestry_goal_type_t;

typedef struct {
    tapestry_goal_type_t  type;
    tapestry_position_t    target;   /* MOVE / CONVERGE destination           */
    float                 radius;   /* FORM radius; DISPERSE minimum spacing */
    tapestry_bse_shape_t  shape;    /* FORM shape (circle / line / grid)     */
} tapestry_goal_t;

typedef enum {
    TAPESTRY_STATUS_IDLE     = 0,   /* no goal submitted                     */
    TAPESTRY_STATUS_ACTIVE   = 1,   /* goal in progress                      */
    TAPESTRY_STATUS_ACHIEVED = 2,   /* goal satisfied (commercial BSE only)  */
    TAPESTRY_STATUS_FAILED   = 3,   /* quorum lost or goal unachievable      */
} tapestry_status_t;

/* ── SDK API ──────────────────────────────────────────────────────────────── */

/*
 * tapestry_init — Initialise the SDK for this element.
 * Must be called once, before any other tapestry_* function.
 */
void tapestry_init(element_id_t self_id);

/*
 * tapestry_submit_goal — Submit a new goal, replacing any current one.
 * The BSE begins decomposing it on the next tapestry_tick().
 * Returns 0 on success, -1 if goal is NULL or type is unrecognised.
 */
int tapestry_submit_goal(const tapestry_goal_t *goal);

/*
 * tapestry_cancel_goal — Cancel the current goal and return to IDLE.
 */
void tapestry_cancel_goal(void);

/*
 * tapestry_goal_status — Return the status of the current goal.
 */
tapestry_status_t tapestry_goal_status(void);

/*
 * tapestry_tick — Drive L6 decomposition for this cycle.
 *
 * Call once per main-loop cycle, after wm_tick() and scr_tick().
 * Updates the directive returned by tapestry_get_directive().
 */
void tapestry_tick(const world_model_t *wm, const scr_state_t *scr);

/*
 * tapestry_get_directive — Return the current per-element behavioral directive.
 *
 * Valid after the first tapestry_tick().  Never returns NULL.
 * The directive is recomputed each tick; do not cache across cycles.
 */
const tapestry_bse_directive_t *tapestry_get_directive(void);

#endif /* TAPESTRY_APP_H */
