/*
 * bse.h — Tapestry L6 Behavior Synthesis Engine interface
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  STUB IMPLEMENTATION — NOT FOR PRODUCTION USE                            ║
 * ║                                                                          ║
 * ║  This header defines the full L6 interface contract.  The open-source    ║
 * ║  stub (bse_stub.c) implements the intent-parser tier only:               ║
 * ║    ✓  Intent parsing — declarative goal → formal behavioral spec         ║
 * ║    ✗  Physics-aware planner (constraint satisfaction, geometry solver)   ║
 * ║    ✗  ML inference runtime (TFLite / ONNX / custom quantized models)     ║
 * ║    ✗  Simulation bridge (offline training / hardware-in-the-loop)        ║
 * ║    ✗  Feedback controller (closed-loop intended ↔ actual physical state) ║
 * ║                                                                          ║
 * ║  The commercial BSE implements all five tiers and plugs in here.         ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Architecture note — L5 calls into L6, not the other way around.
 *
 *   L7 Application  ──submit_intent──►  bse_submit_intent()
 *   Main loop       ──tick──────────►  bse_tick()      (after scr_tick)
 *   Main loop       ──query──────────►  bse_get_directive()
 *
 * The BSE does not mutate SCR state.  L5 remains the authoritative safety
 * and coordination layer; L6 provides the behavioral program that L5
 * executes within its quorum/role envelope.
 */

#ifndef TAPESTRY_BSE_H
#define TAPESTRY_BSE_H

#include <tapestry/csm.h>
#include <tapestry/scr.h>

/* ── Position ─────────────────────────────────────────────────────────────── */
/* tapestry_position_t is an internal CSM type; BSE and SDK use this instead. */
typedef struct {
    float x;
    float y;
} tapestry_position_t;

/* ── Intent: L7 → L6 ─────────────────────────────────────────────────────── */

typedef enum {
    TAPESTRY_BSE_INTENT_IDLE     = 0,   /* no active goal; hold position  */
    TAPESTRY_BSE_INTENT_FORM     = 1,   /* arrange elements into a shape  */
    TAPESTRY_BSE_INTENT_MOVE     = 2,   /* translate formation to target  */
    TAPESTRY_BSE_INTENT_DISPERSE = 3,   /* spread elements across arena   */
    TAPESTRY_BSE_INTENT_CONVERGE = 4,   /* gather elements at a point     */
} tapestry_bse_intent_type_t;

typedef enum {
    TAPESTRY_BSE_SHAPE_CIRCLE = 1,
    TAPESTRY_BSE_SHAPE_LINE   = 2,
    TAPESTRY_BSE_SHAPE_GRID   = 3,
} tapestry_bse_shape_t;

typedef struct {
    tapestry_bse_intent_type_t type;
    tapestry_position_t         target;   /* MOVE / CONVERGE destination    */
    float                      radius;   /* FORM radius; DISPERSE min dist */
    tapestry_bse_shape_t       shape;    /* FORM shape                     */
} tapestry_bse_intent_t;

/* ── Directive: L6 → main loop ────────────────────────────────────────────── */
/*
 * A directive is the per-element behavioral output of the BSE.  The main
 * loop (or a hardware abstraction layer above L5) consumes it each cycle.
 *
 * Stub produces:
 *   IDLE            — no goal; motors stop
 *   HOLD            — quorum lost; freeze in place
 *   MOVE_TO_POINT   — geometry-only target vertex (no path planning)
 *   MAINTAIN_SPRING — spring-field spacing (DISPERSE)
 *
 * Commercial BSE adds physics-corrected trajectories, obstacle avoidance,
 * force-feedback corrections, and model-predicted targets.
 */
typedef enum {
    TAPESTRY_BSE_DIRECTIVE_IDLE            = 0,
    TAPESTRY_BSE_DIRECTIVE_HOLD            = 1,
    TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT   = 2,
    TAPESTRY_BSE_DIRECTIVE_MAINTAIN_SPRING = 3,
} tapestry_bse_directive_type_t;

typedef struct {
    tapestry_bse_directive_type_t type;
    tapestry_position_t            target;    /* MOVE_TO_POINT destination  */
    float                         spring_k;  /* MAINTAIN_SPRING stiffness  */
    float                         spacing;   /* MAINTAIN_SPRING target dist */
} tapestry_bse_directive_t;

/* ── API ──────────────────────────────────────────────────────────────────── */

/*
 * bse_init — Initialise BSE state for this element.
 * Must be called once before bse_submit_intent / bse_tick.
 */
void bse_init(element_id_t self_id);

/*
 * bse_submit_intent — Accept a new intent from L7.
 * Replaces any current intent immediately.
 * Returns 0 on success, -1 if intent is NULL.
 */
int bse_submit_intent(const tapestry_bse_intent_t *intent);

/*
 * bse_tick — Recompute per-element directive from current world state.
 *
 * Call once per main-loop cycle, after wm_tick() and scr_tick() have run.
 *
 * Stub: performs geometry-only vertex assignment (O(N log N) sort over
 * MAX_ELEMENTS).  Commercial BSE runs the physics planner and ML inference
 * runtime here.
 */
void bse_tick(const world_model_t *wm, const scr_state_t *scr);

/*
 * bse_get_directive — Return the directive computed by the last bse_tick.
 * Never returns NULL.  Defaults to TAPESTRY_BSE_DIRECTIVE_IDLE on startup.
 */
const tapestry_bse_directive_t *bse_get_directive(void);

#endif /* TAPESTRY_BSE_H */
