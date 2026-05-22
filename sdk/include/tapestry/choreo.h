/*
 * tapestry/choreo.h — Tapestry Choreographer SDK, L7
 *
 * This header is the stable interface between user applications and the
 * Tapestry stack.  Application developers code against this API exclusively;
 * they do not call into L4 (CSM), L5 (SCR), or L6 (BSE) directly.
 *
 * ┌─────────────────────────────────────────┐
 * │  L7  Choreographer (your code)          │  ← codes against choreo.h
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
 * ║  The stub backing (tapestry-os/subsys/choreo/choreo.c) delegates to      ║
 * ║  tapestry-os/subsys/bse/bse.c.                                           ║
 * ║  Achievement detection, multi-goal sequencing, and the feedback          ║
 * ║  controller are absent; goal state tracks the lifecycle machine          ║
 * ║  (IDLE / CONFIGURED / RUNNING / SUSPENDED / TERMINATED).                ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#ifndef TAPESTRY_CHOREO_H
#define TAPESTRY_CHOREO_H

#include <tapestry/bse.h>   /* tapestry_bse_directive_t, tapestry_bse_shape_t */

/* ── Lifecycle state ──────────────────────────────────────────────────────── */
/*
 * choreo_state_t — five-stage lifecycle analogous to Android Activity,
 * as described in paper §3.9 (install → configure → deploy → monitor →
 * terminate).
 *
 *   IDLE        No goal loaded; SDK is quiescent.
 *   CONFIGURED  Goal validated and stored; BSE is not yet ticking.
 *               Corresponds to the paper's "install + configure" stage.
 *   RUNNING     BSE ticking; quorum is DEGRADED or HEALTHY.
 *               Corresponds to the paper's "deploy + monitor" stage.
 *   SUSPENDED   Quorum dropped to LOST while RUNNING; goal is preserved.
 *               Resumes automatically to RUNNING when quorum recovers.
 *   TERMINATED  choreo_terminate() called; goal cleared.  Transitions
 *               immediately back to IDLE — callers polling goal_status()
 *               will see IDLE, not TERMINATED, in steady-state.
 */
typedef enum {
    CHOREO_STATE_IDLE       = 0,
    CHOREO_STATE_CONFIGURED = 1,
    CHOREO_STATE_RUNNING    = 2,
    CHOREO_STATE_SUSPENDED  = 3,
    CHOREO_STATE_TERMINATED = 4,
} choreo_state_t;

/* ── Application-level capability declarations ────────────────────────────── */
/*
 * choreo_capabilities_t — bitmask of physical capabilities a Choreo goal
 * requires the executing element to possess.
 *
 * Applications declare required physical capabilities
 * (locomotion, bonding, sensing modalities).  Elements only grant what the
 * task requires.
 *
 * choreo_configure() maps these flags to L5 SCR_CAP_* hardware bits and
 * rejects the goal with -EPERM if the registered element cannot satisfy them:
 *
 *   CHOREO_CAP_LOCOMOTION  → SCR_CAP_ACTUATOR  (physical actuation node)
 *   CHOREO_CAP_SENSING     → SCR_CAP_SENSOR    (observation / sensing node)
 *   CHOREO_CAP_SIGNALING   → SCR_CAP_RELAY     (message-forwarding, best-fit)
 *   CHOREO_CAP_BONDING     → (no SCR equivalent; always unsatisfied by SCR)
 */
typedef uint8_t choreo_capabilities_t;

#define CHOREO_CAP_NONE       ((choreo_capabilities_t)0x00)
#define CHOREO_CAP_LOCOMOTION ((choreo_capabilities_t)0x01)
#define CHOREO_CAP_BONDING    ((choreo_capabilities_t)0x02)
#define CHOREO_CAP_SENSING    ((choreo_capabilities_t)0x04)
#define CHOREO_CAP_SIGNALING  ((choreo_capabilities_t)0x08)

/* ── Goal ─────────────────────────────────────────────────────────────────── */
/*
 * A goal is a declarative desired world state submitted by the application.
 * The BSE decomposes it into per-element directives each cycle.
 *
 * The stub supports one active goal at a time.  The commercial BSE supports
 * a prioritized goal queue with preemption and sequencing.
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
    tapestry_position_t   target;        /* MOVE / CONVERGE destination           */
    float                 radius;        /* FORM radius; DISPERSE minimum spacing */
    tapestry_bse_shape_t  shape;         /* FORM shape (circle / line / grid)     */
    choreo_capabilities_t required_caps; /* capabilities this goal requires       */
} choreo_goal_t;

/* ── SDK API ──────────────────────────────────────────────────────────────── */

/*
 * choreo_init — Initialize the Choreographer for this element.
 * Must be called once, before any other choreo_* function.
 */
void choreo_init(element_id_t self_id);

/*
 * choreo_register_scr — Register the element's SCR state for capability checking.
 *
 * Must be called before choreo_configure() for required_caps enforcement to
 * apply.  The pointer must remain valid for the process lifetime.
 * If not called, choreo_configure() skips the capability check.
 */
void choreo_register_scr(const scr_state_t *scr);

/*
 * choreo_configure — Validate and store a goal without starting execution.
 *
 * Lifecycle transition: IDLE → CONFIGURED.
 *
 * Returns 0 on success.
 * Returns -1 if called from a non-IDLE state, or if goal is NULL / type is
 * CHOREO_GOAL_NONE.
 * Returns -EPERM if the element's registered SCR capabilities do not satisfy
 * goal->required_caps.
 *
 * Does not submit an intent to the BSE or start BSE ticking.
 * Call choreo_deploy() to begin execution.
 */
int choreo_configure(const choreo_goal_t *goal);

/*
 * choreo_deploy — Begin executing the configured goal.
 *
 * Lifecycle transition: CONFIGURED → RUNNING.
 *
 * Submits the stored goal as a BSE intent; subsequent choreo_tick() calls
 * drive the BSE decomposition loop.
 * Returns -1 if the state is not CONFIGURED.
 */
int choreo_deploy(void);

/*
 * choreo_terminate — Abort the current goal and return to IDLE.
 *
 * Valid from any state.  Submits an IDLE intent to the BSE, passes through
 * TERMINATED, and settles in IDLE.
 */
void choreo_terminate(void);

/*
 * choreo_submit_goal — One-shot convenience: configure + deploy.
 *
 * Calls choreo_terminate() first if a goal is already active, then calls
 * choreo_configure(goal) followed by choreo_deploy().
 *
 * Returns 0 on success, -1 on invalid goal, -EPERM on capability mismatch.
 */
int choreo_submit_goal(const choreo_goal_t *goal);

/*
 * choreo_cancel_goal — Cancel the current goal and return to IDLE.
 * Thin wrapper around choreo_terminate().
 */
void choreo_cancel_goal(void);

/*
 * choreo_goal_status — Return the current lifecycle state.
 */
choreo_state_t choreo_goal_status(void);

/*
 * choreo_tick — Drive L6 decomposition for this cycle.
 *
 * Call once per main-loop cycle, after wm_tick() and scr_tick().
 * Only drives the BSE when state is RUNNING or SUSPENDED; no-op otherwise.
 * Transitions RUNNING → SUSPENDED on quorum loss, and back on recovery.
 * Updates the directive returned by choreo_get_directive().
 */
void choreo_tick(const world_model_t *wm, const scr_state_t *scr);

/*
 * choreo_get_directive — Return the current per-element behavioral directive.
 *
 * Valid after the first choreo_tick() in RUNNING state.  Never returns NULL.
 * The directive is recomputed each tick; do not cache across cycles.
 */
const tapestry_bse_directive_t *choreo_get_directive(void);

#endif /* TAPESTRY_CHOREO_H */
