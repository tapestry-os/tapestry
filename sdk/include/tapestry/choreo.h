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
 * │  L3  Transport (UDP / BLE / syslink)    │
 * │  L2  Element Runtime  (Zephyr)          │
 * │  L1  Physical Substrate Interface       │
 * └─────────────────────────────────────────┘
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  v1.0 FEATURE SCOPE                                                      ║
 * ║                                                                          ║
 * ║  The backing implementation (tapestry-os/subsys/choreo/choreo.c)         ║
 * ║  delegates to tapestry-os/subsys/bse/bse.c.                              ║
 * ║  Implemented: single goals, linear goal SCRIPTS (choreo_submit_script)   ║
 * ║  with per-step timeout / advance-on-achieved, the minimal L6             ║
 * ║  achievement predicate (choreo_goal_achieved), the install/configure/    ║
 * ║  deploy/terminate lifecycle stages (choreo_state_t below), and a TOML    ║
 * ║  script authoring/compiler toolchain (sdk/tools/choreoc.py — see         ║
 * ║  sdk/CHOREO_SCRIPTS.md), a hardware-in-the-loop simulation bridge        ║
 * ║  (examples/webots-formation/ — this stack, unmodified, against real      ║
 * ║  Webots physics), and an offline capture/replay harness (opt-in CSV      ║
 * ║  capture of per-tick inputs/outputs — choreo_telemetry.h — replayed      ║
 * ║  offline through sdk/python/tapestry and diffed tick-by-tick against     ║
 * ║  the recording — sdk/tools/choreo_sim.py --replay; see                   ║
 * ║  sdk/CHOREO_SCRIPTS.md's "Parity" section), and a synthetic              ║
 * ║  script-authoring simulator (sdk/tools/choreo_sim.py --simulate — N      ║
 * ║  in-process Choreo instances, perfect shared visibility, no C/Zephyr/    ║
 * ║  network, deliberately no repulsion/leash/arena-clamp physics; not a     ║
 * ║  fidelity simulator, see sdk/CHOREO_SCRIPTS.md's "Script-authoring       ║
 * ║  simulation" section).                                                   ║
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
 *   SUSPENDED   Quorum dropped to LOST while RUNNING; goal is preserved
 *               and script step timers are frozen — a partition pauses the
 *               show rather than timing it out.  Per-goal quorum: SELF-
 *               referential goals (HOLD) still tick the BSE while
 *               suspended (station capture and station-keeping need no
 *               peers); PEER-referential goals (EXCHANGE) are frozen.
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
 * Two goal families:
 *
 *   Coordinate goals (FORM / MOVE / DISPERSE / CONVERGE) reference absolute
 *   coordinates supplied by the application.
 *
 *   Configuration goals (HOLD / EXCHANGE) reference the collective's OWN
 *   current configuration — no coordinates in the goal at all:
 *     HOLD      stay at the current station (captured at activation).
 *     EXCHANGE  rotate stations by slot_shift around the ID-sorted ring of
 *               participants; two elements + slot_shift 1 = swap places.
 *               Stations are frozen snapshots of peer positions at
 *               activation; travel is an arc about the formation centroid
 *               so separation is preserved (see bse.h for the mechanism).
 *
 * The substrate-neutral end of a goal sequence is QUIESCENCE: when a script
 * completes (or the Choreo is terminated) the directive becomes IDLE and
 * each platform maps that to its own inactive posture — an aerial element
 * lands and disarms, a ground robot stops.  "Take off" and "land" never
 * appear in the goal vocabulary.
 */

typedef enum {
    CHOREO_GOAL_NONE     = 0,
    CHOREO_GOAL_FORM     = 1,   /* arrange elements into a geometric shape */
    CHOREO_GOAL_MOVE     = 2,   /* translate formation to target point     */
    CHOREO_GOAL_DISPERSE = 3,   /* spread elements across the arena        */
    CHOREO_GOAL_CONVERGE = 4,   /* gather elements at a point              */
    CHOREO_GOAL_HOLD     = 5,   /* stay at current station                 */
    CHOREO_GOAL_EXCHANGE = 6,   /* rotate stations among participants      */
} choreo_goal_type_t;

typedef struct {
    choreo_goal_type_t    type;
    tapestry_position_t   target;        /* MOVE / CONVERGE destination           */
    float                 radius;        /* FORM radius; DISPERSE minimum spacing */
    tapestry_bse_shape_t  shape;         /* FORM shape (circle / line / grid)     */
    choreo_capabilities_t required_caps; /* capabilities this goal requires       */

    uint8_t               slot_shift;    /* EXCHANGE ring rotation (0 → 1)        */
    bool                  direct_path;   /* EXCHANGE beeline vs centroid arc
                                            (see bse.h; arc is the default)      */
    float                 achieve_eps;   /* achievement radius (0 → BSE default)  */
    uint32_t              achieve_hold_ms; /* sustain time (0 → BSE default)      */
} choreo_goal_t;

/* ── Script: an ordered sequence of goals (minimal Choreo container) ─────── */
/*
 * The paper's Choreo is a collection of Goals; this is its minimal linear
 * form — no priorities, no preemption.  Each step runs until:
 *   - its goal is achieved (if advance_on_achieved — scope decides whose
 *     achievement counts, see choreo_achieve_scope_t below), or
 *   - max_duration_ms elapses (if nonzero) — the timeout doubles as the
 *     step duration for steps that advance on time alone (e.g. HOLD 30 s).
 * A step with advance_on_achieved=false and max_duration_ms=0 never
 * advances — choreo_submit_script rejects such a step (the script would
 * stall by construction).
 *
 * When the last step completes the script terminates: directive IDLE —
 * the quiescence signal (see the goal-family comment above).
 */

/*
 * choreo_achieve_scope_t — whose achievement gates an advance_on_achieved
 *
 *   CHOREO_SCOPE_SELF  (default, zero value) — this element's own
 *                      achievement only (choreo_goal_achieved()).  This is
 *                      all that existed before scope was added, so it is
 *                      the zero-initialized behavior of every existing
 *                      choreo_step_t.
 *   CHOREO_SCOPE_ALL   — the collective predicate: this element's own
 *                      achievement AND every fresh active peer's gossiped
 *                      achieved bit (choreo_collective_achieved()).
 *                      Eventually consistent — a peer's achieved bit is
 *                      only as fresh as its last gossip frame, and a
 *                      solo element (no fresh peers) is vacuously "all
 *                      achieved" so it cannot deadlock alone.  This is
 *                      NOT a barrier/lockstep guarantee — different
 *                      elements can observe "all achieved" on different
 *                      ticks, bounded by gossip latency.  The lockstep
 *                      upgrade (design doc's `barrier = true`) is not
 *                      implemented.
 */
typedef enum {
    CHOREO_SCOPE_SELF = 0,
    CHOREO_SCOPE_ALL  = 1,
} choreo_achieve_scope_t;

typedef struct {
    choreo_goal_t          goal;
    uint32_t               max_duration_ms;     /* 0 = no timeout                   */
    bool                   advance_on_achieved; /* advance when achieved (scope-gated) */
    choreo_achieve_scope_t scope;                /* whose achievement counts; 0=SELF */
} choreo_step_t;

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
 * choreo_terminate — Abort the current goal or script and return to IDLE.
 *
 * Valid from any state.  Submits an IDLE intent to the BSE (the quiescence
 * signal), clears any active script, passes through TERMINATED, and settles
 * in IDLE.  choreo_script_complete() is unaffected — it keeps reporting
 * whether the most recent script ran to completion.
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
 * choreo_submit_script — Load and start a goal sequence.
 *
 * Terminates any active goal or script, validates every step up front
 * (goal validity, capability requirements, and that each step can advance),
 * then deploys step 0.  The steps array must remain valid while the script
 * runs (typically a static const array in the application).
 *
 * Returns 0 on success, -1 on invalid arguments or an unadvanceable step,
 * -EPERM if any step's required_caps are unsatisfied.
 */
int choreo_submit_script(const choreo_step_t *steps, uint8_t n_steps);

/*
 * choreo_script_step — Current step index, or -1 if no script is active
 * (never started, terminated, or completed).
 */
int choreo_script_step(void);

/*
 * choreo_script_complete — True once the most recently submitted script has
 * run all its steps to completion.  Reset by the next submit_script /
 * submit_goal.  This is the application's cue to map the IDLE directive to
 * platform quiescence.
 */
bool choreo_script_complete(void);

/*
 * choreo_goal_achieved — Minimal monitor-stage output: the L6 achievement
 * predicate for the currently executing goal (see bse_goal_achieved()).
 */
bool choreo_goal_achieved(void);

/*
 * choreo_collective_achieved — The scope=all achievement predicate (see
 * choreo_achieve_scope_t): true when choreo_goal_achieved() is true AND
 * every fresh, active, non-self entry in wm has its gossiped achieved bit
 * set.  Vacuously true with no fresh peers (a solo element cannot deadlock
 * on a scope=all step).  Eventually consistent — bounded by gossip
 * latency, not a synchronization barrier.
 */
bool choreo_collective_achieved(const world_model_t *wm);

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
 * choreo_current_goal_type — The goal currently executing (RUNNING or
 * SUSPENDED), or CHOREO_GOAL_NONE otherwise.  Lets the platform layer
 * apply per-goal quorum semantics: a HOLD directive may be tracked even
 * with quorum lost (it references only this element), while peer-
 * referential directives should be frozen.
 */
choreo_goal_type_t choreo_current_goal_type(void);

/*
 * choreo_tick — Drive L6 decomposition for this cycle.
 *
 * Call once per main-loop cycle (WM_CYCLE_MS period — script step timers
 * integrate real time on that assumption), after wm_tick() and scr_tick().
 * Only drives the BSE when state is RUNNING; no-op otherwise.
 * Transitions RUNNING → SUSPENDED on quorum loss (freezing the BSE and any
 * script timers), and back on recovery.
 * Advances the active script per the choreo_step_t rules.
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
