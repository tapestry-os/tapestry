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
 * ║  achievement predicate (choreo_goal_achieved), per-step indicator/       ║
 * ║  telemetry_tag effect annotations (§12 Stage 5 — see choreo_step_t),     ║
 * ║  the install/configure/deploy/terminate lifecycle stages                 ║
 * ║  (choreo_state_t below), and a TOML script authoring/compiler            ║
 * ║  toolchain (sdk/tools/choreoc.py — see sdk/CHOREO_SCRIPTS.md), a         ║
 * ║  hardware-in-the-loop simulation bridge (examples/webots-formation/ —    ║
 * ║  this stack, unmodified, against real Webots physics), and an offline    ║
 * ║  capture/replay harness (opt-in CSV capture of per-tick inputs/outputs   ║
 * ║  — choreo_telemetry.h — replayed offline through sdk/python/tapestry     ║
 * ║  and diffed tick-by-tick against the recording — sdk/tools/              ║
 * ║  choreo_sim.py --replay; see sdk/CHOREO_SCRIPTS.md's "Parity" section),  ║
 * ║  and a synthetic script-authoring simulator (sdk/tools/choreo_sim.py     ║
 * ║  --simulate — N in-process Choreo instances, perfect shared visibility,  ║
 * ║  no C/Zephyr/network, deliberately no repulsion/leash/arena-clamp        ║
 * ║  physics; not a fidelity simulator, see sdk/CHOREO_SCRIPTS.md's          ║
 * ║  "Script-authoring simulation" section).                                 ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#ifndef TAPESTRY_CHOREO_H
#define TAPESTRY_CHOREO_H

#include <tapestry/bse.h>        /* tapestry_bse_directive_t, tapestry_bse_shape_t */
#include <tapestry/substrate.h>  /* substrate_signal_t — reused directly for
                                   * choreo_step_t::indicator, same pattern as
                                   * bse.h's frame/anchor/shape/motion enums
                                   * above: no parallel CHOREO_INDICATOR_*
                                   * enum.  substrate.h itself depends on
                                   * nothing else (its own layering note), so
                                   * this pulls in no transitive cross-layer
                                   * dependency. */

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
 *
 *               A HOLD step's OWN max_duration_ms is the one exception to
 *               "step timers are frozen": it keeps counting down while
 *               suspended too (choreo.c's suspended_hold_timeout()), so a
 *               script can give up on permanent isolation instead of
 *               station-keeping forever with no peer ever left to revive
 *               it — every other goal type's timer stays frozen exactly
 *               as before.  Combine with a CHOREO_EVENT_QUORUM_LOST
 *               transition (choreo_event_t below) on the step that
 *               precedes the HOLD to actively choose a safe fallback
 *               station instead of freezing wherever isolation happened
 *               to strike (e.g. mid-EXCHANGE-arc).
 *
 *               SUSPENDED is deliberately defined as "paused, preserved,
 *               resumes automatically" rather than as "quorum lost".  An
 *               implementation with a prioritised goal queue reports a
 *               preempted goal as SUSPENDED too: the semantics are the
 *               same from the application's side, only the cause differs.
 *               This enum is therefore not expected to grow a PREEMPTED
 *               member — adding one would break exhaustive switches in
 *               existing applications for no gain.  A caller that needs to
 *               distinguish the causes should read the goal's identity
 *               (choreo_goal_t::id) and the monitor/telemetry path, not
 *               the lifecycle state.
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
 *   CHOREO_CAP_LOCOMOTION    → SCR_CAP_ACTUATOR       (physical actuation node)
 *   CHOREO_CAP_SENSING       → SCR_CAP_SENSOR         (observation / sensing node)
 *   CHOREO_CAP_SIGNALING     → SCR_CAP_RELAY          (message-forwarding, best-fit)
 *   CHOREO_CAP_BONDING       → SCR_CAP_BONDING        (physical bonding/docking)
 *   CHOREO_CAP_ABS_POSITION  → SCR_CAP_ABS_POSITION   (lighthouse/GPS/mocap etc)
 *
 * required_caps is a floor the author can raise but not lower: choreo.c's
 * derived_caps() unions it with capabilities implied by the goal's own axis
 * values before checking (Choreo SDK Design doc §11) — motion == SPIN
 * demands CHOREO_CAP_LOCOMOTION, and a FORM/CONVERGE goal with frame ==
 * ABSOLUTE demands CHOREO_CAP_ABS_POSITION, even if the author forgot to
 * declare either.
 *
 * uint16_t, not uint8_t: this is an open, application-level vocabulary (§11
 * of the Choreo SDK Design doc derives capability requirements from
 * every axis value and effect a goal declares), and uint8_t's 4 spare bits
 * (4 of 8 already used above) can't absorb that.  Never gossiped — set
 * once locally via scr_init()'s capabilities param — so widening this is a
 * pure local API change with no wire-protocol consequence.
 */
typedef uint16_t choreo_capabilities_t;

#define CHOREO_CAP_NONE          ((choreo_capabilities_t)0x00)
#define CHOREO_CAP_LOCOMOTION    ((choreo_capabilities_t)0x01)
#define CHOREO_CAP_BONDING       ((choreo_capabilities_t)0x02)
#define CHOREO_CAP_SENSING       ((choreo_capabilities_t)0x04)
#define CHOREO_CAP_SIGNALING     ((choreo_capabilities_t)0x08)
#define CHOREO_CAP_ABS_POSITION  ((choreo_capabilities_t)0x10)

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
    position_t   target;        /* MOVE / CONVERGE destination           */
    float                 radius;        /* FORM radius; DISPERSE minimum spacing */
    tapestry_bse_shape_t  shape;         /* FORM shape (circle / line / grid)     */
    choreo_capabilities_t required_caps; /* capabilities this goal requires       */

    /* FORM / CONVERGE only — Choreo SDK Design doc §5's frame ladder.
     * tapestry_bse_frame_t/tapestry_bse_anchor_selector_t (bse.h) reused
     * directly rather than a parallel CHOREO_FRAME_* enum, same as `shape`
     * above — see that header for the full rationale (ABSOLUTE=0 as the
     * compat-preserving default, why NEWEST/OLDEST aren't here yet, the
     * debounce requirement).  frame == 0 (ABSOLUTE) ignores anchor/
     * anchor_id entirely and uses `target` exactly as today.  choreo_goal_t
     * carries these values opaquely through to the BSE intent
     * (goal_to_intent()); L7 does no frame/anchor resolution of its own. */
    tapestry_bse_frame_t           frame;
    tapestry_bse_anchor_selector_t anchor;     /* frame == ELEMENT only */
    element_id_t                   anchor_id;  /* anchor == ID only     */

    /* FORM only — §6's motion modifiers.  motion == 0 (STATIC) is the
     * default.  CONVERGE never reads this (see bse.c's CONVERGE case). */
    tapestry_bse_motion_t motion;
    float                 spin_rate_radps;  /* motion == SPIN only */

    uint8_t               slot_shift;    /* EXCHANGE ring rotation (0 → 1)        */
    bool                  direct_path;   /* EXCHANGE beeline vs centroid arc
                                            (see bse.h; arc is the default)      */
    float                 achieve_eps;   /* achievement radius (0 → BSE default)  */
    uint32_t              achieve_hold_ms; /* sustain time (0 → BSE default)      */

    /*
     * Caller-assigned goal identity.  Opaque to Tapestry: the SDK never
     * generates, interprets, or requires it, and 0 means "anonymous" —
     * the behavior of every goal written before this field existed.
     *
     * It exists so that goal identity does not have to be retrofitted onto
     * choreo_submit_goal()'s return value later.  This implementation runs
     * one goal at a time, so an application can always say "the current
     * goal"; an implementation with a prioritised goal queue cannot, and
     * needs a way for callers to name which goal to cancel, and a way to
     * report which goal preempted which.  Assigning that name here keeps
     * both additive: no existing signature changes, and no call site has
     * to be touched.  Carried through to tapestry_bse_intent_t::id so L6
     * can attribute a directive to the goal that produced it.
     */
    uint16_t              id;
} choreo_goal_t;

/* ── Script: an ordered sequence of goals (minimal Choreo container) ─────── */
/*
 * The paper's Choreo is a collection of Goals; this is its linear-track
 * form (no numeric priority, no track concurrency — see choreo_preempt_
 * goal() for one-off preemption, and §7 of the design doc for the
 * not-yet-implemented multi-track container).  Each step runs until:
 *   - a declared transition fires (choreo_transition_t below — checked
 *     first, in declaration order; the first match wins), or, absent any
 *     matching transition:
 *   - its goal is achieved (if advance_on_achieved — scope decides whose
 *     achievement counts, see choreo_achieve_scope_t below), or
 *   - max_duration_ms elapses (if nonzero) — the timeout doubles as the
 *     step duration for steps that advance on time alone (e.g. HOLD 30 s).
 * A step with advance_on_achieved=false and max_duration_ms=0 and no
 * transitions never advances — choreo_submit_script rejects such a step
 * (the script would stall by construction).
 *
 * When the last step completes the script terminates: directive IDLE —
 * the quiescence signal (see the goal-family comment above).  A
 * transition may also jump directly to completion — see
 * choreo_transition_t::goto_step_idx.
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
 *                      achievement AND every ACTIVE peer's gossiped
 *                      achieved bit (choreo_collective_achieved()).
 *                      Eventually consistent — a peer's achieved bit is
 *                      only as fresh as its last gossip frame, and a
 *                      genuinely solo element (no active peers at all) is
 *                      vacuously "all achieved" so it cannot deadlock
 *                      alone.  A peer that has merely gone stale still
 *                      votes, from its last-received bit: gating on
 *                      freshness instead let this predicate collapse into
 *                      CHOREO_SCOPE_SELF whenever the link dropped out.
 *                      This is
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

/*
 * choreo_event_t / choreo_transition_t — Choreo SDK Design doc §8.2's
 * event vocabulary, this subset only (the "welcome dance" demo, §8.3's
 * flagship for this feature, uses only ELEMENT_JOINED/ELEMENT_LOST):
 *
 *   CHOREO_EVENT_ACHIEVED        — the step's own achievement predicate
 *                                  (scope-gated, same as advance_on_achieved)
 *                                  as an EXPLICIT transition target, not
 *                                  just an implicit next-index advance.
 *   CHOREO_EVENT_ELEMENT_JOINED  — debounced rise in scr_get_swarm_size().
 *   CHOREO_EVENT_ELEMENT_LOST    — debounced fall in scr_get_swarm_size().
 *   CHOREO_EVENT_COUNT_GTE       — scr_get_swarm_size() >= threshold.
 *   CHOREO_EVENT_COUNT_EQ        — scr_get_swarm_size() == threshold.
 *   CHOREO_EVENT_ANCHOR_LOST     — bse_anchor_lost() (FORM/CONVERGE
 *                                  frame == ELEMENT only).
 *   CHOREO_EVENT_QUORUM_LOST     — scr->quorum_state == SCR_QUORUM_LOST,
 *                                  checked on the same tick as (and
 *                                  before) choreo_tick()'s own automatic
 *                                  RUNNING -> SUSPENDED transition — see
 *                                  below.
 *
 * quorum_degraded/quorum_recovered (§8.2's other two collective events)
 * remain deliberately absent — no concrete use case has been identified
 * for either (recovery already has its own automatic SUSPENDED ->
 * RUNNING resume, which continues the step that was running rather than
 * needing an explicit target). QUORUM_LOST *is* now here, and it turned
 * out NOT to need what the previous version of this comment worried
 * about — suppressing the automatic RUNNING -> SUSPENDED transition for
 * that tick. It doesn't need to: a step's QUORUM_LOST transition runs
 * exactly like any other `on[]` entry, jumping to its target BEFORE the
 * automatic transition is evaluated (choreo.c's script_advance() is
 * called first). If the target step is a HOLD, choreo_tick()'s
 * SUSPENDED case (see choreo_state_t above) already knows how to run a
 * HOLD safely while isolated — including timing it out via its own
 * max_duration_ms — so it's simply correct for the state machine to
 * suspend it immediately afterward on the same tick it activated; the
 * escape hatch's whole job is choosing WHICH goal freezes, not avoiding
 * the freeze. The design question this comment used to flag turned out
 * to dissolve once "SUSPENDED + HOLD" itself stopped meaning "frozen
 * forever."
 *
 * ELEMENT_JOINED/ELEMENT_LOST and the two COUNT_* variants share ONE
 * debounce timer per Choreo instance (continuous across the whole
 * script's lifetime, matching TAPESTRY_BSE_ANCHOR_HOLD_MS's "a selector/
 * count result must be stable before acting on it" lesson) — see
 * choreo.c's membership-debounce comment. QUORUM_LOST has no debounce of
 * its own in choreo.c — it reads scr->quorum_state verbatim, same as the
 * automatic RUNNING -> SUSPENDED transition next to it always has. On
 * cf21bl-formation/webots-formation, scr->quorum_state already carries
 * the flight-tested 2-second upward debounce applied at the app level
 * before choreo_tick() ever sees it; a caller that skips that (e.g. the
 * Python SDK driven directly, without an equivalent filter) gets exactly
 * as much debounce on this event as it gets on the automatic suspend —
 * none — which is a pre-existing property of this state machine, not
 * something new QUORUM_LOST introduces.
 */
typedef enum {
    CHOREO_EVENT_ACHIEVED       = 0,
    CHOREO_EVENT_ELEMENT_JOINED = 1,
    CHOREO_EVENT_ELEMENT_LOST   = 2,
    CHOREO_EVENT_COUNT_GTE      = 3,   /* threshold */
    CHOREO_EVENT_COUNT_EQ       = 4,   /* threshold */
    CHOREO_EVENT_ANCHOR_LOST    = 5,
    CHOREO_EVENT_QUORUM_LOST    = 6,
} choreo_event_t;

/* Bounded, no dynamic allocation — matches every other bounded-array
 * pattern in this codebase (MAX_ELEMENTS etc.). */
#define CHOREO_MAX_TRANSITIONS 4

/*
 * goto_step_idx names a step by INDEX into the same steps array
 * choreo_submit_script() was given — step NAMES (TOML `name = "..."`)
 * are resolved to indices at TOML-compile time (choreoc.py); the C
 * runtime stays string-free.  goto_step_idx == n_steps (one past the
 * last step) is "end" — completes the script from anywhere, exactly like
 * naturally completing the last step.  choreo_submit_script() rejects an
 * out-of-range goto_step_idx up front (goto_step_idx > n_steps).
 */
typedef struct {
    choreo_event_t event;
    uint8_t        threshold;      /* COUNT_GTE / COUNT_EQ only */
    uint8_t        goto_step_idx;
} choreo_transition_t;

typedef struct {
    choreo_goal_t          goal;
    uint32_t               max_duration_ms;     /* 0 = no timeout                   */
    bool                   advance_on_achieved; /* advance when achieved (scope-gated) */
    choreo_achieve_scope_t scope;                /* whose achievement counts; 0=SELF */

    /* §8.2/§8.3 guarded transitions.  Evaluated in order; the FIRST
     * matching event wins and its goto_step_idx becomes the next step —
     * checked before advance_on_achieved/max_duration_ms, which remain
     * the fallback for a step with no matching (or no declared)
     * transition, i.e. every existing choreo_step_t is unaffected. */
    choreo_transition_t on[CHOREO_MAX_TRANSITIONS];
    uint8_t              n_transitions;   /* 0 = none declared (default) */

    /*
     * Choreo SDK Design doc §12 Stage 5: effect step annotations.  Both
     * default to "no effect" (SUBSTRATE_SIGNAL_NONE / NULL) — a step that
     * doesn't set either is byte-identical to every choreo_step_t written
     * before this feature existed.
     *
     * indicator — while this step is active, choreo_current_indicator()
     *   returns this value instead of SUBSTRATE_SIGNAL_NONE.  Choreo
     *   itself never calls substrate_set_signal() (L7 has no L1
     *   dependency beyond this reused enum type — see the substrate.h
     *   include above); the application is expected to read
     *   choreo_current_indicator() once per tick and pass it through, the
     *   same way it already passes choreo_get_directive() to
     *   substrate_move().  This makes the *desired* signal declarative
     *   and script-portable instead of hand-computed per app — see
     *   examples/cf21bl-formation/src/formation.c's demo_set_leds() and
     *   examples/webots-formation/controllers/common/tracker.c's
     *   (previously independently-duplicated, identical) copy, both of
     *   which now take the step's declared indicator as an override and
     *   fall back to their existing wm-quorum heuristic when it's NONE.
     *
     * telemetry_tag — an opaque label surfaced verbatim by
     *   choreo_current_telemetry_tag(), for a platform's telemetry
     *   capture (e.g. examples/webots-formation's choreo_telemetry.h) to
     *   record alongside script_step, so a replay or choreo-sim run can
     *   be identified by which authored step produced a given tick
     *   without depending on step index alone.  This is local capture
     *   only — NOT a wire delivery mechanism to an external consumer
     *   (e.g. a facility monitoring dashboard); no such consumer exists
     *   anywhere in this repo, and building live delivery for one would
     *   be speculative.  Must outlive the step array, same requirement
     *   choreo_submit_script() already places on `steps` itself — a TOML-
     *   authored string literal or a static const char* satisfies this.
     *
     *   ABI note: this is the ONE non-plain-old-data field anywhere in
     *   choreo_goal_t/choreo_transition_t/choreo_step_t — everything else
     *   in this struct tree is scalars, enums, and fixed-size arrays, so
     *   it round-trips through a byte-for-byte copy with no pointer
     *   fixup. A raw C pointer has no stable meaning across a wire
     *   transfer (e.g. a future non-choreoc script delivery path — see
     *   choreo_submit_script()'s doc), so a decoder populating a
     *   choreo_step_t from bytes that did not originate at compile time
     *   on THIS device must special-case this field: leave it NULL, or
     *   replace it with a small fixed-size inline buffer / tag-ID lookup
     *   if the capability is wanted over that path. Do not add a second
     *   pointer field to this struct tree without the same consideration.
     */
    substrate_signal_t indicator;
    const char         *telemetry_tag;
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
 * goal->required_caps (see that field's doc for the derived-floor caveat —
 * some axis values demand a capability whether or not required_caps
 * declares it).
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
 * choreo_terminate — Abort the current goal or script.
 *
 * Valid from any state.  If choreo_preempt_goal() has a goal parked, this
 * RESUMES it instead of going to IDLE: the parked goal (and, if it was a
 * script, its exact step/timer position) becomes active again, exactly as
 * it was at the moment it was preempted, and this returns to RUNNING —
 * repeated calls unwind one preemption level at a time.  With nothing
 * parked (the only case before this feature existed, and the common case
 * today), behavior is unchanged: submits an IDLE intent to the BSE (the
 * quiescence signal), clears any active script, passes through TERMINATED,
 * and settles in IDLE.  choreo_script_complete() is unaffected either way —
 * it keeps reporting whether the most recent script ran to completion.
 */
void choreo_terminate(void);

/*
 * choreo_submit_goal — One-shot convenience: configure + deploy.
 *
 * Unconditionally fully resets state first if anything is active —
 * including discarding any goal parked by choreo_preempt_goal(), unlike
 * choreo_terminate() — then calls choreo_configure(goal) followed by
 * choreo_deploy().  An ordinary new submission always replaces everything;
 * use choreo_preempt_goal() when the previous goal should survive.
 *
 * Returns 0 on success, -1 on invalid goal, -EPERM on capability mismatch.
 */
int choreo_submit_goal(const choreo_goal_t *goal);

/*
 * choreo_submit_script — Load and start a goal sequence.
 *
 * Unconditionally fully resets state first (see choreo_submit_goal() —
 * same "always replaces everything, including any parked goal" rule),
 * validates every step up front (goal validity, capability requirements,
 * and that each step can advance), then deploys step 0.  The steps array
 * must remain valid while the script runs (typically a static const array
 * in the application) — this API takes a pointer and a count, and neither
 * this function nor anything it calls cares how that memory was
 * populated. Every consumer today points it at a `choreoc`-generated
 * `static const` array, but that is a self-imposed convention of today's
 * callers, not a constraint choreo.c enforces — a caller could equally
 * decode a wire-delivered blob into its own persistent buffer first. See
 * choreo_step_t's doc for the one field (`telemetry_tag`) that isn't
 * plain-old-data and would need special handling by any such decoder,
 * and validate_steps()'s doc in choreo.c for what re-validation already
 * happens on-device regardless of where `steps` came from (and the one
 * check — cycle/max_runtime — that does not).
 *
 * Returns 0 on success, -1 on invalid arguments or an unadvanceable step,
 * -EPERM if any step's required_caps are unsatisfied.
 */
int choreo_submit_script(const choreo_step_t *steps, uint8_t n_steps);

/*
 * choreo_preempt_goal — Run `goal` immediately, preserving the currently
 * active goal (or script, including its exact step/timer position) to
 * resume automatically later — via choreo_terminate() / choreo_cancel_goal()
 * once `goal` is done, or naturally when a preempting script (not
 * supported here — `goal` is always a single goal) would complete.
 *
 * Unlike choreo_submit_goal(), this does NOT discard what was running: it
 * is the mechanism side of a goal queue with preemption (v1.0 scope: one
 * level deep — a second choreo_preempt_goal() call while one is already
 * parked returns -EBUSY; nest deeper is a future extension, not a
 * redesign). The preempting goal itself cannot be preempted again by
 * anything other than choreo_terminate()/choreo_cancel_goal() resuming
 * what it displaced.
 *
 * Requires an active goal (RUNNING or SUSPENDED) to preempt — returns -1
 * from IDLE/CONFIGURED/TERMINATED, since there is nothing to preserve.
 *
 * Returns 0 on success, -1 if goal is NULL, type is CHOREO_GOAL_NONE, or
 * nothing is active to preempt; -EBUSY if something is already parked;
 * -EPERM on capability mismatch.
 */
int choreo_preempt_goal(const choreo_goal_t *goal);

/*
 * choreo_is_preempted — True if choreo_preempt_goal() has a goal parked
 * that choreo_terminate()/choreo_cancel_goal() would resume.
 */
bool choreo_is_preempted(void);

/*
 * choreo_parked_goal_id — The choreo_goal_t::id of the parked goal (see
 * that field's doc for why it exists: naming which goal to resume, and
 * reporting which goal preempted which). 0 if nothing is parked, or if the
 * parked goal never set an id.
 */
uint16_t choreo_parked_goal_id(void);

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
 * every ACTIVE, non-self entry in wm has its gossiped achieved bit set.
 * Vacuously true only with no active peers at all (a genuinely solo element
 * cannot deadlock on a scope=all step); a stale-but-active peer still votes
 * from its last-received bit.  Eventually consistent — bounded by gossip
 * latency, not a synchronization barrier.
 */
bool choreo_collective_achieved(const world_model_t *wm);

/*
 * choreo_cancel_goal — Cancel the current goal.
 * Thin wrapper around choreo_terminate() — see that function for the
 * "resumes a parked goal instead of going to IDLE, if one exists" behavior.
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
 *
 * Source selection (remote directives, wire.h v5): while a remote L6 BSE's
 * directives are ADOPTED (see choreo_remote_directive() below), this
 * returns the latest remote directive instead of the local BSE's — but
 * only in CHOREO_STATE_RUNNING.  In every other state, and whenever the
 * remote stream is stale or not yet adopted, it returns the local BSE
 * directive exactly as before the remote path existed.
 */
const tapestry_bse_directive_t *choreo_get_directive(void);

/* ── Remote L6 directives (wire.h v5) ────────────────────────────────────── */
/*
 * A remote BSE host (edge node, or an elected SCR_CAP_BSE_HOST element)
 * can stream per-element directives over the wire.  The element treats
 * them as a refinement of — never a replacement for — its own locally
 * computed behavior:
 *
 *   - The local BSE keeps ticking and the script keeps advancing the whole
 *     time, so falling back is bumpless by construction: the local
 *     directive is always current, never resumed from a freeze.
 *   - Remote directives steer the output only after the stream has been
 *     continuously fresh for CHOREO_REMOTE_ADOPT_HOLD_MS (the same
 *     stability-before-acting lesson as TAPESTRY_BSE_ANCHOR_HOLD_MS and
 *     CHOREO_MEMBERSHIP_HOLD_MS: a flapping edge link must not thrash the
 *     steering source).  This hold applies to first adoption and every
 *     re-adoption after a stale period alike.
 *   - A remote directive older than CHOREO_REMOTE_STALE_MS stops steering
 *     immediately (fall back to local is instant; only adoption is
 *     debounced — asymmetric on purpose, like quorum up/down).
 *   - SUSPENDED (quorum lost) always steers locally: L5 remains the
 *     safety authority, and a remote host's view of a partitioned
 *     collective is exactly what cannot be trusted.
 *
 * Staleness is aged in choreo_tick() (WM_CYCLE_MS per call), measured from
 * the arrival of the last accepted frame — never from sender-side time
 * (elements share no wall clock; see wire.h).
 */

#ifndef CHOREO_REMOTE_STALE_MS
#define CHOREO_REMOTE_STALE_MS       1500u   /* == WM_STALE_THRESHOLD_MS lesson */
#endif
#ifndef CHOREO_REMOTE_ADOPT_HOLD_MS
#define CHOREO_REMOTE_ADOPT_HOLD_MS  2000u   /* == CHOREO_MEMBERSHIP_HOLD_MS   */
#endif

/*
 * choreo_remote_directive — Feed one received remote directive in.
 *
 * Called by the runtime bridge (runtime.c) after transport_poll_directive()
 * accepts a frame; d is the frame's payload converted to a BSE directive,
 * goal_id/src_id are carried for telemetry attribution.  Resets the
 * staleness clock.  Never steers the output by itself — adoption is
 * decided in choreo_tick() per the rules above.
 */
void choreo_remote_directive(const tapestry_bse_directive_t *d,
                             uint16_t goal_id, uint8_t src_id);

/*
 * choreo_remote_active — True while choreo_get_directive() is returning
 * remote directives (adopted, fresh, and state is RUNNING).  For
 * telemetry and tests.
 */
bool choreo_remote_active(void);

/*
 * choreo_current_indicator — The active step's declared indicator effect
 * (§12 Stage 5), or SUBSTRATE_SIGNAL_NONE if the current step (or track
 * step) declared none, or if no script is active (a bare submit_goal()
 * has no step to annotate).  See choreo_step_t::indicator.
 */
substrate_signal_t choreo_current_indicator(void);

/*
 * choreo_current_telemetry_tag — The active step's declared telemetry tag
 * (§12 Stage 5), or NULL under the same conditions
 * choreo_current_indicator() returns SUBSTRATE_SIGNAL_NONE.  See
 * choreo_step_t::telemetry_tag.
 */
const char *choreo_current_telemetry_tag(void);

/* ── Tracks (Choreo SDK Design doc §7) ───────────────────────────────────── */
/*
 * choreo_submit_script() runs ONE implicit "all participants" track — the
 * only case that existed before this feature, and still the byte-
 * identical default (choreo_current_track() reports 0 for it, same as
 * every peer's gossiped current_track on a script with no tracks). Tracks
 * let DIFFERENT elements run DIFFERENT step sequences concurrently,
 * membership determined LOCALLY by each element from its own state — no
 * coordination messages (P4) — e.g. "hold the perimeter, except elements
 * whose battery is low, which return to charge instead" as two tracks
 * rather than one script with an escape hatch.
 *
 * An element runs exactly ONE track at a time — the concurrency is ACROSS
 * elements in different tracks, not multiple simultaneous intents on one
 * element (§7.1: "an element joins the FIRST track whose filter it
 * matches"). This is why choreo_submit_tracks() needs no L6 concurrency
 * support: bse.c still drives a single active intent, exactly as today —
 * only choreo.c's bookkeeping (which track, which step within it) grows.
 *
 * Filters are evaluated in declaration order at each debounced epoch
 * (CHOREO_MEMBERSHIP_HOLD_MS, same "stable before acting" pattern as
 * TAPESTRY_BSE_ANCHOR_HOLD_MS) — EXCEPT the very first determination,
 * made synchronously inside choreo_submit_tracks() itself, which is
 * immediate: there is no prior track to flicker away from on a fresh
 * submission, so nothing to debounce against yet.  Migrating to a
 * different track activates that track's current step FRESH (new
 * snapshot/timers) — not a preserved-
 * state resume like choreo_preempt_goal()'s stack; simpler, since tracks
 * don't nest.  Each track's own step index persists while the element is
 * inactive in it (a small fixed array, not a full activation save) — re-
 * entry resumes at that index, fresh.
 *
 * Track migration evaluation is suspended while a goal is parked
 * (choreo_is_preempted()) — the two mechanisms aren't designed to compose
 * mid-flight; preemption already fully occupies the same underlying step-
 * engine state tracks would otherwise migrate.
 */

/*
 * choreo_track_filter_t — which elements belong to a track, evaluated by
 * each element against its OWN state (never a peer's) — this is why
 * required_caps works without gossiping capabilities (§10's "Track
 * membership needs one small gossiped field" applies to COMMUNICATING
 * the result via current_track, not to evaluating the filter itself).
 * The zero value (both fields false/0) matches every element — the "all"
 * default a script with no [[tracks]] uses.
 */
typedef struct {
    choreo_capabilities_t required_caps;       /* 0 = no capability requirement */
    bool                  requires_energy_low; /* §8.2 self-event: energy_low — */
                                                /* ELEMENT_HEALTH_LOW_BATTERY    */
} choreo_track_filter_t;

typedef struct {
    choreo_track_filter_t filter;
    const choreo_step_t  *steps;
    uint8_t                n_steps;
} choreo_track_t;

/* Bounded, no dynamic allocation — matches CHOREO_MAX_TRANSITIONS/
 * MAX_ELEMENTS elsewhere in this codebase. */
#define CHOREO_MAX_TRACKS 4

/*
 * choreo_submit_tracks — Load and start a multi-track Choreo.
 *
 * Validates every track's every step exactly as choreo_submit_script()
 * does (goal validity, capability requirements, transition targets in
 * range), then determines this element's initial track (first matching
 * filter, undebounced, evaluated against `wm` — the same reason
 * choreo_collective_achieved() takes a world_model_t* instead of caching
 * one: submission is caller-driven, not tick-driven) and deploys its
 * step 0.
 *
 * Returns 0 on success, -1 on invalid arguments, an unadvanceable step,
 * an out-of-range transition target, or too many tracks; -EPERM on
 * capability mismatch; -1 if no track's filter matches this element (no
 * catch-all "all" track declared) — the same "nothing to run" case
 * choreo_configure() rejects for a NONE goal.
 */
int choreo_submit_tracks(const world_model_t *wm, const choreo_track_t *tracks, uint8_t n_tracks);

/*
 * choreo_current_track — This element's active track index (0 if not
 * currently in multi-track mode, or on track 0) — it is what lets OTHER
 * elements' collect_participants() (bse.c) filter to peers on the SAME
 * track; see wire.h's v4 comment. Prefer choreo_publish_state() below
 * over calling this directly; it exists for a caller that genuinely
 * wants only the track index and not the rest.
 */
uint8_t choreo_current_track(void);

/*
 * choreo_publish_state — Copy this tick's gossip-relevant L6/L7 output
 * (goal_achieved, current_track) into own_state, ready for gossip_send().
 *
 * Every application that gossips must call this once per cycle, after
 * choreo_tick() and before gossip_send() — element_state_t::goal_achieved
 * and ::current_track are gossiped fields (so peers can aggregate a
 * scope="all" achievement predicate and filter to same-track peers), but
 * Choreo has no handle on element_state_t itself (an L3/L4 wire struct,
 * not something L7 touches) to keep them current on its own. This one
 * call replaces what used to be two separate manual assignments
 * (own_state.goal_achieved = choreo_goal_achieved(); own_state.
 * current_track = choreo_current_track();), duplicated identically
 * across every app that gossips — an obligation that was silently
 * forgotten twice in this repo's history before either mistake was
 * caught (see tapestry-os/tests/transport's suite header comment).
 * Consolidating to one well-named call doesn't make forgetting it
 * impossible, but it cuts the surface from two fields to remember to
 * one, and gives it a name a reviewer or a grep can actually find.
 *
 * Safe to call in any lifecycle state; choreo_goal_achieved()/
 * choreo_current_track() are themselves state-safe (see their own docs).
 */
void choreo_publish_state(element_state_t *own_state);

#endif /* TAPESTRY_CHOREO_H */
