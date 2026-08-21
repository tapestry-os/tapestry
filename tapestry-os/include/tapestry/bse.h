/*
 * bse.h — Tapestry L6 Behavior Synthesis Engine interface
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  v1.0 FEATURE SCOPE                                                      ║
 * ║                                                                          ║
 * ║  This header defines the full L6 interface contract.  bse.c              ║
 * ║  implements, open-core (public):                                         ║
 * ║    ✓  Intent parsing — declarative goal → per-element behavioral spec    ║
 * ║    ✓  Task decomposition — FORM (CIRCLE/LINE/GRID vertex assignment),    ║
 * ║       MOVE (offset-preserving formation translation), CONVERGE           ║
 * ║       (collapse to a point), EXCHANGE (station rotation over snapshot    ║
 * ║       positions, arc or direct path), HOLD (station-keep), DISPERSE      ║
 * ║       (spring-field spacing)                                             ║
 * ║    ✓  Feedback controller (minimal) — achievement predicate: own         ║
 * ║       position within achieve_eps of the goal point, sustained for       ║
 * ║       achieve_hold_ms (bse_goal_achieved); collective (scope=all)        ║
 * ║       achievement aggregation lives one layer up in choreo.h             ║
 * ║    ✓  Simulation bridge (hardware-in-the-loop) — this unmodified L6/L7   ║
 * ║       stack runs against real Webots physics (examples/webots-formation/)║
 * ║    ✓  Offline replay harness — capture per-tick L6/L7 inputs/outputs to  ║
 * ║       CSV (examples/webots-formation/.../choreo_telemetry.h) and replay  ║
 * ║       them through sdk/python/tapestry offline for tick-by-tick          ║
 * ║       regression testing (sdk/tools/choreo_sim.py --replay)              ║
 * ║    ✓  Script-authoring simulator — synthetic multi-element run of a      ║
 * ║       .choreo.toml through sdk/python/tapestry with no C/Zephyr/network  ║
 * ║       and perfect shared visibility, for sub-second script feedback      ║
 * ║       before a substrate exists or a build toolchain is set up           ║
 * ║       (sdk/tools/choreo_sim.py --simulate); not a fidelity simulator —   ║
 * ║       that remains the simulation bridge's job                           ║
 * ║                                                                          ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Architecture note — L2 drives the call chain; L6 does not call back into L5.
 *
 *   L7 Application  ──submit_intent──►  bse_submit_intent()
 *   L2 Main loop    ──tick───────────►  bse_tick()      (after scr_tick)
 *   L2 Main loop    ──query──────────►  bse_get_directive()
 *
 * The BSE does not mutate SCR state.  L5 (SCR) is the authoritative safety
 * and coordination layer; L6 consumes L5 outputs (role, quorum, task_slot)
 * as inputs each tick.  Both L5 and L6 are sequenced by the L2 main loop.
 *
 * Units: positions are dimensionless floats — meters on the cf21bl
 * lighthouse frame, abstract [0,100] units on the simulated world.  The
 * achievement defaults below are meter-scale; abstract-world applications
 * should set achieve_eps explicitly on each intent.  Positions are full 3D
 * (x, y, z) — z is a real, always-considered coordinate, not an optional
 * or platform-managed extra; see position_t (csm.h).
 */

#ifndef TAPESTRY_BSE_H
#define TAPESTRY_BSE_H

#include <tapestry/csm.h>
#include <tapestry/scr.h>

/* ── Intent: L7 → L6 ─────────────────────────────────────────────────────── */
/*
 * IDLE / FORM / DISPERSE / CONVERGE reference absolute coordinates supplied
 * by the application.  HOLD and EXCHANGE instead reference the collective's
 * OWN current configuration (positions read from the L4 world model) — no
 * coordinates appear in the intent at all.  MOVE is a hybrid: an absolute
 * target, displaced per-element by an offset read from the collective's own
 * configuration at activation (see below) — the formation translates as a
 * rigid body instead of collapsing onto the target.
 *
 *   HOLD      Stay at the current station.  On the first tick the element
 *             captures its own position as its station and station-keeps
 *             there (directive MOVE_TO_POINT to the captured point).
 *
 *   MOVE      Translate the formation to intent.target, preserving shape.
 *             On activation each element snapshots its own offset from the
 *             participant centroid (self + fresh peers); every tick the
 *             commanded point is intent.target + that offset, so the
 *             formation's relative geometry travels as a rigid body instead
 *             of every element collapsing onto the same point (that
 *             collapse is what CONVERGE does instead).  A solo element has
 *             a zero offset, so MOVE degenerates to CONVERGE for it —
 *             correct, there is no formation to preserve.
 *
 *   EXCHANGE  Rotate stations by slot_shift around the ID-sorted ring of
 *             participants (self + fresh peers): element at rank r takes the
 *             station of the element at rank (r + slot_shift) mod N.  With
 *             two elements and slot_shift=1 this is exactly "swap places".
 *
 *             Stations are a SNAPSHOT of participant positions captured from
 *             the world model at intent activation — frozen, never
 *             live-chasing (two elements each flying at the other's live
 *             position never converge).  If no fresh peer is visible at
 *             activation the capture retries each tick and the directive is
 *             HOLD until it succeeds.
 *
 *             The commanded target travels an ARC about the snapshot
 *             centroid (constant angular rate, radius interpolated), all
 *             elements orbiting in the same (CCW) direction — so mutual
 *             separation is preserved by construction instead of every
 *             element cutting straight through the formation center.  This
 *             is the minimal stand-in for the physics-aware planner: pure
 *             geometry, identical on every element, no messages.
 */

typedef enum {
    TAPESTRY_BSE_INTENT_IDLE     = 0,   /* no active goal; quiescent      */
    TAPESTRY_BSE_INTENT_FORM     = 1,   /* arrange elements into a shape  */
    TAPESTRY_BSE_INTENT_MOVE     = 2,   /* translate formation to target  */
    TAPESTRY_BSE_INTENT_DISPERSE = 3,   /* spread elements across arena   */
    TAPESTRY_BSE_INTENT_CONVERGE = 4,   /* gather elements at a point     */
    TAPESTRY_BSE_INTENT_HOLD     = 5,   /* station-keep at own position   */
    TAPESTRY_BSE_INTENT_EXCHANGE = 6,   /* rotate stations by slot_shift  */
} tapestry_bse_intent_type_t;

typedef enum {
    TAPESTRY_BSE_SHAPE_CIRCLE = 1,
    TAPESTRY_BSE_SHAPE_LINE   = 2,
    TAPESTRY_BSE_SHAPE_GRID   = 3,
} tapestry_bse_shape_t;

/* ── Frames and anchors (FORM / CONVERGE only) ───────────────────────────── */
/*
 * Choreo SDK Design doc §5 "frame ladder": what a FORM/CONVERGE target is
 * defined relative to.  ABSOLUTE (0) is the zero value on purpose — every
 * FORM/CONVERGE goal today sets an absolute target with no alternative, so
 * zero-initialized/frame-less intents must keep behaving exactly as they do
 * today.  §12's "frame defaults follow P1 (coordinate-free)" is honored by
 * making the coordinate-free path newly *available*, not retroactively
 * default — an author opts in explicitly via COLLECTIVE or ELEMENT.
 *
 * Only FORM and CONVERGE read this — HOLD/EXCHANGE are already inherently
 * coordinate-free (§5.1's ladder table: "n/a" for both), and MOVE/DISPERSE
 * are not in this stage's scope.
 */
typedef enum {
    TAPESTRY_BSE_FRAME_ABSOLUTE   = 0,  /* target is a literal point (today) */
    TAPESTRY_BSE_FRAME_COLLECTIVE = 1,  /* target := live participant centroid */
    TAPESTRY_BSE_FRAME_ELEMENT    = 2,  /* target := resolved anchor's position */
} tapestry_bse_frame_t;

/*
 * §5.2 anchor selectors, meaningful only when frame == ELEMENT.  NEWEST/
 * OLDEST are deliberately not here yet — they need L4 join-order tracking
 * that doesn't exist; see bse.c's anchor resolution comment.
 */
typedef enum {
    TAPESTRY_BSE_ANCHOR_LEADER        = 0,  /* scr_get_leader()              */
    TAPESTRY_BSE_ANCHOR_ID             = 1, /* explicit intent.anchor_id     */
    TAPESTRY_BSE_ANCHOR_SELF           = 2, /* own position (degenerate)     */
    TAPESTRY_BSE_ANCHOR_LOWEST_ENERGY  = 3, /* min energy_level among fresh  */
} tapestry_bse_anchor_selector_t;

/* Anchor selector re-resolution debounce, §5.2: "a selector result must be
 * stable for a hold time before a switch takes effect, or a gossip flicker
 * would make the whole collective's anchor thrash" — the same lesson (and
 * the same value) as QUORUM_UP_MS elsewhere in this codebase. */
#ifndef TAPESTRY_BSE_ANCHOR_HOLD_MS
#define TAPESTRY_BSE_ANCHOR_HOLD_MS  2000u
#endif

/* ── Motion (FORM only) ───────────────────────────────────────────────────── */
/*
 * §6: a motion modifier evolves the frame's reference over time under
 * authored control — "form a circle" (static) vs. "keep rotating in a
 * circle" (spin) is the same relation with a different temporal character.
 * SPIN rotates each vertex's offset from the frame origin at
 * spin_rate_radps; achievement is evaluated against the ROTATING vertex —
 * the existing eps/hold-time predicate generalizes unchanged (it was
 * already tick-scoped).
 *
 * CONVERGE deliberately ignores this field: its target IS the frame
 * origin (zero offset from it), so "rotating the offset" is a no-op —
 * script_toml.py rejects motion = "spin" on converge outright rather than
 * accept a goal that visibly does nothing.
 *
 * A non-terminal motion (SPIN never "completes" — it's a maintained
 * behavior) requires a real max_duration_ms bound; see
 * choreo_submit_script()'s validation.
 */
typedef enum {
    TAPESTRY_BSE_MOTION_STATIC = 0,   /* reference fixed at activation (today) */
    TAPESTRY_BSE_MOTION_SPIN   = 1,   /* reference rotates about the frame origin */
} tapestry_bse_motion_t;

/* Achievement defaults (meter scale — see the units note above). */
#define TAPESTRY_BSE_ACHIEVE_EPS_DEFAULT      0.5f
#define TAPESTRY_BSE_ACHIEVE_HOLD_MS_DEFAULT  3000u

/* EXCHANGE arc angular rate, rad/s.  0.15 rad/s turns a two-element swap
 * (π radians) in ~21 s.  Override at build time if needed. */
#ifndef TAPESTRY_BSE_EXCHANGE_OMEGA_RADPS
#define TAPESTRY_BSE_EXCHANGE_OMEGA_RADPS     0.15f
#endif

/* EXCHANGE occupied-destination handling (step-skew defense), direct_path
 * ONLY: element scripts advance on per-element clocks, so one element can
 * reach its destination station while its (slower) previous owner still
 * occupies it — 2026-07-19 flight 11: a beeline into an occupied station
 * collapsed separation to 0.09 m, the platform repulsion shoved targets
 * sideways, and achievement fired mid-scrum.  While any fresh peer sits
 * within OCCUPIED_M of the destination, the commanded target holds a
 * STANDOFF_M point on the approach line and achievement is deferred; when
 * the owner vacates (its own exchange moves it), the approach completes.
 * The step timeout still bounds the wait if the owner never leaves.  For
 * a 1 m swap, OCCUPIED_M must stay below half the station spacing or two
 * synchronized elements crossing at the midpoint would stall each other.
 * Never applied to the arc path: the arc preserves separation by
 * construction (see TAPESTRY_BSE_EXCHANGE_OMEGA_RADPS above) and never
 * beelines into a station, so the standoff has nothing to defend against
 * there — applying it anyway broke that guarantee on a symmetric swap
 * (bse.c, ztest choreo_script_test_swap_script_end_to_end). */
#ifndef TAPESTRY_BSE_EXCHANGE_OCCUPIED_M
#define TAPESTRY_BSE_EXCHANGE_OCCUPIED_M      0.35f
#endif
#ifndef TAPESTRY_BSE_EXCHANGE_STANDOFF_M
#define TAPESTRY_BSE_EXCHANGE_STANDOFF_M      0.5f
#endif

typedef struct {
    tapestry_bse_intent_type_t type;
    position_t        target;   /* MOVE / CONVERGE destination    */
    float                      radius;   /* FORM: circumradius (CIRCLE) or
                                           * half-span/cell-spacing (LINE/
                                           * GRID); DISPERSE min dist       */
    tapestry_bse_shape_t       shape;    /* FORM shape                     */

    /* FORM / CONVERGE only (see tapestry_bse_frame_t above).  frame == 0
     * (ABSOLUTE) ignores anchor/anchor_id entirely and uses `target` exactly
     * as today. */
    tapestry_bse_frame_t           frame;
    tapestry_bse_anchor_selector_t anchor;     /* frame == ELEMENT only */
    element_id_t                   anchor_id;  /* anchor == ID only     */

    /* FORM only (see tapestry_bse_motion_t above).  motion == 0 (STATIC)
     * is the default — every existing intent is unaffected. */
    tapestry_bse_motion_t          motion;
    float                          spin_rate_radps;  /* motion == SPIN only */

    /* EXCHANGE: ring rotation amount.  0 is treated as 1 (the common case)
     * so a zero-initialized intent still swaps. */
    uint8_t                    slot_shift;

    /* EXCHANGE never touches z — this element's own altitude stays fixed
     * for the whole maneuver, whichever path is used (see
     * exchange_capture()'s comment in bse.c for why: only x/y stations are
     * exchanged). direct_path beelines straight (in x/y) to the
     * destination station instead of the centroid arc — far faster (~1 m
     * at the tracker's speed limit vs ~21 s of arc) but ONLY safe when
     * elements are already separated some other way (e.g. distinct
     * altitudes, however established) — nothing here establishes that
     * separation for you. The arc stays the default because it preserves
     * mutual separation by construction in x/y — its angular sweep keeps
     * every element rotating the same direction about the centroid — so
     * it protects elements with no altitude separation at all. */
    bool                       direct_path;

    /* Achievement predicate parameters (0 → the defaults above).
     * A goal is "achieved" when own position stays within achieve_eps of
     * the goal point for achieve_hold_ms.  See bse_goal_achieved(). */
    float                      achieve_eps;
    uint32_t                   achieve_hold_ms;

    /* Identity of the L7 goal this intent came from, copied verbatim from
     * choreo_goal_t::id.  Opaque here: this implementation never reads it.
     * It is carried so a BSE that queues intents can report which goal a
     * directive belongs to, and which goal preempted which, without the
     * intent type having to change later.  0 = anonymous. */
    uint16_t                   id;
} tapestry_bse_intent_t;

/* ── Directive: L6 → main loop ────────────────────────────────────────────── */
/*
 * A directive is the per-element behavioral output of the BSE.  The main
 * loop (or a hardware abstraction layer above L5) consumes it each cycle.
 *
 * Produces:
 *   IDLE            — no goal.  The substrate-neutral QUIESCENCE signal:
 *                     each platform maps it to its own inactive posture
 *                     (aerial: land and disarm; ground: stop motors).
 *                     Takeoff is equally unnamed in the other direction —
 *                     an aerial element holding a MOVE_TO_POINT directive
 *                     while parked must activate to comply.
 *   HOLD            — freeze in place (quorum lost, or EXCHANGE awaiting
 *                     its snapshot)
 *   MOVE_TO_POINT   — geometry-only target point (no path planning beyond
 *                     the EXCHANGE arc)
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
    position_t           target;    /* MOVE_TO_POINT destination  */
    float                         spring_k;  /* MAINTAIN_SPRING stiffness  */
    float                         spacing;   /* MAINTAIN_SPRING target dist */
} tapestry_bse_directive_t;

/* ── API ──────────────────────────────────────────────────────────────────── */

/*
 * bse_init — Initialize BSE state for this element.
 * Must be called once before bse_submit_intent / bse_tick.
 */
void bse_init(element_id_t self_id);

/*
 * bse_submit_intent — Accept a new intent from L7.
 *
 * The submitted intent becomes the active one immediately: the next
 * bse_tick() decomposes it, and bse_goal_achieved() reports against it.
 *
 * What happens to the DISPLACED intent is implementation-defined and must
 * not be relied on.  This function always discards it — the achievement
 * predicate and every activation capture (HOLD station, EXCHANGE snapshot
 * and arc progress, MOVE centroid offset) are reset, so re-submitting a
 * previously active intent starts it over.  It also discards anything
 * bse_preempt_intent() had parked (an ordinary submit always fully
 * replaces everything).  Use bse_preempt_intent() below for the
 * alternative: preserve the displaced intent's activation state and
 * resume it later.  See bse_activation_t in bse.c for exactly which state
 * that covers.
 *
 * Returns 0 on success, -1 if intent is NULL.
 */
int bse_submit_intent(const tapestry_bse_intent_t *intent);

/*
 * bse_preempt_intent — Like bse_submit_intent(), but saves the displaced
 * intent and its full activation state instead of discarding it, and
 * bse_resume_intent() restores it verbatim (HOLD station, EXCHANGE
 * snapshot/arc progress, MOVE offset, achievement accumulator all survive
 * the round trip). Bounded stack, depth 1 in this implementation — a
 * second preempt while one is already parked returns -1; nest deeper by
 * raising BSE_MAX_PREEMPT_DEPTH in bse.c.
 *
 * The preempting intent becomes active immediately, exactly like
 * bse_submit_intent(). L7 (choreo.c) owns the POLICY of when to call this
 * instead of bse_submit_intent() — priority comparison, if any, happens
 * there; this is pure mechanism.
 *
 * Returns 0 on success, -1 if intent is NULL or the stack is full.
 */
int bse_preempt_intent(const tapestry_bse_intent_t *intent);

/*
 * bse_resume_intent — Pop the most recently preempted intent back to
 * active, restoring its saved activation state exactly as it was at the
 * moment it was displaced. The intent that was active before this call is
 * discarded (same semantics as bse_submit_intent() displacing it) — call
 * bse_preempt_intent() again first if it also needs to survive.
 *
 * Returns 0 on success, -1 if nothing is parked.
 */
int bse_resume_intent(void);

/*
 * bse_has_parked_intent — True if bse_preempt_intent() has saved an intent
 * that bse_resume_intent() would restore.
 */
bool bse_has_parked_intent(void);

/*
 * bse_tick — Recompute per-element directive from current world state.
 *
 * Call once per main-loop cycle (WM_CYCLE_MS period — the achievement
 * hold-time and EXCHANGE arc integrate real time on that assumption),
 * after wm_tick() and scr_tick() have run.
 *
 * This is the L6 task-decomposition step: it maps a declarative intent
 * (submitted via bse_submit_intent) onto a concrete per-element directive,
 * using the world model and L5 outputs (task_slot, quorum, role) as input.
 * L5 provides an ordinal index (task_slot) but performs no decomposition
 * itself; all goal-to-directive mapping happens here in L6.
 *
 * Own position is read from the world model's self entry — the caller must
 * keep it current via wm_update_self().
 */
void bse_tick(const world_model_t *wm, const scr_state_t *scr);

/*
 * bse_get_directive — Return the directive computed by the last bse_tick.
 * Never returns NULL.  Defaults to TAPESTRY_BSE_DIRECTIVE_IDLE on startup.
 */
const tapestry_bse_directive_t *bse_get_directive(void);

/*
 * bse_goal_achieved — Minimal L6 feedback controller output.
 *
 * True when own position has stayed within the intent's achieve_eps of the
 * goal point for achieve_hold_ms (accumulated across consecutive ticks;
 * leaving the epsilon ball resets the accumulator).  The goal point is:
 *   FORM              — this element's assigned vertex
 *   MOVE              — this element's translated point (intent.target +
 *                       its own offset from the participant centroid)
 *   CONVERGE          — the intent target
 *   EXCHANGE          — the destination station (the snapshot point, not
 *                       the moving arc target)
 *   HOLD              — trivially achieved (staying is the goal; a HOLD
 *                       step's duration is governed by its timeout)
 *   IDLE / DISPERSE   — never achieved (no point-goal; timeout only)
 *
 * Purely local: each element evaluates its OWN goal only.  Collective
 * achievement barriers are future L5/L6 work.
 */
bool bse_goal_achieved(void);

/*
 * bse_anchor_lost — True if the last bse_tick() had a FORM/CONVERGE intent
 * with frame == ELEMENT (§5) and could not resolve any anchor position —
 * never locked one yet, or the previously-locked anchor's peer went
 * stale/inactive.  False for every other frame or intent type.  Tick-
 * scoped, like bse_goal_achieved() — reflects only the most recent
 * bse_tick().  This is CHOREO_EVENT_ANCHOR_LOST's source (choreo.h §8.2).
 */
bool bse_anchor_lost(void);

/*
 * bse_set_track_scope — Choreo SDK Design doc §7 tracks: tell bse.c
 * which track (by index) THIS element is currently active in, so
 * collect_participants() (FORM/EXCHANGE/MOVE) filters peers to only
 * those gossiping the SAME current_track — see that function's comment
 * in bse.c.  Defaults to 0 (bse_init()), matching every peer's gossiped
 * current_track on a script with no tracks — a no-op for every caller
 * that never calls this.  L7-owned concept; bse.c only stores and
 * filters on it, never interprets it.
 */
void bse_set_track_scope(uint8_t track);

#endif /* TAPESTRY_BSE_H */
