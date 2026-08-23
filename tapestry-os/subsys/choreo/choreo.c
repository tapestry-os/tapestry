/*
 * choreo.c — Tapestry Choreographer SDK (L7)
 *
 * Implements choreo.h by delegating to bse.c.  See choreo.h for the v1.0
 * feature scope (what's open-core here vs. deliberately licensed-tier).
 *
 * Lifecycle state machine:
 *   IDLE        → choreo_configure()  → CONFIGURED
 *   CONFIGURED  → choreo_deploy()     → RUNNING
 *   RUNNING     → quorum LOST (tick)  → SUSPENDED   (BSE + timers frozen)
 *   SUSPENDED   → quorum !LOST (tick) → RUNNING
 *   any         → choreo_terminate()  → (TERMINATED →) IDLE
 *
 * Script engine (linear Choreo container): choreo_submit_script() loads an
 * ordered choreo_step_t array; choreo_tick() advances it on achievement or
 * timeout; completing the last step terminates → IDLE directive, the
 * substrate-neutral quiescence signal.
 */

#include <stddef.h>
#include <errno.h>
#include <tapestry/choreo.h>
#include <tapestry/bse.h>
#include <tapestry/scr.h>

/* ── State ────────────────────────────────────────────────────────────────── */

static element_id_t       s_self_id;
static choreo_goal_t      s_goal;
static choreo_state_t     s_state;
static const scr_state_t *s_scr;   /* registered via choreo_register_scr(); NULL skips cap check */

/* Script engine */
static const choreo_step_t *s_steps;
static uint8_t              s_n_steps;
static uint8_t              s_step_idx;
static uint32_t             s_step_ms;
static bool                 s_script_active;
static bool                 s_script_done;

/* ── Membership debounce (CHOREO_EVENT_ELEMENT_JOINED/LOST/COUNT_*) ──────── */
/*
 * One debounce timer per Choreo instance, continuous across the whole
 * script's lifetime (not reset per-step) — a count change is a fact about
 * the collective, independent of which step is currently active.  Same
 * "stable before acting on it" lesson as TAPESTRY_BSE_ANCHOR_HOLD_MS
 * (bse.h): a single lucky/unlucky gossip frame must not fire
 * element_joined/element_lost fleet-wide.
 */
#ifndef CHOREO_MEMBERSHIP_HOLD_MS
#define CHOREO_MEMBERSHIP_HOLD_MS 2000u
#endif
static bool    s_count_locked_valid;
static uint8_t s_count_locked;
static uint8_t s_count_candidate;
static uint32_t s_count_candidate_ms;

/* ── Tracks (choreo.h §7) ──────────────────────────────────────────────────── */
/*
 * s_steps/s_n_steps/s_step_idx/s_step_ms/s_goal (already declared above)
 * are reused AS-IS to mean "the ACTIVE track's step state" — an element
 * runs one track at a time (choreo.h's comment), so there is nothing to
 * add there.  What tracks add is: the track table itself, which track is
 * active, and each INACTIVE track's own remembered step index (so
 * re-entering a track resumes where it left off, per that same comment).
 */
static choreo_track_t s_tracks[CHOREO_MAX_TRACKS];
static uint8_t         s_n_tracks;           /* 0 = not in multi-track mode */
static uint8_t         s_active_track_idx;
static uint8_t         s_track_step_idx[CHOREO_MAX_TRACKS];

/* Debounced track-selection state — same shape as the membership debounce
 * above, but NOT the same timer: track membership is (mostly) evaluated
 * from THIS element's own state (capabilities, health_flags), not gossip-
 * derived peer data, so the very first determination is made synchronously
 * inside choreo_submit_tracks() itself (undebounced — there is no prior
 * track to flicker away from on a fresh submission).  Every subsequent
 * re-evaluation, from update_track_selection() each RUNNING tick, IS
 * debounced through this candidate/hold-timer pair (a sensor reading near
 * a threshold, e.g. energy_low, can jitter tick-to-tick same as anything
 * else). */
static uint8_t s_track_candidate_idx;
static bool    s_track_candidate_valid;
static uint32_t s_track_candidate_ms;

/* ── Preemption stack ─────────────────────────────────────────────────────── */
/*
 * choreo_preempt_goal() saves the goal + script engine state a normal
 * choreo_terminate() would otherwise discard; choreo_terminate() itself
 * (and therefore choreo_cancel_goal() and natural script completion, both
 * of which call it) restores the most recently parked entry instead of
 * going to IDLE, whenever one exists. Bounded stack, depth 1 today,
 * mirroring bse.c's BSE_MAX_PREEMPT_DEPTH — see that file's comment for
 * what raising this constant later does and doesn't require changing.
 */
#define CHOREO_MAX_PREEMPT_DEPTH 1

typedef struct {
    choreo_goal_t        goal;
    const choreo_step_t *steps;
    uint8_t               n_steps;
    uint8_t               step_idx;
    uint32_t              step_ms;
    bool                  script_active;
} choreo_parked_goal_t;

static choreo_parked_goal_t s_parked[CHOREO_MAX_PREEMPT_DEPTH];
static int                  s_parked_depth;

/* ── Internal helpers ─────────────────────────────────────────────────────── */

/*
 * Check whether the registered element hardware satisfies required_caps.
 *
 * Mapping (paper §3.9 / CHOREO_CAP_* documentation in choreo.h):
 *   CHOREO_CAP_LOCOMOTION    → SCR_CAP_ACTUATOR
 *   CHOREO_CAP_SENSING       → SCR_CAP_SENSOR
 *   CHOREO_CAP_SIGNALING     → SCR_CAP_RELAY
 *   CHOREO_CAP_BONDING       → SCR_CAP_BONDING
 *   CHOREO_CAP_ABS_POSITION  → SCR_CAP_ABS_POSITION
 */
static int caps_satisfied(choreo_capabilities_t required)
{
    if (s_scr == NULL || required == CHOREO_CAP_NONE) {
        return 1;
    }
    scr_capability_t hw = s_scr->capabilities;
    if ((required & CHOREO_CAP_LOCOMOTION)   && !(hw & SCR_CAP_ACTUATOR))      return 0;
    if ((required & CHOREO_CAP_SENSING)      && !(hw & SCR_CAP_SENSOR))       return 0;
    if ((required & CHOREO_CAP_SIGNALING)    && !(hw & SCR_CAP_RELAY))        return 0;
    if ((required & CHOREO_CAP_BONDING)      && !(hw & SCR_CAP_BONDING))      return 0;
    if ((required & CHOREO_CAP_ABS_POSITION) && !(hw & SCR_CAP_ABS_POSITION)) return 0;
    return 1;
}

/*
 * Choreo SDK Design doc §11: capability requirements are a derived floor,
 * not solely the author's explicit required_caps — an axis value that
 * demands a capability requires it whether or not the author remembered to
 * declare it ("a derived floor the author can add to but not subtract
 * from"):
 *
 *   motion == SPIN -> CHOREO_CAP_LOCOMOTION.  Maps to SCR_CAP_ACTUATOR;
 *   every real SPIN user already has it in practice.
 *
 *   (type == FORM || type == CONVERGE) && frame == ABSOLUTE ->
 *   CHOREO_CAP_ABS_POSITION.  Only these two goal types read frame at all
 *   (bse.h §5) — HOLD/EXCHANGE are inherently coordinate-free and
 *   MOVE/DISPERSE don't read it either.  Maps to SCR_CAP_ABS_POSITION,
 *   which real lighthouse-based apps (examples/cf21bl-formation,
 *   examples/webots-formation) now declare at scr_init() precisely because
 *   ABSOLUTE is the default frame (§6 of the L6/L7 audit list) — every
 *   FORM/CONVERGE goal in those apps needs it unless it opts into
 *   COLLECTIVE/ELEMENT explicitly.  tapestry-scr-hw and tapestry-scr-sim
 *   never call choreo_register_scr(), so this floor is a no-op for them
 *   regardless (caps_satisfied() short-circuits on s_scr == NULL).
 */
static choreo_capabilities_t derived_caps(const choreo_goal_t *goal)
{
    choreo_capabilities_t caps = goal->required_caps;
    if (goal->motion == TAPESTRY_BSE_MOTION_SPIN) {
        caps |= CHOREO_CAP_LOCOMOTION;
    }
    if ((goal->type == CHOREO_GOAL_FORM || goal->type == CHOREO_GOAL_CONVERGE) &&
        goal->frame == TAPESTRY_BSE_FRAME_ABSOLUTE) {
        caps |= CHOREO_CAP_ABS_POSITION;
    }
    return caps;
}

static tapestry_bse_intent_t goal_to_intent(const choreo_goal_t *goal)
{
    static const tapestry_bse_intent_type_t type_map[] = {
        [CHOREO_GOAL_NONE]     = TAPESTRY_BSE_INTENT_IDLE,
        [CHOREO_GOAL_FORM]     = TAPESTRY_BSE_INTENT_FORM,
        [CHOREO_GOAL_MOVE]     = TAPESTRY_BSE_INTENT_MOVE,
        [CHOREO_GOAL_DISPERSE] = TAPESTRY_BSE_INTENT_DISPERSE,
        [CHOREO_GOAL_CONVERGE] = TAPESTRY_BSE_INTENT_CONVERGE,
        [CHOREO_GOAL_HOLD]     = TAPESTRY_BSE_INTENT_HOLD,
        [CHOREO_GOAL_EXCHANGE] = TAPESTRY_BSE_INTENT_EXCHANGE,
    };

    tapestry_bse_intent_t intent = {0};
    intent.type   = (goal->type < (choreo_goal_type_t)(sizeof(type_map) / sizeof(type_map[0])))
                    ? type_map[goal->type]
                    : TAPESTRY_BSE_INTENT_IDLE;
    intent.target          = goal->target;
    intent.radius          = goal->radius;
    intent.shape           = goal->shape;
    intent.frame           = goal->frame;
    intent.anchor          = goal->anchor;
    intent.anchor_id       = goal->anchor_id;
    intent.motion          = goal->motion;
    intent.spin_rate_radps = goal->spin_rate_radps;
    intent.slot_shift      = goal->slot_shift;
    intent.direct_path     = goal->direct_path;
    intent.achieve_eps     = goal->achieve_eps;
    intent.achieve_hold_ms = goal->achieve_hold_ms;
    intent.id              = goal->id;   /* opaque; see choreo_goal_t::id */
    return intent;
}

static void script_clear(void)
{
    s_steps         = NULL;
    s_n_steps       = 0;
    s_step_idx      = 0;
    s_step_ms       = 0;
    s_script_active = false;
}

/* ── API ──────────────────────────────────────────────────────────────────── */

void choreo_init(element_id_t self_id)
{
    s_self_id      = self_id;
    s_state        = CHOREO_STATE_IDLE;
    s_scr          = NULL;
    s_parked_depth = 0;
    s_count_locked_valid = false;
    s_n_tracks = 0;
    script_clear();
    s_script_done = false;
    bse_init(self_id);
}

void choreo_register_scr(const scr_state_t *scr)
{
    s_scr = scr;
}

int choreo_configure(const choreo_goal_t *goal)
{
    if (goal == NULL || goal->type == CHOREO_GOAL_NONE) {
        return -1;
    }
    if (s_state != CHOREO_STATE_IDLE) {
        return -1;
    }
    if (!caps_satisfied(derived_caps(goal))) {
        return -EPERM;
    }
    s_goal  = *goal;
    s_state = CHOREO_STATE_CONFIGURED;
    return 0;
}

int choreo_deploy(void)
{
    if (s_state != CHOREO_STATE_CONFIGURED) {
        return -1;
    }
    tapestry_bse_intent_t intent = goal_to_intent(&s_goal);
    int rc = bse_submit_intent(&intent);
    if (rc != 0) {
        return rc;
    }
    s_state = CHOREO_STATE_RUNNING;
    return 0;
}

/* Unconditional full reset — drops any parked goal too.  Used for the
 * pre-submit reset in choreo_submit_goal()/choreo_submit_script(): an
 * ordinary new submission always fully replaces everything, whether or
 * not a preemption happens to be active, exactly as before this feature
 * existed.  choreo_terminate() (below) is the pop-aware public path. */
static void terminate_hard(void)
{
    s_state = CHOREO_STATE_TERMINATED;
    script_clear();
    s_parked_depth = 0;   /* bse_submit_intent() below drops BSE's side too */
    s_count_locked_valid = false;   /* a fresh submission starts fresh */
    s_n_tracks = 0;                 /* drop multi-track mode entirely */
    bse_set_track_scope(0);         /* bse.c's filter must not stay stuck nonzero */
    tapestry_bse_intent_t idle = { .type = TAPESTRY_BSE_INTENT_IDLE };
    bse_submit_intent(&idle);
    s_state = CHOREO_STATE_IDLE;
}

void choreo_terminate(void)
{
    if (s_parked_depth > 0) {
        s_parked_depth--;
        const choreo_parked_goal_t *p = &s_parked[s_parked_depth];
        s_goal          = p->goal;
        s_steps         = p->steps;
        s_n_steps       = p->n_steps;
        s_step_idx      = p->step_idx;
        s_step_ms       = p->step_ms;
        s_script_active = p->script_active;
        bse_resume_intent();
        s_state = CHOREO_STATE_RUNNING;
        return;
    }
    terminate_hard();
}

int choreo_preempt_goal(const choreo_goal_t *goal)
{
    if (goal == NULL || goal->type == CHOREO_GOAL_NONE) {
        return -1;
    }
    if (s_state != CHOREO_STATE_RUNNING && s_state != CHOREO_STATE_SUSPENDED) {
        return -1;   /* nothing active to preempt */
    }
    if (s_parked_depth >= CHOREO_MAX_PREEMPT_DEPTH) {
        return -EBUSY;
    }
    if (!caps_satisfied(derived_caps(goal))) {
        return -EPERM;
    }

    tapestry_bse_intent_t intent = goal_to_intent(goal);
    choreo_parked_goal_t *p = &s_parked[s_parked_depth];
    p->goal          = s_goal;
    p->steps         = s_steps;
    p->n_steps       = s_n_steps;
    p->step_idx      = s_step_idx;
    p->step_ms       = s_step_ms;
    p->script_active = s_script_active;

    int rc = bse_preempt_intent(&intent);
    if (rc != 0) {
        return rc;   /* BSE-side stack full — nothing was pushed here yet */
    }
    s_parked_depth++;

    s_goal          = *goal;
    s_script_active = false;   /* a preempting goal is a single goal, not a script */
    s_state         = CHOREO_STATE_RUNNING;
    return 0;
}

bool choreo_is_preempted(void)
{
    return s_parked_depth > 0;
}

uint16_t choreo_parked_goal_id(void)
{
    return s_parked_depth > 0 ? s_parked[s_parked_depth - 1].goal.id : 0;
}

int choreo_submit_goal(const choreo_goal_t *goal)
{
    if (goal == NULL) {
        return -1;
    }
    if (s_state != CHOREO_STATE_IDLE) {
        terminate_hard();
    }
    s_script_done = false;
    int rc = choreo_configure(goal);
    if (rc != 0) {
        return rc;
    }
    return choreo_deploy();
}

/* Validate every step up front — a script (or, per-track, a track) that
 * would stall or fail a capability check mid-show is rejected before
 * anything moves.  Shared by choreo_submit_script() and
 * choreo_submit_tracks() so the two validation paths can't drift. */
static int validate_steps(const choreo_step_t *steps, uint8_t n_steps)
{
    if (steps == NULL || n_steps == 0) {
        return -1;
    }
    for (uint8_t i = 0; i < n_steps; i++) {
        const choreo_step_t *st = &steps[i];
        if (st->goal.type == CHOREO_GOAL_NONE) {
            return -1;
        }
        if (!st->advance_on_achieved && st->max_duration_ms == 0u) {
            return -1;   /* no exit condition — stalls by construction */
        }
        if (st->goal.motion == TAPESTRY_BSE_MOTION_SPIN &&
            st->max_duration_ms == 0u) {
            /* A non-terminal motion never "completes" (bse.h §6) —
             * until=achieved alone is not a sufficient exit for a
             * maintained goal, only a valid early-advance on top of a
             * real duration bound. */
            return -1;
        }
        if (!caps_satisfied(derived_caps(&st->goal))) {
            return -EPERM;
        }
        for (uint8_t j = 0; j < st->n_transitions; j++) {
            /* goto_step_idx == n_steps is "end" (choreo.h) — valid.
             * Anything past that would index off the array in
             * script_advance(). */
            if (st->on[j].goto_step_idx > n_steps) {
                return -1;
            }
        }
    }
    return 0;
}

int choreo_submit_script(const choreo_step_t *steps, uint8_t n_steps)
{
    int rc0 = validate_steps(steps, n_steps);
    if (rc0 != 0) {
        return rc0;
    }

    if (s_state != CHOREO_STATE_IDLE) {
        terminate_hard();
    }
    s_script_done = false;

    int rc = choreo_configure(&steps[0].goal);
    if (rc != 0) {
        return rc;
    }
    rc = choreo_deploy();
    if (rc != 0) {
        return rc;
    }

    /* Set after configure/deploy — choreo_terminate() above cleared it. */
    s_steps         = steps;
    s_n_steps       = n_steps;
    s_step_idx      = 0;
    s_step_ms       = 0;
    s_script_active = true;
    return 0;
}

int choreo_script_step(void)
{
    return s_script_active ? (int)s_step_idx : -1;
}

bool choreo_script_complete(void)
{
    return s_script_done;
}

bool choreo_goal_achieved(void)
{
    return bse_goal_achieved();
}

bool choreo_collective_achieved(const world_model_t *wm)
{
    if (!bse_goal_achieved()) {
        return false;
    }
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_self || !e->is_active || e->is_stale) {
            continue;
        }
        if (!e->state.goal_achieved) {
            return false;
        }
    }
    return true;   /* vacuously true when solo — no fresh peer to disagree */
}

void choreo_cancel_goal(void)
{
    choreo_terminate();
}

choreo_state_t choreo_goal_status(void)
{
    return s_state;
}

choreo_goal_type_t choreo_current_goal_type(void)
{
    if (s_state == CHOREO_STATE_RUNNING || s_state == CHOREO_STATE_SUSPENDED) {
        return s_goal.type;
    }
    return CHOREO_GOAL_NONE;
}

/*
 * Update the membership debounce timer from this tick's swarm size and
 * report one-tick JOINED/LOST pulses (see the state comment above).
 * ELEMENT_JOINED/ELEMENT_LOST fire only on the tick a change is
 * CONFIRMED (stable for CHOREO_MEMBERSHIP_HOLD_MS); COUNT_GTE/COUNT_EQ
 * (checked by the caller) read the raw live count instead — a threshold
 * comparison doesn't have the same "which direction changed" ambiguity a
 * flickering count does, so it isn't debounced the same way.
 */
static void update_membership_debounce(const scr_state_t *scr,
                                       bool *joined, bool *lost)
{
    *joined = false;
    *lost   = false;
    uint8_t raw = scr_get_swarm_size(scr);

    if (!s_count_locked_valid) {
        s_count_locked_valid = true;
        s_count_locked       = raw;
        s_count_candidate_ms = 0;
        return;
    }
    if (raw == s_count_locked) {
        s_count_candidate_ms = 0;   /* stable — no pending change */
        return;
    }
    if (raw == s_count_candidate) {
        s_count_candidate_ms += WM_CYCLE_MS;
    } else {
        s_count_candidate    = raw;
        s_count_candidate_ms = 0;
    }
    if (s_count_candidate_ms >= CHOREO_MEMBERSHIP_HOLD_MS) {
        *joined         = raw > s_count_locked;
        *lost           = raw < s_count_locked;
        s_count_locked  = raw;
    }
}

/* Does `event` fire this tick?  threshold/scope/wm/scr/joined/lost carry
 * everything the CHOREO_EVENT_* variants need (choreo.h). */
static bool event_fires(choreo_event_t event, uint8_t threshold,
                        const choreo_step_t *st, const world_model_t *wm,
                        const scr_state_t *scr, bool joined, bool lost)
{
    switch (event) {
    case CHOREO_EVENT_ACHIEVED:
        return (st->scope == CHOREO_SCOPE_ALL)
               ? choreo_collective_achieved(wm)
               : bse_goal_achieved();
    case CHOREO_EVENT_ELEMENT_JOINED:
        return joined;
    case CHOREO_EVENT_ELEMENT_LOST:
        return lost;
    case CHOREO_EVENT_COUNT_GTE:
        return scr_get_swarm_size(scr) >= threshold;
    case CHOREO_EVENT_COUNT_EQ:
        return scr_get_swarm_size(scr) == threshold;
    case CHOREO_EVENT_ANCHOR_LOST:
        return bse_anchor_lost();
    case CHOREO_EVENT_QUORUM_LOST:
        return scr->quorum_state == SCR_QUORUM_LOST;
    default:
        return false;
    }
}

/* Shared tail of an advance: activate step target_idx, or complete the
 * script if target_idx has run off the end.  Factored out of
 * script_advance() so suspended_hold_timeout() below (an isolated HOLD
 * giving up on its own timeout) can reach the exact same completion/
 * next-step logic without duplicating it. */
static void advance_to(int target_idx)
{
    s_step_ms = 0;

    if (target_idx >= (int)s_n_steps) {
        /* Script complete → quiescence: terminate submits the IDLE intent,
         * so the directive goes IDLE and the platform maps it to its own
         * inactive posture.  s_script_done survives the terminate. */
        s_script_done = true;
        choreo_terminate();
        return;
    }

    s_step_idx = (uint8_t)target_idx;
    s_goal = s_steps[s_step_idx].goal;
    tapestry_bse_intent_t intent = goal_to_intent(&s_goal);
    bse_submit_intent(&intent);
}

/* Advance the script if the current step's exit condition is met —
 * either an explicit transition (checked first, first match wins) or,
 * absent any match, the legacy advance_on_achieved/max_duration_ms rule
 * (implicit next-index advance) every step had before transitions
 * existed. */
static void script_advance(const world_model_t *wm, const scr_state_t *scr)
{
    if (!s_script_active) {
        return;
    }

    const choreo_step_t *st = &s_steps[s_step_idx];
    s_step_ms += WM_CYCLE_MS;

    bool joined, lost;
    update_membership_debounce(scr, &joined, &lost);

    int target_idx = -1;
    for (uint8_t i = 0; i < st->n_transitions; i++) {
        const choreo_transition_t *t = &st->on[i];
        if (event_fires(t->event, t->threshold, st, wm, scr, joined, lost)) {
            target_idx = t->goto_step_idx;
            break;
        }
    }

    if (target_idx < 0) {
        bool advance = false;
        if (st->advance_on_achieved) {
            bool achieved = (st->scope == CHOREO_SCOPE_ALL)
                            ? choreo_collective_achieved(wm)
                            : bse_goal_achieved();
            if (achieved) {
                advance = true;
            }
        }
        if (st->max_duration_ms > 0u && s_step_ms >= st->max_duration_ms) {
            advance = true;
        }
        if (!advance) {
            return;
        }
        target_idx = (int)s_step_idx + 1;
    }

    advance_to(target_idx);
}

/*
 * Isolated (SUSPENDED) timeout carve-out — HOLD only.  A HOLD step
 * already ticks the BSE while suspended (choreo_tick()'s SUSPENDED case)
 * because it's self-referential and needs no peers; this extends that
 * same reasoning to its own max_duration_ms, so a script can give up on
 * permanent isolation (all peers gone, out-of-mesh-range, a partition
 * that never heals) instead of station-keeping forever with nothing left
 * to revive it — see choreo_state_t's SUSPENDED doc.
 *
 * Deliberately narrow: unlike script_advance(), this does NOT evaluate
 * on[] transitions (ELEMENT_JOINED/LOST, COUNT_*, ANCHOR_LOST all need
 * live peer/anchor data that doesn't exist while isolated) or
 * advance_on_achieved (bse_goal_achieved() for HOLD is unconditionally
 * true — see choreo_goal_achieved()'s test coverage — so combining it
 * with SUSPENDED would fire on the very first isolated tick, which is
 * never useful; the TOML authoring surface already forbids
 * until/eps/settle on hold for exactly this reason).  Only the time
 * bound itself is unfrozen.
 */
static void suspended_hold_timeout(void)
{
    if (!s_script_active || s_goal.type != CHOREO_GOAL_HOLD) {
        return;
    }
    const choreo_step_t *st = &s_steps[s_step_idx];
    s_step_ms += WM_CYCLE_MS;
    if (st->max_duration_ms > 0u && s_step_ms >= st->max_duration_ms) {
        advance_to((int)s_step_idx + 1);
    }
}

/* Does `f` match THIS element's own state?  Never inspects a peer — track
 * membership is always self-evaluated (choreo.h's track-filter comment). */
static bool track_matches(const choreo_track_filter_t *f, const world_model_t *wm)
{
    if (!caps_satisfied(f->required_caps)) {
        return false;
    }
    if (f->requires_energy_low) {
        bool low = false;
        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm->entries[i];
            if (e->is_self) {
                low = (e->state.health_flags & ELEMENT_HEALTH_LOW_BATTERY) != 0;
                break;
            }
        }
        if (!low) {
            return false;
        }
    }
    return true;
}

/* First declared track whose filter matches, or -1 if none do (no
 * catch-all "all" track declared — choreo_submit_tracks() rejects this at
 * submission; update_track_selection() below falls back to staying put). */
static int first_matching_track(const world_model_t *wm)
{
    for (uint8_t i = 0; i < s_n_tracks; i++) {
        if (track_matches(&s_tracks[i].filter, wm)) {
            return (int)i;
        }
    }
    return -1;
}

/* Switch the active track to `new_track`, saving the outgoing track's step
 * index and resuming the incoming one's — then activate its current step
 * FRESH (new snapshot/timers), per choreo.h's "not a preserved-state
 * resume like preemption" design. */
static void migrate_to_track(uint8_t new_track)
{
    s_track_step_idx[s_active_track_idx] = s_step_idx;

    s_active_track_idx = new_track;
    s_steps            = s_tracks[new_track].steps;
    s_n_steps          = s_tracks[new_track].n_steps;
    s_step_idx         = s_track_step_idx[new_track];
    s_step_ms          = 0;
    s_script_active     = true;


    s_goal = s_steps[s_step_idx].goal;
    tapestry_bse_intent_t intent = goal_to_intent(&s_goal);
    bse_submit_intent(&intent);
}

/* Re-evaluate track membership once per RUNNING tick (choreo_tick()) while
 * in multi-track mode.  Debounced exactly like update_membership_debounce()
 * — a threshold reading (energy_low) can jitter tick-to-tick same as a
 * swarm-size count can. */
static void update_track_selection(const world_model_t *wm)
{
    int found = first_matching_track(wm);
    if (found < 0) {
        /* No track currently claims this element (e.g. energy_low cleared
         * and no other filter matches) — stay on the active track rather
         * than migrating anywhere; there's nowhere well-defined to go. */
        s_track_candidate_valid = false;
        return;
    }
    uint8_t match = (uint8_t)found;

    if (match == s_active_track_idx) {
        s_track_candidate_valid = false;
        return;
    }
    if (s_track_candidate_valid && match == s_track_candidate_idx) {
        s_track_candidate_ms += WM_CYCLE_MS;
    } else {
        s_track_candidate_valid = true;
        s_track_candidate_idx   = match;
        s_track_candidate_ms    = 0;
    }
    if (s_track_candidate_ms >= CHOREO_MEMBERSHIP_HOLD_MS) {
        migrate_to_track(match);
        s_track_candidate_valid = false;
    }
}

int choreo_submit_tracks(const world_model_t *wm, const choreo_track_t *tracks, uint8_t n_tracks)
{
    if (wm == NULL || tracks == NULL || n_tracks == 0 || n_tracks > CHOREO_MAX_TRACKS) {
        return -1;
    }
    for (uint8_t i = 0; i < n_tracks; i++) {
        int rc0 = validate_steps(tracks[i].steps, tracks[i].n_steps);
        if (rc0 != 0) {
            return rc0;
        }
    }

    if (s_state != CHOREO_STATE_IDLE) {
        terminate_hard();
    }
    s_script_done = false;

    for (uint8_t i = 0; i < n_tracks; i++) {
        s_tracks[i]          = tracks[i];
        s_track_step_idx[i]  = 0;
    }
    s_n_tracks              = n_tracks;
    s_track_candidate_valid = false;

    int found = first_matching_track(wm);
    if (found < 0) {
        return -1;   /* no track's filter matches this element */
    }
    uint8_t initial = (uint8_t)found;
    s_active_track_idx = initial;
    s_steps             = s_tracks[initial].steps;
    s_n_steps           = s_tracks[initial].n_steps;

    int rc = choreo_configure(&s_steps[0].goal);
    if (rc != 0) {
        s_n_tracks = 0;
        return rc;
    }
    rc = choreo_deploy();
    if (rc != 0) {
        s_n_tracks = 0;
        return rc;
    }

    s_step_idx      = 0;
    s_step_ms       = 0;
    s_script_active = true;
    return 0;
}

uint8_t choreo_current_track(void)
{
    return s_n_tracks > 0 ? s_active_track_idx : 0;
}

void choreo_tick(const world_model_t *wm, const scr_state_t *scr)
{
    switch (s_state) {
    case CHOREO_STATE_RUNNING:
        if (s_n_tracks > 0 && s_parked_depth == 0) {
            update_track_selection(wm);
        }
        bse_set_track_scope(choreo_current_track());
        bse_tick(wm, scr);
        script_advance(wm, scr);
        /* script_advance may have terminated → IDLE; quorum check only
         * applies while still RUNNING. */
        if (s_state == CHOREO_STATE_RUNNING &&
            scr->quorum_state == SCR_QUORUM_LOST) {
            s_state = CHOREO_STATE_SUSPENDED;
        }
        break;

    case CHOREO_STATE_SUSPENDED:
        /* Script timers frozen — a partition pauses the show rather than
         * timing it out.  Per-goal quorum: a SELF-referential goal (HOLD —
         * it references only this element's own position) still ticks the
         * BSE, so its station is captured as soon as a position is
         * available and station-keeping stays live; deferring the capture
         * to quorum recovery would capture whatever position the element
         * has drifted to by then (2026-07-19 flight finding).  PEER-
         * referential goals (EXCHANGE) stay frozen: their snapshots are
         * meaningless without fresh peers.  HOLD's own max_duration_ms is
         * the one timer that isn't frozen either (suspended_hold_timeout()
         * — see choreo_state_t's doc): permanent isolation needs a way
         * out even for a step that never freezes at an unsafe position. */
        if (s_goal.type == CHOREO_GOAL_HOLD) {
            bse_tick(wm, scr);
            suspended_hold_timeout();
        }
        /* suspended_hold_timeout() may have terminated -> IDLE, or
         * advanced to a new step (still SUSPENDED — advance_to() doesn't
         * touch s_state); the quorum-recovery check below only applies if
         * still actually SUSPENDED, same guard RUNNING uses above. */
        if (s_state == CHOREO_STATE_SUSPENDED &&
            scr->quorum_state != SCR_QUORUM_LOST) {
            s_state = CHOREO_STATE_RUNNING;
        }
        break;

    case CHOREO_STATE_IDLE:
    case CHOREO_STATE_CONFIGURED:
    case CHOREO_STATE_TERMINATED:
    default:
        break;   /* BSE not driven; directive remains at its last value */
    }
}

const tapestry_bse_directive_t *choreo_get_directive(void)
{
    return bse_get_directive();
}

substrate_signal_t choreo_current_indicator(void)
{
    if (!s_script_active) {
        return SUBSTRATE_SIGNAL_NONE;
    }
    return s_steps[s_step_idx].indicator;
}

const char *choreo_current_telemetry_tag(void)
{
    if (!s_script_active) {
        return NULL;
    }
    return s_steps[s_step_idx].telemetry_tag;
}
