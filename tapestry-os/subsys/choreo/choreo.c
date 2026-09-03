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

/* ── Departure policy ─────────────────────────────────────────────────────── */
/*
 * Reason/identity-aware, layered ON TOP of (not instead of) the count-only
 * membership debounce above — that one stays exactly as it was, still
 * driving CHOREO_EVENT_ELEMENT_JOINED/LOST for script-AUTHORED transitions
 * only.  This is a second, independent consumer of the same
 * element_is_participating() (csm.h) predicate the ghost-vote fix uses,
 * but needs to know WHICH peer dropped out and WHY (csm.h's
 * tapestry_departure_reason_t) to support reasons filtering and
 * min_participants — a debounced swarm SIZE can't answer either question.
 * No extra debounce of its own: the DEPARTED bit is monotonic (main.c sets
 * it once, entering FLIGHT_LANDING, and never clears it) so it cannot
 * flicker, and the LOST-inferred case is already gated by
 * WM_EXPIRE_THRESHOLD_MS's own 5 s stability requirement.
 */
static choreo_departure_policy_t   s_departure_mode = CHOREO_DEPARTURE_CONTINUE;
static choreo_departure_reasons_t  s_departure_reasons = CHOREO_DEPARTURE_REASONS_ALL;
static uint8_t                     s_departure_min_participants;
static choreo_departure_recall_point_fn s_departure_recall_fn;

static bool                       s_departure_triggered;
static choreo_departure_policy_t  s_departure_triggered_policy;
static bool                       s_recall_in_progress;
static bool                       s_hold_in_progress;
static uint32_t                   s_hold_ms;

/* Snapshot of which peer IDs were participating as of the last evaluated
 * tick — a bit per element_id (MAX_ELEMENTS <= 32, csm.h).  Compared
 * against this tick's snapshot to find exactly which peer(s) newly
 * stopped participating (a departure edge), never re-triggering on a
 * peer that was already gone last tick. */
static uint32_t s_prev_participating_mask;
static bool     s_prev_participating_mask_valid;

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

/* ── Remote L6 directives (wire.h v5) ─────────────────────────────────────── */
/*
 * See choreo.h's remote-directive section for the design.  The local BSE
 * is never paused while remote directives steer — bse_tick() runs every
 * RUNNING tick either way, which is what makes fallback bumpless: the
 * local directive is always current, never resumed from a freeze.
 *
 * s_remote_age_ms is the time since the last ACCEPTED frame (reset by
 * choreo_remote_directive(), aged by choreo_tick()); s_remote_fresh_ms is
 * the continuous-freshness streak that gates adoption.  Adoption is
 * debounced (CHOREO_REMOTE_ADOPT_HOLD_MS); falling back is immediate —
 * the same up/down asymmetry as quorum.
 */
static tapestry_bse_directive_t s_remote_dir;
static uint16_t s_remote_goal_id;
static uint8_t  s_remote_src;
static bool     s_remote_seen;       /* any frame ever accepted           */
static uint32_t s_remote_age_ms;     /* since last accepted frame         */
static uint32_t s_remote_fresh_ms;   /* continuous freshness streak       */
static bool     s_remote_adopted;

static bool remote_fresh(void)
{
    return s_remote_seen && s_remote_age_ms < CHOREO_REMOTE_STALE_MS;
}

/* Age the remote stream one cycle and update adoption.  Runs every
 * choreo_tick() regardless of lifecycle state — link freshness is a fact
 * about the transport, not about the script. */
static void remote_directive_tick(void)
{
    if (!s_remote_seen) {
        return;
    }
    s_remote_age_ms += WM_CYCLE_MS;

    if (!remote_fresh()) {
        if (s_remote_adopted) {
            s_remote_adopted = false;   /* immediate local fallback */
        }
        s_remote_fresh_ms = 0;
        return;
    }
    s_remote_fresh_ms += WM_CYCLE_MS;
    if (s_remote_fresh_ms >= CHOREO_REMOTE_ADOPT_HOLD_MS) {
        s_remote_adopted = true;
    }
}

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
    s_remote_seen     = false;
    s_remote_adopted  = false;
    s_remote_age_ms   = 0;
    s_remote_fresh_ms = 0;
    s_departure_mode             = CHOREO_DEPARTURE_CONTINUE;
    s_departure_reasons          = CHOREO_DEPARTURE_REASONS_ALL;
    s_departure_min_participants = 0;
    s_departure_recall_fn        = NULL;
    s_departure_triggered        = false;
    s_departure_triggered_policy = CHOREO_DEPARTURE_CONTINUE;
    s_recall_in_progress         = false;
    s_hold_in_progress           = false;
    s_hold_ms                    = 0;
    s_prev_participating_mask_valid = false;
    bse_init(self_id);
}

void choreo_set_departure_recall_point_fn(choreo_departure_recall_point_fn fn)
{
    s_departure_recall_fn = fn;
}

void choreo_set_departure_policy(choreo_departure_policy_t policy,
                                 choreo_departure_reasons_t reasons,
                                 uint8_t min_participants)
{
    s_departure_mode             = policy;
    s_departure_reasons          = reasons;
    s_departure_min_participants = min_participants;
}

bool choreo_departure_triggered(void)
{
    return s_departure_triggered;
}

choreo_departure_policy_t choreo_departure_triggered_policy(void)
{
    return s_departure_triggered_policy;
}

void choreo_remote_directive(const tapestry_bse_directive_t *d,
                             uint16_t goal_id, uint8_t src_id)
{
    if (d == NULL) {
        return;
    }
    s_remote_dir     = *d;
    s_remote_goal_id = goal_id;
    s_remote_src     = src_id;
    s_remote_seen    = true;
    s_remote_age_ms  = 0;
}

bool choreo_remote_active(void)
{
    return s_remote_adopted && remote_fresh() &&
           s_state == CHOREO_STATE_RUNNING;
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
    s_prev_participating_mask_valid = false;   /* ditto, for departure detection */
    s_recall_in_progress = false;
    s_hold_in_progress   = false;
    s_hold_ms            = 0;
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
    s_departure_triggered = false;
    int rc = choreo_configure(goal);
    if (rc != 0) {
        return rc;
    }
    return choreo_deploy();
}

/* Validate every step up front — a script (or, per-track, a track) that
 * would stall or fail a capability check mid-show is rejected before
 * anything moves.  Shared by choreo_submit_script() and
 * choreo_submit_tracks() so the two validation paths can't drift.
 *
 * KNOWN GAP: unlike choreoc.py's compile-time _has_cycle()/max_runtime
 * check (sdk/python/tapestry/script_toml.py), this does NOT detect a
 * step graph that loops back on itself with no overall runtime bound —
 * choreoc refuses to emit such a script, so every steps array compiled
 * by the only delivery path that exists today is already guaranteed
 * acyclic-or-bounded before it ever reaches here. If a second delivery
 * path is ever added (choreo_submit_script() takes any pointer+count —
 * see its doc), a maliciously or accidentally cyclic, unbounded script
 * submitted that way would run forever undetected. Add the same check
 * here (or require the wire format to carry a precomputed total-timeout
 * value this function can enforce) before trusting scripts from any
 * source other than choreoc. */
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
    s_departure_triggered = false;

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
        /* ACTIVE, not fresh: a peer that has merely gone stale
         * (WM_STALE_THRESHOLD_MS) still gets a vote, cast from its
         * last-received goal_achieved.  Skipping stale peers here made
         * CHOREO_SCOPE_ALL silently degrade to per-element achievement
         * under packet loss.  scr_tick() drops quorum on that same
         * staleness threshold, but script_advance() runs BEFORE
         * choreo_tick() flips the state to SUSPENDED, so every
         * quorum-loss transition opened a one-tick window in which an
         * element that had personally arrived advanced alone — nobody
         * left "to disagree".  Two partners on a 17%-delivery link
         * finished a scope=all step 6.3 s apart (2026-08-24 flight 15).
         * Waiting on the last-known vote is the conservative direction: a
         * peer that had not yet reported achieved still blocks.  Expiry
         * (WM_EXPIRE_THRESHOLD_MS) takes 5 s of silence, by which point
         * quorum has been LOST for 3.5 s and the script is SUSPENDED —
         * script_advance() is unreachable — so an expired peer cannot
         * reopen the window either. */
        /* element_is_participating() (not is_active alone) also excludes a
         * peer that has self-declared departure (ELEMENT_HEALTH_DEPARTED):
         * a landed element's gossip stays alive on purpose (main.c's
         * flight-12 deadlock fix) with a frozen `achieved` bit that would
         * otherwise ghost-vote this step forever — either blocking it (bit
         * frozen false) or passing it on a step the departed element was
         * never part of (bit frozen true). */
        if (e->is_self || !element_is_participating(e)) {
            continue;
        }
        /* Track-filtered to match bse.c's collect_participants() (§7,
         * wire v4): a peer on a DIFFERENT track is running a different
         * collective activity and must not block this one's scope=all
         * step. choreo_current_track() defaults to 0, matching every
         * peer's gossiped current_track on a script with no tracks, so
         * this is a no-op filter for every existing (non-tracked)
         * caller. */
        if (e->state.current_track != choreo_current_track()) {
            continue;
        }
        if (!e->state.goal_achieved) {
            return false;
        }
    }
    return true;   /* vacuously true when genuinely solo — no peer to disagree */
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

/*
 * departure_policy_should_fire — reason/identity-aware departure edge
 * detector (see the "Departure policy" state comment above for why this
 * exists alongside, not instead of, update_membership_debounce()).
 *
 * Rebuilds this tick's participating-peer bitmask, diffs it against the
 * previous tick's, and for every peer that just dropped out determines
 * its reason bit (csm.h's tapestry_departure_reason_t, or the LOST bit
 * for an entry that expired outright).  Fires only if: something newly
 * departed this tick (the edge — never re-fires on a peer already gone
 * last tick); the resolved policy (per-step override, else the script
 * default) is not CONTINUE; at least one of this tick's departure
 * reasons passes the reasons filter; and, if min_participants > 0, the
 * surviving count (self + still-participating peers) has dropped to at
 * or below it.  Writes the resolved policy to *out_policy regardless of
 * whether it fires, so callers can log it either way.
 */
static bool departure_policy_should_fire(const world_model_t *wm,
                                         const choreo_step_t *st,
                                         choreo_departure_policy_t *out_policy)
{
    uint32_t mask = 0;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_self || !e->is_active) {
            continue;
        }
        if (element_is_participating(e)) {
            mask |= (1u << e->state.id);
        }
    }

    choreo_departure_reasons_t departed_reasons = 0;
    if (s_prev_participating_mask_valid) {
        uint32_t dropped = s_prev_participating_mask & ~mask;
        for (int id = 0; id < MAX_ELEMENTS; id++) {
            if ((dropped & (1u << id)) == 0) {
                continue;
            }
            const wm_entry_t *e = wm_get_entry(wm, (element_id_t)id);
            if (e == NULL || !e->is_active) {
                departed_reasons |= CHOREO_DEPARTURE_REASON_LOST_BIT;
            } else {
                departed_reasons |= CHOREO_DEPARTURE_REASON_BIT(
                    element_health_departed_reason(e->state.health_flags));
            }
        }
    }
    s_prev_participating_mask       = mask;
    s_prev_participating_mask_valid = true;

    choreo_departure_policy_t policy = (st->on_departure_set)
                                        ? st->on_departure : s_departure_mode;
    *out_policy = policy;

    if (departed_reasons == 0 || policy == CHOREO_DEPARTURE_CONTINUE) {
        return false;
    }
    if ((departed_reasons & s_departure_reasons) == 0) {
        return false;   /* every reason that fired is filtered out */
    }
    if (s_departure_min_participants > 0) {
        uint8_t surviving = 1;   /* self */
        for (int id = 0; id < MAX_ELEMENTS; id++) {
            if (mask & (1u << id)) {
                surviving++;
            }
        }
        /* min_participants is a floor that is still OK to be AT — "this
         * show needs at least N" is satisfied by exactly N, only firing
         * once a departure pushes the surviving count STRICTLY below it. */
        if (surviving >= s_departure_min_participants) {
            return false;
        }
    }
    return true;
}

/*
 * trigger_departure_policy — execute the resolved policy.  LAND_IN_PLACE
 * terminates exactly like normal script completion (advance_to()'s own
 * end-of-script path), just latching s_departure_triggered/
 * _triggered_policy first so the application can tell the two apart.
 * RECALL and HOLD preempt (choreo_preempt_goal()); their eventual
 * termination happens later, in choreo_tick() — see the RUNNING case —
 * once the recall point is reached, or the hold timeout elapses. Either
 * one falls back to landing immediately if it cannot even start (no
 * recall point registered/available, or something else already parked)
 * rather than silently doing nothing.
 */
static void land_for_departure(choreo_departure_policy_t executed_policy)
{
    s_departure_triggered        = true;
    s_departure_triggered_policy = executed_policy;
    s_script_done                = true;
    choreo_terminate();
}

static void trigger_departure_policy(choreo_departure_policy_t policy)
{
    switch (policy) {
    case CHOREO_DEPARTURE_RECALL: {
        position_t recall_pt;
        if (s_departure_recall_fn != NULL &&
            s_departure_recall_fn(&recall_pt)) {
            choreo_goal_t recall_goal = {
                .type   = CHOREO_GOAL_CONVERGE,
                .target = recall_pt,
            };
            if (choreo_preempt_goal(&recall_goal) == 0) {
                s_recall_in_progress = true;
                return;
            }
        }
        land_for_departure(CHOREO_DEPARTURE_LAND_IN_PLACE);
        return;
    }

    case CHOREO_DEPARTURE_HOLD: {
        choreo_goal_t hold_goal = { .type = CHOREO_GOAL_HOLD };
        if (choreo_preempt_goal(&hold_goal) == 0) {
            s_hold_in_progress = true;
            s_hold_ms          = 0;
            return;
        }
        land_for_departure(CHOREO_DEPARTURE_LAND_IN_PLACE);
        return;
    }

    case CHOREO_DEPARTURE_LAND_IN_PLACE:
        land_for_departure(CHOREO_DEPARTURE_LAND_IN_PLACE);
        return;

    case CHOREO_DEPARTURE_CONTINUE:
    default:
        return;
    }
}

/* Advance the script if the current step's exit condition is met —
 * either an explicit transition (checked first, first match wins) or,
 * absent any match, the legacy advance_on_achieved/max_duration_ms rule
 * (implicit next-index advance) every step had before transitions
 * existed.  The departure-policy dial ([choreo] mode / [on_departure] —
 * see choreo_set_departure_policy()) is checked next, but ONLY if no
 * explicit transition claimed the tick: a script's own authored
 * CHOREO_EVENT_ELEMENT_LOST handling is a deliberate choice by the
 * script author and takes priority over the coarse dial. */
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

    /* Unconditional: departure_policy_should_fire() must run every tick
     * regardless of target_idx, to keep its participating-peer snapshot
     * current — skipping it on a tick an explicit transition also fires
     * would corrupt next tick's edge diff (a real departure could be
     * silently missed, or re-detected a tick late against a stale
     * baseline). Only ACTING on the result is conditional on no explicit
     * transition having already claimed the tick. */
    choreo_departure_policy_t departure_policy;
    bool departure_fires =
        departure_policy_should_fire(wm, st, &departure_policy);
    if (target_idx < 0 && departure_fires) {
        trigger_departure_policy(departure_policy);
        return;
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
    s_departure_triggered = false;

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

void choreo_publish_state(element_state_t *own_state)
{
    own_state->goal_achieved = choreo_goal_achieved();
    own_state->current_track = choreo_current_track();
}

void choreo_tick(const world_model_t *wm, const scr_state_t *scr)
{
    remote_directive_tick();

    switch (s_state) {
    case CHOREO_STATE_RUNNING:
        if (s_n_tracks > 0 && s_parked_depth == 0) {
            update_track_selection(wm);
        }
        bse_set_track_scope(choreo_current_track());
        bse_tick(wm, scr);
        script_advance(wm, scr);
        /* RECALL preempts to CHOREO_GOAL_CONVERGE (script_advance() ->
         * trigger_departure_policy()) with s_script_active now false, so
         * script_advance() no longer runs while it's in progress — this
         * is the only place its arrival is ever observed.  terminate_hard()
         * (not choreo_terminate()) on purpose: the parked ORIGINAL script
         * must be discarded, not resumed — recall landing is meant to be
         * final, the same as LAND_IN_PLACE. */
        if (s_recall_in_progress && s_state == CHOREO_STATE_RUNNING &&
            choreo_goal_achieved()) {
            s_recall_in_progress = false;
            s_departure_triggered        = true;
            s_departure_triggered_policy = CHOREO_DEPARTURE_RECALL;
            s_script_done = true;
            terminate_hard();
        }
        /* HOLD preempts to CHOREO_GOAL_HOLD for up to
         * CHOREO_DEPARTURE_HOLD_TIMEOUT_MS, then gives up and lands —
         * departure is one-directional (a peer that left does not come
         * back), so this buys time rather than waiting for recovery.
         * terminate_hard(), same reasoning as RECALL above: discard the
         * parked script, don't resume it. */
        if (s_hold_in_progress && s_state == CHOREO_STATE_RUNNING) {
            s_hold_ms += WM_CYCLE_MS;
            if (s_hold_ms >= CHOREO_DEPARTURE_HOLD_TIMEOUT_MS) {
                s_hold_in_progress = false;
                s_departure_triggered        = true;
                s_departure_triggered_policy = CHOREO_DEPARTURE_LAND_IN_PLACE;
                s_script_done = true;
                terminate_hard();
            }
        }
        /* script_advance/the two blocks above may have terminated → IDLE;
         * quorum check only applies while still RUNNING. */
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
    /* Remote steering only while adopted, fresh, and RUNNING — SUSPENDED
     * (quorum lost) always steers locally: L5 is the safety authority and
     * a remote host's view of a partitioned collective is exactly what
     * cannot be trusted (choreo.h). */
    if (choreo_remote_active()) {
        return &s_remote_dir;
    }
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
