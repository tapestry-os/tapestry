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

/* ── Internal helpers ─────────────────────────────────────────────────────── */

/*
 * Check whether the registered element hardware satisfies required_caps.
 *
 * Mapping (paper §3.9 / CHOREO_CAP_* documentation in choreo.h):
 *   CHOREO_CAP_LOCOMOTION → SCR_CAP_ACTUATOR
 *   CHOREO_CAP_SENSING    → SCR_CAP_SENSOR
 *   CHOREO_CAP_SIGNALING  → SCR_CAP_RELAY
 *   CHOREO_CAP_BONDING    → no SCR equivalent; always unsatisfied
 */
static int caps_satisfied(choreo_capabilities_t required)
{
    if (s_scr == NULL || required == CHOREO_CAP_NONE) {
        return 1;
    }
    scr_capability_t hw = s_scr->capabilities;
    if ((required & CHOREO_CAP_LOCOMOTION) && !(hw & SCR_CAP_ACTUATOR)) return 0;
    if ((required & CHOREO_CAP_SENSING)    && !(hw & SCR_CAP_SENSOR))   return 0;
    if ((required & CHOREO_CAP_SIGNALING)  && !(hw & SCR_CAP_RELAY))    return 0;
    if  (required & CHOREO_CAP_BONDING)                                 return 0;
    return 1;
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
    intent.slot_shift      = goal->slot_shift;
    intent.direct_path     = goal->direct_path;
    intent.achieve_eps     = goal->achieve_eps;
    intent.achieve_hold_ms = goal->achieve_hold_ms;
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
    s_self_id = self_id;
    s_state   = CHOREO_STATE_IDLE;
    s_scr     = NULL;
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
    if (!caps_satisfied(goal->required_caps)) {
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

void choreo_terminate(void)
{
    s_state = CHOREO_STATE_TERMINATED;
    script_clear();
    tapestry_bse_intent_t idle = { .type = TAPESTRY_BSE_INTENT_IDLE };
    bse_submit_intent(&idle);
    s_state = CHOREO_STATE_IDLE;
}

int choreo_submit_goal(const choreo_goal_t *goal)
{
    if (goal == NULL) {
        return -1;
    }
    if (s_state != CHOREO_STATE_IDLE) {
        choreo_terminate();
    }
    s_script_done = false;
    int rc = choreo_configure(goal);
    if (rc != 0) {
        return rc;
    }
    return choreo_deploy();
}

int choreo_submit_script(const choreo_step_t *steps, uint8_t n_steps)
{
    if (steps == NULL || n_steps == 0) {
        return -1;
    }

    /* Validate every step up front — a script that would stall or fail a
     * capability check mid-show is rejected before anything moves. */
    for (uint8_t i = 0; i < n_steps; i++) {
        const choreo_step_t *st = &steps[i];
        if (st->goal.type == CHOREO_GOAL_NONE) {
            return -1;
        }
        if (!st->advance_on_achieved && st->max_duration_ms == 0u) {
            return -1;   /* no exit condition — stalls by construction */
        }
        if (!caps_satisfied(st->goal.required_caps)) {
            return -EPERM;
        }
    }

    if (s_state != CHOREO_STATE_IDLE) {
        choreo_terminate();
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

/* Advance the script if the current step's exit condition is met. */
static void script_advance(const world_model_t *wm)
{
    if (!s_script_active) {
        return;
    }

    const choreo_step_t *st = &s_steps[s_step_idx];
    s_step_ms += WM_CYCLE_MS;

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

    s_step_idx++;
    s_step_ms = 0;

    if (s_step_idx >= s_n_steps) {
        /* Script complete → quiescence: terminate submits the IDLE intent,
         * so the directive goes IDLE and the platform maps it to its own
         * inactive posture.  s_script_done survives the terminate. */
        s_script_done = true;
        choreo_terminate();
        return;
    }

    s_goal = s_steps[s_step_idx].goal;
    tapestry_bse_intent_t intent = goal_to_intent(&s_goal);
    bse_submit_intent(&intent);
}

void choreo_tick(const world_model_t *wm, const scr_state_t *scr)
{
    switch (s_state) {
    case CHOREO_STATE_RUNNING:
        bse_tick(wm, scr);
        script_advance(wm);
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
         * meaningless without fresh peers. */
        if (s_goal.type == CHOREO_GOAL_HOLD) {
            bse_tick(wm, scr);
        }
        if (scr->quorum_state != SCR_QUORUM_LOST) {
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
