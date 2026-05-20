/*
 * choreo.c — Tapestry Choreographer SDK stub (L7)
 *
 * NOT FOR PRODUCTION USE.  Implements choreo.h by delegating to bse.c.
 *
 * Lifecycle state machine:
 *   IDLE        → choreo_configure()  → CONFIGURED
 *   CONFIGURED  → choreo_deploy()     → RUNNING
 *   RUNNING     → quorum LOST (tick)  → SUSPENDED
 *   SUSPENDED   → quorum !LOST (tick) → RUNNING
 *   any         → choreo_terminate()  → (TERMINATED →) IDLE
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
    };

    tapestry_bse_intent_t intent = {0};
    intent.type   = (goal->type < (choreo_goal_type_t)(sizeof(type_map) / sizeof(type_map[0])))
                    ? type_map[goal->type]
                    : TAPESTRY_BSE_INTENT_IDLE;
    intent.target = goal->target;
    intent.radius = goal->radius;
    intent.shape  = goal->shape;
    return intent;
}

/* ── API ──────────────────────────────────────────────────────────────────── */

void choreo_init(element_id_t self_id)
{
    s_self_id = self_id;
    s_state   = CHOREO_STATE_IDLE;
    s_scr     = NULL;
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
    int rc = choreo_configure(goal);
    if (rc != 0) {
        return rc;
    }
    return choreo_deploy();
}

void choreo_cancel_goal(void)
{
    choreo_terminate();
}

choreo_state_t choreo_goal_status(void)
{
    return s_state;
}

void choreo_tick(const world_model_t *wm, const scr_state_t *scr)
{
    switch (s_state) {
    case CHOREO_STATE_RUNNING:
        bse_tick(wm, scr);
        if (scr->quorum_state == SCR_QUORUM_LOST) {
            s_state = CHOREO_STATE_SUSPENDED;
        }
        break;

    case CHOREO_STATE_SUSPENDED:
        bse_tick(wm, scr);   /* BSE issues HOLD directive while quorum is LOST */
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
