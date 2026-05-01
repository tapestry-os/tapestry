/*
 * choreo_stub.c — Tapestry Choreography SDK stub (L7)
 *
 * NOT FOR PRODUCTION USE.  Implements choreo.h by delegating to bse_stub.c.
 *
 * Goal status behaviour in this stub:
 *   IDLE     — no goal submitted
 *   ACTIVE   — goal submitted; quorum DEGRADED or HEALTHY
 *   FAILED   — quorum LOST while goal was active
 *   ACHIEVED — never set (requires the BSE feedback controller)
 */

#include <stddef.h>
#include <tapestry/choreo.h>
#include <tapestry/bse.h>
#include <tapestry/scr.h>

/* ── State ────────────────────────────────────────────────────────────────── */

static element_id_t    s_self_id;
static choreo_goal_t   s_goal;
static bool            s_has_goal;
static choreo_status_t s_status;

/* ── Internal helpers ─────────────────────────────────────────────────────── */

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
    s_self_id  = self_id;
    s_has_goal = false;
    s_status   = CHOREO_STATUS_IDLE;
    bse_init(self_id);
}

int choreo_submit_goal(const choreo_goal_t *goal)
{
    if (goal == NULL) {
        return -1;
    }
    s_goal     = *goal;
    s_has_goal = true;
    s_status   = CHOREO_STATUS_ACTIVE;

    tapestry_bse_intent_t intent = goal_to_intent(goal);
    return bse_submit_intent(&intent);
}

void choreo_cancel_goal(void)
{
    s_has_goal = false;
    s_status   = CHOREO_STATUS_IDLE;

    tapestry_bse_intent_t idle = { .type = TAPESTRY_BSE_INTENT_IDLE };
    bse_submit_intent(&idle);
}

choreo_status_t choreo_goal_status(void)
{
    return s_status;
}

void choreo_tick(const world_model_t *wm, const scr_state_t *scr)
{
    bse_tick(wm, scr);

    /* Stub failure detection: quorum lost while a goal is active. */
    if (s_has_goal && s_status == CHOREO_STATUS_ACTIVE) {
        if (scr->quorum_state == SCR_QUORUM_LOST) {
            s_status = CHOREO_STATUS_FAILED;
        }
    }
}

const tapestry_bse_directive_t *choreo_get_directive(void)
{
    return bse_get_directive();
}
