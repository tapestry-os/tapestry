/*
 * bse_stub.c — Tapestry L6 Behavior Synthesis Engine (STUB)
 *
 * NOT FOR PRODUCTION USE.  See bse.h for the full interface contract and
 * a description of which BSE tiers are absent from this implementation.
 *
 * What this stub does:
 *   - Intent parsing: reads the active intent type.
 *   - Task decomposition (FORM): maps the FORM goal to per-element vertex
 *     assignments — a regular N-gon centred on intent.target, N = active
 *     fresh element count.  Each element independently derives its own
 *     vertex from its peer-rank ordinal; no coordination messages needed.
 *   - For MOVE / CONVERGE: emits MOVE_TO_POINT to intent.target for all
 *     elements (no formation-relative offset; stub limitation).
 *   - For DISPERSE: emits MAINTAIN_SPRING with intent.radius as spacing.
 *   - For IDLE / unknown: emits IDLE.
 *
 * What this stub does NOT do:
 *   - Optimization across swarm (physics-aware planning, ML inference).
 *   - Path planning or obstacle avoidance.
 *   - Closed-loop feedback between intended and actual position.
 *   - Achievement detection (that belongs to the feedback controller).
 */

#include <tapestry/bse.h>
#include <string.h>
#include <math.h>

#define BSE_PI  3.14159265f

static element_id_t            s_self_id;
static tapestry_bse_intent_t   s_intent;
static tapestry_bse_directive_t s_directive;

/* ── API ──────────────────────────────────────────────────────────────────── */

void bse_init(element_id_t self_id)
{
    s_self_id = self_id;
    memset(&s_intent,    0, sizeof(s_intent));
    memset(&s_directive, 0, sizeof(s_directive));
    s_intent.type    = TAPESTRY_BSE_INTENT_IDLE;
    s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
}

int bse_submit_intent(const tapestry_bse_intent_t *intent)
{
    if (intent == NULL) {
        return -1;
    }
    s_intent = *intent;
    return 0;
}

void bse_tick(const world_model_t *wm, const scr_state_t *scr)
{
    (void)scr;   /* stub does not use SCR role for directive synthesis */

    switch (s_intent.type) {

    case TAPESTRY_BSE_INTENT_IDLE:
        s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
        break;

    case TAPESTRY_BSE_INTENT_FORM: {
        /*
         * Task decomposition (L6): map FORM intent onto a per-element vertex.
         * Each element independently sorts the active participant set and
         * claims the vertex at its own rank, producing a collision-free
         * geometry assignment without coordination messages.
         * L5 provides task_slot (an ordinal in the sorted peer list) but
         * does not perform this decomposition; the geometry mapping is L6's
         * sole responsibility.
         */
        element_id_t ids[MAX_ELEMENTS + 1];
        uint8_t      count = 0;

        ids[count++] = s_self_id;

        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm->entries[i];
            if (e->is_self || !e->is_active || e->is_stale) {
                continue;
            }
            ids[count++] = e->state.id;
        }

        /* Insertion sort — MAX_ELEMENTS is small (≤ 20) */
        for (int i = 1; i < (int)count; i++) {
            element_id_t key = ids[i];
            int j = i - 1;
            while (j >= 0 && ids[j] > key) {
                ids[j + 1] = ids[j];
                j--;
            }
            ids[j + 1] = key;
        }

        /* Find self's rank */
        int rank = -1;
        for (int i = 0; i < (int)count; i++) {
            if (ids[i] == s_self_id) {
                rank = i;
                break;
            }
        }
        if (rank < 0 || count == 0) {
            s_directive.type = TAPESTRY_BSE_DIRECTIVE_HOLD;
            break;
        }

        float angle = 2.0f * BSE_PI * (float)rank / (float)count;
        s_directive.type     = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target.x = s_intent.target.x + s_intent.radius * cosf(angle);
        s_directive.target.y = s_intent.target.y + s_intent.radius * sinf(angle);
        break;
    }

    case TAPESTRY_BSE_INTENT_MOVE:
    case TAPESTRY_BSE_INTENT_CONVERGE:
        /* Stub: move every element to the same target point.
         * A physics-aware planner would compute formation-relative offsets. */
        s_directive.type   = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target = s_intent.target;
        break;

    case TAPESTRY_BSE_INTENT_DISPERSE:
        s_directive.type     = TAPESTRY_BSE_DIRECTIVE_MAINTAIN_SPRING;
        s_directive.spring_k = 5.0f;
        s_directive.spacing  = s_intent.radius > 0.0f ? s_intent.radius : 30.0f;
        break;

    default:
        s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
        break;
    }
}

const tapestry_bse_directive_t *bse_get_directive(void)
{
    return &s_directive;
}
