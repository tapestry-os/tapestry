/*
 * bse.c — Tapestry L6 Behavior Synthesis Engine (STUB)
 *
 * NOT FOR PRODUCTION USE.  See bse.h for the full interface contract and
 * a description of which BSE tiers are absent from this implementation.
 *
 * What this stub does:
 *   - Intent parsing: reads the active intent type.
 *   - Task decomposition (FORM): maps the FORM goal to per-element vertex
 *     assignments — a regular N-gon centered on intent.target, N = active
 *     fresh element count.  Each element independently derives its own
 *     vertex from its peer-rank ordinal; no coordination messages needed.
 *   - Task decomposition (EXCHANGE): rotate stations by slot_shift around
 *     the ID-sorted participant ring, over a SNAPSHOT of positions captured
 *     at activation; the commanded target travels a CCW arc about the
 *     snapshot centroid so mutual separation is preserved by construction.
 *   - HOLD: captures own position at activation and station-keeps there.
 *   - Feedback controller (minimal): achievement predicate — own position
 *     within achieve_eps of the goal point for achieve_hold_ms.
 *   - For MOVE / CONVERGE: emits MOVE_TO_POINT to intent.target for all
 *     elements (no formation-relative offset; stub limitation).  Do not
 *     build on MOVE ≡ CONVERGE: per SDK design v0.2 §4, MOVE becomes
 *     offset-preserving translation of the current configuration
 *     (shape + drift), not all-to-point.
 *   - For DISPERSE: emits MAINTAIN_SPRING with intent.radius as spacing.
 *   - For IDLE / unknown: emits IDLE.
 *
 * What this stub does NOT do:
 *   - Optimization across swarm (physics-aware planning, ML inference) —
 *     the EXCHANGE arc is a fixed geometric deconfliction rule, not a
 *     planner.
 *   - Path planning or obstacle avoidance.
 *   - Collective achievement barriers (achievement is own-goal only).
 */

#include <tapestry/bse.h>
#include <string.h>
#include <math.h>

#define BSE_PI  3.14159265f

static element_id_t            s_self_id;
static tapestry_bse_intent_t   s_intent;
static tapestry_bse_directive_t s_directive;

/* ── Achievement predicate state ──────────────────────────────────────────── */

static bool     s_achieved;
static bool     s_goal_pt_valid;    /* goal point computed this tick        */
static tapestry_position_t s_goal_pt;
static uint32_t s_achieve_accum_ms;

/* ── HOLD station capture ─────────────────────────────────────────────────── */

static bool                s_hold_captured;
static tapestry_position_t s_hold_station;

/* ── EXCHANGE snapshot + arc state ───────────────────────────────────────── */

static bool                s_ex_captured;
static tapestry_position_t s_ex_dest;       /* destination station (snapshot) */
static tapestry_position_t s_ex_centroid;   /* snapshot centroid              */
static float               s_ex_theta0;     /* own start angle about centroid */
static float               s_ex_dtheta;     /* total CCW angle to travel      */
static float               s_ex_r0;         /* own start radius               */
static float               s_ex_r1;         /* destination radius             */
static float               s_ex_progress;   /* radians travelled so far       */

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static bool own_position(const world_model_t *wm, tapestry_position_t *out)
{
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_self) {
            out->x = e->state.position.x;
            out->y = e->state.position.y;
            return true;
        }
    }
    return false;
}

/* Collect self + fresh active peers, ID-sorted.  Returns count; fills ids[]
 * and positions[] in sorted order, and writes self's rank to *own_rank
 * (-1 if self entry missing). */
static int collect_participants(const world_model_t *wm,
                                element_id_t ids[MAX_ELEMENTS],
                                tapestry_position_t pos[MAX_ELEMENTS],
                                int *own_rank)
{
    int count = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        bool participant = e->is_self ||
                           (e->is_active && !e->is_stale);
        if (!participant) {
            continue;
        }
        if (count >= MAX_ELEMENTS) {
            break;
        }
        ids[count]   = e->is_self ? s_self_id : e->state.id;
        pos[count].x = e->state.position.x;
        pos[count].y = e->state.position.y;
        count++;
    }

    /* Insertion sort by ID, positions carried along — MAX_ELEMENTS is small */
    for (int i = 1; i < count; i++) {
        element_id_t        kid = ids[i];
        tapestry_position_t kp  = pos[i];
        int j = i - 1;
        while (j >= 0 && ids[j] > kid) {
            ids[j + 1] = ids[j];
            pos[j + 1] = pos[j];
            j--;
        }
        ids[j + 1] = kid;
        pos[j + 1] = kp;
    }

    *own_rank = -1;
    for (int i = 0; i < count; i++) {
        if (ids[i] == s_self_id) {
            *own_rank = i;
            break;
        }
    }
    return count;
}

/*
 * Capture the EXCHANGE snapshot: participant stations frozen at this
 * instant, own → destination arc parameters about the centroid.
 * Requires at least one fresh peer (N >= 2).  Returns true on success.
 *
 * Each element captures independently from its own world model; the
 * snapshots differ by at most one gossip interval of peer motion, which
 * the achievement epsilon absorbs.  Frozen targets — never live-chasing.
 */
static bool exchange_capture(const world_model_t *wm)
{
    element_id_t        ids[MAX_ELEMENTS];
    tapestry_position_t pos[MAX_ELEMENTS];
    int                 own_rank;

    int n = collect_participants(wm, ids, pos, &own_rank);
    if (n < 2 || own_rank < 0) {
        return false;
    }

    uint8_t shift = s_intent.slot_shift != 0u ? s_intent.slot_shift : 1u;
    int     dest  = (own_rank + (int)shift) % n;

    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < n; i++) {
        cx += pos[i].x;
        cy += pos[i].y;
    }
    cx /= (float)n;
    cy /= (float)n;

    tapestry_position_t own = pos[own_rank];

    s_ex_centroid.x = cx;
    s_ex_centroid.y = cy;
    s_ex_dest       = pos[dest];
    s_ex_theta0     = atan2f(own.y - cy, own.x - cx);
    s_ex_r0         = sqrtf((own.x - cx) * (own.x - cx)
                            + (own.y - cy) * (own.y - cy));
    s_ex_r1         = sqrtf((s_ex_dest.x - cx) * (s_ex_dest.x - cx)
                            + (s_ex_dest.y - cy) * (s_ex_dest.y - cy));

    /* CCW angular travel to the destination station, in (0, 2π].  All
     * elements rotate the same direction, so pairwise angular offsets —
     * and therefore separation — are preserved throughout the maneuver. */
    float theta1 = atan2f(s_ex_dest.y - cy, s_ex_dest.x - cx);
    float dtheta = theta1 - s_ex_theta0;
    while (dtheta <= 0.0f) {
        dtheta += 2.0f * BSE_PI;
    }
    /* Degenerate: destination is own station (shift ≡ 0 mod N, or
     * coincident snapshot points) — no travel. */
    if (dest == own_rank) {
        dtheta = 0.0f;
    }
    /* Direct path: skip the arc entirely — the commanded target is the
     * destination from the first tick (see the intent field's comment for
     * when that is safe).  dtheta = 0 also makes the achievement gate
     * active immediately, which is correct here: with a stationary
     * target, "within eps, sustained" measures real arrival. */
    if (s_intent.direct_path) {
        dtheta = 0.0f;
    }

    s_ex_dtheta   = dtheta;
    s_ex_progress = 0.0f;
    s_ex_captured = true;
    return true;
}

/* Advance the arc by one tick and return the commanded target. */
static tapestry_position_t exchange_arc_target(void)
{
    s_ex_progress += TAPESTRY_BSE_EXCHANGE_OMEGA_RADPS
                     * ((float)WM_CYCLE_MS * 0.001f);

    if (s_ex_dtheta <= 0.0f || s_ex_progress >= s_ex_dtheta) {
        return s_ex_dest;   /* arc complete — exact snapshot station */
    }

    float frac  = s_ex_progress / s_ex_dtheta;
    float theta = s_ex_theta0 + s_ex_progress;
    float r     = s_ex_r0 + (s_ex_r1 - s_ex_r0) * frac;

    tapestry_position_t t;
    t.x = s_ex_centroid.x + r * cosf(theta);
    t.y = s_ex_centroid.y + r * sinf(theta);
    return t;
}

/* ── API ──────────────────────────────────────────────────────────────────── */

void bse_init(element_id_t self_id)
{
    s_self_id = self_id;
    memset(&s_intent,    0, sizeof(s_intent));
    memset(&s_directive, 0, sizeof(s_directive));
    s_intent.type    = TAPESTRY_BSE_INTENT_IDLE;
    s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;

    s_achieved         = false;
    s_goal_pt_valid    = false;
    s_achieve_accum_ms = 0;
    s_hold_captured    = false;
    s_ex_captured      = false;
}

int bse_submit_intent(const tapestry_bse_intent_t *intent)
{
    if (intent == NULL) {
        return -1;
    }
    s_intent = *intent;

    /* New goal — reset captures and the achievement predicate. */
    s_achieved         = false;
    s_goal_pt_valid    = false;
    s_achieve_accum_ms = 0;
    s_hold_captured    = false;
    s_ex_captured      = false;

    /* An IDLE intent (quiescence) takes effect immediately, not at the next
     * tick — choreo_terminate() submits it and then stops ticking the BSE,
     * so waiting for a tick would leave the previous directive latched. */
    if (s_intent.type == TAPESTRY_BSE_INTENT_IDLE) {
        s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
    }
    return 0;
}

void bse_tick(const world_model_t *wm, const scr_state_t *scr)
{
    (void)scr;   /* stub does not use SCR role for directive synthesis */

    s_goal_pt_valid = false;

    switch (s_intent.type) {

    case TAPESTRY_BSE_INTENT_IDLE:
        s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
        break;

    case TAPESTRY_BSE_INTENT_HOLD: {
        /* Coordinate-free: the station is wherever the element is when the
         * goal activates.  Captured once, then actively station-kept. */
        if (!s_hold_captured) {
            tapestry_position_t own;
            if (!own_position(wm, &own)) {
                s_directive.type = TAPESTRY_BSE_DIRECTIVE_HOLD;
                break;
            }
            s_hold_station  = own;
            s_hold_captured = true;
        }
        s_directive.type   = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target = s_hold_station;
        s_goal_pt          = s_hold_station;
        s_goal_pt_valid    = true;
        break;
    }

    case TAPESTRY_BSE_INTENT_EXCHANGE: {
        if (!s_ex_captured && !exchange_capture(wm)) {
            /* No fresh peer visible — cannot know whose station to take.
             * Hold and retry the capture next tick. */
            s_directive.type = TAPESTRY_BSE_DIRECTIVE_HOLD;
            break;
        }
        s_directive.type   = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target = exchange_arc_target();
        /* Achievement is measured against the DESTINATION station, and only
         * once the arc itself has completed — otherwise a body tracking the
         * arc perfectly "achieves" while still up to eps short of the
         * station, and settles offset from it. */
        s_goal_pt          = s_ex_dest;
        s_goal_pt_valid    = (s_ex_dtheta <= 0.0f
                              || s_ex_progress >= s_ex_dtheta);

        /* Occupied destination (see the OCCUPIED_M/STANDOFF_M rationale in
         * bse.h): hold a standoff point on the approach line and defer
         * achievement while a fresh peer still sits on the station. */
        {
            bool occupied = false;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm->entries[i];
                if (!e->is_active || e->is_self || e->is_stale) {
                    continue;
                }
                float dx = e->state.position.x - s_ex_dest.x;
                float dy = e->state.position.y - s_ex_dest.y;
                if (sqrtf(dx * dx + dy * dy)
                        < TAPESTRY_BSE_EXCHANGE_OCCUPIED_M) {
                    occupied = true;
                    break;
                }
            }
            if (occupied) {
                tapestry_position_t own;
                if (own_position(wm, &own)) {
                    float dx = own.x - s_ex_dest.x;
                    float dy = own.y - s_ex_dest.y;
                    float d  = sqrtf(dx * dx + dy * dy);
                    if (d > 1e-3f) {
                        s_directive.target.x = s_ex_dest.x
                            + dx / d * TAPESTRY_BSE_EXCHANGE_STANDOFF_M;
                        s_directive.target.y = s_ex_dest.y
                            + dy / d * TAPESTRY_BSE_EXCHANGE_STANDOFF_M;
                    }
                }
                s_goal_pt_valid = false;
            }
        }
        break;
    }

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
        element_id_t        ids[MAX_ELEMENTS];
        tapestry_position_t pos[MAX_ELEMENTS];
        int                 rank;

        int count = collect_participants(wm, ids, pos, &rank);
        if (rank < 0 || count == 0) {
            s_directive.type = TAPESTRY_BSE_DIRECTIVE_HOLD;
            break;
        }

        float angle = 2.0f * BSE_PI * (float)rank / (float)count;
        s_directive.type     = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target.x = s_intent.target.x + s_intent.radius * cosf(angle);
        s_directive.target.y = s_intent.target.y + s_intent.radius * sinf(angle);
        s_goal_pt            = s_directive.target;
        s_goal_pt_valid      = true;
        break;
    }

    case TAPESTRY_BSE_INTENT_MOVE:
    case TAPESTRY_BSE_INTENT_CONVERGE:
        /* Stub: move every element to the same target point.
         * A physics-aware planner would compute formation-relative offsets. */
        s_directive.type   = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target = s_intent.target;
        s_goal_pt          = s_intent.target;
        s_goal_pt_valid    = true;
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

    /* ── Feedback controller (minimal): achievement predicate ───────────── */

    if (s_intent.type == TAPESTRY_BSE_INTENT_HOLD && s_hold_captured) {
        /* Staying is the goal — trivially achieved; duration governs. */
        s_achieved = true;
    } else if (s_goal_pt_valid) {
        float    eps  = s_intent.achieve_eps > 0.0f
                        ? s_intent.achieve_eps
                        : TAPESTRY_BSE_ACHIEVE_EPS_DEFAULT;
        uint32_t hold = s_intent.achieve_hold_ms > 0u
                        ? s_intent.achieve_hold_ms
                        : TAPESTRY_BSE_ACHIEVE_HOLD_MS_DEFAULT;

        tapestry_position_t own;
        if (own_position(wm, &own)) {
            float dx = own.x - s_goal_pt.x;
            float dy = own.y - s_goal_pt.y;
            if (sqrtf(dx * dx + dy * dy) <= eps) {
                s_achieve_accum_ms += WM_CYCLE_MS;
            } else {
                s_achieve_accum_ms = 0;
            }
            s_achieved = s_achieve_accum_ms >= hold;
        }
    } else {
        s_achieve_accum_ms = 0;
        s_achieved         = false;
    }
}

const tapestry_bse_directive_t *bse_get_directive(void)
{
    return &s_directive;
}

bool bse_goal_achieved(void)
{
    return s_achieved;
}
