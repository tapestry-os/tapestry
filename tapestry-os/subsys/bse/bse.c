/*
 * bse.c — Tapestry L6 Behavior Synthesis Engine
 *
 * See bse.h for the full interface contract and v1.0 feature scope (what's
 * open-core here vs. deliberately licensed-tier).
 *
 * What this implements:
 *   - Intent parsing: reads the active intent type.
 *   - Task decomposition (FORM): maps the FORM goal to per-element vertex
 *     assignments, N = active fresh element count, using intent.shape:
 *     CIRCLE (regular N-gon), LINE (evenly spaced on the X axis), or GRID
 *     (near-square rows/cols, radius as cell spacing) — all centered on
 *     intent.target.  Each element independently derives its own vertex
 *     from its peer-rank ordinal; no coordination messages needed.
 *   - Task decomposition (EXCHANGE): rotate stations by slot_shift around
 *     the ID-sorted participant ring, over a SNAPSHOT of positions captured
 *     at activation; the commanded target travels a CCW arc about the
 *     snapshot centroid so mutual separation is preserved by construction.
 *   - HOLD: captures own position at activation and station-keeps there.
 *   - Feedback controller (minimal): achievement predicate — own position
 *     within achieve_eps of the goal point for achieve_hold_ms, for every
 *     goal type except DISPERSE, which has no single goal point (see below).
 *   - For MOVE: offset-preserving translation — the
 *     formation's shape is preserved while its centroid travels to
 *     intent.target (own offset from the participant centroid is snapshot
 *     at activation).  A solo element has zero offset, so MOVE degenerates
 *     to CONVERGE — correct, there is nothing to preserve.
 *   - For CONVERGE: emits MOVE_TO_POINT to intent.target for all elements
 *     (deliberately collapses the formation — this is the "gather" goal).
 *   - For DISPERSE: emits MAINTAIN_SPRING with intent.radius as spacing.
 *     Achievement predicate is nearest-fresh-trusted-peer distance >=
 *     spacing - achieve_eps, sustained for achieve_hold_ms; vacuously
 *     achieved with no peer visible.
 *   - For IDLE / unknown: emits IDLE.
 *
 * What this does NOT do:
 *   - Optimization across swarm (physics-aware planning, ML inference) —
 *     the EXCHANGE arc is a fixed geometric deconfliction rule, not a
 *     planner.  Licensed-tier scope, not a gap to fill here.
 *   - Path planning or obstacle avoidance.
 *   - Collective achievement aggregation: bse_goal_achieved() is own-goal
 *     only by design — the scope=all aggregation across peers is L7's
 *     job (choreo_collective_achieved() in choreo.c), one layer up.
 */

#include <tapestry/bse.h>
#include <string.h>
#include <math.h>

#define BSE_PI  3.14159265f

static element_id_t            s_self_id;
static tapestry_bse_intent_t   s_intent;
static tapestry_bse_directive_t s_directive;

/* ── Per-activation state ─────────────────────────────────────────────────── */
/*
 * bse_activation_t — everything an intent accumulates between the moment it
 * becomes active and the moment it is displaced.  Grouped rather than left
 * as loose file-scope statics for one reason: this is exactly the state a
 * preempting BSE has to save and restore.
 *
 * bse_submit_intent() resets this on every call, so a displaced intent is
 * simply forgotten. bse_preempt_intent()/bse_resume_intent() (below)
 * instead push/pop this struct on a bounded stack (s_parked_act) and
 * restore the preempted entry when the preempting intent completes —
 * nothing outside this struct needs to be saved for that to work, which is
 * why the push/pop pair only ever touches s_intent and this struct.
 *
 * Note what is deliberately NOT here: s_goal_pt / s_goal_pt_valid are
 * recomputed from scratch by every bse_tick(), so they are tick-scoped, not
 * activation-scoped, and a resumed intent regenerates them on its first
 * tick back.
 */
typedef struct {
    /* Feedback controller (achievement predicate) */
    bool     achieved;
    uint32_t achieve_accum_ms;

    /* HOLD: own station, captured at activation */
    bool                hold_captured;
    tapestry_position_t hold_station;

    /* EXCHANGE: frozen snapshot + arc progress */
    bool                ex_captured;
    tapestry_position_t ex_dest;       /* destination station (snapshot) */
    tapestry_position_t ex_centroid;   /* snapshot centroid              */
    float               ex_theta0;     /* own start angle about centroid */
    float               ex_dtheta;     /* total CCW angle to travel      */
    float               ex_r0;         /* own start radius               */
    float               ex_r1;         /* destination radius             */
    float               ex_progress;   /* radians travelled so far       */

    /* MOVE: own offset from the participant centroid, snapshot at activation */
    bool                move_captured;
    tapestry_position_t move_offset;   /* own anchor - snapshot centroid */
} bse_activation_t;

static bse_activation_t s_act;

/* Tick-scoped: recomputed by every bse_tick(), never carried across one. */
static bool                s_goal_pt_valid;   /* goal point computed this tick */
static tapestry_position_t s_goal_pt;

/* ── Preemption stack ─────────────────────────────────────────────────────── */
/*
 * bse_preempt_intent()/bse_resume_intent() implement the extension point
 * bse_submit_intent()'s doc comment and bse_activation_t's comment above
 * describe: instead of discarding the displaced intent, save it and restore
 * it verbatim when the preempting intent completes.  A bounded stack, not a
 * singleton, on purpose — BSE_MAX_PREEMPT_DEPTH is 1 today (matches
 * choreo.c's CHOREO_MAX_PREEMPT_DEPTH), but nesting to N-deep later is
 * raising this constant and loosening one bound check, not a redesign: the
 * push/pop shape and s_intent/s_act split already generalize.  Tick-scoped
 * state (s_goal_pt/s_goal_pt_valid) is deliberately NOT saved here — it is
 * recomputed by every bse_tick() regardless of which intent is active, so
 * there is nothing to preserve across a push/pop.
 */
#define BSE_MAX_PREEMPT_DEPTH 1
static tapestry_bse_intent_t s_parked_intent[BSE_MAX_PREEMPT_DEPTH];
static bse_activation_t      s_parked_act[BSE_MAX_PREEMPT_DEPTH];
static int                   s_parked_depth;

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

/* Collect self + fresh, trusted active peers, ID-sorted.  Returns count;
 * fills ids[] and positions[] in sorted order, and writes self's rank to
 * *own_rank (-1 if self entry missing).
 *
 * Trust-filtered via scr_peer_is_trusted() — the same whitelist/anomaly
 * check scr_tick() applies when building its own candidate set — so the
 * rank and count computed here agree with scr->task_slot/swarm_size, and an
 * anomaly-excluded or non-whitelisted peer cannot steer formation geometry
 * (centroid, arc, vertex assignment) despite being excluded from quorum. */
static int collect_participants(const world_model_t *wm,
                                const scr_state_t *scr,
                                element_id_t ids[MAX_ELEMENTS],
                                tapestry_position_t pos[MAX_ELEMENTS],
                                int *own_rank)
{
    int count = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        bool participant = e->is_self ||
                           (e->is_active && !e->is_stale &&
                            scr_peer_is_trusted(scr, e->state.id));
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
static bool exchange_capture(const world_model_t *wm, const scr_state_t *scr)
{
    element_id_t        ids[MAX_ELEMENTS];
    tapestry_position_t pos[MAX_ELEMENTS];
    int                 own_rank;

    int n = collect_participants(wm, scr, ids, pos, &own_rank);
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

    s_act.ex_centroid.x = cx;
    s_act.ex_centroid.y = cy;
    s_act.ex_dest       = pos[dest];
    s_act.ex_theta0     = atan2f(own.y - cy, own.x - cx);
    s_act.ex_r0         = sqrtf((own.x - cx) * (own.x - cx)
                            + (own.y - cy) * (own.y - cy));
    s_act.ex_r1         = sqrtf((s_act.ex_dest.x - cx) * (s_act.ex_dest.x - cx)
                            + (s_act.ex_dest.y - cy) * (s_act.ex_dest.y - cy));

    /* CCW angular travel to the destination station, in (0, 2π].  All
     * elements rotate the same direction, so pairwise angular offsets —
     * and therefore separation — are preserved throughout the maneuver. */
    float theta1 = atan2f(s_act.ex_dest.y - cy, s_act.ex_dest.x - cx);
    float dtheta = theta1 - s_act.ex_theta0;
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

    s_act.ex_dtheta   = dtheta;
    s_act.ex_progress = 0.0f;
    s_act.ex_captured = true;
    return true;
}

/* Advance the arc by one tick and return the commanded target. */
static tapestry_position_t exchange_arc_target(void)
{
    s_act.ex_progress += TAPESTRY_BSE_EXCHANGE_OMEGA_RADPS
                     * ((float)WM_CYCLE_MS * 0.001f);

    if (s_act.ex_dtheta <= 0.0f || s_act.ex_progress >= s_act.ex_dtheta) {
        return s_act.ex_dest;   /* arc complete — exact snapshot station */
    }

    float frac  = s_act.ex_progress / s_act.ex_dtheta;
    float theta = s_act.ex_theta0 + s_act.ex_progress;
    float r     = s_act.ex_r0 + (s_act.ex_r1 - s_act.ex_r0) * frac;

    tapestry_position_t t;
    t.x = s_act.ex_centroid.x + r * cosf(theta);
    t.y = s_act.ex_centroid.y + r * sinf(theta);
    return t;
}

/* ── API ──────────────────────────────────────────────────────────────────── */

/* Shared by bse_init(), bse_submit_intent(), and bse_preempt_intent() — a
 * fresh intent always starts with a clean activation. */
static void reset_activation(void)
{
    s_act.achieved         = false;
    s_goal_pt_valid        = false;
    s_act.achieve_accum_ms = 0;
    s_act.hold_captured    = false;
    s_act.ex_captured      = false;
    s_act.move_captured    = false;
}

void bse_init(element_id_t self_id)
{
    s_self_id = self_id;
    memset(&s_intent,    0, sizeof(s_intent));
    memset(&s_directive, 0, sizeof(s_directive));
    s_intent.type    = TAPESTRY_BSE_INTENT_IDLE;
    s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
    s_parked_depth   = 0;

    reset_activation();
}

int bse_submit_intent(const tapestry_bse_intent_t *intent)
{
    if (intent == NULL) {
        return -1;
    }
    s_intent = *intent;
    reset_activation();

    /* An ordinary submit always fully discards — including anything
     * bse_preempt_intent() had parked.  Only bse_preempt_intent()/
     * bse_resume_intent() grow/shrink the parked stack; this is the
     * "hard reset to exactly this one intent" path (see choreo.c's
     * terminate_hard(), which relies on this to drop a parked goal on an
     * ordinary new submission even if a preemption happens to be active). */
    s_parked_depth = 0;

    /* An IDLE intent (quiescence) takes effect immediately, not at the next
     * tick — choreo_terminate() submits it and then stops ticking the BSE,
     * so waiting for a tick would leave the previous directive latched. */
    if (s_intent.type == TAPESTRY_BSE_INTENT_IDLE) {
        s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
    }
    return 0;
}

int bse_preempt_intent(const tapestry_bse_intent_t *intent)
{
    if (intent == NULL) {
        return -1;
    }
    if (s_parked_depth >= BSE_MAX_PREEMPT_DEPTH) {
        return -1;   /* stack full */
    }
    s_parked_intent[s_parked_depth] = s_intent;
    s_parked_act[s_parked_depth]    = s_act;
    s_parked_depth++;

    s_intent = *intent;
    reset_activation();

    if (s_intent.type == TAPESTRY_BSE_INTENT_IDLE) {
        s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
    }
    return 0;
}

int bse_resume_intent(void)
{
    if (s_parked_depth <= 0) {
        return -1;   /* nothing parked */
    }
    s_parked_depth--;
    s_intent = s_parked_intent[s_parked_depth];
    s_act    = s_parked_act[s_parked_depth];

    /* Tick-scoped goal point was never saved (see the preemption-stack
     * comment above) — invalidate rather than leave it referencing the
     * intent that was active a moment ago. bse_tick() recomputes it fresh
     * on the next tick either way. */
    s_goal_pt_valid = false;
    return 0;
}

bool bse_has_parked_intent(void)
{
    return s_parked_depth > 0;
}

void bse_tick(const world_model_t *wm, const scr_state_t *scr)
{
    s_goal_pt_valid = false;

    switch (s_intent.type) {

    case TAPESTRY_BSE_INTENT_IDLE:
        s_directive.type = TAPESTRY_BSE_DIRECTIVE_IDLE;
        break;

    case TAPESTRY_BSE_INTENT_HOLD: {
        /* Coordinate-free: the station is wherever the element is when the
         * goal activates.  Captured once, then actively station-kept. */
        if (!s_act.hold_captured) {
            tapestry_position_t own;
            if (!own_position(wm, &own)) {
                s_directive.type = TAPESTRY_BSE_DIRECTIVE_HOLD;
                break;
            }
            s_act.hold_station  = own;
            s_act.hold_captured = true;
        }
        s_directive.type   = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target = s_act.hold_station;
        s_goal_pt          = s_act.hold_station;
        s_goal_pt_valid    = true;
        break;
    }

    case TAPESTRY_BSE_INTENT_EXCHANGE: {
        if (!s_act.ex_captured && !exchange_capture(wm, scr)) {
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
        s_goal_pt          = s_act.ex_dest;
        s_goal_pt_valid    = (s_act.ex_dtheta <= 0.0f
                              || s_act.ex_progress >= s_act.ex_dtheta);

        /* Occupied destination (see the OCCUPIED_M/STANDOFF_M rationale in
         * bse.h): hold a standoff point on the approach line and defer
         * achievement while a fresh peer still sits on the station.
         *
         * direct_path only.  The arc already preserves separation by
         * construction (shared rotation direction, radius interpolated
         * from the snapshot — see exchange_arc_target()), so it never
         * beelines into an occupied station in the first place.  Applying
         * this unconditionally broke that guarantee: on a symmetric swap
         * the peer starts exactly at this element's destination, the
         * standoff fires on tick 1, and the commanded target gets pulled
         * off the arc toward the centroid — collapsing separation instead
         * of preserving it.  Caught by
         * choreo_script_test_swap_script_end_to_end (arc-mode script). */
        if (s_intent.direct_path) {
            bool occupied = false;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm->entries[i];
                if (!e->is_active || e->is_self || e->is_stale) {
                    continue;
                }
                float dx = e->state.position.x - s_act.ex_dest.x;
                float dy = e->state.position.y - s_act.ex_dest.y;
                if (sqrtf(dx * dx + dy * dy)
                        < TAPESTRY_BSE_EXCHANGE_OCCUPIED_M) {
                    occupied = true;
                    break;
                }
            }
            if (occupied) {
                tapestry_position_t own;
                if (own_position(wm, &own)) {
                    float dx = own.x - s_act.ex_dest.x;
                    float dy = own.y - s_act.ex_dest.y;
                    float d  = sqrtf(dx * dx + dy * dy);
                    if (d > 1e-3f) {
                        s_directive.target.x = s_act.ex_dest.x
                            + dx / d * TAPESTRY_BSE_EXCHANGE_STANDOFF_M;
                        s_directive.target.y = s_act.ex_dest.y
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
         * rank/count come directly from L5's scr_state_t — task_slot is the
         * element's ordinal in L5's own trusted-fresh-peer sort, swarm_size
         * its count (scr.h) — rather than being re-derived from a second,
         * independently-filtered participant scan.  Reusing L5's own values
         * guarantees the vertex assignment agrees with L5's quorum/election
         * view by construction — no second copy of the trust-filtering/
         * sorting logic to keep in lockstep with scr.c's — so an anomaly-
         * excluded or unwhitelisted peer that L5 already excludes from
         * candidates cannot end up claiming a vertex here.  Callers must
         * scr_tick() before bse_tick() for this to reflect the current wm;
         * task_slot/swarm_size are valid only when quorum >= DEGRADED
         * (scr.h) — swarm_size == 0 otherwise, handled below like the prior
         * "no participants" case.
         */
        int rank  = scr_get_task_slot(scr);
        int count = scr_get_swarm_size(scr);
        if (count == 0) {
            s_directive.type = TAPESTRY_BSE_DIRECTIVE_HOLD;
            break;
        }

        tapestry_position_t tgt;
        switch (s_intent.shape) {
        case TAPESTRY_BSE_SHAPE_LINE: {
            /* Evenly spaced along the X axis, centered on intent.target,
             * spanning [-radius, +radius]. */
            if (count > 1) {
                float step = (2.0f * s_intent.radius) / (float)(count - 1);
                tgt.x = s_intent.target.x - s_intent.radius + step * (float)rank;
            } else {
                tgt.x = s_intent.target.x;
            }
            tgt.y = s_intent.target.y;
            break;
        }
        case TAPESTRY_BSE_SHAPE_GRID: {
            /* Near-square layout; radius reused as cell spacing. */
            int cols = (int)ceilf(sqrtf((float)count));
            int rows = (int)ceilf((float)count / (float)cols);
            int col  = rank % cols;
            int row  = rank / cols;
            tgt.x = s_intent.target.x
                    + ((float)col - 0.5f * (float)(cols - 1)) * s_intent.radius;
            tgt.y = s_intent.target.y
                    + ((float)row - 0.5f * (float)(rows - 1)) * s_intent.radius;
            break;
        }
        case TAPESTRY_BSE_SHAPE_CIRCLE:
        default: {
            float angle = 2.0f * BSE_PI * (float)rank / (float)count;
            tgt.x = s_intent.target.x + s_intent.radius * cosf(angle);
            tgt.y = s_intent.target.y + s_intent.radius * sinf(angle);
            break;
        }
        }

        s_directive.type   = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target = tgt;
        s_goal_pt           = s_directive.target;
        s_goal_pt_valid     = true;
        break;
    }

    case TAPESTRY_BSE_INTENT_MOVE: {
        /*
         * Offset-preserving translation: "move" is
         * shape + drift, not collapse-to-point.  On activation, snapshot
         * this element's own offset from the participant centroid; every
         * tick after that the commanded point is intent.target displaced
         * by that same offset, so the formation's relative geometry is
         * preserved while its centroid travels to intent.target.  A solo
         * element has offset (0,0) — MOVE degenerates to CONVERGE, which
         * is correct: there is no formation to preserve.
         */
        if (!s_act.move_captured) {
            element_id_t        ids[MAX_ELEMENTS];
            tapestry_position_t pos[MAX_ELEMENTS];
            int                 rank;
            int count = collect_participants(wm, scr, ids, pos, &rank);
            if (rank < 0 || count == 0) {
                s_directive.type = TAPESTRY_BSE_DIRECTIVE_HOLD;
                break;
            }
            float cx = 0.0f, cy = 0.0f;
            for (int i = 0; i < count; i++) {
                cx += pos[i].x;
                cy += pos[i].y;
            }
            cx /= (float)count;
            cy /= (float)count;
            s_act.move_offset.x = pos[rank].x - cx;
            s_act.move_offset.y = pos[rank].y - cy;
            s_act.move_captured = true;
        }
        s_directive.type     = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT;
        s_directive.target.x = s_intent.target.x + s_act.move_offset.x;
        s_directive.target.y = s_intent.target.y + s_act.move_offset.y;
        s_goal_pt             = s_directive.target;
        s_goal_pt_valid       = true;
        break;
    }

    case TAPESTRY_BSE_INTENT_CONVERGE:
        /* All elements gather at the identical point — deliberately
         * different from MOVE (see above), which preserves formation. */
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

    if (s_intent.type == TAPESTRY_BSE_INTENT_HOLD && s_act.hold_captured) {
        /* Staying is the goal — trivially achieved; duration governs. */
        s_act.achieved = true;
    } else if (s_intent.type == TAPESTRY_BSE_INTENT_DISPERSE) {
        /* DISPERSE has no single goal point — achievement is "spread out",
         * measured as this element's nearest fresh trusted peer being at
         * least spacing (less eps slack) away, sustained for the hold
         * duration.  Vacuously achieved with no peer visible (nothing to
         * disperse from), matching choreo_collective_achieved()'s solo
         * convention. Without this, bse_goal_achieved() was permanently
         * false for DISPERSE, and a script step with
         * advance_on_achieved=true and no timeout would never advance. */
        float    eps  = s_intent.achieve_eps > 0.0f
                        ? s_intent.achieve_eps
                        : TAPESTRY_BSE_ACHIEVE_EPS_DEFAULT;
        uint32_t hold = s_intent.achieve_hold_ms > 0u
                        ? s_intent.achieve_hold_ms
                        : TAPESTRY_BSE_ACHIEVE_HOLD_MS_DEFAULT;

        tapestry_position_t own;
        if (own_position(wm, &own)) {
            float min_dist  = -1.0f;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm->entries[i];
                if (e->is_self || !e->is_active || e->is_stale ||
                    !scr_peer_is_trusted(scr, e->state.id)) {
                    continue;
                }
                float dx = e->state.position.x - own.x;
                float dy = e->state.position.y - own.y;
                float d  = sqrtf(dx * dx + dy * dy);
                if (min_dist < 0.0f || d < min_dist) {
                    min_dist = d;
                }
            }
            bool spread = min_dist < 0.0f
                          || min_dist >= (s_directive.spacing - eps);
            if (spread) {
                s_act.achieve_accum_ms += WM_CYCLE_MS;
            } else {
                s_act.achieve_accum_ms = 0;
            }
            s_act.achieved = s_act.achieve_accum_ms >= hold;
        }
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
                s_act.achieve_accum_ms += WM_CYCLE_MS;
            } else {
                s_act.achieve_accum_ms = 0;
            }
            s_act.achieved = s_act.achieve_accum_ms >= hold;
        }
    } else {
        s_act.achieve_accum_ms = 0;
        s_act.achieved         = false;
    }
}

const tapestry_bse_directive_t *bse_get_directive(void)
{
    return &s_directive;
}

bool bse_goal_achieved(void)
{
    return s_act.achieved;
}
