/*
 * scr.c — Tapestry L5 Swarm Coordination Runtime
 *
 * Pure C99. No OS dependencies. No dynamic allocation.
 * All state fits in a caller-allocated scr_state_t.
 */

#include "scr.h"
#include <string.h>

/* ── Public API ──────────────────────────────────────────────────────────── */

void scr_init(scr_state_t *scr,
              element_id_t own_id,
              uint8_t quorum_min,
              uint8_t quorum_target)
{
    memset(scr, 0, sizeof(*scr));
    scr->own_id        = own_id;
    scr->quorum_min    = quorum_min;
    scr->quorum_target = quorum_target;
    scr->role          = SCR_ROLE_NONE;
    scr->leader_id     = ELEMENT_ID_INVALID;
    scr->leader_valid  = false;
    scr->quorum_state  = SCR_QUORUM_LOST;
    scr->fresh_count   = 0;
}

void scr_tick(scr_state_t *scr, const world_model_t *wm)
{
    /*
     * ── Step 1: Collect fresh non-self peers ────────────────────────────
     *
     * A peer counts toward quorum only if its world model entry is active
     * (not expired) AND not stale.  Stale entries mean we have not heard
     * from that peer within WM_STALE_THRESHOLD_MS — it may be partitioned.
     *
     * Self is always included as a candidate in the election but is NOT
     * counted toward fresh_count (which measures external reachability).
     */
    element_id_t candidates[MAX_ELEMENTS];
    uint8_t      n_candidates = 0;
    uint8_t      fresh_count  = 0;

    /* Self always participates in the leader election. */
    candidates[n_candidates++] = scr->own_id;

    for (element_id_t i = 0; i < MAX_ELEMENTS; i++) {
        if (i == scr->own_id) {
            continue;
        }
        const wm_entry_t *e = wm_get_entry(wm, i);
        if (e == NULL || !e->is_active || e->is_stale) {
            continue;
        }
        candidates[n_candidates++] = i;
        fresh_count++;
    }

    scr->fresh_count = fresh_count;

    /*
     * ── Step 2: Classify quorum ─────────────────────────────────────────
     *
     * Thresholds are peer counts (not fractions) — the L5 caller has
     * domain knowledge about expected swarm size that L4 does not.
     */
    if (fresh_count >= scr->quorum_target) {
        scr->quorum_state = SCR_QUORUM_HEALTHY;
    } else if (fresh_count >= scr->quorum_min) {
        scr->quorum_state = SCR_QUORUM_DEGRADED;
    } else {
        scr->quorum_state = SCR_QUORUM_LOST;
        scr->leader_id    = ELEMENT_ID_INVALID;
        scr->leader_valid = false;
        scr->role         = SCR_ROLE_NONE;
        return;
    }

    /*
     * ── Step 3: Elect leader ─────────────────────────────────────────────
     *
     * Deterministic lowest-ID election over the fresh candidate set
     * (self + fresh peers).  All elements compute the same result from
     * their (converged) world model without extra messaging.
     *
     * Property: if two elements A and B are both fresh to each other,
     * they will independently elect the same leader (the one with the
     * lower ID appears in both candidate sets).
     *
     * Convergence time: bounded by WM_STALE_THRESHOLD_MS.  If a leader
     * goes stale (lost), the next scr_tick() automatically elects a new
     * leader from the remaining fresh set.
     */
    element_id_t leader = candidates[0];
    for (uint8_t i = 1; i < n_candidates; i++) {
        if (candidates[i] < leader) {
            leader = candidates[i];
        }
    }

    scr->leader_id    = leader;
    scr->leader_valid = true;
    scr->role = (scr->own_id == leader) ? SCR_ROLE_LEADER : SCR_ROLE_FOLLOWER;
}

scr_role_t scr_get_role(const scr_state_t *scr)
{
    return scr->role;
}

element_id_t scr_get_leader(const scr_state_t *scr)
{
    return scr->leader_valid ? scr->leader_id : ELEMENT_ID_INVALID;
}

scr_quorum_state_t scr_get_quorum(const scr_state_t *scr)
{
    return scr->quorum_state;
}
