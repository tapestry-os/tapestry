/*
 * scr.c — Tapestry L5 Swarm Coordination Runtime
 *
 * Pure C99. No OS dependencies. No dynamic allocation.
 * All state fits in a caller-allocated scr_state_t.
 */

#include "scr.h"
#include <string.h>

/* ── Internal helpers ─────────────────────────────────────────────────────── */

/*
 * peer_is_trusted — Returns true if peer_id passes whitelist and anomaly
 * checks.  Self is always trusted and should never be passed here.
 */
static bool peer_is_trusted(const scr_state_t *scr, element_id_t id)
{
    if (id >= 32u) {
        return false;
    }
    if (scr->peer_whitelist_mask != 0u &&
        !(scr->peer_whitelist_mask & (1u << id))) {
        return false;
    }
    if (scr->anomaly_mask & (1u << id)) {
        return false;
    }
    return true;
}

/*
 * insertion_sort_ids — Sort element_id_t array in-place, ascending.
 * N is bounded by MAX_ELEMENTS (32); insertion sort is appropriate.
 */
static void insertion_sort_ids(element_id_t *arr, uint8_t n)
{
    for (uint8_t i = 1; i < n; i++) {
        element_id_t key = arr[i];
        int j = (int)i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void scr_init(scr_state_t *scr,
              element_id_t own_id,
              uint8_t quorum_min,
              uint8_t quorum_target,
              scr_capability_t capabilities)
{
    memset(scr, 0, sizeof(*scr));
    scr->own_id             = own_id;
    scr->quorum_min         = quorum_min;
    scr->quorum_target      = quorum_target;
    scr->capabilities       = capabilities;
    scr->role               = SCR_ROLE_NONE;
    scr->leader_id          = ELEMENT_ID_INVALID;
    scr->leader_valid       = false;
    scr->quorum_state       = SCR_QUORUM_LOST;
    scr->fresh_count        = 0;
    scr->task_slot          = 0;
    scr->swarm_size         = 0;
    scr->abort_state        = SCR_ABORT_NONE;
    scr->_prev_quorum_state = SCR_QUORUM_LOST;
    scr->peer_whitelist_mask = 0;
    scr->anomaly_mask        = 0;
}

void scr_tick(scr_state_t *scr, const world_model_t *wm)
{
    /*
     * ── Step 1: Collect trusted fresh non-self peers ───────────────────
     *
     * A peer counts toward quorum only if its world model entry is active
     * (not expired), not stale, and passes the whitelist + anomaly checks.
     *
     * Self is always included as a candidate in the election but is NOT
     * counted toward fresh_count (which measures external reachability).
     */
    element_id_t candidates[MAX_ELEMENTS];
    uint8_t      n_candidates = 0;
    uint8_t      fresh_count  = 0;

    candidates[n_candidates++] = scr->own_id;

    for (element_id_t i = 0; i < MAX_ELEMENTS; i++) {
        if (i == scr->own_id) {
            continue;
        }
        if (!peer_is_trusted(scr, i)) {
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
    scr_quorum_state_t new_quorum;
    if (fresh_count >= scr->quorum_target) {
        new_quorum = SCR_QUORUM_HEALTHY;
    } else if (fresh_count >= scr->quorum_min) {
        new_quorum = SCR_QUORUM_DEGRADED;
    } else {
        new_quorum = SCR_QUORUM_LOST;
    }

    /*
     * ── Step 3: Abort state machine ──────────────────────────────────────
     *
     * Two transition types are reported as one-tick signals:
     *
     *   DEGRADED/HEALTHY → LOST  : SCR_ABORT_TRIGGERED
     *     Held until quorum recovers.  Signals to L6 to halt motion.
     *
     *   LOST → DEGRADED/HEALTHY  : SCR_ABORT_CLEARED (one-tick pulse)
     *     Only fires after a prior TRIGGERED event.  Signals recovery.
     *
     * On startup (prev = LOST, new = LOST) neither signal fires, so
     * normal startup quorum acquisition does not produce false events.
     */
    if (scr->abort_state == SCR_ABORT_CLEARED) {
        /* CLEARED was held for one tick; reset before re-evaluating */
        scr->abort_state = SCR_ABORT_NONE;
    }
    if (scr->_prev_quorum_state >= SCR_QUORUM_DEGRADED &&
        new_quorum == SCR_QUORUM_LOST) {
        scr->abort_state = SCR_ABORT_TRIGGERED;
    } else if (scr->abort_state == SCR_ABORT_TRIGGERED &&
               scr->_prev_quorum_state == SCR_QUORUM_LOST &&
               new_quorum >= SCR_QUORUM_DEGRADED) {
        scr->abort_state = SCR_ABORT_CLEARED;
    }

    scr->_prev_quorum_state = new_quorum;
    scr->quorum_state       = new_quorum;

    /*
     * ── Step 4: Early return if LOST ──────────────────────────────────────
     */
    if (new_quorum == SCR_QUORUM_LOST) {
        scr->leader_id    = ELEMENT_ID_INVALID;
        scr->leader_valid = false;
        scr->role         = SCR_ROLE_NONE;
        scr->task_slot    = 0;
        scr->swarm_size   = 0;
        return;
    }

    /*
     * ── Step 5: Sort candidate set ──────────────────────────────────────
     *
     * Insertion sort over self + fresh peers.  After sorting:
     *   candidates[0]  = lowest ID = elected leader
     *   own task_slot  = position of own_id in sorted array (0 = leader)
     *
     * All elements compute the same sorted order from their converged
     * world models without extra messaging.
     */
    insertion_sort_ids(candidates, n_candidates);

    uint8_t task_slot = 0;
    for (uint8_t i = 0; i < n_candidates; i++) {
        if (candidates[i] == scr->own_id) {
            task_slot = i;
            break;
        }
    }
    scr->task_slot  = task_slot;
    scr->swarm_size = n_candidates;

    /*
     * ── Step 6: Elect leader ─────────────────────────────────────────────
     *
     * candidates[0] is the minimum ID after sorting.  All elements compute
     * the same leader from their (converged) world model snapshots.
     *
     * Convergence time: bounded by WM_STALE_THRESHOLD_MS.  If the leader
     * goes stale, the next scr_tick() elects a new leader from the
     * remaining fresh set.
     */
    scr->leader_id    = candidates[0];
    scr->leader_valid = true;

    /*
     * ── Step 7: Role assignment ──────────────────────────────────────────
     *
     * Leader retains SCR_ROLE_LEADER regardless of capability flags.
     * Followers self-assign an extended role from their capability bits:
     *   RELAY > SENSOR > ACTUATOR > FOLLOWER (generic fallback).
     * No messaging required — each element derives its own role independently.
     */
    if (scr->own_id == scr->leader_id) {
        scr->role = SCR_ROLE_LEADER;
    } else if (scr->capabilities & SCR_CAP_RELAY) {
        scr->role = SCR_ROLE_RELAY;
    } else if (scr->capabilities & SCR_CAP_SENSOR) {
        scr->role = SCR_ROLE_SENSOR;
    } else if (scr->capabilities & SCR_CAP_ACTUATOR) {
        scr->role = SCR_ROLE_ACTUATOR;
    } else {
        scr->role = SCR_ROLE_FOLLOWER;
    }
}

/* ── Accessors ───────────────────────────────────────────────────────────── */

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

uint8_t scr_get_task_slot(const scr_state_t *scr)
{
    return scr->task_slot;
}

uint8_t scr_get_swarm_size(const scr_state_t *scr)
{
    return scr->swarm_size;
}

scr_abort_state_t scr_get_abort_state(const scr_state_t *scr)
{
    return scr->abort_state;
}

/* ── BFT peer filtering ──────────────────────────────────────────────────── */

void scr_set_peer_whitelist(scr_state_t *scr, uint32_t mask)
{
    scr->peer_whitelist_mask = mask;
}

void scr_report_anomaly(scr_state_t *scr, element_id_t peer_id)
{
    if (peer_id < 32u) {
        scr->anomaly_mask |= (1u << peer_id);
    }
}

void scr_clear_anomaly(scr_state_t *scr, element_id_t peer_id)
{
    if (peer_id < 32u) {
        scr->anomaly_mask &= ~(1u << peer_id);
    }
}

void scr_clear_all_anomalies(scr_state_t *scr)
{
    scr->anomaly_mask = 0;
}
