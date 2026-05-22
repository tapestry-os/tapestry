/*
 * world_model.c — Tapestry L4 Simulation
 *
 * Implementation of the Collective State Manager (CSM).
 * See <tapestry/csm.h> for full API documentation and design rationale.
 */

#include <string.h>
#include <math.h>
#include <tapestry/csm.h>

/* ── Internal helpers ────────────────────────────────────────────────────── */

/*
 * recompute_metric — Scan all non-self entries and derive fresh/stale/inactive
 * counts, fresh_ratio, quorum status, and CP freeze flag.
 * Called at the end of wm_tick and wm_reconcile.
 */
static void recompute_metric(world_model_t *wm)
{
    wm_consistency_metric_t *m = &wm->metric;

    uint8_t active_total   = 0;
    uint8_t active_fresh   = 0;
    uint8_t active_stale   = 0;
    uint8_t inactive_total = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];

        /* Skip unpopulated slots and own entry */
        if (e->state.id == ELEMENT_ID_INVALID || e->is_self) {
            continue;
        }

        if (e->is_active) {
            active_total++;
            if (e->is_stale) {
                active_stale++;
            } else {
                active_fresh++;
            }
        } else {
            inactive_total++;
        }
    }

    m->active_total   = active_total;
    m->active_fresh   = active_fresh;
    m->active_stale   = active_stale;
    m->inactive_total = inactive_total;
    /* collision_count is set by wm_check_collisions — not touched here */

    float quorum_threshold = wm->consistency_bias * WM_QUORUM_FRACTION;

    if (active_total > 0) {
        m->fresh_ratio = (float)active_fresh / (float)active_total;
        m->quorum_held = (m->fresh_ratio >= quorum_threshold);
        if (quorum_threshold > 0.0f) {
            m->confidence = m->fresh_ratio / quorum_threshold;
            if (m->confidence > 1.0f) { m->confidence = 1.0f; }
        } else {
            m->confidence = 1.0f;   /* bias=0.0: pure AP, always confident */
        }
    } else {
        /* No peers known — no partition to detect */
        m->fresh_ratio = 1.0f;
        m->quorum_held = true;
        m->confidence  = 1.0f;
    }

    m->degraded = !m->quorum_held;
}

/* ── API implementation ──────────────────────────────────────────────────── */

void wm_init(world_model_t *wm,
             element_id_t owner_id,
             const element_state_t *own_state,
             float consistency_bias)
{
    memset(wm, 0, sizeof(*wm));

    /* Mark every slot as unpopulated before touching any of them */
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        wm->entries[i].state.id = ELEMENT_ID_INVALID;
    }

    wm->owner_id          = owner_id;
    wm->consistency_bias  = consistency_bias;

    /* Own entry is always authoritative and always active */
    wm_entry_t *self = &wm->entries[owner_id];
    self->state        = *own_state;
    self->age_ms       = 0;
    self->is_active    = true;
    self->is_stale     = false;
    self->is_self      = true;
    self->update_count = 0;

    wm->known_count = 1;

    recompute_metric(wm);
}

void wm_update_self(world_model_t *wm, element_state_t *own_state)
{
    wm_entry_t *self = &wm->entries[wm->owner_id];

    self->state        = *own_state;
    self->state.logical_clock++;   /* Lamport: increment on every local event */
    self->age_ms       = 0;
    self->is_active    = true;
    self->is_stale     = false;
    self->update_count++;

    /* Write clock back so caller's copy stays in sync with the WM entry. */
    own_state->logical_clock = self->state.logical_clock;
}

bool wm_receive_gossip(world_model_t *wm, const element_state_t *received)
{
    element_id_t id = received->id;

    if (id >= MAX_ELEMENTS || id == ELEMENT_ID_INVALID) {
        return false;
    }

    /* Never let gossip overwrite the owner's own authoritative entry */
    if (id == wm->owner_id) {
        return false;
    }

    wm_entry_t *e        = &wm->entries[id];
    bool        is_new      = (e->state.id == ELEMENT_ID_INVALID);
    bool        is_inactive = !is_new && !e->is_active;

    /* Reject if received clock is not strictly newer than what we hold.
     * Skip for inactive entries: the peer may have rebooted and reset its
     * clock — accept any clock when the entry was presumed dead. */
    if (!is_new && !is_inactive && received->logical_clock <= e->state.logical_clock) {
        return false;
    }

    /*
     * Lamport receive rule: merged_clock = max(local, received) + 1.
     * For inactive entries use local=0: the pre-reboot clock is from a dead
     * process instance and must not block subsequent post-reboot gossip.
     */
    uint32_t local_clock  = (is_new || is_inactive) ? 0 : e->state.logical_clock;
    uint32_t merged_clock = (local_clock > received->logical_clock
                             ? local_clock
                             : received->logical_clock) + 1;

    if (is_new) {
        wm->known_count++;
    }

    e->state               = *received;
    e->state.logical_clock = merged_clock;
    e->age_ms              = 0;
    e->is_active           = true;
    e->is_stale            = false;
    e->is_self             = false;
    e->update_count++;

    return true;
}

void wm_tick(world_model_t *wm, uint32_t elapsed_ms)
{
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        wm_entry_t *e = &wm->entries[i];

        /* Skip unpopulated slots and own entry (self never expires) */
        if (e->state.id == ELEMENT_ID_INVALID || e->is_self) {
            continue;
        }

        /* Don't age entries that are already inactive */
        if (!e->is_active) {
            continue;
        }

        e->age_ms += elapsed_ms;

        if (e->age_ms >= WM_EXPIRE_THRESHOLD_MS) {
            e->is_active = false;
            e->is_stale  = true;
        } else if (e->age_ms >= WM_STALE_THRESHOLD_MS) {
            e->is_stale = true;
        }
    }

    wm->cycle_count++;

    /* Reset per-cycle collision count before caller runs wm_check_collisions */
    wm->metric.collision_count = 0;

    recompute_metric(wm);
}

const wm_entry_t *wm_get_entry(const world_model_t *wm, element_id_t id)
{
    if (id >= MAX_ELEMENTS) {
        return NULL;
    }

    const wm_entry_t *e = &wm->entries[id];

    if (e->state.id == ELEMENT_ID_INVALID) {
        return NULL;   /* This ID has never been seen */
    }

    return e;
}

const wm_consistency_metric_t *wm_get_metric(const world_model_t *wm)
{
    return &wm->metric;
}

/*
 * Temporary pair used only inside wm_nearest_elements for sorting.
 */
typedef struct {
    element_id_t id;
    float        dist;
} near_candidate_t;

uint8_t wm_nearest_elements(const world_model_t *wm,
                             const position_t *pos,
                             float radius,
                             element_id_t *out_ids,
                             uint8_t max_count)
{
    near_candidate_t candidates[MAX_ELEMENTS];
    uint8_t count = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];

        /* Skip unpopulated, inactive, and self entries */
        if (e->state.id == ELEMENT_ID_INVALID || !e->is_active || e->is_self) {
            continue;
        }

        float d = position_distance(pos, &e->state.position);
        if (d <= radius) {
            candidates[count].id   = e->state.id;
            candidates[count].dist = d;
            count++;
        }
    }

    /* Insertion sort by ascending distance — MAX_ELEMENTS is 32, cost is trivial */
    for (int i = 1; i < (int)count; i++) {
        near_candidate_t key = candidates[i];
        int j = i - 1;
        while (j >= 0 && candidates[j].dist > key.dist) {
            candidates[j + 1] = candidates[j];
            j--;
        }
        candidates[j + 1] = key;
    }

    uint8_t out_count = (count < max_count) ? count : max_count;
    for (int i = 0; i < out_count; i++) {
        out_ids[i] = candidates[i].id;
    }

    return out_count;
}

uint8_t wm_check_collisions(const world_model_t *wm,
                             const position_t *own_pos,
                             collision_event_t *out_events,
                             uint8_t max_events)
{
    uint8_t count = 0;
    uint32_t own_clock = wm->entries[wm->owner_id].state.logical_clock;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        if (count >= max_events) {
            break;
        }

        const wm_entry_t *e = &wm->entries[i];

        /* Skip unpopulated, self, and inactive entries */
        if (e->state.id == ELEMENT_ID_INVALID || e->is_self || !e->is_active) {
            continue;
        }

        float d = position_distance(own_pos, &e->state.position);
        if (d < MIN_SEPARATION) {
            out_events[count].element_a     = wm->owner_id;
            out_events[count].element_b     = e->state.id;
            out_events[count].distance      = d;
            out_events[count].logical_clock = own_clock;
            count++;
        }
    }

    /*
     * Update metric.collision_count. The metric pointer is logically mutable
     * telemetry state even though the world model is otherwise read-only here.
     */
    ((world_model_t *)wm)->metric.collision_count = count;

    return count;
}

void wm_reconcile(world_model_t *wm,
                  const wm_entry_t *received_entries,
                  uint8_t entry_count,
                  uint32_t now_ms)
{
    (void)now_ms;   /* Reserved for future wall-clock duration tracking */

    for (int i = 0; i < entry_count; i++) {
        const wm_entry_t *recv = &received_entries[i];
        element_id_t id = recv->state.id;

        if (id >= MAX_ELEMENTS || id == ELEMENT_ID_INVALID) {
            continue;
        }

        /* Never overwrite own authoritative state during reconciliation */
        if (id == wm->owner_id) {
            continue;
        }

        wm_entry_t *e    = &wm->entries[id];
        bool        is_new = (e->state.id == ELEMENT_ID_INVALID);

        /* Keep whichever version has the higher logical clock */
        if (is_new || recv->state.logical_clock > e->state.logical_clock) {
            if (is_new) {
                wm->known_count++;
            }
            *e         = *recv;
            e->is_self = false;
        }
    }

    wm->last_reconcile_cycle  = wm->cycle_count;
    wm->reconcile_duration_ms = 0;   /* Caller sets this after timing the heal */

    recompute_metric(wm);
}

uint8_t wm_snapshot(const world_model_t *wm,
                    element_state_t *out_states,
                    uint8_t max_count)
{
    uint8_t count = 0;

    for (int i = 0; i < MAX_ELEMENTS && count < max_count; i++) {
        const wm_entry_t *e = &wm->entries[i];

        if (e->state.id == ELEMENT_ID_INVALID || !e->is_active) {
            continue;
        }

        out_states[count++] = e->state;
    }

    return count;
}
