/*
 * facts.c — Tapestry L4 shared facts
 *
 * The replicated fact table, its merge/rank/eviction rules, the digest and
 * the Trickle timer that paces fact frames.  OS-free like world_model.c;
 * see <tapestry/csm.h>'s "Shared facts" section for the rules, and
 * sdk/python/tapestry/facts.py for the Python mirror (parity-tested).
 */

#include <string.h>
#include <math.h>
#include <tapestry/csm.h>

/* ── Ordering ────────────────────────────────────────────────────────────── */

static bool is_live(const fact_t *f)
{
    return (f->flags & FACT_RETRACTED) == 0u;
}

/* Rank order among live facts: (stamp, origin, epoch, seq) ascending.
 * Returns <0, 0, >0 like strcmp. */
static int rank_cmp(const fact_t *a, const fact_t *b)
{
    if (a->stamp != b->stamp) {
        return a->stamp < b->stamp ? -1 : 1;
    }
    if (a->key.origin != b->key.origin) {
        return a->key.origin < b->key.origin ? -1 : 1;
    }
    if (a->key.epoch != b->key.epoch) {
        return a->key.epoch < b->key.epoch ? -1 : 1;
    }
    if (a->key.seq != b->key.seq) {
        return a->key.seq < b->key.seq ? -1 : 1;
    }
    return 0;
}

/* Eviction order: (live, priority, then rank order) ascending — the
 * smallest is evicted first.  Built only from fields every element holds
 * identically, so the whole swarm forgets the same facts. */
static int evict_cmp(const fact_t *a, const fact_t *b)
{
    int la = is_live(a) ? 1 : 0;
    int lb = is_live(b) ? 1 : 0;
    if (la != lb) {
        return la < lb ? -1 : 1;
    }
    uint8_t pa = FACT_PRIO_OF(a->flags);
    uint8_t pb = FACT_PRIO_OF(b->flags);
    if (pa != pb) {
        return pa < pb ? -1 : 1;
    }
    return rank_cmp(a, b);
}

static bool type_matches(uint8_t type, const fact_t *f)
{
    return type == FACT_TYPE_NONE || f->key.type == type;
}

/* ── Slots ───────────────────────────────────────────────────────────────── */

static wm_fact_slot_t *find_slot(world_model_t *wm, const fact_key_t *key)
{
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        wm_fact_slot_t *s = &wm->facts.slots[i];
        if (s->used && fact_key_equal(&s->fact.key, key)) {
            return s;
        }
    }
    return NULL;
}

static void evict(world_model_t *wm, wm_fact_slot_t *s, uint8_t reason)
{
    wm_facts_t *t = &wm->facts;
    t->evictions++;
    t->last_evicted        = s->fact.key;
    t->last_evict_reason   = reason;
    t->last_evicted_age_ms = s->held_ms;
    t->last_evicted_flags  = s->fact.flags;
    s->used = false;
}

/* Find room for f and store it.  Returns false (f dropped) when f would
 * itself be the entry evicted. */
static bool admit(world_model_t *wm, const fact_t *f)
{
    wm_facts_t *t = &wm->facts;

    if (is_live(f)) {
        int              n_origin = 0;
        wm_fact_slot_t  *oldest   = NULL;
        for (int i = 0; i < WM_MAX_FACTS; i++) {
            wm_fact_slot_t *s = &t->slots[i];
            if (!s->used || !is_live(&s->fact) ||
                s->fact.key.origin != f->key.origin ||
                s->fact.key.epoch != f->key.epoch) {
                continue;
            }
            n_origin++;
            if (oldest == NULL || evict_cmp(&s->fact, &oldest->fact) < 0) {
                oldest = s;
            }
        }
        if (n_origin >= (int)FACT_MAX_PER_ORIGIN && oldest != NULL) {
            if (evict_cmp(f, &oldest->fact) < 0) {
                return false;
            }
            evict(wm, oldest, WM_FACT_EVICT_PER_ORIGIN);
        }
    }

    wm_fact_slot_t *free_slot = NULL;
    wm_fact_slot_t *lowest    = NULL;
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        wm_fact_slot_t *s = &t->slots[i];
        if (!s->used) {
            free_slot = s;
            break;
        }
        if (lowest == NULL || evict_cmp(&s->fact, &lowest->fact) < 0) {
            lowest = s;
        }
    }
    if (free_slot == NULL) {
        if (evict_cmp(f, &lowest->fact) < 0) {
            return false;
        }
        evict(wm, lowest, WM_FACT_EVICT_CAPACITY);
        free_slot = lowest;
    }

    free_slot->fact    = *f;
    free_slot->used    = true;
    free_slot->held_ms = 0;
    free_slot->tomb_ms = 0;
    return true;
}

/* A live fact of f's type, other than f's own key, within the type's merge
 * radius of pos — the lowest-ranked such fact, or NULL. */
static wm_fact_slot_t *nearby_duplicate(world_model_t *wm, uint8_t type,
                                        const fact_key_t *skip,
                                        const position_t *pos)
{
    float r = wm_fact_merge_radius(type);
    if (r <= 0.0f) {
        return NULL;
    }
    wm_fact_slot_t *best = NULL;
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        wm_fact_slot_t *s = &wm->facts.slots[i];
        if (!s->used || !is_live(&s->fact) || s->fact.key.type != type) {
            continue;
        }
        if (skip != NULL && fact_key_equal(&s->fact.key, skip)) {
            continue;
        }
        if (position_distance(&s->fact.pos, pos) > r) {
            continue;
        }
        if (best == NULL || rank_cmp(&s->fact, &best->fact) < 0) {
            best = s;
        }
    }
    return best;
}

/* ── Trickle ─────────────────────────────────────────────────────────────── */

static void trickle_start_interval(wm_facts_t *t, uint32_t i_ms, uint32_t rnd)
{
    uint32_t half = i_ms / 2u;
    uint32_t span = i_ms - half;

    t->trickle_i_ms       = i_ms;
    t->trickle_t_ms       = half + (span > 0u ? rnd % span : 0u);
    t->trickle_elapsed_ms = 0u;
    t->trickle_c          = 0u;
    t->trickle_fired      = false;
    t->trickle_started    = true;
}

/* RFC 6206 "inconsistency": back to FACT_IMIN_MS unless already there.
 * The new interval starts on the next wm_fact_trickle_tick(), which has
 * a random value to pick its send point with. */
static void trickle_reset(wm_facts_t *t)
{
    if (t->trickle_started && t->trickle_i_ms > FACT_IMIN_MS) {
        t->trickle_i_ms    = 0u;   /* marks "restart at IMIN" */
    }
}

/* ── API ─────────────────────────────────────────────────────────────────── */

void wm_facts_init(world_model_t *wm)
{
    memset(&wm->facts, 0, sizeof(wm->facts));
}

void wm_fact_set_epoch(world_model_t *wm, uint16_t epoch)
{
    wm->facts.epoch     = epoch;
    wm->facts.epoch_set = true;
}

float wm_fact_merge_radius(uint8_t type)
{
    return type == FACT_TYPE_TARGET ? FACT_MERGE_RADIUS_TARGET : 0.0f;
}

int wm_fact_publish(world_model_t *wm, uint8_t type, const position_t *pos,
                    uint16_t attr, uint8_t flags, fact_key_t *out)
{
    wm_facts_t *t = &wm->facts;

    if (type == FACT_TYPE_NONE || pos == NULL || wm->owner_id >= MAX_ELEMENTS) {
        return WM_FACT_ERR_INVALID;
    }
    if (!t->epoch_set) {
        return WM_FACT_ERR_NOEPOCH;
    }
    if (!t->synced) {
        return WM_FACT_ERR_UNSYNCED;
    }

    wm_fact_slot_t *dup = nearby_duplicate(wm, type, NULL, pos);
    if (dup != NULL) {
        if (out != NULL) {
            *out = dup->fact.key;
        }
        return WM_FACT_REUSED;
    }

    fact_t f = {
        .key   = { .type = type, .origin = wm->owner_id,
                   .epoch = t->epoch, .seq = t->next_seq },
        .flags = (uint8_t)(flags & (FACT_IMMUTABLE | FACT_PRIO_MASK)),
        .stamp = t->clock + 1u,
        .pos   = *pos,
        .attr  = attr,
    };
    if (!admit(wm, &f)) {
        return WM_FACT_ERR_FULL;
    }
    t->clock = f.stamp;
    t->next_seq++;
    trickle_reset(t);
    if (out != NULL) {
        *out = f.key;
    }
    return WM_FACT_CREATED;
}

int wm_fact_retract(world_model_t *wm, const fact_key_t *key)
{
    wm_fact_slot_t *s = find_slot(wm, key);
    if (s == NULL || !is_live(&s->fact)) {
        return -1;
    }
    wm->facts.clock++;
    s->fact.flags |= FACT_RETRACTED;
    s->fact.stamp  = wm->facts.clock;
    s->tomb_ms     = 0u;
    trickle_reset(&wm->facts);
    return 0;
}

/* The merge rules of wm_fact_receive(); true if the table changed. */
static bool merge_record(world_model_t *wm, const fact_t *fact)
{
    wm_facts_t *t = &wm->facts;

    if (fact->key.type == FACT_TYPE_NONE || fact->key.origin >= MAX_ELEMENTS) {
        return false;
    }
    if (fact->stamp > t->clock) {
        t->clock = fact->stamp;   /* Lamport receive rule */
    }

    wm_fact_slot_t *held = find_slot(wm, &fact->key);
    if (held != NULL) {
        fact_t *h = &held->fact;
        if (!is_live(h)) {
            /* Retraction beats any live version.  Two tombstones of one
             * key keep the earlier, so every element converges on the
             * same (stamp, flags) and digests can match. */
            if (!is_live(fact) && fact->stamp < h->stamp) {
                *h = *fact;
                return true;
            }
            return false;
        }
        if (!is_live(fact)) {
            *h = *fact;
            held->tomb_ms = 0u;
            return true;
        }
        bool immutable = (h->flags & FACT_IMMUTABLE) != 0u;
        int  c         = rank_cmp(fact, h);
        if ((immutable && c < 0) || (!immutable && c > 0)) {
            *h = *fact;
            return true;
        }
        return false;
    }

    if (is_live(fact)) {
        /* Two creators who had not heard each other: the lower (stamp,
         * origin) stands, here and everywhere else. */
        for (;;) {
            wm_fact_slot_t *dup = nearby_duplicate(wm, fact->key.type,
                                                   &fact->key, &fact->pos);
            if (dup == NULL) {
                break;
            }
            if (rank_cmp(fact, &dup->fact) > 0) {
                return false;
            }
            dup->used = false;
        }
    }

    return admit(wm, fact);
}

/* New data is a Trickle inconsistency (RFC 6206): an element that has just
 * learned a fact, or its retraction, restarts at FACT_IMIN_MS and passes it
 * on within a fraction of a second, so a peer that missed the originator's
 * burst hears it from its neighbours instead of waiting out their long
 * intervals (hardware, 2026-10-22: a robot that missed three bursts from a
 * stationary sensor heard the leak ~5 s late).  K still bounds the chatter:
 * a neighbour that hears K consistent frames in an interval stays quiet. */
bool wm_fact_receive(world_model_t *wm, const fact_t *fact)
{
    if (!merge_record(wm, fact)) {
        return false;
    }
    trickle_reset(&wm->facts);
    return true;
}

void wm_fact_heard_digest(world_model_t *wm, uint16_t digest)
{
    wm_facts_t *t = &wm->facts;

    t->synced = true;
    t->frames_heard++;
    if (digest == wm_fact_digest(wm)) {
        if (t->trickle_c < 0xFFu) {
            t->trickle_c++;
        }
    } else {
        trickle_reset(t);
    }
}

void wm_fact_tick(world_model_t *wm, uint32_t elapsed_ms)
{
    wm_facts_t *t = &wm->facts;

    t->since_init_ms += elapsed_ms;
    if (t->since_init_ms >= FACT_SYNC_TIMEOUT_MS) {
        t->synced = true;
    }
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        wm_fact_slot_t *s = &t->slots[i];
        if (!s->used) {
            continue;
        }
        s->held_ms += elapsed_ms;
        if (!is_live(&s->fact)) {
            s->tomb_ms += elapsed_ms;
            if (s->tomb_ms >= FACT_TOMBSTONE_TTL_MS) {
                s->used = false;
            }
        }
    }
}

bool wm_fact_trickle_tick(world_model_t *wm, uint32_t elapsed_ms, uint32_t rnd)
{
    wm_facts_t *t = &wm->facts;

    if (!t->trickle_started || t->trickle_i_ms == 0u) {
        trickle_start_interval(t, FACT_IMIN_MS, rnd);
    }

    t->trickle_elapsed_ms += elapsed_ms;

    bool send = false;
    if (!t->trickle_fired && t->trickle_elapsed_ms >= t->trickle_t_ms) {
        t->trickle_fired = true;
        send = t->trickle_c < FACT_K;
    }
    if (t->trickle_elapsed_ms >= t->trickle_i_ms) {
        uint32_t next = t->trickle_i_ms * 2u;
        trickle_start_interval(t, next > FACT_IMAX_MS ? FACT_IMAX_MS : next, rnd);
    }
    return send;
}

uint8_t wm_fact_pack(world_model_t *wm, fact_t out[2])
{
    wm_facts_t *t = &wm->facts;
    uint8_t     n = 0;

    for (int k = 0; k < WM_MAX_FACTS && n < 2u; k++) {
        uint8_t i = (uint8_t)((t->cursor + k) % WM_MAX_FACTS);
        if (t->slots[i].used) {
            out[n++] = t->slots[i].fact;
            if (n == 2u) {
                t->cursor = (uint8_t)((i + 1u) % WM_MAX_FACTS);
            }
        }
    }
    if (n == 1u) {
        t->cursor = 0u;   /* one record held: nothing to rotate */
    }
    return n;
}

/* FNV-1a over the record's identity and version, folded to 16 bits.  The
 * byte order is fixed (little-endian) so the Python mirror computes the
 * same value. */
static uint16_t record_hash(const fact_t *f)
{
    uint8_t b[10] = {
        f->key.type, f->key.origin,
        (uint8_t)(f->key.epoch & 0xFFu), (uint8_t)(f->key.epoch >> 8),
        f->key.seq, f->flags,
        (uint8_t)(f->stamp & 0xFFu), (uint8_t)((f->stamp >> 8) & 0xFFu),
        (uint8_t)((f->stamp >> 16) & 0xFFu), (uint8_t)(f->stamp >> 24),
    };
    uint32_t h = 2166136261u;
    for (int i = 0; i < 10; i++) {
        h ^= b[i];
        h *= 16777619u;
    }
    return (uint16_t)((h ^ (h >> 16)) & 0xFFFFu);
}

uint16_t wm_fact_digest(const world_model_t *wm)
{
    uint16_t d = 0;
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        const wm_fact_slot_t *s = &wm->facts.slots[i];
        if (s->used) {
            d = (uint16_t)(d + record_hash(&s->fact));
        }
    }
    return d;
}

bool wm_fact_get(const world_model_t *wm, const fact_key_t *key, fact_t *out)
{
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        const wm_fact_slot_t *s = &wm->facts.slots[i];
        if (s->used && fact_key_equal(&s->fact.key, key)) {
            if (out != NULL) {
                *out = s->fact;
            }
            return true;
        }
    }
    return false;
}

bool wm_fact_find(const world_model_t *wm, uint8_t type, int rank, fact_t *out)
{
    if (rank < 0) {
        return false;
    }
    /* Selection by repeated minimum — WM_MAX_FACTS is small. */
    const fact_t *prev = NULL;
    for (int r = 0; r <= rank; r++) {
        const fact_t *best = NULL;
        for (int i = 0; i < WM_MAX_FACTS; i++) {
            const wm_fact_slot_t *s = &wm->facts.slots[i];
            if (!s->used || !is_live(&s->fact) || !type_matches(type, &s->fact)) {
                continue;
            }
            if (prev != NULL && rank_cmp(&s->fact, prev) <= 0) {
                continue;
            }
            if (best == NULL || rank_cmp(&s->fact, best) < 0) {
                best = &s->fact;
            }
        }
        if (best == NULL) {
            return false;
        }
        prev = best;
    }
    if (out != NULL) {
        *out = *prev;
    }
    return true;
}

int wm_fact_count(const world_model_t *wm, uint8_t type)
{
    int n = 0;
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        const wm_fact_slot_t *s = &wm->facts.slots[i];
        if (s->used && is_live(&s->fact) && type_matches(type, &s->fact)) {
            n++;
        }
    }
    return n;
}

int wm_fact_tombstones(const world_model_t *wm, uint8_t type)
{
    int n = 0;
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        const wm_fact_slot_t *s = &wm->facts.slots[i];
        if (s->used && !is_live(&s->fact) && type_matches(type, &s->fact)) {
            n++;
        }
    }
    return n;
}
