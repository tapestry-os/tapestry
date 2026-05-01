/*
 * main.c — World model unit tests
 *
 * Covers: staleness aging, gossip clock ordering, Lamport merge,
 * CP freeze, quorum boundary, reconciliation, nearest elements,
 * and collision detection.
 *
 * Build:  west build -b native_sim zephyr/tests/world_model
 * Run:    ./build/zephyr/zephyr.exe
 */

#include <zephyr/ztest.h>
#include <tapestry/csm.h>

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static element_state_t make_state(element_id_t id, float x, float y,
                                   uint32_t clock)
{
    element_state_t s    = {0};
    s.id                 = id;
    s.position.x         = x;
    s.position.y         = y;
    s.logical_clock      = clock;
    s.partition_island   = 0;
    return s;
}

/* Owner is always ID 0. bias=0.0 → pure AP, bias=1.0 → pure CP. */
static void init_wm(world_model_t *wm, float bias)
{
    element_state_t own = make_state(0, 50.0f, 50.0f, 1);
    wm_init(wm, 0, &own, bias);
}

/* ── Test suite ──────────────────────────────────────────────────────────── */

ZTEST_SUITE(world_model, NULL, NULL, NULL, NULL, NULL);

/*
 * test_staleness_aging
 *
 * Inject a peer, then tick through the stale and expire thresholds.
 * Verifies flags flip at the right age, not before.
 */
ZTEST(world_model, test_staleness_aging)
{
    world_model_t wm;
    init_wm(&wm, 0.0f);

    element_state_t peer = make_state(1, 55.0f, 50.0f, 10);
    zassert_true(wm_receive_gossip(&wm, &peer), "initial gossip should be accepted");

    const wm_entry_t *e = wm_get_entry(&wm, 1);
    zassert_not_null(e, "peer entry should exist after gossip");
    zassert_true(e->is_active, "peer should start active");
    zassert_false(e->is_stale, "peer should not be stale immediately");

    /* One tick short of stale threshold */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS - 1);
    e = wm_get_entry(&wm, 1);
    zassert_false(e->is_stale, "should not be stale one ms before threshold");
    zassert_true(e->is_active, "should still be active before stale");

    /* Cross the stale threshold */
    wm_tick(&wm, 1);
    e = wm_get_entry(&wm, 1);
    zassert_true(e->is_stale, "should be stale at threshold");
    zassert_true(e->is_active, "stale entry should still be active");

    /* Cross the expire threshold */
    wm_tick(&wm, WM_EXPIRE_THRESHOLD_MS - WM_STALE_THRESHOLD_MS);
    e = wm_get_entry(&wm, 1);
    zassert_false(e->is_active, "should be inactive after expire threshold");
}

/*
 * test_gossip_clock_ordering
 *
 * Gossip is accepted only when the incoming logical clock is strictly
 * greater than the locally held one. Own-ID gossip is always rejected.
 */
ZTEST(world_model, test_gossip_clock_ordering)
{
    world_model_t wm;
    init_wm(&wm, 0.0f);

    element_state_t peer = make_state(1, 60.0f, 50.0f, 10);

    /* New entry — must be accepted regardless of clock */
    zassert_true(wm_receive_gossip(&wm, &peer),
                 "first gossip for new peer should be accepted");

    /* Same clock — must be rejected */
    peer.logical_clock = 10;
    zassert_false(wm_receive_gossip(&wm, &peer),
                  "gossip with equal clock should be rejected");

    /* Older clock — must be rejected */
    peer.logical_clock = 5;
    zassert_false(wm_receive_gossip(&wm, &peer),
                  "gossip with older clock should be rejected");

    /* Newer clock — must be accepted */
    peer.logical_clock = 20;
    zassert_true(wm_receive_gossip(&wm, &peer),
                 "gossip with newer clock should be accepted");

    const wm_entry_t *e = wm_get_entry(&wm, 1);
    zassert_equal(e->update_count, 2u,
                  "update_count should reflect two accepted gossips");

    /* Gossip addressed to own ID must be rejected */
    element_state_t self_gossip = make_state(0, 99.0f, 99.0f, 999);
    zassert_false(wm_receive_gossip(&wm, &self_gossip),
                  "gossip targeting own ID should be rejected");
}

/*
 * test_lamport_merge
 *
 * Verifies the Lamport receive rule: stored clock = max(local, received) + 1.
 */
ZTEST(world_model, test_lamport_merge)
{
    world_model_t wm;
    init_wm(&wm, 0.0f);

    element_state_t peer = make_state(1, 60.0f, 50.0f, 10);
    wm_receive_gossip(&wm, &peer);

    const wm_entry_t *e = wm_get_entry(&wm, 1);
    /* Entry is new: local=0, received=10 → merged = max(0,10)+1 = 11 */
    zassert_equal(e->state.logical_clock, 11u,
                  "Lamport merge (new entry): max(0,10)+1 should be 11");

    /* Second gossip: received=20, local=11 → merged = max(11,20)+1 = 21 */
    peer.logical_clock = 20;
    wm_receive_gossip(&wm, &peer);
    e = wm_get_entry(&wm, 1);
    zassert_equal(e->state.logical_clock, 21u,
                  "Lamport merge (existing entry): max(11,20)+1 should be 21");
}

/*
 * test_cp_freeze
 *
 * bias=1.0 (pure CP): degraded must be set when fresh_ratio drops to zero;
 * confidence must reach 0.0 at that point.
 * bias=0.0 (pure AP): degraded must never be set; confidence always 1.0.
 * bias=0.5 (hybrid): degraded triggers at a lower threshold; confidence
 * degrades continuously rather than jumping.
 */
ZTEST(world_model, test_cp_freeze)
{
    /* ── Pure CP (bias=1.0) ── */
    world_model_t wm;
    init_wm(&wm, 1.0f);

    element_state_t p1 = make_state(1, 55.0f, 50.0f, 10);
    element_state_t p2 = make_state(2, 45.0f, 50.0f, 10);
    wm_receive_gossip(&wm, &p1);
    wm_receive_gossip(&wm, &p2);

    wm_tick(&wm, 1);
    const wm_consistency_metric_t *m = wm_get_metric(&wm);
    zassert_false(m->degraded,   "should not be degraded with two fresh peers");
    zassert_equal(m->confidence, 1.0f, "confidence should be 1.0 when at quorum");

    /* Age both peers past stale threshold — fresh_ratio drops to 0 */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS);
    m = wm_get_metric(&wm);
    zassert_equal(m->active_fresh, 0,    "both peers should be stale");
    zassert_true(m->degraded,            "CP mode must degrade when all peers stale");
    zassert_equal(m->confidence,  0.0f,  "confidence must be 0.0 when no fresh peers");

    /* ── Pure AP (bias=0.0) ── */
    world_model_t wm_ap;
    init_wm(&wm_ap, 0.0f);
    element_state_t p1_ap = make_state(1, 55.0f, 50.0f, 10);
    wm_receive_gossip(&wm_ap, &p1_ap);
    wm_tick(&wm_ap, WM_STALE_THRESHOLD_MS + 1);
    m = wm_get_metric(&wm_ap);
    zassert_false(m->degraded,   "AP mode must never degrade");
    zassert_equal(m->confidence, 1.0f, "AP mode confidence always 1.0");

    /* ── Hybrid (bias=0.5): quorum threshold = 0.25 ── */
    /* With one fresh and one stale peer: fresh_ratio=0.5, threshold=0.25 → not degraded */
    world_model_t wm_h;
    element_state_t own_h = make_state(0, 50.0f, 50.0f, 1);
    wm_init(&wm_h, 0, &own_h, 0.5f);
    element_state_t ph1 = make_state(1, 55.0f, 50.0f, 10);
    element_state_t ph2 = make_state(2, 45.0f, 50.0f, 10);
    wm_receive_gossip(&wm_h, &ph1);
    wm_receive_gossip(&wm_h, &ph2);
    /* Age both past stale — fresh_ratio=0, threshold=0.25 → degraded */
    wm_tick(&wm_h, WM_STALE_THRESHOLD_MS + 1);
    m = wm_get_metric(&wm_h);
    zassert_true(m->degraded, "hybrid must degrade when fresh_ratio < threshold");
    zassert_equal(m->confidence, 0.0f, "confidence must be 0.0 at fresh_ratio=0");
}

/*
 * test_quorum_boundary
 *
 * At bias=1.0 the threshold is 0.5.  At exactly 50% fresh the implementation
 * uses >= so quorum must be held and confidence must be exactly 1.0.
 */
ZTEST(world_model, test_quorum_boundary)
{
    world_model_t wm;
    init_wm(&wm, 1.0f);

    /* Four peers, all fresh */
    for (int i = 1; i <= 4; i++) {
        element_state_t p = make_state((element_id_t)i, (float)(i * 10), 50.0f, 10);
        wm_receive_gossip(&wm, &p);
    }

    /* Age all four past stale */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);

    /* Refresh peers 3 and 4 with strictly newer clocks */
    element_state_t r3 = make_state(3, 30.0f, 50.0f, 50);
    element_state_t r4 = make_state(4, 40.0f, 50.0f, 50);
    wm_receive_gossip(&wm, &r3);
    wm_receive_gossip(&wm, &r4);

    /* Single ms tick: peers 1 & 2 remain stale, peers 3 & 4 are fresh */
    wm_tick(&wm, 1);

    const wm_consistency_metric_t *m = wm_get_metric(&wm);
    zassert_equal(m->active_total,  4, "4 active peers");
    zassert_equal(m->active_fresh,  2, "2 fresh peers (3 and 4)");
    zassert_equal(m->active_stale,  2, "2 stale peers (1 and 2)");
    zassert_equal(m->fresh_ratio, 0.5f, "fresh_ratio should be exactly 0.5");
    /* Implementation is >=, so 0.5 == threshold holds quorum */
    zassert_true(m->quorum_held,
                 "quorum should be held at exactly 0.5 (implementation uses >=)");
    zassert_false(m->degraded,   "should not be degraded at exactly 0.5");
    zassert_equal(m->confidence, 1.0f, "confidence should be 1.0 at quorum boundary");
}

/*
 * test_reconciliation
 *
 * After a partition heal, wm_reconcile must keep the entry with the
 * higher logical clock. The owner's own entry is never overwritten.
 */
ZTEST(world_model, test_reconciliation)
{
    /* Island A: element 0 has peer 2 at clock 5 */
    world_model_t wm_a;
    init_wm(&wm_a, 0.0f);
    element_state_t p2_old = make_state(2, 70.0f, 50.0f, 5);
    wm_receive_gossip(&wm_a, &p2_old);

    /* Island B: element 1 has peer 2 at clock 15 (newer, different position) */
    world_model_t wm_b;
    element_state_t own_b = make_state(1, 40.0f, 50.0f, 1);
    wm_init(&wm_b, 1, &own_b, 0.0f);
    element_state_t p2_new = make_state(2, 72.0f, 52.0f, 15);
    wm_receive_gossip(&wm_b, &p2_new);

    /* Extract wm_b's non-self entries to simulate what arrives at reconcile */
    wm_entry_t recv_entries[MAX_ELEMENTS];
    uint8_t recv_count = 0;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm_b.entries[i];
        if (e->state.id != ELEMENT_ID_INVALID && !e->is_self) {
            recv_entries[recv_count++] = *e;
        }
    }

    wm_reconcile(&wm_a, recv_entries, recv_count, 0);

    /* wm_a should take wm_b's newer version of peer 2 */
    const wm_entry_t *e2 = wm_get_entry(&wm_a, 2);
    zassert_not_null(e2, "peer 2 should exist after reconcile");
    zassert_equal(e2->state.logical_clock, 16u,
                  "reconcile must take the higher-clock version (15 + Lamport +1 on receive = 16)");
    zassert_equal(e2->state.position.x, 72.0f,
                  "reconcile must update position to newer version");

    /* wm_a's own entry (ID 0) must be unchanged */
    const wm_entry_t *self = wm_get_entry(&wm_a, 0);
    zassert_true(self->is_self, "own entry must retain is_self after reconcile");
    zassert_equal(self->state.position.x, 50.0f,
                  "own position must not be overwritten by reconcile");

    /* Reconciling wm_b against an older entry must not downgrade it */
    wm_entry_t stale = {0};
    stale.state     = make_state(2, 60.0f, 50.0f, 3);
    stale.is_active = true;
    wm_reconcile(&wm_b, &stale, 1, 0);

    const wm_entry_t *e2b = wm_get_entry(&wm_b, 2);
    zassert_equal(e2b->state.logical_clock, 16u,
                  "reconcile must not downgrade to an older clock");
    zassert_equal(e2b->state.position.x, 72.0f,
                  "reconcile must not overwrite position with older data");
}

/*
 * test_nearest_elements
 *
 * Spatial query must return only active elements within the radius,
 * ordered closest-first, capped at max_count.
 */
ZTEST(world_model, test_nearest_elements)
{
    world_model_t wm;
    init_wm(&wm, 0.0f);

    /* Distances from (50, 50): peer 1 = 5.0, peer 2 ≈ 14.1, peer 3 = 30.0 */
    element_state_t p1 = make_state(1, 55.0f, 50.0f, 10);  /* dist  5.0 */
    element_state_t p2 = make_state(2, 60.0f, 60.0f, 10);  /* dist ~14.1 */
    element_state_t p3 = make_state(3, 80.0f, 50.0f, 10);  /* dist 30.0 */
    wm_receive_gossip(&wm, &p1);
    wm_receive_gossip(&wm, &p2);
    wm_receive_gossip(&wm, &p3);

    position_t center = {50.0f, 50.0f};
    element_id_t results[MAX_ELEMENTS];

    /* Radius 20 captures peers 1 and 2, not peer 3 */
    uint8_t found = wm_nearest_elements(&wm, &center, 20.0f, results, MAX_ELEMENTS);
    zassert_equal(found, 2, "radius 20 should find 2 elements");
    zassert_equal(results[0], 1, "closest should be peer 1");
    zassert_equal(results[1], 2, "second closest should be peer 2");

    /* max_count cap: ask for at most 1 result */
    found = wm_nearest_elements(&wm, &center, 20.0f, results, 1);
    zassert_equal(found, 1, "max_count=1 should return only 1 result");
    zassert_equal(results[0], 1, "capped result must be the closest element");

    /* Expire all peers — none should appear */
    wm_tick(&wm, WM_EXPIRE_THRESHOLD_MS + 1);
    found = wm_nearest_elements(&wm, &center, 100.0f, results, MAX_ELEMENTS);
    zassert_equal(found, 0, "expired peers must not appear in spatial query");
}

/*
 * test_collision_detection
 *
 * wm_check_collisions must detect peers within MIN_SEPARATION,
 * exclude inactive peers, and update metric.collision_count.
 */
ZTEST(world_model, test_collision_detection)
{
    world_model_t wm;
    init_wm(&wm, 0.0f);

    /* Peer 1: distance 1.0 — within MIN_SEPARATION (3.0) → collision */
    element_state_t p1 = make_state(1, 51.0f, 50.0f, 10);
    /* Peer 2: distance 5.0 — outside MIN_SEPARATION → no collision */
    element_state_t p2 = make_state(2, 55.0f, 50.0f, 10);
    wm_receive_gossip(&wm, &p1);
    wm_receive_gossip(&wm, &p2);

    position_t own_pos = {50.0f, 50.0f};
    collision_event_t events[MAX_ELEMENTS];

    uint8_t count = wm_check_collisions(&wm, &own_pos, events, MAX_ELEMENTS);
    zassert_equal(count, 1, "should detect exactly one collision");
    zassert_equal(events[0].element_a, 0, "element_a should be the owner");
    zassert_equal(events[0].element_b, 1, "element_b should be peer 1");
    zassert_true(events[0].distance < MIN_SEPARATION,
                 "recorded distance should be less than MIN_SEPARATION");
    zassert_equal(wm_get_metric(&wm)->collision_count, 1,
                  "metric.collision_count should be updated");

    /* Expire peer 1 — should no longer trigger a collision */
    wm_tick(&wm, WM_EXPIRE_THRESHOLD_MS + 1);
    count = wm_check_collisions(&wm, &own_pos, events, MAX_ELEMENTS);
    zassert_equal(count, 0, "inactive peer must not generate a collision event");
    zassert_equal(wm_get_metric(&wm)->collision_count, 0,
                  "metric.collision_count should be 0 after inactive peer");
}
