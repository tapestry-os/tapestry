/*
 * facts_test.c — L4 shared facts unit tests (tapestry-os/subsys/csm/facts.c)
 *
 * Mirrors sdk/tests/test_facts.py scenario for scenario; the digest and
 * Trickle literals are the same numbers, so either engine drifting from
 * the other fails one suite.  The per-origin cap test is Python-only (the
 * cap is a compile-time constant here).
 */

#include <zephyr/ztest.h>
#include <tapestry/csm.h>

#define HIGH ((uint8_t)(FACT_IMMUTABLE | FACT_PRIO(3)))

static world_model_t wm;

static void init(element_id_t owner, uint16_t epoch)
{
    element_state_t own = {0};
    own.id = owner;
    wm_init(&wm, owner, &own, 0.0f);
    wm_fact_set_epoch(&wm, epoch);
}

static void init_synced(element_id_t owner, uint16_t epoch)
{
    init(owner, epoch);
    wm_fact_heard_digest(&wm, 0);
}

static fact_t mk(uint8_t type, element_id_t origin, uint16_t epoch,
                 uint8_t seq, uint8_t flags, uint32_t stamp, float x, float y)
{
    fact_t f = {
        .key   = { .type = type, .origin = origin, .epoch = epoch, .seq = seq },
        .flags = flags,
        .stamp = stamp,
        .pos   = { x, y, 0.0f },
    };
    return f;
}

static int send_ticks(int ticks, uint32_t rnd, int *out, int max)
{
    int n = 0;
    for (int i = 0; i < ticks; i++) {
        if (wm_fact_trickle_tick(&wm, 100u, rnd) && n < max) {
            out[n++] = i;
        }
    }
    return n;
}

ZTEST_SUITE(facts, NULL, NULL, NULL, NULL, NULL);

/* ── Publishing ──────────────────────────────────────────────────────────── */

ZTEST(facts, test_publish_needs_a_synced_table)
{
    position_t p = { 69.0f, 68.4f, 0.0f };
    fact_key_t k;
    fact_t     f;

    init(0, 0x1234);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, HIGH, &k),
                  WM_FACT_ERR_UNSYNCED);
    wm_fact_heard_digest(&wm, 0);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, HIGH, &k),
                  WM_FACT_CREATED);
    zassert_equal(k.type, FACT_TYPE_TARGET);
    zassert_equal(k.origin, 0);
    zassert_equal(k.epoch, 0x1234);
    zassert_equal(k.seq, 0);
    zassert_true(wm_fact_get(&wm, &k, &f));
    zassert_equal(f.stamp, 1u);
    zassert_equal(f.flags, HIGH);
}

ZTEST(facts, test_publish_needs_an_epoch)
{
    element_state_t own = {0};
    position_t p = { 10.0f, 10.0f, 0.0f };

    /* wm_init() leaves the epoch unset; gossip_drain() seeds it in the real
     * stack, so an app that publishes before ever draining must be told. */
    wm_init(&wm, 0, &own, 0.0f);
    wm_fact_heard_digest(&wm, 0);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, 0, NULL),
                  WM_FACT_ERR_NOEPOCH);
    wm_fact_set_epoch(&wm, 0x1234);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, 0, NULL),
                  WM_FACT_CREATED);
}

ZTEST(facts, test_sync_timeout_allows_a_solo_element_to_publish)
{
    position_t p = { 10.0f, 10.0f, 0.0f };

    init(0, 1);
    wm_tick(&wm, FACT_SYNC_TIMEOUT_MS - 100u);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, 0, NULL),
                  WM_FACT_ERR_UNSYNCED);
    wm_tick(&wm, 100u);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, 0, NULL),
                  WM_FACT_CREATED);
}

ZTEST(facts, test_publish_within_merge_radius_reuses_the_known_fact)
{
    position_t a = { 69.0f, 68.4f, 0.0f };
    position_t b = { 75.0f, 70.0f, 0.0f };
    position_t c = { 90.0f, 90.0f, 0.0f };
    fact_key_t k0, k1, k2;

    init_synced(0, 0x1234);
    wm_fact_publish(&wm, FACT_TYPE_TARGET, &a, 0, HIGH, &k0);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &b, 0, HIGH, &k1),
                  WM_FACT_REUSED);
    zassert_true(fact_key_equal(&k0, &k1));
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &c, 0, HIGH, &k2),
                  WM_FACT_CREATED);
    zassert_false(fact_key_equal(&k0, &k2));
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_ZONE, &a, 0, 0, NULL),
                  WM_FACT_CREATED);
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_ZONE, &a, 0, 0, NULL),
                  WM_FACT_CREATED, "zones never merge by distance");
}

/* ── Merge rules ─────────────────────────────────────────────────────────── */

ZTEST(facts, test_immutable_lower_stamp_wins_mutable_higher_wins)
{
    fact_t f;

    init_synced(9, 1);
    fact_t t5 = mk(FACT_TYPE_TARGET, 1, 7, 0, HIGH, 5, 10, 10);
    fact_t t6 = mk(FACT_TYPE_TARGET, 1, 7, 0, HIGH, 6, 11, 10);
    fact_t t4 = mk(FACT_TYPE_TARGET, 1, 7, 0, HIGH, 4, 12, 10);
    wm_fact_receive(&wm, &t5);
    zassert_false(wm_fact_receive(&wm, &t6));
    zassert_true(wm_fact_receive(&wm, &t4));
    zassert_true(wm_fact_get(&wm, &t4.key, &f));
    zassert_equal(f.pos.x, 12.0f);

    fact_t z5 = mk(FACT_TYPE_ZONE, 1, 7, 1, 0, 5, 10, 10);
    fact_t z6 = mk(FACT_TYPE_ZONE, 1, 7, 1, 0, 6, 20, 10);
    fact_t z3 = mk(FACT_TYPE_ZONE, 1, 7, 1, 0, 3, 30, 10);
    wm_fact_receive(&wm, &z5);
    zassert_true(wm_fact_receive(&wm, &z6));
    zassert_false(wm_fact_receive(&wm, &z3));
    zassert_true(wm_fact_get(&wm, &z5.key, &f));
    zassert_equal(f.pos.x, 20.0f);
}

ZTEST(facts, test_retraction_beats_live_and_any_element_may_retract)
{
    static world_model_t b;
    element_state_t own = { .id = 3 };
    position_t p = { 50.0f, 50.0f, 0.0f };
    fact_key_t key;
    fact_t     live, tomb;

    init_synced(0, 0x1234);
    wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, HIGH, &key);
    wm_fact_get(&wm, &key, &live);

    wm_init(&b, 3, &own, 0.0f);
    wm_fact_heard_digest(&b, 0);
    wm_fact_receive(&b, &live);
    zassert_equal(wm_fact_retract(&b, &key), 0, "a non-origin may retract");
    wm_fact_get(&b, &key, &tomb);
    zassert_true(tomb.flags & FACT_RETRACTED);

    zassert_true(wm_fact_receive(&wm, &tomb));
    zassert_equal(wm_fact_count(&wm, FACT_TYPE_TARGET), 0);
    zassert_equal(wm_fact_tombstones(&wm, FACT_TYPE_TARGET), 1);
    zassert_false(wm_fact_receive(&wm, &live), "stale live copy cannot revive");
    zassert_equal(wm_fact_count(&wm, FACT_TYPE_TARGET), 0);
}

ZTEST(facts, test_two_tombstones_of_one_key_keep_the_earlier)
{
    fact_t f;

    init_synced(9, 1);
    fact_t t9  = mk(FACT_TYPE_TARGET, 1, 7, 0, HIGH | FACT_RETRACTED, 9, 1, 1);
    fact_t t8  = mk(FACT_TYPE_TARGET, 1, 7, 0, HIGH | FACT_RETRACTED, 8, 1, 1);
    fact_t t10 = mk(FACT_TYPE_TARGET, 1, 7, 0, HIGH | FACT_RETRACTED, 10, 1, 1);
    wm_fact_receive(&wm, &t9);
    zassert_true(wm_fact_receive(&wm, &t8));
    zassert_false(wm_fact_receive(&wm, &t10));
    wm_fact_get(&wm, &t8.key, &f);
    zassert_equal(f.stamp, 8u);
}

ZTEST(facts, test_tombstone_dropped_after_ttl)
{
    init_synced(9, 1);
    fact_t t = mk(FACT_TYPE_TARGET, 1, 7, 0, FACT_RETRACTED, 3, 1, 1);
    wm_fact_receive(&wm, &t);
    wm_tick(&wm, FACT_TOMBSTONE_TTL_MS - 100u);
    zassert_equal(wm_fact_tombstones(&wm, FACT_TYPE_NONE), 1);
    wm_tick(&wm, 100u);
    zassert_equal(wm_fact_tombstones(&wm, FACT_TYPE_NONE), 0);
    zassert_false(wm_fact_get(&wm, &t.key, NULL));
}

ZTEST(facts, test_duplicates_from_two_finders_converge_on_the_lower_stamp)
{
    fact_t f;

    init_synced(9, 1);
    fact_t a = mk(FACT_TYPE_TARGET, 2, 7, 0, HIGH, 3, 70, 68);
    fact_t b = mk(FACT_TYPE_TARGET, 1, 8, 0, HIGH, 4, 72, 66);
    fact_t c = mk(FACT_TYPE_TARGET, 1, 8, 1, HIGH, 3, 71, 67);
    wm_fact_receive(&wm, &a);
    zassert_false(wm_fact_receive(&wm, &b));
    zassert_true(wm_fact_receive(&wm, &c));
    zassert_equal(wm_fact_count(&wm, FACT_TYPE_TARGET), 1);
    zassert_true(wm_fact_find(&wm, FACT_TYPE_TARGET, 0, &f));
    zassert_true(fact_key_equal(&f.key, &c.key));
}

/* ── Ranking ─────────────────────────────────────────────────────────────── */

ZTEST(facts, test_rank_orders_by_stamp_then_origin)
{
    fact_t f;

    init_synced(9, 1);
    fact_t a = mk(FACT_TYPE_TARGET, 4, 1, 0, HIGH, 7, 10, 10);
    fact_t b = mk(FACT_TYPE_TARGET, 2, 1, 0, HIGH, 7, 40, 10);
    fact_t c = mk(FACT_TYPE_TARGET, 5, 1, 0, HIGH, 2, 80, 10);
    wm_fact_receive(&wm, &a);
    wm_fact_receive(&wm, &b);
    wm_fact_receive(&wm, &c);
    zassert_true(wm_fact_find(&wm, FACT_TYPE_TARGET, 0, &f));
    zassert_equal(f.key.origin, 5);
    zassert_true(wm_fact_find(&wm, FACT_TYPE_TARGET, 1, &f));
    zassert_equal(f.key.origin, 2);
    zassert_true(wm_fact_find(&wm, FACT_TYPE_TARGET, 2, &f));
    zassert_equal(f.key.origin, 4);
    zassert_false(wm_fact_find(&wm, FACT_TYPE_TARGET, 3, &f));
    zassert_equal(wm_fact_count(&wm, FACT_TYPE_TARGET), 3);
}

/* ── Capacity ────────────────────────────────────────────────────────────── */

ZTEST(facts, test_full_table_evicts_lowest_priority_oldest_first)
{
    init_synced(9, 1);
    for (int i = 0; i < WM_MAX_FACTS; i++) {
        fact_t w = mk(FACT_TYPE_WAYPOINT, (element_id_t)(i % 8), 1,
                      (uint8_t)i, 0, 100u + (uint32_t)i, (float)i, 0);
        wm_fact_receive(&wm, &w);
    }
    zassert_equal(wm_fact_count(&wm, FACT_TYPE_NONE), WM_MAX_FACTS);

    fact_t t = mk(FACT_TYPE_TARGET, 3, 2, 0, HIGH, 1, 50, 50);
    zassert_true(wm_fact_receive(&wm, &t));
    zassert_equal(wm.facts.evictions, 1u);
    zassert_equal(wm.facts.last_evicted.type, FACT_TYPE_WAYPOINT);
    zassert_equal(wm.facts.last_evicted.origin, 0);
    zassert_equal(wm.facts.last_evicted.seq, 0);
    zassert_equal(wm.facts.last_evict_reason, WM_FACT_EVICT_CAPACITY);

    fact_t back = mk(FACT_TYPE_WAYPOINT, 0, 1, 0, 0, 100, 0, 0);
    zassert_false(wm_fact_receive(&wm, &back), "evicted fact must not bounce back");
    zassert_equal(wm.facts.evictions, 1u);
    zassert_equal(wm_fact_count(&wm, FACT_TYPE_TARGET), 1);
}

/* ── Digest ──────────────────────────────────────────────────────────────── */

ZTEST(facts, test_digest_literals_and_order_independence)
{
    static world_model_t b;
    element_state_t own = { .id = 8 };

    init(9, 1);
    zassert_equal(wm_fact_digest(&wm), 0);
    fact_t f1 = mk(FACT_TYPE_TARGET, 0, 0x1234, 0, HIGH, 1, 69.0f, 68.4f);
    fact_t f2 = mk(FACT_TYPE_HAZARD, 3, 0xBEEF, 2, FACT_PRIO(1), 7, 10, 20);

    wm_fact_receive(&wm, &f1);
    zassert_equal(wm_fact_digest(&wm), 0xBC77);
    wm_fact_receive(&wm, &f2);

    wm_init(&b, 8, &own, 0.0f);
    wm_fact_receive(&b, &f2);
    zassert_equal(wm_fact_digest(&b), 0x7BAC);
    wm_fact_receive(&b, &f1);
    zassert_equal(wm_fact_digest(&wm), 0x3823);
    zassert_equal(wm_fact_digest(&b), 0x3823);
    wm_fact_retract(&b, &f1.key);
    zassert_not_equal(wm_fact_digest(&b), wm_fact_digest(&wm));
}

/* ── Trickle ─────────────────────────────────────────────────────────────── */

ZTEST(facts, test_trickle_backs_off_to_imax_when_alone)
{
    static const int want[] = { 2, 6, 16, 31, 61, 121, 201, 281, 361 };
    int got[16];

    init(0, 1);
    int n = send_ticks(400, 12345u, got, 16);
    zassert_equal(n, (int)ARRAY_SIZE(want));
    for (int i = 0; i < n; i++) {
        zassert_equal(got[i], want[i], "send %d", i);
    }
}

ZTEST(facts, test_trickle_suppresses_after_k_consistent_digests)
{
    int sent = 0;

    init(0, 1);
    for (int i = 0; i < 200; i++) {
        wm_fact_trickle_tick(&wm, 100u, 12345u);
    }
    for (int i = 0; i < 400; i++) {
        if (i % 10 == 0) {
            wm_fact_heard_digest(&wm, wm_fact_digest(&wm));
            wm_fact_heard_digest(&wm, wm_fact_digest(&wm));
        }
        sent += wm_fact_trickle_tick(&wm, 100u, 12345u) ? 1 : 0;
    }
    zassert_equal(sent, 0);
}

ZTEST(facts, test_trickle_restarts_at_imin_on_an_inconsistent_digest_or_new_fact)
{
    position_t p = { 1.0f, 1.0f, 0.0f };
    int got[4];

    init(0, 1);
    for (int i = 0; i < 200; i++) {
        wm_fact_trickle_tick(&wm, 100u, 12345u);
    }
    zassert_equal(wm.facts.trickle_i_ms, FACT_IMAX_MS);
    wm_fact_heard_digest(&wm, 0xABCD);
    zassert_equal(send_ticks(3, 12345u, got, 4), 1);
    zassert_equal(got[0], 2);

    for (int i = 0; i < 200; i++) {
        wm_fact_trickle_tick(&wm, 100u, 12345u);
    }
    wm_fact_publish(&wm, FACT_TYPE_TARGET, &p, 0, 0, NULL);
    zassert_equal(send_ticks(3, 12345u, got, 4), 1);
    zassert_equal(got[0], 2);
}

ZTEST(facts, test_pack_rotates_two_records_at_a_time)
{
    fact_t out[2];

    init_synced(9, 1);
    for (int i = 0; i < 3; i++) {
        fact_t w = mk(FACT_TYPE_WAYPOINT, 1, 1, (uint8_t)i, 0,
                      (uint32_t)i + 1u, 0, 0);
        wm_fact_receive(&wm, &w);
    }
    zassert_equal(wm_fact_pack(&wm, out), 2);
    zassert_equal(out[0].key.seq, 0);
    zassert_equal(out[1].key.seq, 1);
    zassert_equal(wm_fact_pack(&wm, out), 2);
    zassert_equal(out[0].key.seq, 2);
    zassert_equal(out[1].key.seq, 0);
    zassert_equal(wm_fact_pack(&wm, out), 2);
    zassert_equal(out[0].key.seq, 1);
    zassert_equal(out[1].key.seq, 2);
}

/* ── The rejoin case (logs/ble-20261006-112812) ──────────────────────────── */

ZTEST(facts, test_rejoiner_adopts_the_gossiped_point_not_a_peer_position)
{
    static world_model_t finder;
    element_state_t f0 = { .id = 0 };
    position_t leak = { 69.0f, 68.4f, 0.0f };
    position_t over = { 70.0f, 69.0f, 0.0f };
    fact_key_t key, k2;
    fact_t recs[2], f;

    wm_init(&finder, 0, &f0, 0.0f);
    wm_fact_set_epoch(&finder, 0x0A0A);
    wm_fact_heard_digest(&finder, 0);
    wm_fact_publish(&finder, FACT_TYPE_TARGET, &leak, 0, HIGH, &key);

    init(3, 0x7777);                            /* power-cycled: empty */
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &over, 0, HIGH, NULL),
                  WM_FACT_ERR_UNSYNCED, "no low-stamped duplicate before sync");
    uint8_t n = wm_fact_pack(&finder, recs);
    for (uint8_t i = 0; i < n; i++) {
        wm_fact_receive(&wm, &recs[i]);
    }
    wm_fact_heard_digest(&wm, wm_fact_digest(&finder));

    zassert_true(wm_fact_find(&wm, FACT_TYPE_TARGET, 0, &f));
    zassert_true(fact_key_equal(&f.key, &key));
    zassert_equal(f.pos.x, 69.0f);
    zassert_equal(f.pos.y, 68.4f);
    zassert_equal(wm_fact_digest(&wm), wm_fact_digest(&finder));
    zassert_equal(wm_fact_publish(&wm, FACT_TYPE_TARGET, &over, 0, HIGH, &k2),
                  WM_FACT_REUSED);
    zassert_true(fact_key_equal(&k2, &key));
}
