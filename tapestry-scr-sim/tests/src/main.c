/*
 * main.c — Ztest suite for L5 Swarm Coordination Runtime
 *
 * Tests are independent: each builds its own world model and SCR state
 * from scratch, injects peers via wm_receive_gossip(), and asserts the
 * expected quorum state, leader, and role.
 *
 * Conventions:
 *   - quorum_min    = 1  (need >= 1 fresh peer for DEGRADED)
 *   - quorum_target = 3  (need >= 3 fresh peers for HEALTHY)
 *   - Own ID is 5 in most tests so we can inject lower-ID peers (0–4)
 *     and higher-ID peers (6–9) to test leader direction.
 *   - wm_tick(wm, WM_STALE_THRESHOLD_MS + 1) ages all entries to stale.
 *   - wm_tick(wm, WM_EXPIRE_THRESHOLD_MS + 1) ages entries to inactive.
 */

#include <zephyr/ztest.h>
#include <tapestry/scr.h>

/* ── Test constants ───────────────────────────────────────────────────────── */

#define OWN_ID         5u
#define QUORUM_MIN     1u
#define QUORUM_TARGET  3u

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/*
 * make_wm — Initialise a world model for element `owner_id`.
 * Spreads the owner position deterministically (same formula as main.c).
 */
static void make_wm(world_model_t *wm, element_id_t owner_id)
{
    element_state_t s = {0};
    s.id          = owner_id;
    s.power_state = POWER_ACTIVE;
    s.position.x  = 10.0f + (float)((owner_id * 17) % 80);
    s.position.y  = 10.0f + (float)((owner_id * 23) % 80);
    wm_init(wm, owner_id, &s, 0.0f); /* AP bias: never L4-freezes */
}

/*
 * inject_peer — Gossip a state for peer `id` into wm with a given clock.
 *
 * wm_receive_gossip rejects messages where logical_clock <= the stored
 * Lamport-merged clock.  After a first inject with clock=1 the WM stores
 * max(0,1)+1 = 2.  Re-injections after a partition must pass a clock > 2;
 * use a value well above the merge result to be safe (e.g. 100).
 */
static void inject_peer(world_model_t *wm, element_id_t id, uint32_t clock)
{
    element_state_t s = {0};
    s.id            = id;
    s.power_state   = POWER_ACTIVE;
    s.logical_clock = clock;
    s.position.x    = 10.0f + (float)((id * 17) % 80);
    s.position.y    = 10.0f + (float)((id * 23) % 80);
    wm_receive_gossip(wm, &s);
}

/*
 * make_scr — Initialise SCR with standard test parameters.
 */
static void make_scr(scr_state_t *scr, element_id_t own_id)
{
    scr_init(scr, own_id, QUORUM_MIN, QUORUM_TARGET);
}

/* ── Test suite ───────────────────────────────────────────────────────────── */

/*
 * test_init_state — After scr_init, state is quorum LOST, no leader, NONE role.
 */
ZTEST(scr_suite, test_init_state)
{
    scr_state_t scr;
    make_scr(&scr, OWN_ID);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST,
                  "Initial quorum should be LOST");
    zassert_equal(scr_get_leader(&scr), ELEMENT_ID_INVALID,
                  "Initial leader should be ELEMENT_ID_INVALID");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_NONE,
                  "Initial role should be NONE");
}

/*
 * test_quorum_lost_no_peers — With zero fresh peers, quorum stays LOST.
 */
ZTEST(scr_suite, test_quorum_lost_no_peers)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST,
                  "No peers → LOST");
    zassert_equal(scr_get_leader(&scr), ELEMENT_ID_INVALID,
                  "No peers → no valid leader");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_NONE,
                  "No peers → NONE role");
    zassert_equal(scr.fresh_count, 0, "No peers → fresh_count == 0");
}

/*
 * test_quorum_degraded — Exactly quorum_min (1) fresh peer → DEGRADED.
 * Leader is elected and role is assigned.
 */
ZTEST(scr_suite, test_quorum_degraded)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 7u, 1); /* One peer, higher ID than own */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_DEGRADED,
                  "1 peer >= quorum_min(1) → DEGRADED");
    zassert_equal(scr.fresh_count, 1, "fresh_count == 1");
    /* Own ID (5) < peer ID (7) → own is leader */
    zassert_equal(scr_get_leader(&scr), OWN_ID,
                  "Own ID wins as leader (lower than peer 7)");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_LEADER,
                  "Should be LEADER");
    zassert_true(scr.leader_valid, "Leader should be valid");
}

/*
 * test_quorum_healthy — quorum_target (3) or more fresh peers → HEALTHY.
 */
ZTEST(scr_suite, test_quorum_healthy)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    inject_peer(&wm, 8u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY,
                  "3 peers >= quorum_target(3) → HEALTHY");
    zassert_equal(scr.fresh_count, 3, "fresh_count == 3");
    /* Own ID (5) < all peers (6, 7, 8) → own is leader */
    zassert_equal(scr_get_leader(&scr), OWN_ID, "OWN_ID is lowest");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_LEADER, "Should be LEADER");
}

/*
 * test_leader_is_lowest_id — Inject a peer with lower ID than self.
 * That peer becomes leader; self is follower.
 */
ZTEST(scr_suite, test_leader_is_lowest_id)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 2u, 1); /* Lower ID → should win election */
    inject_peer(&wm, 8u, 1);
    inject_peer(&wm, 9u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "3 peers → HEALTHY");
    zassert_equal(scr_get_leader(&scr), 2u,
                  "Peer 2 is lowest ID → elected leader");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_FOLLOWER,
                  "Own ID 5 > leader 2 → FOLLOWER");
}

/*
 * test_leader_stability — Adding a higher-ID peer does not change leader.
 */
ZTEST(scr_suite, test_leader_stability)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 3u, 1); /* Lower than own (5) */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_leader(&scr), 3u, "Leader is 3");

    /* Add higher-ID peer — should not disturb election */
    inject_peer(&wm, 9u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_leader(&scr), 3u,
                  "Adding higher-ID peer 9 leaves leader 3 unchanged");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_FOLLOWER,
                  "Own ID 5 > leader 3 → FOLLOWER");
}

/*
 * test_stale_peer_not_counted — A peer that goes stale is excluded from
 * quorum count and election.
 */
ZTEST(scr_suite, test_stale_peer_not_counted)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    /* Inject only one peer — initially counts for quorum */
    inject_peer(&wm, 4u, 1); /* Lower ID than own (5) */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_DEGRADED,
                  "Before stale: DEGRADED with 1 fresh peer");
    zassert_equal(scr_get_leader(&scr), 4u,
                  "Before stale: peer 4 is leader");

    /* Age the peer past the stale threshold — no new gossip */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);

    zassert_equal(scr.fresh_count, 0,
                  "After stale: fresh_count drops to 0");
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST,
                  "After stale: quorum LOST");
    zassert_equal(scr_get_leader(&scr), ELEMENT_ID_INVALID,
                  "After stale: no valid leader");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_NONE,
                  "After stale: role NONE");
}

/*
 * test_partition_and_heal — Simulate a partition where peers become stale,
 * then recover when gossip resumes.
 *
 * Phase 1 (connected):  3 fresh peers → HEALTHY, peer 1 is leader.
 * Phase 2 (partition):  all peers go stale → LOST, own becomes sole candidate.
 * Phase 3 (heal):       peers gossip again → HEALTHY, peer 1 re-elected.
 */
ZTEST(scr_suite, test_partition_and_heal)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    /* ── Phase 1: connected ── */
    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "Phase 1: HEALTHY");
    zassert_equal(scr_get_leader(&scr), 1u, "Phase 1: peer 1 is leader");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_FOLLOWER, "Phase 1: FOLLOWER");

    /* ── Phase 2: partition (no new gossip) ── */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "Phase 2: LOST");
    zassert_equal(scr_get_leader(&scr), ELEMENT_ID_INVALID,
                  "Phase 2: no leader");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_NONE, "Phase 2: NONE");

    /* ── Phase 3: heal (gossip resumes with higher clocks) ── */
    inject_peer(&wm, 1u, 100);
    inject_peer(&wm, 6u, 100);
    inject_peer(&wm, 7u, 100);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "Phase 3: HEALTHY");
    zassert_equal(scr_get_leader(&scr), 1u, "Phase 3: peer 1 re-elected");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_FOLLOWER, "Phase 3: FOLLOWER");
}

/*
 * test_leader_reelection_on_loss — When the current leader's entry goes stale,
 * the next lowest-ID fresh peer is elected.
 */
ZTEST(scr_suite, test_leader_reelection_on_loss)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 2u, 1); /* Current leader: peer 2 (lowest) */
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_leader(&scr), 2u, "Initial leader is peer 2");

    /*
     * Advance time past stale threshold without refreshing.
     * All peers go stale → quorum LOST.
     * Then re-inject only peers 6 and 7 (not peer 2), with higher clocks
     * so wm_receive_gossip accepts them (stored Lamport clock is 2 after
     * the initial inject with clock=1; re-inject needs clock > 2).
     */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);

    inject_peer(&wm, 6u, 100);
    inject_peer(&wm, 7u, 100);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    /* Peer 2 is stale; fresh candidates are: own(5), peer 6, peer 7.
     * Lowest is own ID (5) → own is elected leader. */
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_DEGRADED,
                  "2 fresh peers → DEGRADED (< quorum_target=3)");
    zassert_equal(scr_get_leader(&scr), OWN_ID,
                  "Own ID 5 is lowest among {5, 6, 7}");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_LEADER,
                  "Should be LEADER after re-election");
}

/*
 * test_expired_peer_not_counted — An expired (inactive) peer is treated
 * the same as never seen: not in quorum, not in election.
 */
ZTEST(scr_suite, test_expired_peer_not_counted)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 3u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_DEGRADED,
                  "Before expiry: 1 peer → DEGRADED");

    /* Age peer past expiry threshold */
    wm_tick(&wm, WM_EXPIRE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);

    zassert_equal(scr.fresh_count, 0, "Expired peer not counted");
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST,
                  "Expired peer → LOST");
    zassert_equal(scr_get_leader(&scr), ELEMENT_ID_INVALID,
                  "Expired peer → no valid leader");
}

/* ── Suite registration ─────────────────────────────────────────────────── */

ZTEST_SUITE(scr_suite, NULL, NULL, NULL, NULL, NULL);
