/*
 * main.c — Ztest suite for L5 Swarm Coordination Runtime
 *
 * Tests are independent: each builds its own world model and SCR state
 * from scratch, injects peers via wm_receive_gossip(), and asserts the
 * expected quorum state, leader, role, task slot, abort state, and BFT
 * filtering behavior.
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
 * make_wm — Initialize a world model for element `owner_id`.
 * Spreads the owner position deterministically (same formula as main.c).
 */
static void make_wm(world_model_t *wm, element_id_t owner_id)
{
    element_state_t s = {0};
    s.id          = owner_id;
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
    s.logical_clock = clock;
    s.position.x    = 10.0f + (float)((id * 17) % 80);
    s.position.y    = 10.0f + (float)((id * 23) % 80);
    wm_receive_gossip(wm, &s);
}

/*
 * make_scr — Initialize SCR with standard test parameters and no capabilities.
 */
static void make_scr(scr_state_t *scr, element_id_t own_id)
{
    scr_init(scr, own_id, QUORUM_MIN, QUORUM_TARGET, SCR_CAP_NONE);
}

/*
 * make_scr_with_caps — Initialize SCR with specific capability flags.
 */
static void make_scr_with_caps(scr_state_t *scr, element_id_t own_id,
                                scr_capability_t caps)
{
    scr_init(scr, own_id, QUORUM_MIN, QUORUM_TARGET, caps);
}

/* ── Original test suite ──────────────────────────────────────────────────── */

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
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_NONE,
                  "Initial abort state should be NONE");
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
    zassert_equal(scr.swarm_size, 0, "No peers → swarm_size == 0");
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
    zassert_equal(scr_get_task_slot(&scr), 0, "Leader has task_slot 0");
    zassert_equal(scr_get_swarm_size(&scr), 2, "swarm_size == 2");
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
    zassert_equal(scr_get_task_slot(&scr), 0, "Leader has task_slot 0");
    zassert_equal(scr_get_swarm_size(&scr), 4, "swarm_size == 4");
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
    /* Sorted: [2, 5, 8, 9] → own slot = 1 */
    zassert_equal(scr_get_task_slot(&scr), 1, "Own slot is 1 in [2,5,8,9]");
    zassert_equal(scr_get_swarm_size(&scr), 4, "swarm_size == 4");
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
 * Phase 2 (partition):  all peers go stale → LOST.
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

/* ── Gap 3: Extended roles ────────────────────────────────────────────────── */

/*
 * test_extended_role_relay — Follower with SCR_CAP_RELAY gets SCR_ROLE_RELAY.
 */
ZTEST(scr_suite, test_extended_role_relay)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr_with_caps(&scr, OWN_ID, SCR_CAP_RELAY);

    inject_peer(&wm, 2u, 1); /* Lower ID → peer 2 is leader */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_leader(&scr), 2u, "Peer 2 is leader");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_RELAY,
                  "Follower with RELAY capability → RELAY role");
}

/*
 * test_extended_role_sensor — Follower with SCR_CAP_SENSOR gets SCR_ROLE_SENSOR.
 */
ZTEST(scr_suite, test_extended_role_sensor)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr_with_caps(&scr, OWN_ID, SCR_CAP_SENSOR);

    inject_peer(&wm, 2u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_role(&scr), SCR_ROLE_SENSOR,
                  "SENSOR capability → SENSOR role");
}

/*
 * test_extended_role_actuator — Follower with SCR_CAP_ACTUATOR gets
 * SCR_ROLE_ACTUATOR.
 */
ZTEST(scr_suite, test_extended_role_actuator)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr_with_caps(&scr, OWN_ID, SCR_CAP_ACTUATOR);

    inject_peer(&wm, 2u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_role(&scr), SCR_ROLE_ACTUATOR,
                  "ACTUATOR capability → ACTUATOR role");
}

/*
 * test_extended_role_relay_priority — When multiple capability bits are set,
 * RELAY takes priority over SENSOR and ACTUATOR.
 */
ZTEST(scr_suite, test_extended_role_relay_priority)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr_with_caps(&scr, OWN_ID,
                        SCR_CAP_RELAY | SCR_CAP_SENSOR | SCR_CAP_ACTUATOR);

    inject_peer(&wm, 2u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_role(&scr), SCR_ROLE_RELAY,
                  "RELAY > SENSOR > ACTUATOR priority");
}

/*
 * test_leader_keeps_leader_role_despite_caps — When own ID wins the election,
 * role is LEADER regardless of capability flags.
 */
ZTEST(scr_suite, test_leader_keeps_leader_role_despite_caps)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5; all peers have higher IDs */
    make_scr_with_caps(&scr, OWN_ID,
                        SCR_CAP_RELAY | SCR_CAP_SENSOR | SCR_CAP_ACTUATOR);

    inject_peer(&wm, 7u, 1); /* Higher ID → own wins */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_leader(&scr), OWN_ID, "Own ID is leader");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_LEADER,
                  "Leader role overrides capability-derived role");
}

/* ── Gap 2: Task slot ─────────────────────────────────────────────────────── */

/*
 * test_task_slot_assignment — Verifies that each candidate receives a unique
 * slot in the sorted peer list and the leader always occupies slot 0.
 *
 * Swarm: own=5, peers 1, 8, 9.  Sorted: [1, 5, 8, 9].
 * Expected: peer 1 → leader/slot 0; own 5 → slot 1.
 */
ZTEST(scr_suite, test_task_slot_assignment)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 8u, 1);
    inject_peer(&wm, 9u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "3 peers → HEALTHY");
    zassert_equal(scr_get_leader(&scr), 1u, "Peer 1 is leader");
    zassert_equal(scr_get_task_slot(&scr), 1,
                  "Own ID 5 is at slot 1 in sorted [1, 5, 8, 9]");
    zassert_equal(scr_get_swarm_size(&scr), 4, "swarm_size == 4");
}

/*
 * test_task_slot_leader_is_zero — When own ID is the leader, task slot is 0.
 */
ZTEST(scr_suite, test_task_slot_leader_is_zero)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5; all peers higher */
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    inject_peer(&wm, 8u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_leader(&scr), OWN_ID, "Own ID is leader");
    zassert_equal(scr_get_task_slot(&scr), 0, "Leader always has task_slot 0");
}

/*
 * test_task_slot_single_peer — With one fresh peer, own slot depends on
 * relative ID ordering.  Peer 4 (lower) → own slot = 1.
 */
ZTEST(scr_suite, test_task_slot_single_peer)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 4u, 1); /* Lower ID → peer 4 is leader */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_leader(&scr), 4u, "Peer 4 is leader");
    zassert_equal(scr_get_task_slot(&scr), 1,
                  "Own ID 5 is at slot 1 in sorted [4, 5]");
    zassert_equal(scr_get_swarm_size(&scr), 2, "swarm_size == 2");
}

/* ── Gap 4: Abort protocol ────────────────────────────────────────────────── */

/*
 * test_abort_none_on_startup — Starting with no peers produces LOST quorum
 * but SCR_ABORT_NONE; the abort protocol does not fire on initial startup.
 */
ZTEST(scr_suite, test_abort_none_on_startup)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "Startup quorum LOST");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_NONE,
                  "No abort on startup — quorum was never established");
}

/*
 * test_abort_triggered_on_quorum_loss — When quorum drops from HEALTHY to
 * LOST, abort state becomes TRIGGERED and stays TRIGGERED while LOST.
 */
ZTEST(scr_suite, test_abort_triggered_on_quorum_loss)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    /* Establish healthy quorum */
    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "Phase 1: HEALTHY");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_NONE,
                  "No abort in steady HEALTHY state");

    /* Partition: peers go stale */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "Phase 2: LOST");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_TRIGGERED,
                  "Abort TRIGGERED on HEALTHY→LOST transition");

    /* Abort stays TRIGGERED while quorum remains LOST */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_TRIGGERED,
                  "Abort remains TRIGGERED while quorum is still LOST");
}

/*
 * test_abort_triggered_from_degraded — TRIGGERED also fires when quorum
 * drops from DEGRADED (not just HEALTHY) to LOST.
 */
ZTEST(scr_suite, test_abort_triggered_from_degraded)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    /* Establish degraded quorum (1 fresh peer) */
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_DEGRADED, "DEGRADED");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_NONE, "No abort yet");

    /* Drop to LOST */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "LOST");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_TRIGGERED,
                  "Abort TRIGGERED on DEGRADED→LOST transition");
}

/*
 * test_abort_cleared_on_recovery — When quorum recovers from LOST, abort
 * state becomes CLEARED for exactly one tick, then resets to NONE.
 */
ZTEST(scr_suite, test_abort_cleared_on_recovery)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    /* Reach TRIGGERED state */
    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_TRIGGERED,
                  "Pre-condition: abort is TRIGGERED");

    /* Re-inject peers → quorum recovers */
    inject_peer(&wm, 1u, 100);
    inject_peer(&wm, 6u, 100);
    inject_peer(&wm, 7u, 100);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY,
                  "Recovery tick: HEALTHY");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_CLEARED,
                  "Recovery tick: abort CLEARED");

    /* One more tick with no changes: CLEARED resets to NONE */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_NONE,
                  "Tick after recovery: abort resets to NONE");
}

/* ── Quorum-recovery hold (scr_set_quorum_hold_ms()) ─────────────────────── */

/*
 * test_quorum_hold_disabled_matches_prior_behavior — hold_ms == 0
 * (scr_init()'s default) must reproduce test_abort_cleared_on_recovery
 * exactly: recovery is reported the very tick sustained peers reappear.
 */
ZTEST(scr_suite, test_quorum_hold_disabled_matches_prior_behavior)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);   /* quorum_hold_ms == 0 */

    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "Phase 1: HEALTHY");

    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "Phase 2: LOST");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_TRIGGERED, "TRIGGERED");

    inject_peer(&wm, 1u, 100);
    inject_peer(&wm, 6u, 100);
    inject_peer(&wm, 7u, 100);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY,
                  "hold disabled: recovery reported immediately");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_CLEARED, "CLEARED");
}

/*
 * test_quorum_hold_delays_recovery_until_sustained — a real LOST -> HEALTHY
 * recovery must be held for quorum_hold_ms of SUSTAINED >=DEGRADED
 * readings before scr_get_quorum()/scr_get_abort_state() report it.
 */
ZTEST(scr_suite, test_quorum_hold_delays_recovery_until_sustained)
{
    world_model_t wm;
    scr_state_t   scr;
    uint32_t      clock;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);   /* hold disabled for warm-up — cold-boot
                               * acquisition is its own held case too
                               * (see scr_set_quorum_hold_ms()'s doc);
                               * isolate recovery-from-a-real-loss here. */

    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "warm-up: HEALTHY");

    scr_set_quorum_hold_ms(&scr, 1000u);   /* 10 ticks at WM_CYCLE_MS=100 */

    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "partitioned");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_TRIGGERED, "TRIGGERED");

    /* Peers come back fresh; re-inject every 5 ticks (500ms) to stay
     * under WM_STALE_THRESHOLD_MS (1500ms), same as real gossip would. */
    clock = 100;
    for (int tick = 1; tick <= 9; tick++) {
        if (tick == 1 || tick % 5 == 1) {
            clock += 100;
            inject_peer(&wm, 1u, clock);
            inject_peer(&wm, 6u, clock);
            inject_peer(&wm, 7u, clock);
        }
        wm_tick(&wm, WM_CYCLE_MS);
        scr_tick(&scr, &wm);
        zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST,
                      "tick %d: recovery not yet sustained %d ms", tick,
                      tick * (int)WM_CYCLE_MS);
        zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_TRIGGERED,
                      "abort must stay TRIGGERED while held");
    }
    /* 10th tick: 1000ms sustained -> hold satisfied */
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY,
                  "recovery sustained for the full hold -> reported");
    zassert_equal(scr_get_abort_state(&scr), SCR_ABORT_CLEARED, "CLEARED");
}

/*
 * test_quorum_hold_resets_on_flicker_back_to_lost — a recovery that drops
 * back to real LOST mid-hold gets no partial credit; the next recovery
 * attempt needs the full hold_ms again.
 */
ZTEST(scr_suite, test_quorum_hold_resets_on_flicker_back_to_lost)
{
    world_model_t wm;
    scr_state_t   scr;
    uint32_t      clock;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "warm-up: HEALTHY");

    scr_set_quorum_hold_ms(&scr, 1000u);

    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "partitioned");

    /* Recover for 500ms (half the hold)... */
    clock = 100;
    for (int tick = 0; tick < 5; tick++) {
        clock += 100;
        inject_peer(&wm, 1u, clock);
        inject_peer(&wm, 6u, clock);
        inject_peer(&wm, 7u, clock);
        wm_tick(&wm, WM_CYCLE_MS);
        scr_tick(&scr, &wm);
        zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "still held");
    }

    /* ...then genuinely drop back to LOST before the hold elapses */
    wm_tick(&wm, WM_STALE_THRESHOLD_MS + 1);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST, "lost again");

    /* Recover again — must need the FULL 1000ms again, not "500ms more" */
    clock = 1000;
    for (int tick = 1; tick <= 9; tick++) {
        if (tick == 1 || tick % 5 == 1) {
            clock += 100;
            inject_peer(&wm, 1u, clock);
            inject_peer(&wm, 6u, clock);
            inject_peer(&wm, 7u, clock);
        }
        wm_tick(&wm, WM_CYCLE_MS);
        scr_tick(&scr, &wm);
        zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_LOST,
                      "no partial credit from the aborted recovery");
    }
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY,
                  "second recovery completes after its own full hold");
}

/*
 * test_quorum_hold_does_not_affect_fluctuation_within_recovered_zone —
 * once already recovered, DEGRADED<->HEALTHY fluctuation (never touching
 * LOST) is reported live; the hold only ever gates the LOST edge.
 */
ZTEST(scr_suite, test_quorum_hold_does_not_affect_fluctuation_within_recovered_zone)
{
    world_model_t wm;
    scr_state_t   scr;
    uint32_t      clock;
    uint32_t      elapsed;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 1u, 1);
    inject_peer(&wm, 6u, 1);
    inject_peer(&wm, 7u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_HEALTHY, "warm-up: HEALTHY");

    scr_set_quorum_hold_ms(&scr, 1000u);

    /* Let peer 7 go stale by simply not re-injecting it, while peers 1
     * and 6 keep gossiping (re-injected each tick, like a real element
     * would). After WM_STALE_THRESHOLD_MS, peer 7 alone goes stale ->
     * DEGRADED (2 fresh, still >= quorum_min=1, < quorum_target=3). This
     * must be reported LIVE (no hold) — we're already recovered, not
     * coming from LOST. */
    clock   = 1;
    elapsed = 0;
    while (elapsed <= WM_STALE_THRESHOLD_MS) {
        clock++;
        inject_peer(&wm, 1u, clock);
        inject_peer(&wm, 6u, clock);
        wm_tick(&wm, WM_CYCLE_MS);
        scr_tick(&scr, &wm);
        elapsed += WM_CYCLE_MS;
    }
    zassert_equal(scr_get_quorum(&scr), SCR_QUORUM_DEGRADED,
                  "DEGRADED<->HEALTHY fluctuation must not be held");
}

/*
 * test_quorum_hold_matches_old_app_level_filter — direct equivalence
 * check against the flight-tested app-level QUORUM_UP_MS filter this
 * generalizes (cf21bl-formation/src/main.c, pre-migration; see
 * CHANGELOG), under that filter's own config (quorum_min=quorum_target=1
 * -> a binary threshold, DEGRADED never appears, matching the old
 * filter's own hardcoded fresh_count>=1 assumption). Runs the OLD
 * algorithm (replicated inline, byte-for-byte) and the NEW L5 mechanism
 * side by side against the SAME wm/gossip sequence and asserts they
 * agree on the reported quorum every single tick, over a scripted
 * sequence covering startup acquisition, sustained health, a real
 * partition, a single lucky flicker packet, and a real recovery.
 */
ZTEST(scr_suite, test_quorum_hold_matches_old_app_level_filter)
{
    world_model_t wm;
    scr_state_t   raw_scr;    /* hold disabled -- feeds the OLD algorithm */
    scr_state_t   new_scr;    /* hold enabled  -- the NEW mechanism       */
    uint32_t      old_fresh_streak_ms = 0;
    uint32_t      clock = 0;

    make_wm(&wm, OWN_ID);
    scr_init(&raw_scr, OWN_ID, 1u, 1u, SCR_CAP_NONE);
    scr_init(&new_scr, OWN_ID, 1u, 1u, SCR_CAP_NONE);
    scr_set_quorum_hold_ms(&new_scr, 2000u);   /* matches QUORUM_UP_MS */

    for (int tick = 0; tick < 40; tick++) {
        bool     inject        = false;
        uint32_t extra_stale_ms = 0;

        if (tick < 5) {
            inject = true;                              /* acquiring quorum */
        } else if (tick < 15) {
            if (tick % 3 == 0) inject = true;            /* sustained contact */
        } else if (tick < 20) {
            extra_stale_ms = WM_STALE_THRESHOLD_MS + 1;  /* partition */
        } else if (tick == 22) {
            inject = true;                               /* one lucky packet */
        } else if (tick >= 28) {
            if (tick == 28 || tick % 4 == 0) inject = true;  /* real heal */
        }

        if (inject) {
            clock += 100;
            inject_peer(&wm, 1u, clock);
        }
        wm_tick(&wm, WM_CYCLE_MS + extra_stale_ms);
        scr_tick(&raw_scr, &wm);
        scr_tick(&new_scr, &wm);

        /* OLD algorithm, replicated exactly from the deleted app code */
        if (raw_scr.fresh_count >= 1) {
            if (old_fresh_streak_ms < 2000u) old_fresh_streak_ms += 100u;
        } else {
            old_fresh_streak_ms = 0;
        }
        bool old_quorum_up = old_fresh_streak_ms >= 2000u;
        scr_quorum_state_t old_view = old_quorum_up ? SCR_QUORUM_HEALTHY
                                                     : SCR_QUORUM_LOST;

        zassert_equal(old_view, scr_get_quorum(&new_scr),
                      "tick %d: old=%d new=%d fresh=%u", tick, old_view,
                      scr_get_quorum(&new_scr), raw_scr.fresh_count);
    }
}

/* ── Gap 1: Lightweight BFT ──────────────────────────────────────────────── */

/*
 * test_whitelist_excludes_non_member — A peer not in the whitelist is
 * excluded from election and quorum count.
 */
ZTEST(scr_suite, test_whitelist_excludes_non_member)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    /* Inject two peers: 2 (lower ID, not trusted) and 7 (trusted) */
    inject_peer(&wm, 2u, 1);
    inject_peer(&wm, 7u, 1);

    /* Whitelist: only trust peer 7 (bit 7 set) */
    scr_set_peer_whitelist(&scr, 1u << 7);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    /* Only peer 7 is in the candidate set: sorted [5, 7] */
    zassert_equal(scr.fresh_count, 1,
                  "Only whitelisted peer 7 counted; peer 2 excluded");
    zassert_equal(scr_get_leader(&scr), OWN_ID,
                  "Own ID 5 wins: peer 2 excluded by whitelist");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_LEADER, "Own is leader");
    zassert_equal(scr_get_swarm_size(&scr), 2,
                  "swarm_size == 2 (self + peer 7)");
}

/*
 * test_whitelist_zero_allows_all — A whitelist mask of 0 disables filtering;
 * all fresh peers participate.
 */
ZTEST(scr_suite, test_whitelist_zero_allows_all)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 2u, 1);
    inject_peer(&wm, 7u, 1);
    inject_peer(&wm, 9u, 1);

    scr_set_peer_whitelist(&scr, 0u); /* No restriction */

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr.fresh_count, 3, "All 3 peers counted");
    zassert_equal(scr_get_leader(&scr), 2u, "Peer 2 wins (lowest ID)");
}

/*
 * test_anomaly_excludes_peer — A peer reported as anomalous is excluded from
 * election and quorum count on subsequent ticks.
 */
ZTEST(scr_suite, test_anomaly_excludes_peer)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 2u, 1); /* Would normally be leader */
    inject_peer(&wm, 7u, 1);
    inject_peer(&wm, 8u, 1);
    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    zassert_equal(scr_get_leader(&scr), 2u, "Pre-condition: peer 2 is leader");

    /* Transport layer detects anomaly from peer 2 */
    scr_report_anomaly(&scr, 2u);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    /* Peer 2 excluded; candidates: [5, 7, 8] → own ID 5 wins */
    zassert_equal(scr.fresh_count, 2,
                  "Peer 2 excluded by anomaly; fresh_count == 2");
    zassert_equal(scr_get_leader(&scr), OWN_ID,
                  "Own ID 5 wins after peer 2 excluded");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_LEADER, "Own is now leader");
}

/*
 * test_anomaly_clear_readmits_peer — After clearing an anomaly, the peer
 * re-enters the candidate set on the next tick.
 */
ZTEST(scr_suite, test_anomaly_clear_readmits_peer)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID); /* own_id = 5 */
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 2u, 1);
    inject_peer(&wm, 7u, 1);
    inject_peer(&wm, 8u, 1);

    scr_report_anomaly(&scr, 2u);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr_get_leader(&scr), OWN_ID,
                  "Pre-condition: own is leader while peer 2 excluded");

    /* Clear the anomaly */
    scr_clear_anomaly(&scr, 2u);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);

    /* Peer 2 re-admitted; all three peers fresh; peer 2 wins */
    zassert_equal(scr.fresh_count, 3,
                  "Peer 2 re-admitted; fresh_count == 3");
    zassert_equal(scr_get_leader(&scr), 2u,
                  "Peer 2 re-elected after anomaly cleared");
    zassert_equal(scr_get_role(&scr), SCR_ROLE_FOLLOWER,
                  "Own is follower again");
}

/*
 * test_anomaly_clear_all — scr_clear_all_anomalies re-admits all excluded peers.
 */
ZTEST(scr_suite, test_anomaly_clear_all)
{
    world_model_t wm;
    scr_state_t   scr;

    make_wm(&wm, OWN_ID);
    make_scr(&scr, OWN_ID);

    inject_peer(&wm, 2u, 1);
    inject_peer(&wm, 3u, 1);
    inject_peer(&wm, 7u, 1);

    scr_report_anomaly(&scr, 2u);
    scr_report_anomaly(&scr, 3u);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr.fresh_count, 1, "Only peer 7 counts; 2 and 3 excluded");

    scr_clear_all_anomalies(&scr);

    wm_tick(&wm, WM_CYCLE_MS);
    scr_tick(&scr, &wm);
    zassert_equal(scr.fresh_count, 3, "All three peers re-admitted");
    zassert_equal(scr_get_leader(&scr), 2u, "Peer 2 (lowest) re-elected");
}

/* ── Suite registration ─────────────────────────────────────────────────── */

ZTEST_SUITE(scr_suite, NULL, NULL, NULL, NULL, NULL);
