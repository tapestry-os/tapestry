/*
 * main.c — L3 gossip wire round-trip tests
 *
 * Covers the two hops nothing else does: gossip_send() packing an
 * element_state_t into on-wire bytes, and gossip_drain() unpacking those
 * bytes back into a peer's world model.
 *
 * Why this suite exists.  Every other test stubs the wire.  The L6/L7
 * collective-achievement tests in examples/cf21bl-formation/tests write
 * wm.entries[1].state.goal_achieved directly and say so in a comment —
 * they prove the CONSUMER of a gossiped bit is correct while assuming the
 * bit ever arrives.  That assumption has failed twice: the webots
 * controller never published goal_achieved before gossiping (fixed in
 * 827fe4b), and choreo_telemetry.c never recorded it (fixed 2026-08-17).
 * Both were single hops in a chain everyone had inspected hop by hop and
 * nobody had followed end to end.  These tests close hops 3 and 4 by
 * putting real bytes through a loopback transceiver, so a field that is
 * packed but not unpacked (or silently dropped by a size, version or auth
 * check) fails here rather than in flight.
 *
 * Built three ways in CI (see .github/workflows/ci.yml):
 *   default        — plain framing
 *   auth.conf      — CONFIG_TAPESTRY_WIRE_AUTH_ENABLED, HMAC sign+verify
 *   relay.conf     — CONFIG_TAPESTRY_MESH_RELAY, two-hop forwarding
 * The auth and relay code paths had never been compiled anywhere before,
 * let alone run.
 *
 * Build:  west build -b native_sim/native/64 tapestry/tapestry-os/tests/transport
 * Run:    ./build/zephyr/zephyr.exe
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <tapestry/csm.h>
#include <tapestry/transceiver.h>
#include <tapestry/wire.h>

#include "gossip.h"

/* ── Loopback transceiver ────────────────────────────────────────────────── */
/*
 * Stands in for a radio: tx() appends the exact bytes gossip.c handed it to
 * a FIFO, rx() pops them back.  Nothing is reinterpreted in between, so the
 * bytes a test observes are the bytes that would have gone on air — which
 * is the whole point of testing this layer rather than mocking it.
 */

#define LOOP_DEPTH   16
#define LOOP_MAX_LEN (TAPESTRY_GOSSIP_WIRE_SIZE + 8u)

static uint8_t  loop_buf[LOOP_DEPTH][LOOP_MAX_LEN];
static uint16_t loop_len[LOOP_DEPTH];
static int      loop_head;       /* next slot to read  */
static int      loop_tail;       /* next slot to write */
static int      loop_tx_calls;

static int loop_init(void)
{
    return 0;
}

static int loop_tx(const uint8_t *data, uint16_t len)
{
    loop_tx_calls++;
    if (loop_tail >= LOOP_DEPTH) {
        return -ENOSPC;
    }
    if (len > LOOP_MAX_LEN) {
        len = LOOP_MAX_LEN;
    }
    memcpy(loop_buf[loop_tail], data, len);
    loop_len[loop_tail] = len;
    loop_tail++;
    return 0;
}

static int loop_rx(uint8_t *buf, uint16_t max_len)
{
    if (loop_head >= loop_tail) {
        return 0;                       /* non-blocking: nothing pending */
    }
    uint16_t len = loop_len[loop_head];

    if (len > max_len) {
        len = max_len;
    }
    memcpy(buf, loop_buf[loop_head], len);
    loop_head++;
    return (int)len;
}

static void loop_set_power(float level)
{
    ARG_UNUSED(level);
}

static const tapestry_transceiver_t loop_transceiver = {
    .type      = TRANSCEIVER_TYPE_UDP,
    .init      = loop_init,
    .tx        = loop_tx,
    .rx        = loop_rx,
    .set_power = loop_set_power,
};

static const tapestry_transceiver_t *const loop_set[] = { &loop_transceiver };

static void loop_reset(void)
{
    loop_head = 0;
    loop_tail = 0;
    loop_tx_calls = 0;
    memset(loop_buf, 0, sizeof(loop_buf));
    memset(loop_len, 0, sizeof(loop_len));
}

/* Number of frames queued but not yet drained. */
static int loop_pending(void)
{
    return loop_tail - loop_head;
}

/* The most recently transmitted frame, as raw bytes a test can inspect or
 * corrupt before draining it back. */
static uint8_t *loop_last(void)
{
    zassert_true(loop_tail > 0, "no frame has been transmitted");
    return loop_buf[loop_tail - 1];
}

/* ── Fixtures ────────────────────────────────────────────────────────────── */

/* Distinctive values in every field: a pack/unpack that crosses two fields
 * over, or drops one, cannot produce these by accident. */
static element_state_t sender_state(element_id_t id, bool achieved)
{
    element_state_t s  = {0};

    s.id               = id;
    s.position.x       = 12.25f;      /* exact in binary32 — no rounding    */
    s.position.y       = -3.5f;       /* signed, to catch a sign-dropping   */
    s.position.z       = 6.25f;       /* exact, distinct from x/y           */
    s.orientation.w    = 0.5f;        /* exact; not identity, and every     */
    s.orientation.x    = -0.5f;       /* component distinct/signed so a     */
    s.orientation.y    = 0.25f;       /* pack/unpack that crosses two of    */
    s.orientation.z    = -0.25f;      /* these cannot produce these by luck */
    s.logical_clock    = 0x0A0B0C0Du; /* all four bytes distinct            */
    s.update_seq       = 0xDEADBEEFu;
    s.energy_level     = 77u;
    s.health_flags     = ELEMENT_HEALTH_LOW_BATTERY | ELEMENT_HEALTH_DEGRADED;
    s.goal_achieved    = achieved;
    s.current_track    = 5u;          /* v4; distinct from id/energy above  */
    return s;
}

/* A receiver whose own id differs from the sender's, so drained frames are
 * not swallowed by the self-echo filter. */
static void receiver_init(world_model_t *wm, element_id_t own_id)
{
    element_state_t own = {0};

    own.id            = own_id;
    own.position.x    = 50.0f;
    own.position.y    = 50.0f;
    own.logical_clock = 1u;
    wm_init(wm, own_id, &own, 0.0f);
}

/* The relay flush is jittered 0-50 ms from first enqueue; sleeping past
 * that window is the only way to make a flush deterministic. */
#define RELAY_JITTER_SETTLE_MS 60

static void suite_before(void *fixture)
{
    ARG_UNUSED(fixture);
    gossip_register_transceivers(loop_set, 1);

#ifdef CONFIG_TAPESTRY_MESH_RELAY
    /* gossip.c's relay ring buffer and last-relayed clock table are file
     * statics with no reset hook, so anything a previous test queued would
     * be transmitted by the NEXT test's flush and counted as its own.
     * Drain the residue before zeroing the loopback, which makes each test
     * independent of the order the suite happens to run in.  Peers below
     * still use distinct element ids, so the last-relayed clock table
     * cannot leak between tests either. */
    k_msleep(RELAY_JITTER_SETTLE_MS);
    gossip_relay_flush();
#endif

    loop_reset();
}

ZTEST_SUITE(gossip_wire, NULL, NULL, suite_before, NULL, NULL);

/* ── Wire contract ───────────────────────────────────────────────────────── */

/*
 * The frame layout is mirrored by hand in three Python files (checked by
 * gen_wire_protocol.py --check) and is parsed byte-for-byte by peers that
 * may be running older firmware.  Pin the size here so a field added in the
 * middle of the struct is a test failure, not a silent interop break.
 */
ZTEST(gossip_wire, test_frame_sizes_match_the_documented_wire_contract)
{
    zassert_equal(TAPESTRY_GOSSIP_FRAME_SIZE, 43u,
                  "gossip frame must stay 43 bytes (wire.h v4 documents "
                  "'<BfffffffIIBBBBBB'); got %u", TAPESTRY_GOSSIP_FRAME_SIZE);
    /* Pinned together on purpose: a layout change that forgets the version
     * bump leaves older peers parsing the new frame as the old one, and a
     * version bump that forgets the Python mirrors leaves the orchestrators
     * decoding garbage.  Change the frame, change both, and run
     * tapestry-os/tools/gen_wire_protocol.py. */
    zassert_equal(TAPESTRY_WIRE_VERSION, 4u,
                  "wire version must be bumped with the frame layout; got %u",
                  TAPESTRY_WIRE_VERSION);
    zassert_equal(TAPESTRY_MSG_HEADER_SIZE, 5u,
                  "message header must stay 5 bytes ('<BBBH'); got %u",
                  TAPESTRY_MSG_HEADER_SIZE);

    /* version must stay LAST and id FIRST — transceiver_udp.c extracts
     * src_id from byte 0 before the header is populated, and new fields are
     * inserted before version, never after (wire.h). */
    tapestry_gossip_frame_t f = {0};
    const uint8_t *raw = (const uint8_t *)&f;

    f.id = 0xABu;
    zassert_equal(raw[0], 0xABu, "id must be the first byte on the wire");
    f.version = 0xCDu;
    zassert_equal(raw[TAPESTRY_GOSSIP_FRAME_SIZE - 1u], 0xCDu,
                  "version must be the last byte on the wire");
}

/* ── Hops 3-4: pack → wire → unpack ──────────────────────────────────────── */

/*
 * The core round trip.  Every field the sender owns must arrive in the
 * receiver's world model unchanged.
 */
ZTEST(gossip_wire, test_every_state_field_survives_the_round_trip)
{
    world_model_t wm;
    element_state_t own = sender_state(3, false);

    receiver_init(&wm, 0);

    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);
    zassert_equal(loop_pending(), 1, "send should queue exactly one frame");

    zassert_equal(gossip_drain(&wm, 0), 1, "drain should accept one frame");

    const wm_entry_t *e = wm_get_entry(&wm, 3);

    zassert_not_null(e, "sender should now have a world model entry");
    zassert_equal(e->state.id, 3, "id");
    zassert_true(e->state.position.x == 12.25f,
                 "position.x should survive unchanged (exact in binary32)");
    zassert_true(e->state.position.y == -3.5f,
                 "position.y should survive unchanged, sign included");
    zassert_true(e->state.position.z == 6.25f,
                 "position.z should survive unchanged");
    zassert_true(e->state.orientation.w ==  0.5f &&
                 e->state.orientation.x == -0.5f &&
                 e->state.orientation.y ==  0.25f &&
                 e->state.orientation.z == -0.25f,
                 "orientation (qw,qx,qy,qz) should survive unchanged, "
                 "signs included, with no cross-wiring between components");
    /* The world model applies the Lamport receive rule on the way in —
     * merged = max(local, received) + 1, and local is 0 for an entry we
     * have never seen — so the stored clock is the sent one plus one.
     * Asserting the raw value would be asserting the wrong layer. */
    zassert_equal(e->state.logical_clock, 0x0A0B0C0Du + 1u,
                  "logical_clock should arrive Lamport-merged, not raw");
    zassert_equal(e->state.update_seq, 0xDEADBEEFu, "update_seq");
    zassert_equal(e->state.energy_level, 77u, "energy_level");
    zassert_equal(e->state.health_flags,
                  ELEMENT_HEALTH_LOW_BATTERY | ELEMENT_HEALTH_DEGRADED,
                  "health_flags");
    /* Added to the frame in v4 and packed by gossip.c ever since, but
     * asserted nowhere until now — this test's name was already promising
     * it. */
    zassert_equal(e->state.current_track, 5u, "current_track");
}

/*
 * The bit this suite was written for.  A scope="all" step advances on
 * choreo_collective_achieved(), which reads nothing but peers' gossiped
 * goal_achieved.  Both polarities, because a hop that hardcodes false is
 * indistinguishable from a working one until somebody achieves something.
 */
ZTEST(gossip_wire, test_the_achieved_bit_round_trips_true)
{
    world_model_t wm;
    element_state_t own = sender_state(3, true);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);
    zassert_equal(gossip_drain(&wm, 0), 1, "frame should be accepted");

    const wm_entry_t *e = wm_get_entry(&wm, 3);

    zassert_not_null(e, "sender entry");
    zassert_true(e->state.goal_achieved,
                 "an achieved peer must arrive achieved — scope=\"all\" "
                 "advances on exactly this bit");
}

ZTEST(gossip_wire, test_the_achieved_bit_round_trips_false)
{
    world_model_t wm;
    element_state_t own = sender_state(3, false);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);
    zassert_equal(gossip_drain(&wm, 0), 1, "frame should be accepted");

    const wm_entry_t *e = wm_get_entry(&wm, 3);

    zassert_not_null(e, "sender entry");
    zassert_false(e->state.goal_achieved,
                  "an unachieved peer must not arrive achieved — a stuck-true "
                  "bit would advance a scope=\"all\" step early");
}

/*
 * The achieved bit is carried in its own byte and must not be confused with
 * the neighbouring relay_qos / health_flags bytes.
 */
ZTEST(gossip_wire, test_the_achieved_bit_occupies_its_own_wire_byte)
{
    element_state_t own = sender_state(3, true);

    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);

    const tapestry_gossip_frame_t *f =
        (const tapestry_gossip_frame_t *)loop_last();

    zassert_equal(f->achieved, 1u, "achieved must be encoded as 0/1");
    zassert_equal(f->health_flags,
                  ELEMENT_HEALTH_LOW_BATTERY | ELEMENT_HEALTH_DEGRADED,
                  "health_flags must not be clobbered by achieved");
}

/*
 * The QoS tier passed to gossip_send() must survive onto the wire, packed
 * into relay_qos alongside hop_count without disturbing it — both fields
 * share one byte (wire.h's TAPESTRY_PACK_RELAY_QOS).
 */
ZTEST(gossip_wire, test_qos_tier_is_packed_into_relay_qos_without_disturbing_hop_count)
{
    element_state_t own = sender_state(3, false);

    gossip_send(&own, TAPESTRY_QOS_HARD_RT);

    const tapestry_gossip_frame_t *f =
        (const tapestry_gossip_frame_t *)loop_last();

    zassert_equal(TAPESTRY_QOS_TIER(f->relay_qos), TAPESTRY_QOS_HARD_RT,
                  "qos tier must round-trip onto the wire");
    zassert_equal(TAPESTRY_HOP_COUNT(f->relay_qos),
                  IS_ENABLED(CONFIG_TAPESTRY_MESH_RELAY) ? 2u : 0u,
                  "packing qos must not disturb hop_count's bits");
    zassert_equal(f->relay_qos & ~(TAPESTRY_RELAY_QOS_HOP_MASK |
                                   TAPESTRY_RELAY_QOS_QOS_MASK), 0u,
                  "bits [7:4] of relay_qos are reserved and must be zero");
}

/*
 * Several elements gossiping in one cycle: the drain loop must consume the
 * whole queue, not just the first frame.
 */
ZTEST(gossip_wire, test_drain_consumes_every_queued_frame)
{
    world_model_t wm;

    receiver_init(&wm, 0);

    for (element_id_t id = 1; id <= 4; id++) {
        element_state_t s = sender_state(id, (id % 2u) == 0u);

        gossip_send(&s, TAPESTRY_QOS_SOFT_RT);
    }

    zassert_equal(gossip_drain(&wm, 0), 4, "all four frames should be drained");
    zassert_equal(loop_pending(), 0, "drain must leave the queue empty");

    for (element_id_t id = 1; id <= 4; id++) {
        const wm_entry_t *e = wm_get_entry(&wm, id);

        zassert_not_null(e, "peer %u should be known", id);
        zassert_equal(e->state.goal_achieved, (id % 2u) == 0u,
                      "peer %u achieved bit should not be cross-wired with "
                      "another peer's", id);
    }
}

/* ── Frames that must be rejected ────────────────────────────────────────── */

/*
 * A peer built against a different wire schema must be ignored rather than
 * misparsed — this is checked here (medium-agnostic) because BLE and
 * syslink advertise the frame with no header wrapper at all.
 */
ZTEST(gossip_wire, test_a_version_mismatch_is_dropped)
{
    world_model_t wm;
    element_state_t own = sender_state(3, true);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);

    ((tapestry_gossip_frame_t *)loop_last())->version =
        (uint8_t)(TAPESTRY_WIRE_VERSION + 1u);

    zassert_equal(gossip_drain(&wm, 0), 0, "wrong-version frame must drop");
    zassert_is_null(wm_get_entry(&wm, 3),
                    "a dropped frame must not create a world model entry");
}

/*
 * A truncated frame (medium cut it short) must not be parsed off the end of
 * the buffer.
 */
ZTEST(gossip_wire, test_a_short_frame_is_dropped)
{
    world_model_t wm;
    element_state_t own = sender_state(3, true);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);
    loop_len[loop_tail - 1] = (uint16_t)(sizeof(tapestry_gossip_frame_t) - 1u);

    zassert_equal(gossip_drain(&wm, 0), 0, "short frame must drop");
    zassert_is_null(wm_get_entry(&wm, 3), "no entry from a short frame");
}

/*
 * Hearing our own id back is an auto-ID collision on every medium that
 * cannot hear itself — 2026-07-19 flight 4 failed exactly this way.  The
 * frame must be dropped AND counted, because the count is what the
 * application reacts to when the console is gone.
 */
ZTEST(gossip_wire, test_an_own_id_frame_is_dropped_and_counted)
{
    world_model_t wm;
    element_state_t own = sender_state(7, true);
    uint32_t before = gossip_own_id_frames();

    receiver_init(&wm, 7);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);

    zassert_equal(gossip_drain(&wm, 7), 0, "own-id frame must not be applied");

    if (!IS_ENABLED(CONFIG_BT)) {
        zassert_equal(gossip_own_id_frames(), before + 1u,
                      "own-id frames must be counted as duplicate-ID "
                      "evidence (gossip_own_id_frames)");
    }
}

/* ── Discovery beacons ───────────────────────────────────────────────────── */

/*
 * Auto-ID rides the same frame with id = ELEMENT_ID_INVALID and the nonce
 * in update_seq, so it travels over any transceiver.  Round-trip it too:
 * the split between beacons and already-claimed ids happens on the same
 * bytes this suite already covers.
 */
ZTEST(gossip_wire, test_discovery_beacons_and_claimed_ids_split_correctly)
{
    uint32_t nonces[4] = {0};
    bool claimed[MAX_ELEMENTS] = {false};

    gossip_send_discovery(0xC0FFEE01u);
    gossip_send_discovery(0xC0FFEE02u);

    element_state_t running = sender_state(5, false);

    gossip_send(&running, TAPESTRY_QOS_SOFT_RT);

    int n = gossip_drain_discovery(nonces, 4, claimed, MAX_ELEMENTS);

    zassert_equal(n, 2, "both beacons should yield a nonce");
    zassert_equal(nonces[0], 0xC0FFEE01u, "first nonce");
    zassert_equal(nonces[1], 0xC0FFEE02u, "second nonce");
    zassert_true(claimed[5], "an already-running element claims its id");
    zassert_false(claimed[3], "no other id should be claimed");
}

/* ── Authenticated framing (auth.conf) ───────────────────────────────────── */

#ifdef CONFIG_TAPESTRY_WIRE_AUTH_ENABLED

/*
 * With auth on, the frame is followed by a 4-byte truncated HMAC-SHA256
 * tag.  Before this suite the whole path was compiled by nothing.
 */
ZTEST(gossip_wire, test_auth_appends_a_tag_and_still_round_trips)
{
    world_model_t wm;
    element_state_t own = sender_state(3, true);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);

    zassert_equal(loop_len[loop_tail - 1], TAPESTRY_GOSSIP_WIRE_SIZE,
                  "an authenticated frame carries %u tag bytes",
                  TAPESTRY_WIRE_AUTH_TAG_SIZE);
    zassert_equal(gossip_drain(&wm, 0), 1,
                  "a correctly signed frame must verify and be applied");

    const wm_entry_t *e = wm_get_entry(&wm, 3);

    zassert_not_null(e, "sender entry");
    zassert_true(e->state.goal_achieved,
                 "authentication must not disturb the payload");
}

/*
 * The point of the tag: a modified payload no longer verifies.  Flipping
 * the achieved bit is the interesting case — an attacker (or a corrupting
 * medium) advancing everyone's scope="all" step.
 */
ZTEST(gossip_wire, test_auth_rejects_a_tampered_payload)
{
    world_model_t wm;
    element_state_t own = sender_state(3, false);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);

    ((tapestry_gossip_frame_t *)loop_last())->achieved = 1u;

    zassert_equal(gossip_drain(&wm, 0), 0,
                  "a payload edited after signing must not verify");
    zassert_is_null(wm_get_entry(&wm, 3), "no entry from a rejected frame");
}

ZTEST(gossip_wire, test_auth_rejects_a_corrupt_tag)
{
    world_model_t wm;
    element_state_t own = sender_state(3, true);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);
    loop_last()[sizeof(tapestry_gossip_frame_t)] ^= 0xFFu;

    zassert_equal(gossip_drain(&wm, 0), 0, "a bad tag must drop the frame");
}

/*
 * An unauthenticated frame from a peer with auth off is exactly
 * TAPESTRY_GOSSIP_FRAME_SIZE bytes — no tag to check.  It must be dropped,
 * not read past the end of.
 */
ZTEST(gossip_wire, test_auth_rejects_an_untagged_frame)
{
    world_model_t wm;
    element_state_t own = sender_state(3, true);

    receiver_init(&wm, 0);
    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);
    loop_len[loop_tail - 1] = TAPESTRY_GOSSIP_FRAME_SIZE;

    zassert_equal(gossip_drain(&wm, 0), 0,
                  "a frame with no auth tag must drop when auth is required");
}

#endif /* CONFIG_TAPESTRY_WIRE_AUTH_ENABLED */

/* ── Opportunistic relay (relay.conf) ────────────────────────────────────── */

#ifdef CONFIG_TAPESTRY_MESH_RELAY

/*
 * First-party frames start at hop_count 2 only when relay is compiled in —
 * the TTL that caps relay depth at two hops.
 */
ZTEST(gossip_wire, test_relay_frames_start_with_two_hops)
{
    element_state_t own = sender_state(3, false);

    gossip_send(&own, TAPESTRY_QOS_SOFT_RT);

    const tapestry_gossip_frame_t *f =
        (const tapestry_gossip_frame_t *)loop_last();

    zassert_equal(TAPESTRY_HOP_COUNT(f->relay_qos), 2u,
                  "first-party frames start at hop_count 2 with relay on");
}

/*
 * The forwarding path: a frame heard from a peer is re-advertised with its
 * TTL decremented, so an element out of direct range can hear it.
 */
ZTEST(gossip_wire, test_a_received_frame_is_re_advertised_with_one_less_hop)
{
    world_model_t wm;
    element_state_t peer = sender_state(9, true);

    receiver_init(&wm, 0);
    peer.logical_clock = 0x1000u;      /* newer than anything relayed before */
    gossip_send(&peer, TAPESTRY_QOS_SOFT_RT);

    zassert_equal(gossip_drain(&wm, 0), 1, "peer frame should be applied");

    int queued_before = loop_pending();

    k_msleep(RELAY_JITTER_SETTLE_MS);
    gossip_relay_flush();

    zassert_equal(loop_pending(), queued_before + 1,
                  "the frame should be re-advertised exactly once");

    const tapestry_gossip_frame_t *r =
        (const tapestry_gossip_frame_t *)loop_last();

    zassert_equal(r->id, 9, "the relay must preserve the ORIGINATOR's id");
    zassert_equal(TAPESTRY_HOP_COUNT(r->relay_qos), 1u,
                  "hop_count must be decremented");
    zassert_true(r->achieved == 1u,
                 "a relayed frame must carry the originator's achieved bit — "
                 "an element two hops out reads it for scope=\"all\"");
}

/*
 * Depth cap: a frame that has already used its TTL is not forwarded again,
 * which is what keeps a dense mesh from broadcast-storming.
 */
ZTEST(gossip_wire, test_an_exhausted_ttl_is_not_relayed)
{
    world_model_t wm;
    element_state_t peer = sender_state(10, false);

    receiver_init(&wm, 0);
    peer.logical_clock = 0x2000u;
    gossip_send(&peer, TAPESTRY_QOS_SOFT_RT);
    ((tapestry_gossip_frame_t *)loop_last())->relay_qos =
        TAPESTRY_PACK_RELAY_QOS(0u, TAPESTRY_QOS_SOFT_RT);

    zassert_equal(gossip_drain(&wm, 0), 1, "the frame is still applied");

    int queued_before = loop_pending();

    k_msleep(RELAY_JITTER_SETTLE_MS);
    gossip_relay_flush();

    zassert_equal(loop_pending(), queued_before,
                  "a frame with no hops left must not be re-advertised");
}

/*
 * Duplicate suppression: the same clock heard twice (directly and via a
 * relay) must be forwarded once, or two elements relaying each other would
 * amplify every frame indefinitely.
 */
ZTEST(gossip_wire, test_a_repeated_clock_is_relayed_only_once)
{
    world_model_t wm;
    element_state_t peer = sender_state(11, false);

    receiver_init(&wm, 0);
    peer.logical_clock = 0x3000u;

    /* Count transmissions, not queue depth: gossip_drain consumes the
     * queue, so depth cannot distinguish "relayed then drained" from
     * "never relayed". */
    gossip_send(&peer, TAPESTRY_QOS_SOFT_RT);          /* tx 1 */
    zassert_equal(gossip_drain(&wm, 0), 1, "first copy applied");

    k_msleep(RELAY_JITTER_SETTLE_MS);
    gossip_relay_flush();                              /* tx 2 — the relay */
    zassert_equal(loop_tx_calls, 2, "the first copy should be relayed once");

    gossip_send(&peer, TAPESTRY_QOS_SOFT_RT);          /* tx 3 — same clock */
    gossip_drain(&wm, 0);

    k_msleep(RELAY_JITTER_SETTLE_MS);
    gossip_relay_flush();

    zassert_equal(loop_tx_calls, 3,
                  "a clock already relayed must not be relayed again — two "
                  "elements relaying each other would otherwise amplify "
                  "every frame indefinitely");
}

/*
 * Priority under pressure: when the relay queue (depth 8) is full, a
 * higher-qos incoming frame must evict the lowest-qos queued frame rather
 * than being dropped itself — a HARD_RT frame must never be the one
 * silently lost.
 */
ZTEST(gossip_wire, test_a_higher_qos_frame_evicts_the_lowest_qos_queued_frame)
{
    world_model_t wm;

    receiver_init(&wm, 0);

    /* Element ids 21-29, used nowhere else in this suite: relay_clock[] is
     * a file-static in gossip.c with no per-test reset hook (see
     * suite_before's comment), so a prior test's frame for a REUSED id
     * could already have set relay_clock[id] above whatever this test
     * sends, silently suppressing the enqueue this test depends on.
     * Ids never touched elsewhere start at relay_clock's zero default,
     * so any positive logical_clock here is unambiguously newer. */

    /* Fill the relay queue with 8 distinct-id SOFT_RT frames. */
    for (element_id_t id = 21; id <= 28; id++) {
        element_state_t peer = sender_state(id, false);

        peer.logical_clock = 1u;
        gossip_send(&peer, TAPESTRY_QOS_SOFT_RT);
        zassert_equal(gossip_drain(&wm, 0), 1, "peer %u applied", id);
    }

    /* One more, higher-qos, distinct id: the queue is already full, so
     * this must evict a queued SOFT_RT frame instead of being dropped. */
    element_state_t urgent = sender_state(29, false);

    urgent.logical_clock = 1u;
    gossip_send(&urgent, TAPESTRY_QOS_HARD_RT);
    zassert_equal(gossip_drain(&wm, 0), 1, "urgent peer applied");

    /* Reclaim the loopback's own fixed depth (LOOP_DEPTH) before the
     * flush — it is a test-harness limit, unrelated to the relay ring
     * buffer under test, and the 9 sends above already used most of it. */
    loop_reset();

    k_msleep(RELAY_JITTER_SETTLE_MS);
    gossip_relay_flush();

    int relayed_count = 0;
    bool saw_urgent    = false;

    for (int i = loop_head; i < loop_tail; i++) {
        const tapestry_gossip_frame_t *r =
            (const tapestry_gossip_frame_t *)loop_buf[i];

        relayed_count++;
        if (r->id == 29) {
            saw_urgent = true;
        }
    }
    zassert_equal(relayed_count, 8,
                  "queue depth still caps relayed frames at 8 — one SOFT_RT "
                  "frame must have been evicted, not appended past capacity");
    zassert_true(saw_urgent,
                 "the HARD_RT frame must be relayed — it must not be the "
                 "one dropped for capacity");
}

#endif /* CONFIG_TAPESTRY_MESH_RELAY */
