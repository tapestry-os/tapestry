"""
test_facts.py — L4 shared facts, Python mirror (sdk/python/tapestry/facts.py).

The numeric expectations here (digests, Trickle send ticks) are literals on
purpose: tapestry-csm-sim/tests/src/facts_test.c asserts the SAME literals
against the C engine, so a divergence in either engine fails one side.
"""


from tapestry import facts as F
from tapestry.facts import Fact, FactTable


def synced(owner, epoch=0x1234):
    t = FactTable(owner, epoch)
    t.heard_digest(0)          # an empty peer table: synced, now inconsistent
    return t


TARGET = F.FACT_TYPE_TARGET
HIGH = F.FACT_IMMUTABLE | F.fact_prio(3)


# ── Publishing ──────────────────────────────────────────────────────────────

def test_publish_needs_a_synced_table():
    t = FactTable(0, 0x1234)
    rc, _ = t.publish(TARGET, (69.0, 68.4, 0.0), 0, HIGH)
    assert rc == F.WM_FACT_ERR_UNSYNCED
    t.heard_digest(0)
    rc, key = t.publish(TARGET, (69.0, 68.4, 0.0), 0, HIGH)
    assert rc == F.WM_FACT_CREATED
    assert key == (TARGET, 0, 0x1234, 0)
    f = t.get(key)
    assert f.stamp == 1 and f.flags == HIGH and f.live


def test_publish_needs_an_epoch():
    """Nothing has seeded the epoch (the C side does it in the first
    gossip_drain()) and none was set: publishing must fail loudly, not
    silently reuse epoch 0."""
    t = FactTable(0)
    t.heard_digest(0)
    assert t.publish(TARGET, (10, 10, 0))[0] == F.WM_FACT_ERR_NOEPOCH
    t.set_epoch(0x1234)
    assert t.publish(TARGET, (10, 10, 0))[0] == F.WM_FACT_CREATED


def test_sync_timeout_allows_a_solo_element_to_publish():
    t = FactTable(0, 1)
    t.tick(F.FACT_SYNC_TIMEOUT_MS - 100)
    assert t.publish(TARGET, (10, 10, 0))[0] == F.WM_FACT_ERR_UNSYNCED
    t.tick(100)
    assert t.publish(TARGET, (10, 10, 0))[0] == F.WM_FACT_CREATED


def test_publish_within_merge_radius_reuses_the_known_fact():
    t = synced(0)
    _, k0 = t.publish(TARGET, (69.0, 68.4, 0.0), 0, HIGH)
    rc, k1 = t.publish(TARGET, (75.0, 70.0, 0.0), 0, HIGH)
    assert rc == F.WM_FACT_REUSED and k1 == k0
    rc, k2 = t.publish(TARGET, (90.0, 90.0, 0.0), 0, HIGH)   # 29 units away
    assert rc == F.WM_FACT_CREATED and k2 != k0
    # Zones never merge by distance.
    assert t.publish(F.FACT_TYPE_ZONE, (69.0, 68.4, 0.0))[0] == F.WM_FACT_CREATED
    assert t.publish(F.FACT_TYPE_ZONE, (69.0, 68.4, 0.0))[0] == F.WM_FACT_CREATED


# ── Merge rules ─────────────────────────────────────────────────────────────

def test_immutable_lower_stamp_wins_mutable_higher_wins():
    t = synced(9)
    t.receive(Fact(TARGET, 1, 7, 0, HIGH, 5, (10, 10, 0)))
    assert not t.receive(Fact(TARGET, 1, 7, 0, HIGH, 6, (11, 10, 0)))
    assert t.receive(Fact(TARGET, 1, 7, 0, HIGH, 4, (12, 10, 0)))
    assert t.get((TARGET, 1, 7, 0)).pos == (12, 10, 0)

    z = F.FACT_TYPE_ZONE
    t.receive(Fact(z, 1, 7, 1, 0, 5, (10, 10, 0)))
    assert t.receive(Fact(z, 1, 7, 1, 0, 6, (20, 10, 0)))
    assert not t.receive(Fact(z, 1, 7, 1, 0, 3, (30, 10, 0)))
    assert t.get((z, 1, 7, 1)).pos == (20, 10, 0)


def test_retraction_beats_live_and_any_element_may_retract():
    a = synced(0)
    _, key = a.publish(TARGET, (50, 50, 0), 0, HIGH)
    b = synced(3)
    b.receive(a.get(key))
    assert b.retract(key) == 0                      # not the origin
    tomb = b.get(key)
    assert not tomb.live
    assert a.receive(tomb)
    assert a.count(TARGET) == 0 and a.tombstones(TARGET) == 1
    # A stale live copy cannot revive it.
    live = Fact(TARGET, 0, a.epoch, 0, HIGH, 1, (50, 50, 0))
    assert not a.receive(live)
    assert a.count(TARGET) == 0


def test_two_tombstones_of_one_key_keep_the_earlier():
    t = synced(9)
    t.receive(Fact(TARGET, 1, 7, 0, HIGH | F.FACT_RETRACTED, 9, (1, 1, 0)))
    assert t.receive(Fact(TARGET, 1, 7, 0, HIGH | F.FACT_RETRACTED, 8, (1, 1, 0)))
    assert not t.receive(Fact(TARGET, 1, 7, 0, HIGH | F.FACT_RETRACTED, 10, (1, 1, 0)))
    assert t.get((TARGET, 1, 7, 0)).stamp == 8


def test_tombstone_dropped_after_ttl():
    t = synced(9)
    t.receive(Fact(TARGET, 1, 7, 0, F.FACT_RETRACTED, 3, (1, 1, 0)))
    t.tick(F.FACT_TOMBSTONE_TTL_MS - 100)
    assert t.tombstones() == 1
    t.tick(100)
    assert t.tombstones() == 0 and t.held() == []


def test_duplicates_from_two_finders_converge_on_the_lower_stamp():
    t = synced(9)
    t.receive(Fact(TARGET, 2, 7, 0, HIGH, 3, (70, 68, 0)))
    # Higher stamp, within 15 units: dropped on arrival.
    assert not t.receive(Fact(TARGET, 1, 8, 0, HIGH, 4, (72, 66, 0)))
    # Same stamp, lower origin: it wins and replaces.
    assert t.receive(Fact(TARGET, 1, 8, 1, HIGH, 3, (71, 67, 0)))
    assert [f.key for f in t.live(TARGET)] == [(TARGET, 1, 8, 1)]


# ── Ranking ─────────────────────────────────────────────────────────────────

def test_rank_orders_by_stamp_then_origin():
    t = synced(9)
    t.receive(Fact(TARGET, 4, 1, 0, HIGH, 7, (10, 10, 0)))
    t.receive(Fact(TARGET, 2, 1, 0, HIGH, 7, (40, 10, 0)))
    t.receive(Fact(TARGET, 5, 1, 0, HIGH, 2, (80, 10, 0)))
    assert t.find(TARGET, 0).origin == 5
    assert t.find(TARGET, 1).origin == 2
    assert t.find(TARGET, 2).origin == 4
    assert t.find(TARGET, 3) is None
    assert t.count(TARGET) == 3


# ── Capacity ────────────────────────────────────────────────────────────────

def _fill_low(t, n):
    for i in range(n):
        t.receive(Fact(F.FACT_TYPE_WAYPOINT, i % 8, 1, i, 0, 100 + i,
                       (float(i), 0.0, 0.0)))


def test_full_table_evicts_lowest_priority_oldest_first():
    t = synced(9)
    _fill_low(t, F.WM_MAX_FACTS)
    assert len(t.held()) == F.WM_MAX_FACTS
    assert t.receive(Fact(TARGET, 3, 2, 0, HIGH, 1, (50, 50, 0)))
    assert t.evictions == 1
    assert t.last_evicted == (F.FACT_TYPE_WAYPOINT, 0, 1, 0)   # stamp 100
    assert t.last_evict_reason == F.WM_FACT_EVICT_CAPACITY
    # The evicted fact re-sent by a peer ranks below everything: dropped.
    assert not t.receive(Fact(F.FACT_TYPE_WAYPOINT, 0, 1, 0, 0, 100, (0, 0, 0)))
    assert t.evictions == 1
    assert t.count(TARGET) == 1


def test_per_origin_cap_replaces_the_origins_own_oldest(monkeypatch):
    monkeypatch.setattr(F, "FACT_MAX_PER_ORIGIN", 2)
    t = synced(9)
    w = F.FACT_TYPE_WAYPOINT
    t.receive(Fact(w, 1, 5, 0, 0, 10, (0, 0, 0)))
    t.receive(Fact(w, 1, 5, 1, 0, 11, (0, 0, 0)))
    t.receive(Fact(w, 2, 5, 0, 0, 1, (0, 0, 0)))
    assert t.receive(Fact(w, 1, 5, 2, 0, 12, (0, 0, 0)))
    assert t.last_evicted == (w, 1, 5, 0)
    assert t.last_evict_reason == F.WM_FACT_EVICT_PER_ORIGIN
    assert t.get((w, 2, 5, 0)) is not None
    # Same id, new boot (epoch): a separate allowance.
    assert t.receive(Fact(w, 1, 6, 0, 0, 13, (0, 0, 0)))
    assert t.evictions == 1


# ── Digest ──────────────────────────────────────────────────────────────────

def test_digest_literals_and_order_independence():
    a = FactTable(9)
    assert a.digest() == 0
    f1 = Fact(TARGET, 0, 0x1234, 0, HIGH, 1, (69.0, 68.4, 0.0))
    f2 = Fact(F.FACT_TYPE_HAZARD, 3, 0xBEEF, 2, F.fact_prio(1), 7, (10, 20, 0))
    assert F.record_hash(f1) == 0xBC77
    assert F.record_hash(f2) == 0x7BAC
    a.receive(f1)
    a.receive(f2)
    b = FactTable(8)
    b.receive(f2)
    b.receive(f1)
    assert a.digest() == b.digest() == 0x3823
    b.retract(f1.key)
    assert b.digest() != a.digest()


# ── Trickle ─────────────────────────────────────────────────────────────────

def _send_ticks(t, ticks, rnd=12345, hear=None):
    out = []
    for i in range(ticks):
        if hear is not None:
            hear(t, i)
        if t.trickle_tick(100, rnd):
            out.append(i)
    return out


def test_trickle_backs_off_to_imax_when_alone():
    t = FactTable(0)
    sends = _send_ticks(t, 400)
    # IMIN 250 ms doubling to 8 s; with rnd fixed the send point is the
    # same fraction of each interval.
    assert sends == [2, 6, 16, 31, 61, 121, 201, 281, 361]


def test_trickle_suppresses_after_k_consistent_digests():
    t = FactTable(0)
    _send_ticks(t, 200)                       # settled at IMAX
    def hear(tbl, i):
        if i % 10 == 0:                       # two matching neighbours
            tbl.heard_digest(tbl.digest())    # (FACT_K = 2) every second
            tbl.heard_digest(tbl.digest())
    assert _send_ticks(t, 400, hear=hear) == []


def test_trickle_restarts_at_imin_on_an_inconsistent_digest_or_new_fact():
    t = FactTable(0, 1)
    _send_ticks(t, 200)
    assert t.trickle_i_ms == F.FACT_IMAX_MS
    t.heard_digest(0xABCD)
    assert _send_ticks(t, 3) == [2]
    _send_ticks(t, 200)
    t.publish(TARGET, (1, 1, 0))
    assert _send_ticks(t, 3) == [2]


def test_trickle_restarts_at_imin_when_a_received_record_is_new():
    """Mirror of the C test: new data restarts Trickle, a repeat does not."""
    t = synced(0, 1)
    _send_ticks(t, 200)
    assert t.trickle_i_ms == F.FACT_IMAX_MS
    leak = Fact(TARGET, 31, 0x2d67, 0, F.FACT_IMMUTABLE, 1, (70, 70, 0))
    assert t.receive(leak)
    assert _send_ticks(t, 3) == [2]
    _send_ticks(t, 200)
    assert not t.receive(leak)
    assert t.trickle_i_ms == F.FACT_IMAX_MS
    cleared = Fact(TARGET, 31, 0x2d67, 0, F.FACT_IMMUTABLE | F.FACT_RETRACTED,
                   2, (70, 70, 0))
    assert t.receive(cleared)
    assert _send_ticks(t, 3) == [2]


def test_pack_rotates_two_records_at_a_time():
    t = synced(9)
    for i in range(3):
        t.receive(Fact(F.FACT_TYPE_WAYPOINT, 1, 1, i, 0, i + 1, (0, 0, 0)))
    assert [f.seq for f in t.pack()] == [0, 1]
    assert [f.seq for f in t.pack()] == [2, 0]
    assert [f.seq for f in t.pack()] == [1, 2]


# ── The rejoin case (logs/ble-20261006-112812) ──────────────────────────────

def test_rejoiner_adopts_the_gossiped_point_not_a_peer_position():
    finder = synced(0, epoch=0x0A0A)
    _, key = finder.publish(TARGET, (69.0, 68.4, 0.0), 0, HIGH)

    rejoiner = FactTable(3, epoch=0x7777)     # power-cycled: empty table
    for rec in finder.pack():
        rejoiner.receive(rec)
    rejoiner.heard_digest(finder.digest())
    f = rejoiner.find(TARGET, 0)
    assert f.key == key and f.pos == (69.0, 68.4, 0.0)
    assert rejoiner.digest() == finder.digest()
    # Driving over the marker afterwards reuses the agreed fact.
    assert rejoiner.publish(TARGET, (70.0, 69.0, 0.0), 0, HIGH) == \
        (F.WM_FACT_REUSED, key)


def test_rejoiner_cannot_publish_a_low_stamped_duplicate_before_sync():
    rejoiner = FactTable(3, epoch=0x7777)
    rc, _ = rejoiner.publish(TARGET, (70.0, 69.0, 0.0), 0, HIGH)
    assert rc == F.WM_FACT_ERR_UNSYNCED
