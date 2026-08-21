"""
test_bse.py — L6 Behavior Synthesis Engine (sdk/python/tapestry/bse.py).

The Python BSE is a tick-for-tick mirror of tapestry-os/subsys/bse/bse.c,
so these tests pin the geometry and the feedback controller: which vertex
an element is assigned, which station it swaps to, when the achievement
predicate fires.  A silent change here changes what every Choreo script
does in flight without any C file being touched.
"""

import math

import pytest
from helpers import QUORUM_HEALTHY, scr, solo, wm

from tapestry.bse import (ACHIEVE_EPS_DEFAULT, ACHIEVE_HOLD_MS_DEFAULT,
                          ANCHOR_HOLD_MS, EXCHANGE_OCCUPIED_M,
                          EXCHANGE_OMEGA_RADPS, EXCHANGE_STANDOFF_M,
                          WM_CYCLE_MS, BSE, BSEAnchorSelector, BSEDirective,
                          BSEDirectiveType, BSEFrame, BSEIntent,
                          BSEIntentType, BSEMotion, BSEShape)

HEALTHY = scr(QUORUM_HEALTHY)


def run(element_id, intent, entries, ticks=1, state=HEALTHY):
    """Submit an intent and tick it `ticks` times against a fixed world."""
    b = BSE(element_id)
    b.submit_intent(intent)
    for _ in range(ticks):
        b.tick(entries, state)
    return b


def approx_xy(target, expected, tol=1e-9):
    assert target[0] == pytest.approx(expected[0], abs=tol)
    assert target[1] == pytest.approx(expected[1], abs=tol)


# ── IDLE ─────────────────────────────────────────────────────────────────────

def test_idle_intent_takes_effect_before_the_first_tick():
    """Quiescence is immediate — the Choreographer stops ticking after
    terminate(), so submit_intent() itself must publish IDLE."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.IDLE))
    assert b.get_directive().type == BSEDirectiveType.IDLE


def test_idle_is_never_achieved():
    b = run(0, BSEIntent(type=BSEIntentType.IDLE), solo(), ticks=50)
    assert b.goal_achieved() is False


def test_unknown_intent_falls_back_to_idle():
    b = run(0, BSEIntent(type=99), solo())          # type: ignore[arg-type]
    assert b.get_directive().type == BSEDirectiveType.IDLE


# ── FORM ─────────────────────────────────────────────────────────────────────

def test_form_circle_places_each_rank_on_its_ngon_vertex():
    entries = wm([(0, 0)] * 4, self_id=0)
    for rank in range(4):
        b = run(rank, BSEIntent(type=BSEIntentType.FORM, target=(10.0, 20.0),
                                radius=5.0, shape=BSEShape.CIRCLE),
                wm([(0, 0)] * 4, self_id=rank))
        angle = 2.0 * math.pi * rank / 4
        approx_xy(b.get_directive().target,
                  (10.0 + 5.0 * math.cos(angle), 20.0 + 5.0 * math.sin(angle)))
    assert len(entries) == 4


def test_form_line_spreads_ranks_evenly_across_the_diameter():
    """N ranks on [target.x - radius, target.x + radius], y untouched."""
    targets = [run(r, BSEIntent(type=BSEIntentType.FORM, target=(0.0, 7.0),
                                radius=3.0, shape=BSEShape.LINE),
                   wm([(0, 0)] * 4, self_id=r)).get_directive().target
               for r in range(4)]
    assert [t[0] for t in targets] == [-3.0, -1.0, 1.0, 3.0]
    assert all(t[1] == 7.0 for t in targets)


def test_form_line_of_one_stays_on_the_target():
    """n == 1 would divide by zero in the even-spacing step; the guard
    leaves x on the target instead."""
    b = run(0, BSEIntent(type=BSEIntentType.FORM, target=(4.0, 9.0),
                         radius=3.0, shape=BSEShape.LINE), solo())
    approx_xy(b.get_directive().target, (4.0, 9.0))


def test_form_grid_is_a_centered_near_square():
    targets = [run(r, BSEIntent(type=BSEIntentType.FORM, target=(0.0, 0.0),
                                radius=2.0, shape=BSEShape.GRID),
                   wm([(0, 0)] * 4, self_id=r)).get_directive().target
               for r in range(4)]
    assert targets == [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)]


def test_form_grid_handles_a_non_square_count():
    """N=5 → 3 cols x 2 rows; radius is the cell spacing, not a radius."""
    targets = [run(r, BSEIntent(type=BSEIntentType.FORM, target=(0.0, 0.0),
                                radius=1.0, shape=BSEShape.GRID),
                   wm([(0, 0)] * 5, self_id=r)).get_directive().target
               for r in range(5)]
    assert targets[0] == (-1.0, -0.5)
    assert targets[2] == (1.0, -0.5)
    assert targets[3] == (-1.0, 0.5)


def test_form_ranks_by_sorted_element_id_not_world_model_order():
    """Vertex assignment must be order-independent: every element derives
    the same ring from the same ID set regardless of gossip arrival order."""
    entries = wm([(0, 0)] * 3, self_id=2)
    b_forward = run(2, BSEIntent(type=BSEIntentType.FORM, radius=10.0), entries)
    b_reverse = run(2, BSEIntent(type=BSEIntentType.FORM, radius=10.0),
                    list(reversed(entries)))
    assert b_forward.get_directive().target == b_reverse.get_directive().target


def test_form_excludes_stale_and_inactive_peers_from_the_ring():
    """A stale peer must not hold a vertex — three bodies, one stale, is a
    two-element formation."""
    b = run(0, BSEIntent(type=BSEIntentType.FORM, target=(0.0, 0.0),
                         radius=1.0), wm([(0, 0)] * 3, self_id=0, stale=[2]))
    two_up = run(0, BSEIntent(type=BSEIntentType.FORM, target=(0.0, 0.0),
                              radius=1.0), wm([(0, 0)] * 2, self_id=0))
    assert b.get_directive().target == two_up.get_directive().target


def test_form_includes_self_even_when_absent_from_the_world_model():
    """A solo element still gets vertex 0 of a 1-gon."""
    b = run(7, BSEIntent(type=BSEIntentType.FORM, target=(1.0, 2.0),
                         radius=4.0), [])
    approx_xy(b.get_directive().target, (5.0, 2.0))


# ── MOVE / CONVERGE / DISPERSE ───────────────────────────────────────────────

def test_move_translates_the_formation_rigidly():
    """Offset from the participant centroid, captured once, added to the
    target every tick — the shape survives the translation."""
    entries = wm([(-1, 0), (1, 0)], self_id=0)
    b = run(0, BSEIntent(type=BSEIntentType.MOVE, target=(10.0, 10.0)), entries)
    approx_xy(b.get_directive().target, (9.0, 10.0))


def test_move_offset_is_frozen_at_activation():
    """Re-deriving the offset each tick would let the formation drift with
    its own motion; the snapshot is what makes MOVE rigid."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.MOVE, target=(10.0, 10.0)))
    b.tick(wm([(-1, 0), (1, 0)], self_id=0), HEALTHY)
    first = b.get_directive().target
    b.tick(wm([(5, 5), (6, 6)], self_id=0), HEALTHY)
    assert b.get_directive().target == first


def test_solo_move_degenerates_to_converge():
    """Zero offset from a one-element centroid — documented in bse.py."""
    b = run(0, BSEIntent(type=BSEIntentType.MOVE, target=(10.0, 10.0)),
            solo(3.0, 4.0))
    approx_xy(b.get_directive().target, (10.0, 10.0))


def test_move_holds_when_self_is_not_a_participant():
    b = run(5, BSEIntent(type=BSEIntentType.MOVE, target=(1.0, 1.0)), [])
    assert b.get_directive().type == BSEDirectiveType.HOLD


def test_converge_sends_every_element_to_the_same_point():
    for rank in range(3):
        b = run(rank, BSEIntent(type=BSEIntentType.CONVERGE, target=(2.0, 3.0)),
                wm([(0, 0), (5, 5), (9, 9)], self_id=rank))
        assert b.get_directive().target == (2.0, 3.0)


# ── Frames + anchors (Choreo SDK Design doc §5, FORM/CONVERGE only) ────────

def test_frame_absolute_is_unchanged_default():
    """frame left at BSEFrame.ABSOLUTE (the default) must behave exactly
    like every intent submitted before this feature existed."""
    b = run(0, BSEIntent(type=BSEIntentType.CONVERGE, target=(7.0, 3.0)),
            solo())
    d = b.get_directive()
    assert d.type == BSEDirectiveType.MOVE_TO_POINT
    approx_xy(d.target, (7.0, 3.0))


def test_frame_collective_converge_gathers_at_centroid():
    b = run(0, BSEIntent(type=BSEIntentType.CONVERGE, frame=BSEFrame.COLLECTIVE),
            wm([(0, 0), (4, 0)], self_id=0))
    approx_xy(b.get_directive().target, (2.0, 0.0))


def test_frame_collective_form_centers_shape_on_centroid():
    b = run(0, BSEIntent(type=BSEIntentType.FORM, shape=BSEShape.LINE,
                         radius=3.0, frame=BSEFrame.COLLECTIVE),
            wm([(0, 0), (10, 0)], self_id=0))
    # self is rank 0 of 2 on a LINE spanning [-3,+3] around centroid (5,0)
    assert b.get_directive().target[0] == pytest.approx(2.0)


def test_frame_element_anchor_debounces_before_locking():
    """A brand-new anchor resolution must not drive a directive until it
    has been stable for ANCHOR_HOLD_MS — same lesson as QUORUM_UP_MS."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE,
                              frame=BSEFrame.ELEMENT,
                              anchor=BSEAnchorSelector.SELF))
    entries = solo(3.0, 4.0)
    n_before = ANCHOR_HOLD_MS // WM_CYCLE_MS - 1
    for _ in range(n_before):
        b.tick(entries, HEALTHY)
    assert b.get_directive().type == BSEDirectiveType.HOLD, \
        "still debouncing before the hold time elapses"

    b.tick(entries, HEALTHY)
    b.tick(entries, HEALTHY)   # past ANCHOR_HOLD_MS
    d = b.get_directive()
    assert d.type == BSEDirectiveType.MOVE_TO_POINT
    approx_xy(d.target, (3.0, 4.0))


def test_frame_element_leader_anchor_tracks_live_and_falls_back_on_loss():
    b = BSE(1)
    entries = wm([(5, 5), (9, 9)], self_id=1)   # id0=(5,5), id1(self)=(9,9)
    scr_state = scr(QUORUM_HEALTHY, leader_id=0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE,
                              frame=BSEFrame.ELEMENT,
                              anchor=BSEAnchorSelector.LEADER))
    for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS + 1):
        b.tick(entries, scr_state)
    approx_xy(b.get_directive().target, (5.0, 5.0))

    # Leader goes stale — must fall back to HOLD immediately (undebounced
    # loss; debouncing only governs switching between VALID candidates).
    entries[0]['is_stale'] = True
    b.tick(entries, scr_state)
    assert b.get_directive().type == BSEDirectiveType.HOLD

    entries[0]['is_stale'] = False
    for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS + 1):
        b.tick(entries, scr_state)
    approx_xy(b.get_directive().target, (5.0, 5.0))


def test_frame_element_id_anchor_live_no_lag():
    """§5.3: element-frame anchors bind LIVE — once locked, the same
    anchor's position tracks every tick with no additional debounce."""
    b = BSE(0)
    entries = wm([(0, 0), (1, 1)], self_id=0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE,
                              frame=BSEFrame.ELEMENT,
                              anchor=BSEAnchorSelector.ID, anchor_id=1))
    for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS + 1):
        b.tick(entries, HEALTHY)
    assert b.get_directive().target[0] == pytest.approx(1.0)

    entries[1]['x'], entries[1]['y'] = 8.0, 8.0
    b.tick(entries, HEALTHY)
    assert b.get_directive().target[0] == pytest.approx(8.0)


def test_frame_element_lowest_energy_anchor():
    b = BSE(0)
    entries = wm([(0, 0), (5, 5), (9, 9)], self_id=0)
    entries[0]['energy_level'] = 90
    entries[1]['energy_level'] = 80
    entries[2]['energy_level'] = 20   # lowest
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE,
                              frame=BSEFrame.ELEMENT,
                              anchor=BSEAnchorSelector.LOWEST_ENERGY))
    for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS + 1):
        b.tick(entries, HEALTHY)
    approx_xy(b.get_directive().target, (9.0, 9.0))


# ── Motion: spin (Choreo SDK Design doc §6, FORM only) ─────────────────────

def test_motion_spin_rotates_the_form_vertex():
    b = BSE(0)
    entries = solo(5.0, 0.0)
    b.submit_intent(BSEIntent(type=BSEIntentType.FORM, shape=BSEShape.CIRCLE,
                              radius=5.0, target=(0.0, 0.0),
                              motion=BSEMotion.SPIN, spin_rate_radps=0.5))
    b.tick(entries, HEALTHY)   # t=0.1s
    theta = 0.5 * 0.1
    approx_xy(b.get_directive().target,
             (5.0 * math.cos(theta), 5.0 * math.sin(theta)), tol=1e-4)

    for _ in range(19):
        b.tick(entries, HEALTHY)   # t=2.0s total
    theta = 0.5 * 2.0
    approx_xy(b.get_directive().target,
             (5.0 * math.cos(theta), 5.0 * math.sin(theta)), tol=1e-4)


def test_motion_spin_ignored_by_converge():
    b = run(0, BSEIntent(type=BSEIntentType.CONVERGE, target=(4.0, 4.0),
                         motion=BSEMotion.SPIN, spin_rate_radps=1.0),
            solo(), ticks=50)
    approx_xy(b.get_directive().target, (4.0, 4.0))


def test_disperse_is_a_spring_field_at_the_requested_spacing():
    d = run(0, BSEIntent(type=BSEIntentType.DISPERSE, radius=7.0),
            wm([(0, 0), (1, 1)], self_id=0)).get_directive()
    assert d.type == BSEDirectiveType.MAINTAIN_SPRING
    assert d.spacing == 7.0


def test_disperse_falls_back_to_the_default_spacing_at_radius_zero():
    d = run(0, BSEIntent(type=BSEIntentType.DISPERSE, radius=0.0),
            solo()).get_directive()
    assert d.spacing == 30.0


def test_disperse_is_vacuously_achieved_solo():
    """No peer to disperse from — achieved once the hold duration elapses,
    same convention as choreo_collective_achieved()'s solo case."""
    b = run(0, BSEIntent(type=BSEIntentType.DISPERSE, radius=5.0), solo(),
            ticks=100)
    assert b.goal_achieved() is True


def test_disperse_is_not_achieved_while_a_peer_is_within_spacing():
    """Peer 1.0 apart, spacing requested is 5.0 — never spread enough."""
    b = run(0, BSEIntent(type=BSEIntentType.DISPERSE, radius=5.0),
            wm([(0, 0), (1, 0)], self_id=0), ticks=100)
    assert b.goal_achieved() is False


def test_disperse_is_achieved_once_the_nearest_peer_clears_spacing():
    """Peer 5.0 apart, spacing requested is 5.0 — right at the boundary,
    within the default eps slack — achieved after the hold duration."""
    b = run(0, BSEIntent(type=BSEIntentType.DISPERSE, radius=5.0),
            wm([(0, 0), (5, 0)], self_id=0), ticks=100)
    assert b.goal_achieved() is True


# ── HOLD ─────────────────────────────────────────────────────────────────────

def test_hold_captures_the_station_once_and_keeps_it():
    b = BSE(3)
    b.submit_intent(BSEIntent(type=BSEIntentType.HOLD))
    b.tick([{"id": 3, "x": 4.0, "y": 5.0, "is_active": True,
             "is_stale": False, "is_self": True}], HEALTHY)
    assert b.get_directive().target == (4.0, 5.0)
    # Drift must not move the station — that is the whole point of HOLD.
    b.tick([{"id": 3, "x": 9.0, "y": 9.0, "is_active": True,
             "is_stale": False, "is_self": True}], HEALTHY)
    assert b.get_directive().target == (4.0, 5.0)


def test_hold_is_trivially_achieved():
    """Staying is the goal; the step's duration governs (bse.py)."""
    b = run(0, BSEIntent(type=BSEIntentType.HOLD), solo(1.0, 1.0))
    assert b.goal_achieved() is True


def test_hold_without_a_known_own_position_defers_the_capture():
    """Capturing (0, 0) because the position was missing would command a
    flight to the origin — emit the HOLD directive and retry instead."""
    b = BSE(3)
    b.submit_intent(BSEIntent(type=BSEIntentType.HOLD))
    b.tick([{"id": 3, "is_active": True, "is_stale": False, "is_self": True}],
           HEALTHY)
    assert b.get_directive().type == BSEDirectiveType.HOLD
    assert b.goal_achieved() is False
    b.tick(solo(2.0, 2.0, element_id=3), HEALTHY)
    assert b.get_directive().target == (2.0, 2.0)


# ── EXCHANGE ─────────────────────────────────────────────────────────────────

PAIR = [(-1.0, 0.0), (1.0, 0.0)]


def test_exchange_needs_a_fresh_peer_before_it_can_capture():
    b = run(0, BSEIntent(type=BSEIntentType.EXCHANGE), solo())
    assert b.get_directive().type == BSEDirectiveType.HOLD


def test_exchange_ignores_stale_peers_when_capturing():
    b = run(0, BSEIntent(type=BSEIntentType.EXCHANGE),
            wm(PAIR, self_id=0, stale=[1]))
    assert b.get_directive().type == BSEDirectiveType.HOLD


def test_exchange_retries_the_capture_on_a_later_tick():
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE))
    b.tick(solo(-1.0, 0.0), HEALTHY)
    assert b.get_directive().type == BSEDirectiveType.HOLD
    b.tick(wm(PAIR, self_id=0), HEALTHY)
    assert b.get_directive().type == BSEDirectiveType.MOVE_TO_POINT


def test_exchange_destination_is_the_next_station_on_the_id_ring():
    """slot_shift 0 means 1 — rank+1 modulo the participant count."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE))
    b.tick(wm([(0, 0), (1, 0), (2, 0)], self_id=0), HEALTHY)
    assert b._ex["dest"] == (1.0, 0.0)

    b2 = BSE(0)
    b2.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, slot_shift=2))
    b2.tick(wm([(0, 0), (1, 0), (2, 0)], self_id=0), HEALTHY)
    assert b2._ex["dest"] == (2.0, 0.0)


def test_exchange_stations_are_a_frozen_snapshot():
    """Live-chasing a moving partner would never converge; the snapshot is
    what makes the swap terminate."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, direct_path=True))
    b.tick(wm(PAIR, self_id=0), HEALTHY)
    dest = b._ex["dest"]
    b.tick(wm([(-1, 0), (40, 40)], self_id=0), HEALTHY)
    assert b._ex["dest"] == dest


def test_exchange_arc_travels_ccw_and_lands_exactly_on_the_station():
    """Every element rotates the same direction, so pairwise separation is
    preserved for the whole maneuver."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE))
    b.tick(wm(PAIR, self_id=0), HEALTHY)
    assert b._ex["dtheta"] == pytest.approx(math.pi)
    # Peer moved off the destination, so the standoff no longer applies and
    # the raw arc is visible.
    clear = wm([(-1, 0), (40, 40)], self_id=0)
    b.tick(clear, HEALTHY)
    x, y = b.get_directive().target
    assert y < 0.0 and x < 0.0             # increasing angle from theta0 = pi

    ticks = 1
    while b._ex["progress"] < b._ex["dtheta"]:
        b.tick(clear, HEALTHY)
        ticks += 1
    approx_xy(b.get_directive().target, (1.0, 0.0), tol=1e-9)
    # ~pi radians at EXCHANGE_OMEGA_RADPS on a WM_CYCLE_MS tick.
    expected = math.pi / (EXCHANGE_OMEGA_RADPS * WM_CYCLE_MS * 0.001)
    assert ticks == pytest.approx(expected, rel=0.05)


def test_exchange_direct_path_skips_the_arc_entirely():
    """path="direct" beelines — safe only where deconfliction is vertical."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, direct_path=True))
    b.tick(wm(PAIR, self_id=0), HEALTHY)
    b.tick(wm([(-1, 0), (40, 40)], self_id=0), HEALTHY)
    assert b.get_directive().target == (1.0, 0.0)


def test_exchange_holds_off_an_occupied_destination():
    """Step skew: the partner has not vacated yet.  Wait on the approach
    line at EXCHANGE_STANDOFF_M instead of flying into it."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, direct_path=True))
    b.tick(wm(PAIR, self_id=0), HEALTHY)          # peer still on (1, 0)
    approx_xy(b.get_directive().target, (1.0 - EXCHANGE_STANDOFF_M, 0.0))
    assert b._goal_pt is None                     # achievement deferred


def test_exchange_releases_the_standoff_once_the_station_clears():
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, direct_path=True))
    b.tick(wm(PAIR, self_id=0), HEALTHY)
    vacated = wm([(-1, 0), (1.0 + EXCHANGE_OCCUPIED_M * 2, 0)], self_id=0)
    b.tick(vacated, HEALTHY)
    assert b.get_directive().target == (1.0, 0.0)
    assert b._goal_pt == (1.0, 0.0)


def test_exchange_defers_achievement_until_the_arc_completes():
    """A body tracking the arc perfectly would otherwise 'achieve' while
    still up to eps short of its station."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE))
    b.tick(wm(PAIR, self_id=0), HEALTHY)
    clear = wm([(-1, 0), (40, 40)], self_id=0)
    b.tick(clear, HEALTHY)
    assert b._goal_pt is None
    while b._ex["progress"] < b._ex["dtheta"]:
        b.tick(clear, HEALTHY)
    assert b._goal_pt == (1.0, 0.0)


# ── Achievement predicate ────────────────────────────────────────────────────

def test_achievement_requires_the_hold_time_to_accumulate():
    intent = BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0),
                       achieve_eps=1.0, achieve_hold_ms=300)
    b = BSE(0)
    b.submit_intent(intent)
    on_station = solo(0.1, 0.0)
    for _ in range(2):
        b.tick(on_station, HEALTHY)
        assert b.goal_achieved() is False
    b.tick(on_station, HEALTHY)
    assert b.goal_achieved() is True


def test_leaving_the_epsilon_ball_resets_the_accumulator():
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0),
                              achieve_eps=1.0, achieve_hold_ms=200))
    for _ in range(5):
        b.tick(solo(0.1, 0.0), HEALTHY)
    assert b.goal_achieved() is True
    b.tick(solo(5.0, 5.0), HEALTHY)
    assert b.goal_achieved() is False


def test_zero_valued_achievement_parameters_use_the_defaults():
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0)))
    inside = solo(ACHIEVE_EPS_DEFAULT * 0.9, 0.0)
    for _ in range(ACHIEVE_HOLD_MS_DEFAULT // WM_CYCLE_MS - 1):
        b.tick(inside, HEALTHY)
    assert b.goal_achieved() is False
    b.tick(inside, HEALTHY)
    assert b.goal_achieved() is True


def test_achievement_is_unchanged_while_own_position_is_unknown():
    """No self entry means no evidence either way — hold the last verdict
    rather than fabricating one."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0),
                              achieve_eps=1.0, achieve_hold_ms=100))
    b.tick(solo(0.0, 0.0), HEALTHY)
    assert b.goal_achieved() is True
    b.tick([{"id": 0, "is_active": True, "is_stale": False, "is_self": True}],
           HEALTHY)
    assert b.goal_achieved() is True


def test_submitting_a_new_intent_resets_goal_state():
    """Stale hold stations, exchange snapshots and MOVE offsets from the
    previous step must not leak into the next one."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.HOLD))
    b.tick(solo(4.0, 4.0), HEALTHY)
    assert b.goal_achieved() is True
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0)))
    assert b.goal_achieved() is False
    assert b._hold_station is None and b._ex is None and b._move_offset is None


def test_directive_defaults_are_the_documented_spring_values():
    d = BSEDirective()
    assert (d.type, d.spring_k, d.spacing) == (BSEDirectiveType.IDLE, 5.0, 30.0)


# ── Tracks (§7, bse_set_track_scope / collect_participants filter) ──────────

def test_track_scope_defaults_to_zero_and_counts_every_peer():
    entries = wm([(0.0, 0.0), (10.0, 0.0)], self_id=0)   # current_track absent -> 0
    b = run(0, BSEIntent(type=BSEIntentType.CONVERGE, frame=BSEFrame.COLLECTIVE),
           entries)
    approx_xy(b.get_directive().target, (5.0, 0.0))


def test_set_track_scope_excludes_a_different_track_peer_from_the_centroid():
    entries = wm([(0.0, 0.0), (10.0, 0.0), (100.0, 100.0)], self_id=0)
    entries[1]["current_track"] = 0   # same track as self
    entries[2]["current_track"] = 1   # different track

    b = BSE(0)
    b.set_track_scope(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, frame=BSEFrame.COLLECTIVE))
    b.tick(entries, HEALTHY)
    approx_xy(b.get_directive().target, (5.0, 0.0))


def test_set_track_scope_also_filters_form_rank_and_count():
    entries = wm([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], self_id=0)
    entries[1]["current_track"] = 0
    entries[2]["current_track"] = 1   # excluded from FORM's N and rank

    b = BSE(0)
    b.set_track_scope(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.FORM, shape=BSEShape.LINE,
                              target=(0.0, 0.0), radius=1.0))
    b.tick(entries, HEALTHY)
    # N=2 (self + peer 1 only), self is the lower id -> rank 0 -> x = -radius
    approx_xy(b.get_directive().target, (-1.0, 0.0))


def test_a_track_scope_no_peer_currently_matches_still_includes_self():
    b = BSE(0)
    b.set_track_scope(3)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, frame=BSEFrame.COLLECTIVE))
    b.tick(solo(2.0, 2.0), HEALTHY)
    approx_xy(b.get_directive().target, (2.0, 2.0))
