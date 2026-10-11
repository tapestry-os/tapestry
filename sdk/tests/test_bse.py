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

def form_targets(intent, positions, bounds=None):
    """Every element's FORM target, by id, all ticking the same world."""
    out = []
    for i in range(len(positions)):
        b = BSE(i)
        if bounds is not None:
            b.set_form_bounds(*bounds)
        b.submit_intent(intent)
        b.tick(wm(positions, self_id=i), HEALTHY)
        out.append(b.get_directive().target)
    return out


def assert_same_points(got, expected, tol=1e-9):
    """Same multiset of points, any order."""
    def key(t):
        return (round(t[0], 6), round(t[1], 6))
    assert len(got) == len(expected)
    for g, e in zip(sorted(got, key=key), sorted(expected, key=key),
                    strict=True):
        approx_xy(g, e, tol)


def test_form_circle_puts_one_element_on_each_ngon_vertex():
    """All co-located (every assignment ties): still each vertex once."""
    targets = form_targets(BSEIntent(type=BSEIntentType.FORM,
                                     target=(10.0, 20.0, 0.0), radius=5.0,
                                     shape=BSEShape.CIRCLE), [(0, 0)] * 4)
    assert_same_points(targets, [
        (10.0 + 5.0 * math.cos(2.0 * math.pi * k / 4),
         20.0 + 5.0 * math.sin(2.0 * math.pi * k / 4)) for k in range(4)])


def test_form_line_spreads_elements_evenly_across_the_diameter():
    """N slots on [target.x - radius, target.x + radius], y untouched."""
    targets = form_targets(BSEIntent(type=BSEIntentType.FORM,
                                     target=(0.0, 7.0, 0.0), radius=3.0,
                                     shape=BSEShape.LINE), [(0, 0)] * 4)
    assert sorted(t[0] for t in targets) == [-3.0, -1.0, 1.0, 3.0]
    assert all(t[1] == 7.0 for t in targets)


def test_form_line_of_one_stays_on_the_target():
    """n == 1 would divide by zero in the even-spacing step; the guard
    leaves x on the target instead."""
    b = run(0, BSEIntent(type=BSEIntentType.FORM, target=(4.0, 9.0, 0.0),
                         radius=3.0, shape=BSEShape.LINE), solo())
    approx_xy(b.get_directive().target, (4.0, 9.0))


def test_form_grid_is_a_centered_near_square():
    targets = form_targets(BSEIntent(type=BSEIntentType.FORM,
                                     target=(0.0, 0.0, 0.0), radius=2.0,
                                     shape=BSEShape.GRID), [(0, 0)] * 4)
    assert_same_points(targets, [(-1.0, -1.0), (1.0, -1.0),
                                 (-1.0, 1.0), (1.0, 1.0)])


def test_form_grid_handles_a_non_square_count():
    """N=5 → 3 cols x 2 rows; radius is the cell spacing, not a radius."""
    targets = form_targets(BSEIntent(type=BSEIntentType.FORM,
                                     target=(0.0, 0.0, 0.0), radius=1.0,
                                     shape=BSEShape.GRID), [(0, 0)] * 5)
    assert_same_points(targets, [(-1.0, -0.5), (0.0, -0.5), (1.0, -0.5),
                                 (-1.0, 0.5), (0.0, 0.5)])


RING4 = BSEIntent(type=BSEIntentType.FORM, target=(50.0, 50.0, 0.0),
                  radius=20.0, shape=BSEShape.CIRCLE)
# CIRCLE vertices of RING4: E, N, W, S.
E, N, W, S = (70.0, 50.0), (50.0, 70.0), (30.0, 50.0), (50.0, 30.0)


def test_form_takes_the_vertex_nearest_each_element():
    """Ids out of bearing order: each still takes the vertex beside it,
    not the one its id rank would give (which crosses the ring)."""
    targets = form_targets(RING4, [(48, 25), (75, 52), (27, 49), (52, 74)])
    for t, want in zip(targets, [S, E, W, N], strict=True):
        approx_xy(t, want)


def test_form_assignment_has_no_crossing_paths():
    """Two elements each nearer the other's 'rank' vertex: swap, not cross.
    Element 0 west of centre and 1 east of it; rank order would send 0 east
    and 1 west through each other."""
    targets = form_targets(BSEIntent(type=BSEIntentType.FORM,
                                     target=(50.0, 50.0, 0.0), radius=20.0,
                                     shape=BSEShape.LINE), [(35, 55), (65, 45)])
    approx_xy(targets[0], (30.0, 50.0))
    approx_xy(targets[1], (70.0, 50.0))


def test_form_assignment_holds_while_elements_drive_to_their_vertices():
    """Moving straight at its own vertex only makes the chosen assignment
    cheaper, so it never flips mid-transit."""
    start = [(50, 45), (40, 52), (58, 55), (47, 60)]
    first = form_targets(RING4, start)
    for f in (0.25, 0.5, 0.75, 1.0):
        moved = [(p[0] + f * (t[0] - p[0]), p[1] + f * (t[1] - p[1]))
                 for p, t in zip(start, first, strict=True)]
        assert form_targets(RING4, moved) == first


def test_form_assignment_ignores_world_model_order():
    """Every element derives the same assignment from the same set,
    regardless of gossip arrival order."""
    entries = wm([(48, 25), (75, 52), (27, 49), (52, 74)], self_id=2)
    forward = run(2, RING4, entries)
    reverse = run(2, RING4, list(reversed(entries)))
    assert forward.get_directive().target == reverse.get_directive().target


def test_form_bounds_slide_the_shape_inside():
    """A radius-20 ring around (85, 95) would reach x=105 and y=115: the
    whole ring slides to centre (70, 70) to fit [10, 90], shape unchanged."""
    intent = BSEIntent(type=BSEIntentType.FORM, target=(85.0, 95.0, 0.0),
                       radius=20.0, shape=BSEShape.CIRCLE)
    targets = form_targets(intent, [(0, 0)] * 4, bounds=(10.0, 10.0, 90.0, 90.0))
    assert_same_points(targets, [(90.0, 70.0), (70.0, 90.0),
                                 (50.0, 70.0), (70.0, 50.0)])


def test_form_bounds_centre_a_shape_wider_than_them():
    intent = BSEIntent(type=BSEIntentType.FORM, target=(20.0, 50.0, 0.0),
                       radius=30.0, shape=BSEShape.LINE)
    targets = form_targets(intent, [(0, 0)] * 2, bounds=(0.0, 0.0, 40.0, 100.0))
    assert_same_points(targets, [(-10.0, 50.0), (50.0, 50.0)])


def test_form_bounds_leave_a_shape_already_inside_alone():
    targets = form_targets(RING4, [(0, 0)] * 4, bounds=(10.0, 10.0, 90.0, 90.0))
    assert_same_points(targets, [E, N, W, S])


def test_form_bounds_clear_with_min_above_max():
    b = BSE(0)
    b.set_form_bounds(10.0, 10.0, 90.0, 90.0)
    b.set_form_bounds(1.0, 0.0, 0.0, 0.0)
    b.submit_intent(BSEIntent(type=BSEIntentType.FORM, target=(95.0, 50.0, 0.0),
                              radius=0.0))
    b.tick(wm([(0, 0)]), HEALTHY)
    approx_xy(b.get_directive().target, (95.0, 50.0))


def test_form_excludes_stale_and_inactive_peers_from_the_ring():
    """A stale peer must not hold a vertex — three bodies, one stale, is a
    two-element formation."""
    b = run(0, BSEIntent(type=BSEIntentType.FORM, target=(0.0, 0.0, 0.0),
                         radius=1.0), wm([(0, 0)] * 3, self_id=0, stale=[2]))
    two_up = run(0, BSEIntent(type=BSEIntentType.FORM, target=(0.0, 0.0, 0.0),
                              radius=1.0), wm([(0, 0)] * 2, self_id=0))
    assert b.get_directive().target == two_up.get_directive().target


def test_form_includes_self_even_when_absent_from_the_world_model():
    """A solo element still gets vertex 0 of a 1-gon."""
    b = run(7, BSEIntent(type=BSEIntentType.FORM, target=(1.0, 2.0, 0.0),
                         radius=4.0), [])
    approx_xy(b.get_directive().target, (5.0, 2.0))


# ── MOVE / CONVERGE / DISPERSE ───────────────────────────────────────────────

def test_move_translates_the_formation_rigidly():
    """Offset from the participant centroid, captured once, added to the
    target every tick — the shape survives the translation."""
    entries = wm([(-1, 0), (1, 0)], self_id=0)
    b = run(0, BSEIntent(type=BSEIntentType.MOVE, target=(10.0, 10.0, 0.0)), entries)
    approx_xy(b.get_directive().target, (9.0, 10.0))


def test_move_offset_is_frozen_at_activation():
    """Re-deriving the offset each tick would let the formation drift with
    its own motion; the snapshot is what makes MOVE rigid."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.MOVE, target=(10.0, 10.0, 0.0)))
    b.tick(wm([(-1, 0), (1, 0)], self_id=0), HEALTHY)
    first = b.get_directive().target
    b.tick(wm([(5, 5), (6, 6)], self_id=0), HEALTHY)
    assert b.get_directive().target == first


def test_solo_move_degenerates_to_converge():
    """Zero offset from a one-element centroid — documented in bse.py."""
    b = run(0, BSEIntent(type=BSEIntentType.MOVE, target=(10.0, 10.0, 0.0)),
            solo(3.0, 4.0))
    approx_xy(b.get_directive().target, (10.0, 10.0))


def test_move_holds_when_self_is_not_a_participant():
    b = run(5, BSEIntent(type=BSEIntentType.MOVE, target=(1.0, 1.0, 0.0)), [])
    assert b.get_directive().type == BSEDirectiveType.HOLD


def test_converge_sends_every_element_to_the_same_point():
    for rank in range(3):
        b = run(rank, BSEIntent(type=BSEIntentType.CONVERGE, target=(2.0, 3.0, 0.0)),
                wm([(0, 0), (5, 5), (9, 9)], self_id=rank))
        assert b.get_directive().target == (2.0, 3.0, 0.0)


# ── Frames + anchors (Choreo SDK Design doc §5, FORM/CONVERGE only) ────────

def test_frame_absolute_is_unchanged_default():
    """frame left at BSEFrame.ABSOLUTE (the default) must behave exactly
    like every intent submitted before this feature existed."""
    b = run(0, BSEIntent(type=BSEIntentType.CONVERGE, target=(7.0, 3.0, 0.0)),
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


def _target_table(*facts):
    """A FactTable holding the given (origin, stamp, x, y) target facts."""
    from tapestry import facts as F
    t = F.FactTable(0)
    for origin, stamp, x, y in facts:
        assert t.receive(F.Fact(F.FACT_TYPE_TARGET, origin, 7, 0,
                                F.FACT_IMMUTABLE | F.fact_prio(3), stamp,
                                (x, y, 0.0)))
    return t


def _landmark_converge():
    from tapestry import facts as F
    return BSEIntent(type=BSEIntentType.CONVERGE, frame=BSEFrame.ELEMENT,
                     anchor=BSEAnchorSelector.LANDMARK,
                     landmark_type=F.FACT_TYPE_TARGET, landmark_rank=0)


def test_frame_element_landmark_anchor_resolves_the_fact():
    """Wire v7: LANDMARK resolves to a shared fact's position — never to an
    element — so the finder moving or leaving changes nothing.  Mirrors
    test_frame_element_landmark_anchor_resolves_the_fact (C)."""
    b = BSE(0)
    entries = wm([(0, 0), (5, 5)], self_id=0)
    b.submit_intent(_landmark_converge())
    for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS * 3):
        b.tick(entries, HEALTHY)
    assert b.get_directive().type == BSEDirectiveType.HOLD
    assert b.anchor_lost() is True, "no target fact held yet"

    facts = _target_table((1, 3, 69.0, 68.4))
    b.tick(entries, HEALTHY, facts)
    approx_xy(b.get_directive().target, (69.0, 68.4))   # first lock: at once

    entries[1]['x'], entries[1]['y'] = 90.5, 68.7       # finder moves
    entries[1]['is_stale'] = True                       # ...and leaves
    b.tick(entries, HEALTHY, facts)
    approx_xy(b.get_directive().target, (69.0, 68.4))
    assert b.anchor_lost() is False


def test_frame_element_landmark_rank_change_is_debounced():
    from tapestry import facts as F
    b = BSE(0)
    entries = solo()
    facts = _target_table((2, 5, 70.0, 70.0))
    b.submit_intent(_landmark_converge())
    b.tick(entries, HEALTHY, facts)
    approx_xy(b.get_directive().target, (70.0, 70.0))

    facts.receive(F.Fact(F.FACT_TYPE_TARGET, 3, 7, 0,
                         F.FACT_IMMUTABLE | F.fact_prio(3), 2, (20.0, 20.0, 0.0)))
    for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS):
        b.tick(entries, HEALTHY, facts)
        approx_xy(b.get_directive().target, (70.0, 70.0))
    b.tick(entries, HEALTHY, facts)
    approx_xy(b.get_directive().target, (20.0, 20.0))


def test_frame_element_landmark_retraction_drops_the_anchor():
    b = BSE(0)
    entries = solo()
    facts = _target_table((2, 5, 70.0, 70.0))
    b.submit_intent(_landmark_converge())
    b.tick(entries, HEALTHY, facts)
    approx_xy(b.get_directive().target, (70.0, 70.0))
    facts.retract(facts.find(1, 0).key)
    b.tick(entries, HEALTHY, facts)
    assert b.get_directive().type == BSEDirectiveType.HOLD
    assert b.anchor_lost() is True


# ── Motion: spin (Choreo SDK Design doc §6, FORM only) ─────────────────────

def test_motion_spin_rotates_the_form_vertex():
    b = BSE(0)
    entries = solo(5.0, 0.0)
    b.submit_intent(BSEIntent(type=BSEIntentType.FORM, shape=BSEShape.CIRCLE,
                              radius=5.0, target=(0.0, 0.0, 0.0),
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
    b = run(0, BSEIntent(type=BSEIntentType.CONVERGE, target=(4.0, 4.0, 0.0),
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
    assert b.get_directive().target == (4.0, 5.0, 0.0)
    # Drift must not move the station — that is the whole point of HOLD.
    b.tick([{"id": 3, "x": 9.0, "y": 9.0, "is_active": True,
             "is_stale": False, "is_self": True}], HEALTHY)
    assert b.get_directive().target == (4.0, 5.0, 0.0)


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
    assert b.get_directive().target == (2.0, 2.0, 0.0)


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
    assert b._ex["dest"] == (1.0, 0.0, 0.0)

    b2 = BSE(0)
    b2.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, slot_shift=2))
    b2.tick(wm([(0, 0), (1, 0), (2, 0)], self_id=0), HEALTHY)
    assert b2._ex["dest"] == (2.0, 0.0, 0.0)


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
    x, y, _z = b.get_directive().target
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
    assert b.get_directive().target == (1.0, 0.0, 0.0)


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
    assert b.get_directive().target == (1.0, 0.0, 0.0)
    assert b._goal_pt == (1.0, 0.0, 0.0)


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
    assert b._goal_pt == (1.0, 0.0, 0.0)


# ── Achievement predicate ────────────────────────────────────────────────────

def test_achievement_requires_the_hold_time_to_accumulate():
    intent = BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0, 0.0),
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
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0, 0.0),
                              achieve_eps=1.0, achieve_hold_ms=200))
    for _ in range(5):
        b.tick(solo(0.1, 0.0), HEALTHY)
    assert b.goal_achieved() is True
    b.tick(solo(5.0, 5.0), HEALTHY)
    assert b.goal_achieved() is False


def test_zero_valued_achievement_parameters_use_the_defaults():
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0, 0.0)))
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
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0, 0.0),
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
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, target=(0.0, 0.0, 0.0)))
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
                              target=(0.0, 0.0, 0.0), radius=1.0))
    b.tick(entries, HEALTHY)
    # N=2 (self + peer 1 only), self is the lower id -> rank 0 -> x = -radius
    approx_xy(b.get_directive().target, (-1.0, 0.0))


def test_a_track_scope_no_peer_currently_matches_still_includes_self():
    b = BSE(0)
    b.set_track_scope(3)
    b.submit_intent(BSEIntent(type=BSEIntentType.CONVERGE, frame=BSEFrame.COLLECTIVE))
    b.tick(solo(2.0, 2.0), HEALTHY)
    approx_xy(b.get_directive().target, (2.0, 2.0))


# ── EXCHANGE never touches z ─────────────────────────────────────────────────

def test_exchange_arc_never_drifts_toward_the_peers_altitude():
    """EXCHANGE reassigns x/y stations only -- z stays fixed at this
    element's own altitude for the whole maneuver (bse.py's
    _exchange_capture() comment: whatever vertical separation elements
    have must survive a horizontal crossing, not get walked away)."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE))
    b.tick(wm([(-1.0, 0.0, 0.5), (1.0, 0.0, 1.5)], self_id=0), HEALTHY)
    assert b.get_directive().target[2] == pytest.approx(0.5)
    for _ in range(40):
        b.tick(wm([(-1.0, 0.0, 0.5), (1.0, 0.0, 1.5)], self_id=0), HEALTHY)
        assert b.get_directive().target[2] == pytest.approx(0.5)


def test_exchange_direct_path_staggered_altitude_skips_the_standoff():
    """A peer at a genuinely different altitude never occupies this
    element's real destination (dest x/y at OWN z) -- direct_path
    proceeds immediately."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, direct_path=True))
    b.tick(wm([(-1.0, 0.0, 0.3), (1.0, 0.0, 1.5)], self_id=0), HEALTHY)
    d = b.get_directive()
    assert d.target[0] == pytest.approx(1.0)
    assert d.target[2] == pytest.approx(0.3)


def test_exchange_direct_path_same_altitude_still_gets_a_standoff():
    """Without staggering, the swap partner really is at this element's
    destination -- the OCCUPIED_M/STANDOFF_M step-skew defense still
    applies exactly as it did before positions were 3D."""
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE, direct_path=True))
    b.tick(wm([(-1.0, 0.0, 1.0), (1.0, 0.0, 1.0)], self_id=0), HEALTHY)
    d = b.get_directive()
    assert d.target[2] == pytest.approx(1.0)
    assert -1.0 < d.target[0] < 1.0


def test_rotation_turns_the_form_shape():
    # A solo element is rank 0: its vertex sits along +x without rotation,
    # and rotation turns it about the frame origin.
    for rot in (0.0, math.pi / 2, math.radians(225.0)):
        b = BSE(0)
        b.submit_intent(BSEIntent(type=BSEIntentType.FORM, shape=BSEShape.CIRCLE,
                                  radius=5.0, target=(0.0, 0.0, 0.0),
                                  rotation_rad=rot))
        b.tick(solo(5.0, 0.0), HEALTHY)
        approx_xy(b.get_directive().target,
                  (5.0 * math.cos(rot), 5.0 * math.sin(rot)), tol=1e-4)


def test_rotation_adds_to_spin():
    b = BSE(0)
    b.submit_intent(BSEIntent(type=BSEIntentType.FORM, shape=BSEShape.CIRCLE,
                              radius=5.0, target=(0.0, 0.0, 0.0),
                              motion=BSEMotion.SPIN, spin_rate_radps=0.5,
                              rotation_rad=1.0))
    b.tick(solo(5.0, 0.0), HEALTHY)   # t=0.1s
    theta = 1.0 + 0.5 * 0.1
    approx_xy(b.get_directive().target,
              (5.0 * math.cos(theta), 5.0 * math.sin(theta)), tol=1e-4)
