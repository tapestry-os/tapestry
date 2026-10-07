"""
test_cutebot_discovery_scripts.py — the REAL authored spill / leak scripts
(examples/cutebot-formation/{spill,leak}.choreo.toml) driven end-to-end
through the Python Choreo engine, plus a check that the committed cutebot
ring header stays free of the discovery define.

Kept out of test_choreo.py / test_choreoc.py on purpose: those exercise the
general engine and compiler, while these depend on files that live under
examples/cutebot-formation.
"""

import math

import pytest
from helpers import (QUORUM_HEALTHY, REPO_ROOT, scr, wm)

from tapestry import facts as F
from tapestry.bse import WM_CYCLE_MS, BSEDirectiveType
from tapestry.choreo import Choreo
from tapestry.facts import FactTable

import choreoc

HEALTHY = scr(QUORUM_HEALTHY)


# ── spill.choreo.toml / leak.choreo.toml end-to-end ─────────────────────────
#
# These load and drive the REAL authored scripts (not hand-built inline
# ChoreoStep objects) through this same Python engine — the
# "script-authoring simulation" path sdk/CHOREO_SCRIPTS.md documents, and
# the most direct way to catch a TOML-authoring mistake a purely
# mechanism-level unit test (which never touches the actual file) would
# miss.  Steps are looked up BY NAME so a script edit that adds or reorders
# legs doesn't silently retarget an assertion.
#
# examples/cutebot-formation/tests (the C ztest suite) is NOT where this
# belongs: that suite's choreo_script.h is a single generated artifact
# compiled from whichever ONE script is currently "active" — choreoc.py's
# generated symbol names are fixed regardless of source script, so several
# scripts' compiled output cannot coexist in one translation unit.
# Positions are static here (no motion integration); this validates script
# STRUCTURE.

SPILL_TOML = REPO_ROOT / "examples" / "cutebot-formation" / "spill.choreo.toml"
LEAK_TOML  = REPO_ROOT / "examples" / "cutebot-formation" / "leak.choreo.toml"


def _names(path, track=None):
    from tapestry.script_toml import parse_file
    p = parse_file(path)
    steps = p.tracks[track].steps if track is not None else p.steps
    return [st.name for st in steps]


def test_spill_script_loads_and_self_discovery_ends_the_patrol():
    from tapestry.script_toml import load_steps
    steps = load_steps(SPILL_TOML)
    names = _names(SPILL_TOML)
    settled = names.index("settled")

    c = Choreo(element_id=0)
    entries = wm([(10.0, 20.0), (60.0, 65.0)], self_id=0)
    assert c.submit_script(steps) == 0
    assert c.script_step() == 0, "starts on leg1"

    entries[1]['discovered'] = True   # a peer discovering must not matter
    c.tick(entries, HEALTHY)
    assert c.script_step() == 0, \
        "a peer's discovery must not end this robot's patrol"

    entries[0]['discovered'] = True   # self discovers
    c.tick(entries, HEALTHY)
    assert c.script_step() == settled, \
        "own discovery must jump straight to the settled step, no debounce"

    # The step index updates immediately (_script_advance runs within
    # this same tick), but its own goal's directive lags one tick behind
    # (_script_advance runs AFTER bse.tick() each cycle — same as the real
    # C runtime's choreo_tick()).  One more tick to see the settled step's
    # own fresh capture, not the outgoing leg's stale directive.
    c.tick(entries, HEALTHY)
    d = c.get_directive()
    assert d.type == BSEDirectiveType.MOVE_TO_POINT, \
        "settled is a HOLD goal — station-keeps wherever this element was"
    assert d.target == pytest.approx((10.0, 20.0, 0.0))


def test_spill_patrol_leg_advances_on_own_achievement_not_the_groups():
    """Legs are self-scoped.  A searcher standing on its slot must advance
    even though its peer never reports achieved (the collective predicate
    would strand it)."""
    from tapestry.script_toml import load_steps
    steps = load_steps(SPILL_TOML)

    c = Choreo(element_id=0)
    # leg1: circle radius 20 around (32,32); 2 elements -> id0 is rank 0
    # -> angle 0 -> (52, 32).
    entries = wm([(52.0, 32.0), (12.0, 32.0)], self_id=0)
    assert c.submit_script(steps) == 0
    assert c.script_step() == 0
    for _ in range(40):                       # 4 s: eps + 2 s settle + slack
        c.tick(entries, HEALTHY)
    assert c.script_step() >= 1, \
        "a robot at its own slot must advance without waiting for its peer"


# ── leak.choreo.toml ────────────────────────────────────────────────────────
#
# The finder publishes where it found the leak as a TARGET fact (L4 shared
# facts, wire v7); every robot's table holds the same fact, and the ring
# anchors on it.  These tests hand each robot a table holding that fact, the
# state gossip leaves it in.

LEAK_FOUND_AT = (60.0, 65.0)
HIGH = F.FACT_IMMUTABLE | F.fact_prio(3)


def _finder_table():
    t = FactTable(0, epoch=0x0A0A)
    t.heard_digest(0)
    rc, _ = t.publish(F.FACT_TYPE_TARGET, LEAK_FOUND_AT + (0.0,), 0, HIGH)
    assert rc == F.WM_FACT_CREATED
    return t


def _table_for(me, finder):
    """Robot `me`'s table after the finder's fact frames reached it."""
    if me == 0:
        return finder
    t = FactTable(me, epoch=0x1000 + me)
    for rec in finder.pack():
        t.receive(rec)
    t.heard_digest(finder.digest())
    return t


def _leak_ring_targets(ids, finder_stale=False, positions=None):
    """Drive each element in `ids` through the real leak script: id0 (the
    finder) publishes LEAK_FOUND_AT, then backs out to a new position —
    and optionally drops out of the swarm — before anyone reaches "ring".
    Returns {id: ring vertex}."""
    from tapestry.script_toml import load_steps
    steps = load_steps(LEAK_TOML)
    names = _names(LEAK_TOML)
    found, ring = names.index("found"), names.index("ring")
    finder = _finder_table()
    positions = positions or [LEAK_FOUND_AT, (30.0, 30.0), (40.0, 80.0),
                              (75.0, 30.0)]

    targets = {}
    for me in ids:
        c = Choreo(element_id=me)
        entries = wm(positions, self_id=me)
        facts = _table_for(me, finder)
        assert c.submit_script(steps) == 0
        c.tick(entries, HEALTHY, facts)
        assert c.script_step() == found, \
            "landmark_added must park everyone on 'found' first"

        entries[0]['x'], entries[0]['y'] = 45.0, 50.0   # finder backs out
        if finder_stale:
            entries[0]['is_stale'] = True
        for _ in range(4000 // WM_CYCLE_MS + 2):         # the 4 s found hold
            c.tick(entries, HEALTHY, facts)
        assert c.script_step() == ring
        c.tick(entries, HEALTHY, facts)
        d = c.get_directive()
        assert d.type == BSEDirectiveType.MOVE_TO_POINT
        targets[me] = d.target
    return targets


def _check_ring(targets, side, centre=LEAK_FOUND_AT):
    pts = list(targets.values())
    for t in pts:
        r = math.hypot(t[0] - centre[0], t[1] - centre[1])
        assert r == pytest.approx(20.0, abs=1e-3), \
            f"vertex must be radius 20 from where the leak was FOUND, got {r}"
    near = [min(math.hypot(a[0] - b[0], a[1] - b[1])
                for j, b in enumerate(pts) if j != i)
            for i, a in enumerate(pts)]
    assert near == pytest.approx([side] * len(pts), abs=1e-3)


def test_leak_all_four_ring_the_found_point_including_the_finder():
    """No tracks: the finder takes a slot — four vertices 28.3 apart (a
    square), centred where it FOUND the marker, not where it is now."""
    _check_ring(_leak_ring_targets((0, 1, 2, 3)), 20.0 * math.sqrt(2.0))


def test_leak_three_survivors_make_an_equilateral_triangle():
    """The finder leaves the swarm after publishing: the centre stays put
    and three vertices are 34.6 apart."""
    _check_ring(_leak_ring_targets((1, 2, 3), finder_stale=True),
                20.0 * math.sqrt(3.0))


def test_leak_rejoiner_rings_the_agreed_point_not_a_peers_slot():
    """logs/ble-20261006-112812: id3 was switched off and on while the ring
    stood.  It latched peer id0's ring SLOT (90.5, 68.7) as the leak instead
    of the found point (69.0, 68.4) and drove into id0.  With shared facts
    it relearns the fact from any peer and rings the same point."""
    from tapestry.script_toml import load_steps
    steps = load_steps(LEAK_TOML)
    ring = _names(LEAK_TOML).index("ring")
    leak = (69.0, 68.4)

    finder = FactTable(0, epoch=0x0A0A)
    finder.heard_digest(0)
    finder.publish(F.FACT_TYPE_TARGET, leak + (0.0,), 0, HIGH)
    peer2 = _table_for(2, finder)            # any peer holds the same table

    c = Choreo(element_id=3)                  # rebooted: fresh engine...
    rejoined = FactTable(3, epoch=0x7777)     # ...and an empty table
    entries = wm([(90.5, 68.7), (69.0, 88.4), (48.0, 68.4), (50.0, 50.0)],
                 self_id=3)                   # the ring as it stands
    assert c.submit_script(steps) == 0
    for rec in peer2.pack():                  # hears peer 2 first
        rejoined.receive(rec)
    rejoined.heard_digest(peer2.digest())
    for _ in range(4000 // WM_CYCLE_MS + 3):
        c.tick(entries, HEALTHY, rejoined)
    assert c.script_step() == ring
    c.tick(entries, HEALTHY, rejoined)
    t = c.get_directive().target
    assert math.hypot(t[0] - leak[0], t[1] - leak[1]) == \
        pytest.approx(20.0, abs=1e-3), \
        f"the rejoiner's vertex must sit 20 from {leak}, got {t}"


def test_scripts_end_a_fruitless_lap_on_a_visible_searched_hold():
    """Both scripts must have a "searched" step that is orange (degraded)
    — the lap is a single pass, so a robot that never found anything needs
    somewhere visible to stop instead of looping into drift."""
    from tapestry.script_toml import parse_file
    for path in (SPILL_TOML, LEAK_TOML):
        steps = parse_file(path).steps
        names = [st.name for st in steps]
        s = steps[names.index("searched")]
        assert s.goal == "hold" and s.indicator == "degraded"
        assert names.index("searched") == max(i for i, n in enumerate(names)
                                              if n.startswith("leg")) + 1, \
            "searched must directly follow the last patrol leg"


# ── compiler output for the committed cutebot scripts ───────────────────────

def test_committed_ring_header_has_no_discovery_define():
    ring = REPO_ROOT / "examples" / "cutebot-formation" / "src" / "choreo_script.h"
    assert "CHOREO_USES_DISCOVERY" not in ring.read_text()


def test_spill_and_leak_scripts_use_discovery_and_only_leak_reads_landmarks():
    base = REPO_ROOT / "examples" / "cutebot-formation"
    out = base / "src" / "choreo_script.h"
    spill = choreoc.render(base / "spill.choreo.toml", out)[1]
    leak = choreoc.render(base / "leak.choreo.toml", out)[1]
    assert "#define CHOREO_USES_DISCOVERY" in spill
    assert "#define CHOREO_USES_LANDMARKS" not in spill
    assert "#define CHOREO_USES_DISCOVERY" in leak
    assert "#define CHOREO_USES_LANDMARKS" in leak
