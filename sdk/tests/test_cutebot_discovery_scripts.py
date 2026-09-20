"""
test_cutebot_discovery_scripts.py — the REAL authored king / spill scripts
(examples/cutebot-formation/{king,spill}.choreo.toml) driven end-to-end
through the Python Choreo engine, plus a check that the committed cutebot
ring header stays free of the discovery define.

Kept out of test_choreo.py / test_choreoc.py on purpose: those exercise the
general engine and compiler, while these depend on files that live under
examples/cutebot-formation.
"""

import math

import pytest
from helpers import (QUORUM_HEALTHY, REPO_ROOT, scr, solo, wm)

from tapestry.bse import ANCHOR_HOLD_MS, WM_CYCLE_MS, BSEDirectiveType
from tapestry.choreo import Choreo

import choreoc

HEALTHY = scr(QUORUM_HEALTHY)


# ── king.choreo.toml / spill.choreo.toml end-to-end (2026-09-16/19) ─────────
#
# These load and drive the REAL authored scripts (not hand-built inline
# ChoreoStep/ChoreoTrack objects like the DISCOVERY/track tests above)
# through this same Python engine — the "script-authoring simulation"
# path sdk/CHOREO_SCRIPTS.md documents, and the most direct way to catch
# a TOML-authoring mistake a purely mechanism-level unit test (which
# never touches the actual file) would miss.  Steps are looked up BY NAME
# so a script edit that adds or reorders legs doesn't silently retarget
# an assertion.
#
# examples/cutebot-formation/tests (the C ztest suite) is NOT where this
# belongs: that suite's choreo_script.h is a single generated artifact
# compiled from whichever ONE script is currently "active" (today:
# ring.choreo.toml) — choreoc.py's generated symbol names are fixed
# regardless of source script, so king's and spill's compiled output
# cannot coexist with ring's in one translation unit.  Positions are
# static here (no motion integration); this validates script STRUCTURE.
# Physical behaviour of the whole 4-robot loop was checked with a separate
# host simulation (real bse/choreo/scr/formation, one process per robot).

KING_TOML  = REPO_ROOT / "examples" / "cutebot-formation" / "king.choreo.toml"
SPILL_TOML = REPO_ROOT / "examples" / "cutebot-formation" / "spill.choreo.toml"


def _names(path, track=None):
    from tapestry.script_toml import parse_file
    p = parse_file(path)
    steps = p.tracks[track].steps if track is not None else p.steps
    return [st.name for st in steps]


def test_king_script_loads_and_the_discoverer_migrates_to_the_anchor_track():
    from tapestry.script_toml import load_tracks
    tracks = load_tracks(KING_TOML)

    c = Choreo(element_id=0)
    entries = solo(10.0, 20.0)        # self at a fixed, arbitrary point
    entries[0]['discovered'] = True   # already discovered at submission time
    assert c.submit_tracks(entries, tracks) == 0
    # Track selection is synchronous/undebounced at submission time — see
    # choreo_submit_tracks()'s own doc.
    assert c.current_track() == 0, \
        "an already-discovered element must start on the anchor track"

    c.tick(entries, HEALTHY)
    d = c.get_directive()
    # A HOLD goal's steady-state directive is MOVE_TO_POINT to the
    # captured station (bse.h), not directive-type HOLD — see this
    # file's test_track_discovered_migration_is_debounced above.
    assert d.type == BSEDirectiveType.MOVE_TO_POINT
    assert d.target == pytest.approx((10.0, 20.0, 0.0))


def test_king_searcher_reacts_to_a_peers_discovery_and_rings_them():
    from tapestry.script_toml import load_tracks
    tracks = load_tracks(KING_TOML)
    names = _names(KING_TOML, track=1)
    ring = names.index("ring")

    c = Choreo(element_id=1)
    entries = wm([(60.0, 65.0), (10.0, 20.0)], self_id=1)   # id0 = finder
    assert c.submit_tracks(entries, tracks) == 0
    assert c.current_track() == 1, \
        "an undiscovered element must start on the catch-all searcher track"
    assert c.script_step() == 0, "starts on leg1"

    entries[0]['discovered']    = True    # id0 found it — self did not
    entries[0]['current_track'] = 0       # ...and has migrated to "anchor"
    c.tick(entries, HEALTHY)
    assert c.current_track() == 1, \
        "self must stay on the searcher track — only the finder migrates"
    assert c.script_step() == ring, \
        "discovery_any must jump straight to the ring step from a patrol leg"

    for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS + 2):
        c.tick(entries, HEALTHY)
    d = c.get_directive()
    assert d.type == BSEDirectiveType.MOVE_TO_POINT
    dist = math.hypot(d.target[0] - 60.0, d.target[1] - 65.0)
    assert dist == pytest.approx(20.0, abs=1e-3), \
        f"ring vertex must sit radius=20 from the discoverer, got {dist}"


def test_king_ring_divides_evenly_among_searchers_excluding_the_finder():
    """Fix 4 (track-scoped FORM count) end to end on the real script: two
    searchers plus the finder on the anchor track make a 2-slot ring, so
    the searchers sit exactly opposite each other (40 apart), NOT
    120 degrees apart as if the finder held a slot."""
    from tapestry.script_toml import load_tracks
    tracks = load_tracks(KING_TOML)
    names = _names(KING_TOML, track=1)
    ring = names.index("ring")

    targets = {}
    for me in (1, 2):
        c = Choreo(element_id=me)
        entries = wm([(60.0, 65.0), (30.0, 30.0), (40.0, 80.0)], self_id=me)
        entries[0]['discovered']    = True
        entries[0]['current_track'] = 0
        for e in entries[1:]:
            e['current_track'] = 1
        assert c.submit_tracks(entries, tracks) == 0
        for _ in range(ANCHOR_HOLD_MS // WM_CYCLE_MS + 4):
            c.tick(entries, HEALTHY)
        assert c.script_step() == ring
        targets[me] = c.get_directive().target
    sep = math.hypot(targets[1][0] - targets[2][0], targets[1][1] - targets[2][1])
    assert sep == pytest.approx(40.0, abs=1e-3), \
        f"2-slot ring around the finder must be a diameter apart, got {sep}"


def test_king_patrol_leg_advances_on_own_achievement_not_the_groups():
    """Fix 3: legs are self-scoped.  A searcher standing on its slot must
    advance even though its peer never reports achieved (the collective
    predicate would strand it)."""
    from tapestry.script_toml import load_tracks
    tracks = load_tracks(KING_TOML)

    c = Choreo(element_id=0)
    # leg1: circle radius 20 around (32,32); 2 same-track elements -> id0
    # is rank 0 -> angle 0 -> (52, 32).
    entries = wm([(52.0, 32.0), (12.0, 32.0)], self_id=0)
    for e in entries:
        e['current_track'] = 1
    assert c.submit_tracks(entries, tracks) == 0
    assert c.script_step() == 0
    for _ in range(40):                       # 4 s: eps + 2 s settle + slack
        c.tick(entries, HEALTHY)
    assert c.script_step() >= 1, \
        "a robot at its own slot must advance without waiting for its peer"


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


def test_scripts_end_a_fruitless_lap_on_a_visible_searched_hold():
    """Both scripts must have a "searched" step that is orange (degraded)
    — the lap is a single pass, so a robot that never found anything needs
    somewhere visible to stop instead of looping into drift."""
    from tapestry.script_toml import parse_file
    for path, track in ((KING_TOML, 1), (SPILL_TOML, None)):
        p = parse_file(path)
        steps = p.tracks[track].steps if track is not None else p.steps
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


def test_king_and_spill_scripts_use_discovery():
    base = REPO_ROOT / "examples" / "cutebot-formation"
    for name in ("king", "spill"):
        out = base / "src" / "choreo_script.h"
        text = choreoc.render(base / f"{name}.choreo.toml", out)[1]
        assert "#define CHOREO_USES_DISCOVERY" in text, name
