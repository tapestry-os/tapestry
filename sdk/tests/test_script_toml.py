"""
test_script_toml.py — the Choreo script parser (sdk/python/tapestry/
script_toml.py).

This is the surface a domain expert with no systems background edits: a
.choreo.toml is the ONLY file they touch to change a show.  Every test
below is therefore about the error the author gets back, not just the
value that comes out — a parser that accepts a typo silently is the
failure mode that matters here.
"""

import textwrap
from pathlib import Path

import pytest
from helpers import SCRIPT_TOML, solo

from tapestry.bse import BSEAnchorSelector, BSEFrame, BSEMotion
from tapestry.choreo import ChoreoCapabilities, ChoreoEvent, GoalShape, GoalType
from tapestry.script_toml import (NormalizedTransition, ScriptError,
                                  load_steps, load_tracks, parse_duration_ms,
                                  parse_file, parse_length_m, to_choreo_steps)


def script(body: str, tmp_path, name: str = "unit-test") -> Path:
    path = tmp_path / "unit-test.choreo.toml"
    path.write_text(f'choreo = "{name}"\n\n' + textwrap.dedent(body))
    return path


def one_step(params: str, tmp_path, goal: str = "hold"):
    return script(f"[[steps]]\n{goal} = {{ {params} }}\n", tmp_path)


# ── Duration syntax ──────────────────────────────────────────────────────────

@pytest.mark.parametrize("text,ms", [
    ("30s", 30_000), ("500ms", 500), ("45min", 2_700_000), ("2h", 7_200_000),
    ("0.5s", 500), (5, 5_000), (2.5, 2_500), ("7", 7_000),
    ("  30S  ", 30_000),                      # whitespace and case tolerated
])
def test_durations_parse_to_milliseconds(text, ms):
    assert parse_duration_ms(text, "w") == ms


@pytest.mark.parametrize("bad", ["abc", "s", "30sec", "-1s", 0, -3, True,
                                None, [1]])
def test_bad_durations_raise_scripterror(bad):
    """Including True: a bool is an int in Python, and "duration = true"
    must not silently become 1000 ms."""
    with pytest.raises(ScriptError):
        parse_duration_ms(bad, "steps[0]")


def test_a_duration_error_names_the_step_it_came_from():
    with pytest.raises(ScriptError, match=r"steps\[2\]"):
        parse_duration_ms("nope", "steps[2]")


# ── Length syntax ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("text,m", [
    ("25cm", 0.25), ("250mm", 0.25), ("500um", 0.0005), ("0.25m", 0.25),
    (1.5, 1.5), ("2", 2.0),
])
def test_lengths_parse_to_meters(text, m):
    assert parse_length_m(text, "w") == pytest.approx(m)


@pytest.mark.parametrize("bad", ["xm", "0", "-1cm", 0, -0.5, True, None])
def test_bad_lengths_raise_scripterror(bad):
    with pytest.raises(ScriptError):
        parse_length_m(bad, "steps[0]")


# ── Document shape ───────────────────────────────────────────────────────────

def test_a_minimal_script_parses(tmp_path):
    s = parse_file(script('[[steps]]\nhold = { duration = "10s" }\n', tmp_path))
    assert s.name == "unit-test"
    assert len(s.steps) == 1
    assert s.steps[0].goal == "hold"
    assert s.steps[0].max_duration_ms == 10_000


def test_total_timeout_is_the_sum_of_the_step_bounds(tmp_path):
    s = parse_file(script('''
        [[steps]]
        hold = { duration = "10s" }
        [[steps]]
        hold = { duration = "500ms" }
        ''', tmp_path))
    assert s.total_timeout_ms == 10_500


def test_a_missing_choreo_name_is_rejected(tmp_path):
    path = tmp_path / "x.choreo.toml"
    path.write_text('[[steps]]\nhold = { duration = "1s" }\n')
    with pytest.raises(ScriptError, match="missing 'choreo"):
        parse_file(path)


def test_an_empty_choreo_name_is_rejected(tmp_path):
    path = tmp_path / "x.choreo.toml"
    path.write_text('choreo = ""\n[[steps]]\nhold = { duration = "1s" }\n')
    with pytest.raises(ScriptError, match="missing 'choreo"):
        parse_file(path)


def test_an_unexpected_top_level_key_is_rejected(tmp_path):
    """A typo'd key must not be silently ignored — that is how an author
    thinks they set something they did not."""
    path = tmp_path / "x.choreo.toml"
    path.write_text('choreo = "x"\nspeed = 3\n[[steps]]\n'
                    'hold = { duration = "1s" }\n')
    with pytest.raises(ScriptError, match="unexpected top-level keys"):
        parse_file(path)


def test_a_script_with_no_steps_is_rejected(tmp_path):
    path = tmp_path / "x.choreo.toml"
    path.write_text('choreo = "x"\n')
    with pytest.raises(ScriptError, match="at least one"):
        parse_file(path)


def test_malformed_toml_is_reported_as_a_script_error(tmp_path):
    path = tmp_path / "x.choreo.toml"
    path.write_text('choreo = "x\n[[steps]]\n')
    with pytest.raises(ScriptError, match="not valid TOML"):
        parse_file(path)


def test_a_missing_file_raises_oserror(tmp_path):
    with pytest.raises(OSError):
        parse_file(tmp_path / "absent.choreo.toml")


# ── Step shape ───────────────────────────────────────────────────────────────

def test_a_step_needs_exactly_one_goal_key(tmp_path):
    with pytest.raises(ScriptError, match="exactly one goal key"):
        parse_file(script('[[steps]]\nduration = "1s"\n', tmp_path))
    with pytest.raises(ScriptError, match="exactly one goal key"):
        parse_file(script('[[steps]]\nhold = { duration = "1s" }\n'
                          'converge = { duration = "1s", target = [0, 0] }\n',
                          tmp_path))


def test_a_stray_key_beside_the_goal_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="unexpected keys"):
        parse_file(script('[[steps]]\nhold = { duration = "1s" }\n'
                          'note = "hi"\n', tmp_path))


def test_a_goal_must_be_a_table_of_parameters(tmp_path):
    with pytest.raises(ScriptError, match="must be a table"):
        parse_file(script('[[steps]]\nhold = "10s"\n', tmp_path))


def test_an_unknown_parameter_is_rejected_and_lists_the_allowed_ones(tmp_path):
    with pytest.raises(ScriptError, match=r"unknown parameter\(s\) \['spin'\]"):
        parse_file(one_step('duration = "1s", spin = 2', tmp_path))


def test_every_step_needs_a_time_bound(tmp_path):
    """Stricter than the C API on purpose: the bound is the robustness net
    that keeps a script from stalling in flight."""
    with pytest.raises(ScriptError, match="no time bound"):
        parse_file(script('[[steps]]\nhold = { }\n', tmp_path))


def test_duration_and_timeout_are_the_same_bound_and_conflict(tmp_path):
    with pytest.raises(ScriptError, match="not both"):
        parse_file(one_step('duration = "1s", timeout = "2s"', tmp_path))


def test_timeout_is_accepted_as_the_bound(tmp_path):
    s = parse_file(one_step('timeout = "3s"', tmp_path, goal="exchange"))
    assert s.steps[0].max_duration_ms == 3_000


# ── until / scope ────────────────────────────────────────────────────────────

def test_until_achieved_sets_the_advance_flag(tmp_path):
    s = parse_file(one_step('timeout = "5s", until = "achieved"', tmp_path,
                            goal="exchange"))
    assert s.steps[0].advance_on_achieved is True


def test_until_accepts_only_achieved(tmp_path):
    with pytest.raises(ScriptError, match="only supported value"):
        parse_file(one_step('timeout = "5s", until = "done"', tmp_path,
                            goal="exchange"))


def test_hold_rejects_the_achievement_parameters(tmp_path):
    """hold is trivially achieved in the current runtime, so
    until = "achieved" on a hold would advance on the first tick."""
    for param in ('until = "achieved"', 'eps = "10cm"', 'settle = "1s"'):
        with pytest.raises(ScriptError, match="not allowed on 'hold'"):
            parse_file(one_step(f'duration = "5s", {param}', tmp_path))


@pytest.mark.parametrize("scope,value", [("self", 0), ("all", 1)])
def test_scope_maps_to_the_choreo_scope_enum(scope, value, tmp_path):
    s = parse_file(one_step(
        f'timeout = "5s", until = "achieved", scope = "{scope}"', tmp_path,
        goal="exchange"))
    assert s.steps[0].scope == value


def test_scope_without_until_is_rejected(tmp_path):
    """It would have no effect — silently accepting it would let an author
    believe a step waits for the collective when it does not."""
    with pytest.raises(ScriptError, match="no effect without"):
        parse_file(one_step('timeout = "5s", scope = "all"', tmp_path,
                            goal="exchange"))


def test_an_unknown_scope_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="scope must be"):
        parse_file(one_step('timeout = "5s", until = "achieved", '
                            'scope = "everyone"', tmp_path, goal="exchange"))


# ── eps / settle / shift / path / requires ───────────────────────────────────

def test_eps_and_settle_are_converted_to_meters_and_milliseconds(tmp_path):
    s = parse_file(one_step('timeout = "5s", eps = "25cm", settle = "3s"',
                            tmp_path, goal="exchange"))
    assert s.steps[0].achieve_eps == pytest.approx(0.25)
    assert s.steps[0].achieve_hold_ms == 3_000


def test_shift_must_be_a_positive_integer(tmp_path):
    s = parse_file(one_step('timeout = "5s", shift = 2', tmp_path,
                            goal="exchange"))
    assert s.steps[0].slot_shift == 2
    for bad in ("0", "-1", "1.5", "true"):
        with pytest.raises(ScriptError, match="shift must be"):
            parse_file(one_step(f'timeout = "5s", shift = {bad}', tmp_path,
                                goal="exchange"))


def test_path_selects_the_arc_or_the_beeline(tmp_path):
    assert parse_file(one_step('timeout = "5s", path = "direct"', tmp_path,
                               goal="exchange")).steps[0].direct_path is True
    assert parse_file(one_step('timeout = "5s", path = "arc"', tmp_path,
                               goal="exchange")).steps[0].direct_path is False
    with pytest.raises(ScriptError, match='path must be "arc"'):
        parse_file(one_step('timeout = "5s", path = "sideways"', tmp_path,
                            goal="exchange"))


def test_requires_accumulates_a_capability_mask(tmp_path):
    s = parse_file(one_step('duration = "5s", '
                            'requires = ["locomotion", "sensing"]', tmp_path))
    assert s.steps[0].required_caps == (ChoreoCapabilities.LOCOMOTION
                                        | ChoreoCapabilities.SENSING)


def test_requires_rejects_a_bare_string_and_unknown_names(tmp_path):
    with pytest.raises(ScriptError, match="must be a list"):
        parse_file(one_step('duration = "5s", requires = "locomotion"',
                            tmp_path))
    with pytest.raises(ScriptError, match="unknown capability"):
        parse_file(one_step('duration = "5s", requires = ["telepathy"]',
                            tmp_path))


def test_abs_position_is_a_known_capability_name(tmp_path):
    s = parse_file(one_step('duration = "5s", target = [1, 2, 3], '
                            'requires = ["abs_position"]', tmp_path,
                            goal="converge"))
    assert s.steps[0].required_caps == ChoreoCapabilities.ABS_POSITION


# ── Effects (§12 Stage 5) ─────────────────────────────────────────────────────

def test_indicator_and_telemetry_tag_parse(tmp_path):
    s = parse_file(one_step('duration = "5s", indicator = "active", '
                            'telemetry_tag = "spraying"', tmp_path))
    assert s.steps[0].indicator == "active"
    assert s.steps[0].telemetry_tag == "spraying"


def test_effects_default_to_none(tmp_path):
    s = parse_file(one_step('duration = "5s"', tmp_path))
    assert s.steps[0].indicator is None
    assert s.steps[0].telemetry_tag is None


def test_unknown_indicator_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="unknown indicator"):
        parse_file(one_step('duration = "5s", indicator = "rainbow"',
                            tmp_path))


def test_indicator_none_is_not_a_valid_name(tmp_path):
    """There is no "none" string — a step that wants no override omits
    the key entirely; the runtime default already means that."""
    with pytest.raises(ScriptError, match="unknown indicator"):
        parse_file(one_step('duration = "5s", indicator = "none"', tmp_path))


def test_telemetry_tag_must_be_a_nonempty_string(tmp_path):
    with pytest.raises(ScriptError, match="non-empty string"):
        parse_file(one_step('duration = "5s", telemetry_tag = ""', tmp_path))
    with pytest.raises(ScriptError, match="non-empty string"):
        parse_file(one_step('duration = "5s", telemetry_tag = 5', tmp_path))


def test_effects_are_allowed_on_every_goal_key(tmp_path):
    """Common to every goal, same as requires/on — not restricted to one
    goal type."""
    for goal, extra in [("hold", ""), ("exchange", ""),
                        ("form", ", target = [0, 0, 0], radius = 1"),
                        ("move", ", target = [0, 0, 0]"),
                        ("converge", ", target = [0, 0, 0]"),
                        ("disperse", ", radius = 1")]:
        s = parse_file(one_step(f'duration = "5s", indicator = "idle"{extra}',
                                tmp_path, goal=goal))
        assert s.steps[0].indicator == "idle", goal


def test_effects_convert_to_choreo_step_fields(tmp_path):
    from tapestry.choreo import SubstrateSignal
    s = parse_file(one_step('duration = "5s", indicator = "degraded", '
                            'telemetry_tag = "watch"', tmp_path))
    steps = to_choreo_steps(s)
    assert steps[0].indicator == SubstrateSignal.DEGRADED
    assert steps[0].telemetry_tag == "watch"


def test_a_step_without_effects_converts_to_none_defaults(tmp_path):
    from tapestry.choreo import SubstrateSignal
    s = parse_file(one_step('duration = "5s"', tmp_path))
    steps = to_choreo_steps(s)
    assert steps[0].indicator == SubstrateSignal.NONE
    assert steps[0].telemetry_tag is None


# ── Derived capability floor / satisfiability warnings (§11) ────────────────
# Non-fatal: the runtime derives and enforces these (derived_caps() in
# choreo.c) whether or not `requires` declares them, so an unlisted one is
# not a script error — but it's worth flagging so an author isn't
# surprised by a runtime -EPERM on an element that lacks the capability.

def test_absolute_frame_warns_when_abs_position_not_declared(tmp_path):
    s = parse_file(one_step('duration = "5s", target = [1, 2, 3]', tmp_path,
                            goal="converge"))
    assert len(s.warnings) == 1
    assert "steps[0]" in s.warnings[0]
    assert "abs_position" in s.warnings[0]


def test_absolute_frame_no_warning_when_abs_position_declared(tmp_path):
    s = parse_file(one_step('duration = "5s", target = [1, 2, 3], '
                            'requires = ["abs_position"]', tmp_path,
                            goal="converge"))
    assert s.warnings == []


@pytest.mark.parametrize("frame_params", [
    'frame = "collective"',
    'frame = "element", anchor = { select = "leader" }',
])
def test_non_absolute_frame_has_no_abs_position_warning(frame_params, tmp_path):
    s = parse_file(one_step(f'duration = "5s", {frame_params}', tmp_path,
                            goal="converge"))
    assert s.warnings == []


@pytest.mark.parametrize("goal", ["hold", "exchange"])
def test_coordinate_free_goals_never_warn(goal, tmp_path):
    """HOLD/EXCHANGE never read frame at all (bse.py §5) — no floor to
    derive, regardless of the (unreachable, default) frame value."""
    s = parse_file(one_step('duration = "5s"', tmp_path, goal=goal))
    assert s.warnings == []


def test_spin_warns_when_locomotion_not_declared(tmp_path):
    s = parse_file(one_step(
        'duration = "60s", target = [0, 0, 0], radius = 3, '
        'spin = "0.15rad/s", requires = ["abs_position"]',
        tmp_path, goal="form"))
    assert len(s.warnings) == 1
    assert "locomotion" in s.warnings[0]


def test_spin_no_warning_when_locomotion_declared(tmp_path):
    s = parse_file(one_step(
        'duration = "60s", target = [0, 0, 0], radius = 3, '
        'spin = "0.15rad/s", requires = ["locomotion", "abs_position"]',
        tmp_path, goal="form"))
    assert s.warnings == []


def test_static_form_has_no_locomotion_warning(tmp_path):
    s = parse_file(one_step(
        'duration = "5s", target = [0, 0, 0], radius = 3, '
        'requires = ["abs_position"]', tmp_path, goal="form"))
    assert s.warnings == []


def test_orbit_preset_warns_about_the_locomotion_it_desugars_to(tmp_path):
    """§6.1 sugar goes through the same form+spin parsing as hand-written
    steps — the warning must apply to the desugared result, not be
    bypassed by the preset."""
    s = parse_file(one_step(
        'around = "leader", radius = "1m", rate = "0.15rad/s", '
        'duration = "60s"', tmp_path, goal="orbit"))
    assert len(s.warnings) == 1
    assert "locomotion" in s.warnings[0]


def test_tracks_warnings_use_track_and_step_index(tmp_path):
    # tracks[0] carries a filter so the later catch-all isn't shadowed —
    # this test is about the derived-cap warning's label, and a filterless
    # FIRST track would (correctly) add a §8.4 unreachable-track warning
    # for tracks[1] on top of the one being asserted here.
    p = tracks_script("""\
        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        converge = { target = [0, 0, 0], duration = "60s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "60s" }
        """, tmp_path)
    s = parse_file(p)
    assert len(s.warnings) == 1
    assert "tracks[0].steps[0]" in s.warnings[0]


# ── Coordinate-free vs. coordinate goals ─────────────────────────────────────

@pytest.mark.parametrize("goal", ["hold", "exchange"])
@pytest.mark.parametrize("param", ["target = [1, 2]", "radius = 3",
                                   'shape = "circle"'])
def test_coordinate_free_goals_reject_coordinates(goal, param, tmp_path):
    """hold and exchange reference the collective's own configuration; a
    coordinate on them is a category error, not a tolerable extra."""
    with pytest.raises(ScriptError, match="unknown parameter"):
        parse_file(one_step(f'timeout = "5s", {param}', tmp_path, goal=goal))


@pytest.mark.parametrize("goal", ["form", "move", "converge"])
def test_point_goals_need_a_three_number_target(goal, tmp_path):
    extra = ", radius = 3" if goal == "form" else ""
    s = parse_file(one_step(f'duration = "5s", target = [1.5, -2, 3.0]{extra}',
                            tmp_path, goal=goal))
    assert s.steps[0].target == (1.5, -2.0, 3.0)
    for bad in ("[1]", "[1, 2]", "[1, 2, 3, 4]", '"1,2,3"', "[1, 2, true]"):
        with pytest.raises(ScriptError, match=r"needs .*target = \[x, y, z\]"):
            parse_file(one_step(f'duration = "5s", target = {bad}{extra}',
                                tmp_path, goal=goal))


def test_form_requires_a_radius(tmp_path):
    """radius 0 assigns every element the SAME vertex — the whole
    collective to one point with only platform deconfliction between
    airframes."""
    with pytest.raises(ScriptError, match="'form' needs a radius"):
        parse_file(one_step('duration = "5s", target = [0, 0, 0]', tmp_path,
                            goal="form"))


def test_disperse_requires_a_radius(tmp_path):
    with pytest.raises(ScriptError, match="'disperse' needs a radius"):
        parse_file(one_step('duration = "5s"', tmp_path, goal="disperse"))


@pytest.mark.parametrize("shape", ["circle", "line", "grid"])
def test_form_shapes_are_recognised(shape, tmp_path):
    s = parse_file(one_step(f'duration = "5s", target = [0, 0, 0], '
                            f'radius = "2m", shape = "{shape}"', tmp_path,
                            goal="form"))
    assert s.steps[0].shape == shape


def test_an_unknown_shape_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="unknown shape"):
        parse_file(one_step('duration = "5s", target = [0, 0, 0], radius = 2, '
                            'shape = "spiral"', tmp_path, goal="form"))


# ── Frames + anchors (Choreo SDK Design doc §5, form/converge only) ────────

def test_frame_defaults_to_absolute_and_needs_a_target(tmp_path):
    s = parse_file(one_step('duration = "5s", target = [1, 2, 3]', tmp_path,
                            goal="converge"))
    assert s.steps[0].frame == "absolute"
    assert s.steps[0].target == (1.0, 2.0, 3.0)


def test_frame_collective_forbids_target(tmp_path):
    with pytest.raises(ScriptError, match="'target' has no effect"):
        parse_file(one_step('duration = "5s", frame = "collective", '
                            'target = [1, 2, 3]', tmp_path, goal="converge"))


def test_frame_collective_needs_no_target(tmp_path):
    s = parse_file(one_step('duration = "5s", frame = "collective"',
                            tmp_path, goal="converge"))
    assert s.steps[0].frame == "collective"
    assert s.steps[0].target is None


def test_anchor_without_element_frame_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="'anchor' has no effect"):
        parse_file(one_step('duration = "5s", '
                            'anchor = { select = "leader" }', tmp_path,
                            goal="converge"))
    with pytest.raises(ScriptError, match="'anchor' only applies"):
        parse_file(one_step('duration = "5s", frame = "collective", '
                            'anchor = { select = "leader" }', tmp_path,
                            goal="converge"))


def test_frame_element_needs_an_anchor(tmp_path):
    with pytest.raises(ScriptError, match='needs anchor = \\{ select'):
        parse_file(one_step('duration = "5s", frame = "element"', tmp_path,
                            goal="converge"))


@pytest.mark.parametrize("select", ["leader", "self", "lowest-energy"])
def test_frame_element_anchor_selectors_are_recognised(select, tmp_path):
    s = parse_file(one_step(
        f'duration = "5s", frame = "element", '
        f'anchor = {{ select = "{select}" }}', tmp_path, goal="converge"))
    assert s.steps[0].frame == "element"
    assert s.steps[0].anchor_select == select


def test_frame_element_anchor_id_selector(tmp_path):
    s = parse_file(one_step(
        'duration = "5s", frame = "element", '
        'anchor = { select = "id:7" }', tmp_path, goal="converge"))
    assert s.steps[0].anchor_select == "id"
    assert s.steps[0].anchor_id == 7


def test_frame_element_anchor_id_needs_an_integer(tmp_path):
    with pytest.raises(ScriptError, match="needs an integer"):
        parse_file(one_step(
            'duration = "5s", frame = "element", '
            'anchor = { select = "id:leader" }', tmp_path, goal="converge"))


@pytest.mark.parametrize("select", ["newest", "oldest"])
def test_frame_element_newest_oldest_are_deferred_not_silently_accepted(
        select, tmp_path):
    with pytest.raises(ScriptError, match="not yet implemented"):
        parse_file(one_step(
            f'duration = "5s", frame = "element", '
            f'anchor = {{ select = "{select}" }}', tmp_path, goal="converge"))


def test_an_unknown_anchor_selector_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="unknown anchor select"):
        parse_file(one_step(
            'duration = "5s", frame = "element", '
            'anchor = { select = "closest" }', tmp_path, goal="converge"))


def test_an_unknown_frame_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="unknown frame"):
        parse_file(one_step('duration = "5s", frame = "body"', tmp_path,
                            goal="converge"))


def test_frame_and_anchor_apply_to_form_too(tmp_path):
    s = parse_file(one_step(
        'duration = "5s", radius = 2, frame = "collective"', tmp_path,
        goal="form"))
    assert s.steps[0].frame == "collective"


def test_frame_and_anchor_convert_to_the_sdk_goal(tmp_path):
    s = parse_file(one_step(
        'duration = "5s", frame = "element", '
        'anchor = { select = "id:3" }', tmp_path, goal="converge"))
    goal = to_choreo_steps(s)[0].goal
    assert goal.frame == BSEFrame.ELEMENT
    assert goal.anchor == BSEAnchorSelector.ID
    assert goal.anchor_id == 3


# ── Motion: spin + orbit preset (Choreo SDK Design doc §6, form only) ──────

def test_spin_on_form_sets_motion_and_rate(tmp_path):
    s = parse_file(one_step(
        'duration = "60s", target = [0, 0, 0], radius = 3, spin = "0.15rad/s"',
        tmp_path, goal="form"))
    assert s.steps[0].motion == "spin"
    assert s.steps[0].spin_rate_radps == pytest.approx(0.15)


def test_spin_defaults_to_static(tmp_path):
    s = parse_file(one_step('duration = "5s", target = [0, 0, 0], radius = 3',
                            tmp_path, goal="form"))
    assert s.steps[0].motion == "static"
    assert s.steps[0].spin_rate_radps is None


@pytest.mark.parametrize("goal", ["converge", "hold", "exchange", "move", "disperse"])
def test_spin_only_applies_to_form(goal, tmp_path):
    """converge's target IS the frame origin — rotating the offset would
    be a no-op, so this is a category error, not a tolerable extra; the
    other goals never had coordinate-adjacent params at all."""
    with pytest.raises(ScriptError, match="unknown parameter"):
        parse_file(one_step('duration = "5s", spin = "0.1rad/s"', tmp_path,
                            goal=goal))


def test_spin_converts_to_the_sdk_goal(tmp_path):
    s = parse_file(one_step(
        'duration = "60s", target = [0, 0, 0], radius = 3, spin = "0.2rad/s"',
        tmp_path, goal="form"))
    goal = to_choreo_steps(s)[0].goal
    assert goal.motion == BSEMotion.SPIN
    assert goal.spin_rate_radps == pytest.approx(0.2)


def test_orbit_desugars_to_a_form_step(tmp_path):
    s = parse_file(one_step(
        'around = "leader", radius = "1m", rate = "0.15rad/s", '
        'duration = "60s"', tmp_path, goal="orbit"))
    step = s.steps[0]
    assert step.goal == "form"
    assert step.shape == "circle"
    assert step.frame == "element"
    assert step.anchor_select == "leader"
    assert step.motion == "spin"
    assert step.spin_rate_radps == pytest.approx(0.15)
    assert step.radius == pytest.approx(1.0)
    assert step.max_duration_ms == 60_000


@pytest.mark.parametrize("missing", ["around", "radius", "rate"])
def test_orbit_needs_around_radius_and_rate(missing, tmp_path):
    params = {"around": '"leader"', "radius": '"1m"', "rate": '"0.15rad/s"'}
    del params[missing]
    body = ", ".join(f"{k} = {v}" for k, v in params.items())
    with pytest.raises(ScriptError, match=f"needs {missing}"):
        parse_file(one_step(f'duration = "60s", {body}', tmp_path,
                            goal="orbit"))


def test_orbit_rejects_unknown_parameters(tmp_path):
    with pytest.raises(ScriptError, match="unknown parameter"):
        parse_file(one_step(
            'around = "leader", radius = "1m", rate = "0.15rad/s", '
            'duration = "60s", shape = "line"', tmp_path, goal="orbit"))


# ── Conversion to SDK objects ────────────────────────────────────────────────

def test_to_choreo_steps_carries_every_field_across(tmp_path):
    # Inline tables must stay on one line in TOML; a step with many
    # parameters uses the [steps.<goal>] section form instead — the same
    # shape the shipped change-partners script uses for its exchange step.
    s = parse_file(script('''
        [[steps]]
        [steps.form]
        duration = "12s"
        target   = [4, 5, 6]
        radius   = "2m"
        shape    = "grid"
        requires = ["locomotion"]

        [[steps]]
        [steps.exchange]
        timeout = "30s"
        until   = "achieved"
        scope   = "all"
        path    = "direct"
        eps     = "25cm"
        settle  = "3s"
        shift   = 2
        ''', tmp_path))
    steps = to_choreo_steps(s)

    form = steps[0]
    assert form.goal.type == GoalType.FORM
    assert form.goal.target == (4.0, 5.0, 6.0)
    assert form.goal.radius == 2.0
    assert form.goal.shape == GoalShape.GRID
    assert form.goal.required_caps == ChoreoCapabilities.LOCOMOTION
    assert form.max_duration_ms == 12_000
    assert form.advance_on_achieved is False

    ex = steps[1]
    assert ex.goal.type == GoalType.EXCHANGE
    assert (ex.goal.slot_shift, ex.goal.direct_path) == (2, True)
    assert ex.goal.achieve_eps == pytest.approx(0.25)
    assert ex.goal.achieve_hold_ms == 3_000
    assert (ex.max_duration_ms, ex.advance_on_achieved, ex.scope) \
        == (30_000, True, 1)


def test_unauthored_fields_keep_the_sdk_defaults(tmp_path):
    """Only explicitly authored fields are set; everything else stays at
    the Goal default so the BSE applies its own (documented) fallback."""
    step = to_choreo_steps(parse_file(one_step('duration = "5s"', tmp_path)))[0]
    assert step.goal.type == GoalType.HOLD
    assert step.goal.achieve_eps == 0.0
    assert step.goal.achieve_hold_ms == 0
    assert step.goal.slot_shift == 0
    assert step.goal.direct_path is False


# ── Events + transitions (Choreo SDK Design doc §8, single track) ──────────

def test_named_step_is_a_valid_goto_target(tmp_path):
    s = parse_file(script('''
        [[steps]]
        name = "triangle"
        [steps.hold]
        duration = "60s"
        on = [ { event = "achieved", goto = "welcome" } ]

        [[steps]]
        name = "welcome"
        [steps.hold]
        duration = "60s"
        ''', tmp_path))
    assert s.steps[0].name == "triangle"
    assert s.steps[0].on == [
        NormalizedTransition(event="achieved", goto_step_idx=1, threshold=0)
    ]


def test_goto_end_resolves_to_the_step_count(tmp_path):
    s = parse_file(one_step(
        'duration = "60s", on = [ { event = "achieved", goto = "end" } ]',
        tmp_path))
    assert s.steps[0].on[0].goto_step_idx == 1   # len(steps) == 1


def test_goto_an_unknown_name_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="does not name a step"):
        parse_file(one_step(
            'duration = "60s", on = [ { event = "achieved", goto = "nope" } ]',
            tmp_path))


def test_duplicate_step_names_are_rejected(tmp_path):
    with pytest.raises(ScriptError, match="duplicate step name"):
        parse_file(script('''
            [[steps]]
            name = "a"
            [steps.hold]
            duration = "10s"

            [[steps]]
            name = "a"
            [steps.hold]
            duration = "10s"
            ''', tmp_path))


def test_end_is_a_reserved_step_name(tmp_path):
    with pytest.raises(ScriptError, match="reserved"):
        parse_file(script('''
            [[steps]]
            name = "end"
            [steps.hold]
            duration = "10s"
            ''', tmp_path))


def test_unknown_event_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="unknown event"):
        parse_file(one_step(
            'duration = "60s", on = [ { event = "elevenses", goto = "end" } ]',
            tmp_path))


def test_too_many_transitions_is_rejected(tmp_path):
    entries = ", ".join('{ event = "achieved", goto = "end" }' for _ in range(5))
    with pytest.raises(ScriptError, match="runtime only holds"):
        parse_file(one_step(f'duration = "60s", on = [ {entries} ]', tmp_path))


def test_count_events_need_a_threshold(tmp_path):
    with pytest.raises(ScriptError, match="needs a threshold"):
        parse_file(one_step(
            'duration = "60s", on = [ { event = "count_gte", goto = "end" } ]',
            tmp_path))


def test_threshold_only_applies_to_count_events(tmp_path):
    with pytest.raises(ScriptError, match="only applies to count_gte/count_eq"):
        parse_file(one_step(
            'duration = "60s", '
            'on = [ { event = "achieved", goto = "end", threshold = 2 } ]',
            tmp_path))


def test_a_cyclic_script_requires_max_runtime(tmp_path):
    """The welcome-dance shape (§8.3): step 0 <-> step 1 is a real cycle."""
    body = '''
        [[steps]]
        name = "triangle"
        [steps.hold]
        duration = "300s"
        on = [ { event = "element_joined", goto = "welcome" } ]

        [[steps]]
        name = "welcome"
        [steps.hold]
        duration = "30s"
        on = [ { event = "element_lost", goto = "triangle" } ]
        '''
    with pytest.raises(ScriptError, match="max_runtime"):
        parse_file(script(body, tmp_path))

    s = parse_file(script(f'max_runtime = "10min"\n{body}', tmp_path))
    assert s.total_timeout_ms == 600_000


def test_an_acyclic_script_does_not_need_max_runtime(tmp_path):
    s = parse_file(script('''
        [[steps]]
        hold = { duration = "10s" }
        [[steps]]
        hold = { duration = "5s" }
        ''', tmp_path))
    assert s.total_timeout_ms == 15_000


def test_on_converts_to_choreo_transitions(tmp_path):
    s = parse_file(script('''
        [[steps]]
        name = "a"
        [steps.hold]
        duration = "60s"
        on = [ { event = "count_gte", goto = "end", threshold = 3 } ]
        ''', tmp_path))
    goal_step = to_choreo_steps(s)[0]
    assert len(goal_step.on) == 1
    t = goal_step.on[0]
    assert t.event == ChoreoEvent.COUNT_GTE
    assert t.threshold == 3
    assert t.goto_step_idx == 1


def test_quorum_lost_event_is_recognised(tmp_path):
    s = parse_file(script('''
        [[steps]]
        name = "a"
        [steps.converge]
        target   = [1, 2, 3]
        duration = "60s"
        on = [ { event = "quorum_lost", goto = "b" } ]

        [[steps]]
        name = "b"
        [steps.hold]
        duration = "60s"
        ''', tmp_path))
    assert s.steps[0].on == [
        NormalizedTransition(event="quorum_lost", goto_step_idx=1, threshold=0)
    ]
    t = to_choreo_steps(s)[0].on[0]
    assert t.event == ChoreoEvent.QUORUM_LOST
    assert t.threshold == 0


def test_quorum_lost_needs_no_threshold(tmp_path):
    """Rejected on every event except count_gte/count_eq — quorum_lost
    must not be treated as one of those two."""
    with pytest.raises(ScriptError, match="only applies to count_gte/count_eq"):
        parse_file(one_step(
            'duration = "60s", '
            'on = [ { event = "quorum_lost", goto = "end", threshold = 2 } ]',
            tmp_path))


# ── The committed script ─────────────────────────────────────────────────────

def test_the_shipped_change_partners_script_still_parses():
    """The one script this repository ships and flies.  If a parser change
    ever rejects it, that is a breaking change to a released show."""
    s = parse_file(SCRIPT_TOML)
    assert s.name == "change-partners"
    assert [st.goal for st in s.steps] == ["hold", "exchange", "hold"]
    assert s.total_timeout_ms == 48_000


def test_the_shipped_script_converts_to_a_submittable_sequence():
    from tapestry.choreo import Choreo
    steps = load_steps(SCRIPT_TOML)
    assert Choreo(element_id=0, capabilities=None).submit_script(steps) == 0


# ── Tracks (§7) ──────────────────────────────────────────────────────────────

def tracks_script(body: str, tmp_path, name: str = "unit-test") -> Path:
    path = tmp_path / "unit-test.choreo.toml"
    path.write_text(f'choreo = "{name}"\n\n' + textwrap.dedent(body))
    return path


def test_a_tracks_script_parses_filters_and_per_track_steps(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "300s" }

        [[tracks]]
        [[tracks.steps]]
        converge = { target = [0, 0, 0], duration = "60s" }
        """, tmp_path)
    s = parse_file(p)
    assert s.steps == []
    assert s.tracks is not None and len(s.tracks) == 2
    assert s.tracks[0].required_caps == ChoreoCapabilities.SENSING
    assert s.tracks[0].requires_energy_low is False
    assert [st.goal for st in s.tracks[0].steps] == ["hold"]
    assert s.tracks[1].required_caps == 0
    assert [st.goal for st in s.tracks[1].steps] == ["converge"]


def test_energy_low_filter_parses(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        filter = { energy_low = true }
        [[tracks.steps]]
        converge = { target = [0, 0, 0], duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    s = parse_file(p)
    assert s.tracks[0].requires_energy_low is True
    assert s.tracks[1].requires_energy_low is False


# ── Track shadowing (§8.4 declaration-order warnings) ────────────────────────
# Selection is first-match-wins, so a track subsumed by an EARLIER track's
# filter can never be selected.  Non-fatal (warning, not error) — matches
# the derived-capability warnings' contract.  "unreachable" is the marker
# distinguishing these from the derived-cap warnings in the same list.

def shadowing_warnings(script):
    return [w for w in script.warnings if "unreachable" in w]


def test_catch_all_first_shadows_every_later_track(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    w = shadowing_warnings(parse_file(p))
    assert len(w) == 1
    assert "tracks[1]" in w[0] and "tracks[0]" in w[0]


def test_identical_filters_shadow_the_later_track(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    w = shadowing_warnings(parse_file(p))
    assert len(w) == 1
    assert "tracks[1]" in w[0]


def test_subset_requires_shadows_superset_later_track(tmp_path):
    """An element with sensing AND locomotion already matches the
    sensing-only track, so the stricter track after it is dead."""
    p = tracks_script("""\
        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        filter = { requires = ["sensing", "locomotion"] }
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    w = shadowing_warnings(parse_file(p))
    assert len(w) == 1
    assert "tracks[1]" in w[0]


def test_specific_filters_then_catch_all_last_is_clean(tmp_path):
    """The intended §7 pattern — no shadowing warning."""
    p = tracks_script("""\
        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        filter = { energy_low = true }
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    assert shadowing_warnings(parse_file(p)) == []


def test_energy_low_track_does_not_shadow_catch_all(tmp_path):
    """A healthy-battery element skips an energy_low track, so a later
    catch-all is still reachable — the energy constraint must be compared
    directionally, not just the caps mask."""
    p = tracks_script("""\
        [[tracks]]
        filter = { energy_low = true }
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    assert shadowing_warnings(parse_file(p)) == []


def test_catch_all_does_shadow_later_energy_low_track(tmp_path):
    """The reverse direction: every energy-low element also matches the
    earlier catch-all, so the energy_low track after it is dead."""
    p = tracks_script("""\
        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        filter = { energy_low = true }
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    w = shadowing_warnings(parse_file(p))
    assert len(w) == 1
    assert "tracks[1]" in w[0]


def test_shadowed_track_reports_first_shadower_only(tmp_path):
    """Two catch-alls before a filtered track: one warning for each dead
    track, each naming tracks[0] (the track the runtime would pick), not
    one warning per (shadower, shadowed) pair."""
    p = tracks_script("""\
        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }

        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    w = shadowing_warnings(parse_file(p))
    assert len(w) == 2
    assert "tracks[1]" in w[0] and "tracks[0]" in w[0]
    assert "tracks[2]" in w[1] and "tracks[0]" in w[1]


def test_steps_and_tracks_together_is_rejected(tmp_path):
    p = tracks_script("""\
        [[steps]]
        hold = { duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    with pytest.raises(ScriptError, match="either \\[\\[steps\\]\\] or \\[\\[tracks\\]\\]"):
        parse_file(p)


def test_neither_steps_nor_tracks_is_rejected(tmp_path):
    p = tracks_script("", tmp_path)
    with pytest.raises(ScriptError, match="at least one"):
        parse_file(p)


def test_too_many_tracks_is_rejected(tmp_path):
    body = "\n".join(
        '[[tracks]]\n[[tracks.steps]]\nhold = { duration = "10s" }\n'
        for _ in range(5))
    p = tracks_script(body, tmp_path)
    with pytest.raises(ScriptError, match="runtime only holds"):
        parse_file(p)


def test_a_track_needs_at_least_one_step(tmp_path):
    p = tracks_script("[[tracks]]\n", tmp_path)
    with pytest.raises(ScriptError, match="tracks.steps"):
        parse_file(p)


def test_unknown_filter_key_is_rejected(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        filter = { bogus = true }
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    with pytest.raises(ScriptError, match="unknown filter key"):
        parse_file(p)


def test_top_level_max_runtime_with_tracks_is_rejected(tmp_path):
    p = tracks_script("""\
        max_runtime = "10min"

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    with pytest.raises(ScriptError, match="top-level max_runtime"):
        parse_file(p)


def test_a_cyclic_track_requires_its_own_max_runtime(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        [[tracks.steps]]
        name = "a"
        hold = { duration = "10s", on = [{ event = "achieved", goto = "b" }] }
        [[tracks.steps]]
        name = "b"
        hold = { duration = "10s", on = [{ event = "achieved", goto = "a" }] }
        """, tmp_path)
    with pytest.raises(ScriptError, match="max_runtime"):
        parse_file(p)

    p2 = tracks_script("""\
        [[tracks]]
        max_runtime = "5min"
        [[tracks.steps]]
        name = "a"
        hold = { duration = "10s", on = [{ event = "achieved", goto = "b" }] }
        [[tracks.steps]]
        name = "b"
        hold = { duration = "10s", on = [{ event = "achieved", goto = "a" }] }
        """, tmp_path)
    s = parse_file(p2)
    assert s.tracks[0].total_timeout_ms == 300_000


def test_goto_only_resolves_within_the_same_track(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        [[tracks.steps]]
        name = "only-here"
        hold = { duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s", on = [{ event = "achieved", goto = "only-here" }] }
        """, tmp_path)
    with pytest.raises(ScriptError, match="does not name a step"):
        parse_file(p)


def test_to_choreo_tracks_converts_filters_and_steps(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        filter = { requires = ["sensing"] }
        [[tracks.steps]]
        hold = { duration = "300s" }

        [[tracks]]
        [[tracks.steps]]
        converge = { target = [1, 2, 0], duration = "60s" }
        """, tmp_path)
    tracks = load_tracks(p)
    assert len(tracks) == 2
    assert tracks[0].filter.required_caps == ChoreoCapabilities.SENSING
    assert tracks[0].filter.requires_energy_low is False
    assert tracks[0].steps[0].goal.type == GoalType.HOLD
    assert tracks[1].filter.required_caps == 0
    assert tracks[1].steps[0].goal.type == GoalType.CONVERGE
    assert tracks[1].steps[0].goal.target == (1.0, 2.0, 0.0)


def test_load_steps_rejects_a_tracks_script(tmp_path):
    p = tracks_script("""\
        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    with pytest.raises(ScriptError, match="multi-track script"):
        load_steps(p)


def test_load_tracks_rejects_a_steps_script(tmp_path):
    p = tracks_script("""\
        [[steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    with pytest.raises(ScriptError, match="single-track script"):
        load_tracks(p)


def test_a_tracks_script_is_submittable(tmp_path):
    from tapestry.choreo import Choreo
    p = tracks_script("""\
        [[tracks]]
        filter = { energy_low = true }
        [[tracks.steps]]
        converge = { target = [0, 0, 0], duration = "10s" }

        [[tracks]]
        [[tracks.steps]]
        hold = { duration = "10s" }
        """, tmp_path)
    tracks = load_tracks(p)
    c = Choreo(element_id=0, capabilities=None)
    assert c.submit_tracks(solo(), tracks) == 0
    assert c.current_track() == 1
