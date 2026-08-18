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
from helpers import SCRIPT_TOML

from tapestry.choreo import ChoreoCapabilities, GoalShape, GoalType
from tapestry.script_toml import (ScriptError, load_steps, parse_duration_ms,
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
def test_point_goals_need_a_two_number_target(goal, tmp_path):
    extra = ", radius = 3" if goal == "form" else ""
    s = parse_file(one_step(f'duration = "5s", target = [1.5, -2]{extra}',
                            tmp_path, goal=goal))
    assert s.steps[0].target == (1.5, -2.0)
    for bad in ("[1]", "[1, 2, 3]", '"1,2"', "[1, true]"):
        with pytest.raises(ScriptError, match=r"needs .*target = \[x, y\]"):
            parse_file(one_step(f'duration = "5s", target = {bad}{extra}',
                                tmp_path, goal=goal))


def test_form_requires_a_radius(tmp_path):
    """radius 0 assigns every element the SAME vertex — the whole
    collective to one point with only platform deconfliction between
    airframes."""
    with pytest.raises(ScriptError, match="'form' needs a radius"):
        parse_file(one_step('duration = "5s", target = [0, 0]', tmp_path,
                            goal="form"))


def test_disperse_requires_a_radius(tmp_path):
    with pytest.raises(ScriptError, match="'disperse' needs a radius"):
        parse_file(one_step('duration = "5s"', tmp_path, goal="disperse"))


@pytest.mark.parametrize("shape", ["circle", "line", "grid"])
def test_form_shapes_are_recognised(shape, tmp_path):
    s = parse_file(one_step(f'duration = "5s", target = [0, 0], '
                            f'radius = "2m", shape = "{shape}"', tmp_path,
                            goal="form"))
    assert s.steps[0].shape == shape


def test_an_unknown_shape_is_rejected(tmp_path):
    with pytest.raises(ScriptError, match="unknown shape"):
        parse_file(one_step('duration = "5s", target = [0, 0], radius = 2, '
                            'shape = "spiral"', tmp_path, goal="form"))


# ── Conversion to SDK objects ────────────────────────────────────────────────

def test_to_choreo_steps_carries_every_field_across(tmp_path):
    # Inline tables must stay on one line in TOML; a step with many
    # parameters uses the [steps.<goal>] section form instead — the same
    # shape the shipped change-partners script uses for its exchange step.
    s = parse_file(script('''
        [[steps]]
        [steps.form]
        duration = "12s"
        target   = [4, 5]
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
    assert form.goal.target == (4.0, 5.0)
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
