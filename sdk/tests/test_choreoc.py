"""
test_choreoc.py — the Choreo script compiler (sdk/tools/choreoc.py).

choreoc turns a .choreo.toml into a committed C header that firmware
builds consume without ever running Python.  Two things can go wrong and
both have: the emitted C can be wrong, and a committed header can drift
away from the script it claims to be generated from (examples/
webots-formation's copy sat a release behind examples/cf21bl-formation's).
The `--check` path that catches the second is exercised here end to end.
"""

import subprocess
import sys
import textwrap
from pathlib import Path

import pytest
from helpers import REPO_ROOT, SCRIPT_TOML

import choreoc
from tapestry.script_toml import NormalizedStep, NormalizedTransition, parse_file

CHOREOC = REPO_ROOT / "sdk/tools/choreoc.py"


def run_cli(*args, cwd=None):
    return subprocess.run([sys.executable, str(CHOREOC), *map(str, args)],
                          capture_output=True, text=True, cwd=cwd)


def write_script(tmp_path, body: str, name: str = "unit-test") -> Path:
    path = tmp_path / f"{name}.choreo.toml"
    path.write_text(f'choreo = "{name}"\n\n' + textwrap.dedent(body))
    return path


MINIMAL = '''
    [[steps]]
    hold = { duration = "10s", requires = ["locomotion"] }

    [[steps]]
    exchange = { timeout = "30s", until = "achieved", eps = "25cm" }
    '''


# ── C literal emission ───────────────────────────────────────────────────────

@pytest.mark.parametrize("value,literal", [
    (1.0, "1.0f"), (0.25, "0.25f"), (2, "2.0f"), (-1.5, "-1.5f"),
    (1e-6, "1e-06f"), (1.5e10, "1.5e+10f"), (0.0, "0.0f"),
])
def test_floats_are_emitted_as_c_float_literals(value, literal):
    """Every literal needs the f suffix and a decimal point: an integer
    literal assigned to a float field is a silent type change in C."""
    assert choreoc.c_float(value) == literal


def test_capability_masks_become_or_ed_enum_names():
    assert choreoc.caps_expr(0) == "CHOREO_CAP_NONE"
    assert choreoc.caps_expr(0x01) == "CHOREO_CAP_LOCOMOTION"
    assert choreoc.caps_expr(0x05) == "CHOREO_CAP_LOCOMOTION | CHOREO_CAP_SENSING"
    assert choreoc.caps_expr(0x0F).count("|") == 3


def test_only_authored_fields_are_emitted():
    """Unset fields must stay absent so the C struct's own designated
    initializer defaults apply — emitting a zero would override them."""
    c = choreoc.emit_step(NormalizedStep(goal="hold", max_duration_ms=5000))
    assert ".type = CHOREO_GOAL_HOLD" in c
    assert ".max_duration_ms = 5000u" in c
    assert ".advance_on_achieved = false" in c
    for absent in (".target", ".radius", ".shape", ".slot_shift",
                   ".direct_path", ".achieve_eps", ".achieve_hold_ms",
                   ".required_caps", ".scope"):
        assert absent not in c


def test_a_fully_specified_step_emits_every_field():
    c = choreoc.emit_step(NormalizedStep(
        goal="form", max_duration_ms=12000, advance_on_achieved=True, scope=1,
        slot_shift=2, direct_path=True, achieve_eps=0.25, achieve_hold_ms=3000,
        target=(4.0, 5.0), radius=2.0, shape="grid", required_caps=0x01))
    for fragment in (".type = CHOREO_GOAL_FORM",
                     ".target = { 4.0f, 5.0f }",
                     ".radius = 2.0f",
                     ".shape = TAPESTRY_BSE_SHAPE_GRID",
                     ".required_caps = CHOREO_CAP_LOCOMOTION",
                     ".slot_shift = 2u",
                     ".direct_path = true",
                     ".achieve_eps = 0.25f",
                     ".achieve_hold_ms = 3000u",
                     ".max_duration_ms = 12000u",
                     ".advance_on_achieved = true",
                     ".scope = CHOREO_SCOPE_ALL"):
        assert fragment in c, fragment


def test_frame_absolute_default_is_left_implicit():
    """frame="absolute" is the zero/default value — must not be emitted,
    same reasoning as every other unset-means-default field."""
    c = choreoc.emit_step(NormalizedStep(goal="converge", max_duration_ms=1000,
                                         target=(1.0, 2.0)))
    assert ".frame" not in c
    assert ".anchor" not in c


def test_frame_and_anchor_are_emitted_as_c_enum_names():
    c = choreoc.emit_step(NormalizedStep(
        goal="converge", max_duration_ms=1000, frame="element",
        anchor_select="id", anchor_id=3))
    assert ".frame = TAPESTRY_BSE_FRAME_ELEMENT" in c
    assert ".anchor = TAPESTRY_BSE_ANCHOR_ID" in c
    assert ".anchor_id = 3u" in c


def test_motion_static_default_is_left_implicit():
    c = choreoc.emit_step(NormalizedStep(goal="form", max_duration_ms=1000,
                                         target=(0.0, 0.0), radius=3.0))
    assert ".motion" not in c
    assert ".spin_rate_radps" not in c


def test_spin_motion_and_rate_are_emitted():
    c = choreoc.emit_step(NormalizedStep(
        goal="form", max_duration_ms=60000, target=(0.0, 0.0), radius=3.0,
        motion="spin", spin_rate_radps=0.15))
    assert ".motion = TAPESTRY_BSE_MOTION_SPIN" in c
    assert ".spin_rate_radps = 0.15f" in c


def test_no_transitions_leaves_on_and_n_transitions_implicit():
    c = choreoc.emit_step(NormalizedStep(goal="hold", max_duration_ms=1000))
    assert ".on" not in c
    assert ".n_transitions" not in c


def test_transitions_are_emitted_as_a_designated_initializer_array():
    c = choreoc.emit_step(NormalizedStep(
        goal="hold", max_duration_ms=1000,
        on=[
            NormalizedTransition(event="count_gte", goto_step_idx=2, threshold=3),
            NormalizedTransition(event="achieved", goto_step_idx=0),
        ]))
    assert ".n_transitions = 2u" in c
    assert "CHOREO_EVENT_COUNT_GTE" in c
    assert ".goto_step_idx = 2u" in c
    assert ".threshold = 3u" in c
    assert "CHOREO_EVENT_ACHIEVED" in c


def test_scope_self_is_left_implicit():
    c = choreoc.emit_step(NormalizedStep(goal="hold", max_duration_ms=1000,
                                         scope=0))
    assert ".scope" not in c


# ── Header emission ──────────────────────────────────────────────────────────

def test_the_header_declares_the_name_length_and_total_bound(tmp_path):
    path = write_script(tmp_path, MINIMAL)
    _, text = choreoc.render(path, tmp_path / "choreo_script.h")
    assert '#define CHOREO_NAME                    "unit-test"' in text
    assert "#define CHOREO_SCRIPT_LEN              2u" in text
    assert "#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 40000u" in text
    assert "static const choreo_step_t k_choreo_script[CHOREO_SCRIPT_LEN]" in text
    assert text.startswith("/*")
    assert "DO NOT EDIT" in text


def test_the_banner_carries_a_runnable_regenerate_command(tmp_path):
    """Discovery in --check recovers a header's source from this line, so
    it is load-bearing, not decoration."""
    path = write_script(tmp_path, MINIMAL)
    _, text = choreoc.render(path, tmp_path / "choreo_script.h")
    assert choreoc.REGEN_RE.search(text) is not None


def test_rendering_is_deterministic(tmp_path):
    path = write_script(tmp_path, MINIMAL)
    out = tmp_path / "choreo_script.h"
    assert choreoc.render(path, out)[1] == choreoc.render(path, out)[1]


def test_the_same_script_renders_differently_for_two_consumers(tmp_path):
    """The regenerate banner names the output path — which is exactly why
    check_pair compares against a render targeting the SAME path."""
    path = write_script(tmp_path, MINIMAL)
    a = choreoc.render(path, tmp_path / "a" / "choreo_script.h")[1]
    b = choreoc.render(path, tmp_path / "b" / "choreo_script.h")[1]
    assert a != b


def test_default_output_prefers_a_src_directory(tmp_path):
    script = write_script(tmp_path, MINIMAL)
    assert choreoc.default_output(script) == tmp_path / "choreo_script.h"
    (tmp_path / "src").mkdir()
    assert choreoc.default_output(script) == tmp_path / "src" / "choreo_script.h"


# ── CLI ──────────────────────────────────────────────────────────────────────

def test_compiling_writes_the_header_and_reports_the_time_bound(tmp_path):
    script = write_script(tmp_path, MINIMAL)
    out = tmp_path / "choreo_script.h"
    r = run_cli(script, "-o", out)
    assert r.returncode == 0, r.stderr
    assert out.is_file()
    assert "2 step(s)" in r.stdout and "40 s" in r.stdout


def test_a_script_error_exits_nonzero_without_writing(tmp_path):
    bad = tmp_path / "bad.choreo.toml"
    bad.write_text('choreo = "bad"\n[[steps]]\nhold = { }\n')
    out = tmp_path / "choreo_script.h"
    r = run_cli(bad, "-o", out)
    assert r.returncode == 1
    assert "no time bound" in r.stderr
    assert not out.exists()


def test_a_script_argument_is_required_without_check(tmp_path):
    r = run_cli()
    assert r.returncode != 0
    assert "required unless --check" in r.stderr


def test_check_passes_on_a_freshly_generated_header(tmp_path):
    script = write_script(tmp_path, MINIMAL)
    out = tmp_path / "choreo_script.h"
    assert run_cli(script, "-o", out).returncode == 0
    r = run_cli("--check", script, "-o", out)
    assert r.returncode == 0
    assert "already up to date" in r.stdout


def test_check_fails_on_a_missing_header(tmp_path):
    script = write_script(tmp_path, MINIMAL)
    r = run_cli("--check", script, "-o", tmp_path / "choreo_script.h")
    assert r.returncode == 1
    assert "MISSING" in r.stderr


def test_check_fails_when_the_header_has_drifted(tmp_path):
    """The regression that shipped: a script edited without recompiling
    its consumer's header."""
    script = write_script(tmp_path, MINIMAL)
    out = tmp_path / "choreo_script.h"
    run_cli(script, "-o", out)
    script.write_text(script.read_text().replace('"10s"', '"20s"'))
    r = run_cli("--check", script, "-o", out)
    assert r.returncode == 1
    assert "STALE" in r.stderr


def test_check_writes_nothing(tmp_path):
    script = write_script(tmp_path, MINIMAL)
    out = tmp_path / "choreo_script.h"
    run_cli(script, "-o", out)
    before = out.read_text()
    script.write_text(script.read_text().replace('"10s"', '"20s"'))
    run_cli("--check", script, "-o", out)
    assert out.read_text() == before


# ── Repository-wide discovery ────────────────────────────────────────────────

def test_discovery_finds_every_committed_generated_header():
    """A hardcoded list would have missed the consumer that drifted, so
    choreoc recovers each header's source from its own banner instead."""
    pairs = choreoc.discover_pairs()
    assert pairs, "no generated headers discovered"
    headers = {h.resolve() for _, h in pairs}
    assert (REPO_ROOT / "examples/cf21bl-formation/src/choreo_script.h"
            ).resolve() in headers
    assert (REPO_ROOT / "examples/webots-formation/controllers/cf21bl/"
            "choreo_script.h").resolve() in headers
    assert all(s.is_file() for s, _ in pairs)


def test_discovery_skips_build_trees():
    """Build directories hold copies; checking them would report a stale
    artifact as a source-tree failure."""
    assert not any("build" in h.parts for _, h in choreoc.discover_pairs())


def test_every_committed_header_matches_its_script_today():
    """The same guarantee CI's `choreoc.py --check` gives, as a unit test:
    a script edit that is not recompiled cannot pass silently."""
    r = run_cli("--check", cwd=REPO_ROOT)
    assert r.returncode == 0, r.stdout + r.stderr


def test_the_shipped_script_compiles_to_its_committed_header():
    committed = (REPO_ROOT
                 / "examples/cf21bl-formation/src/choreo_script.h")
    _, rendered = choreoc.render(SCRIPT_TOML, committed)
    assert rendered == committed.read_text()


def test_the_two_consumers_of_the_shipped_script_agree_on_its_steps():
    """The headers differ only in their regenerate banner — the step array
    itself must be identical, which is what drifted before."""
    def body(p: Path) -> str:
        return p.read_text().split("static const choreo_step_t", 1)[1]

    a = REPO_ROOT / "examples/cf21bl-formation/src/choreo_script.h"
    b = (REPO_ROOT / "examples/webots-formation/controllers/cf21bl/"
         "choreo_script.h")
    assert body(a) == body(b)


def test_the_headers_total_bound_matches_the_parsed_script():
    """Mission backstops are sized off CHOREO_SCRIPT_TOTAL_TIMEOUT_MS, so
    it must equal the sum of the step bounds, not an author's estimate."""
    script = parse_file(SCRIPT_TOML)
    header = (REPO_ROOT
              / "examples/cf21bl-formation/src/choreo_script.h").read_text()
    assert (f"#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS "
            f"{script.total_timeout_ms}u") in header


# ── Tracks (§7) ──────────────────────────────────────────────────────────────

TRACKS = '''
    [[tracks]]
    filter = { requires = ["sensing"] }
    [[tracks.steps]]
    hold = { duration = "300s" }

    [[tracks]]
    filter = { energy_low = true }
    [[tracks.steps]]
    converge = { target = [0, 0], duration = "60s" }

    [[tracks]]
    [[tracks.steps]]
    hold = { duration = "10s" }
    '''


def test_a_tracks_header_declares_the_track_table(tmp_path):
    path = write_script(tmp_path, TRACKS)
    _, text = choreoc.render(path, tmp_path / "choreo_script.h")
    assert '#define CHOREO_NAME                    "unit-test"' in text
    assert "#define CHOREO_N_TRACKS                3u" in text
    # worst case across tracks, not the sum: 300s, 60s, 10s -> 300s
    assert "#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 300000u" in text
    assert "CHOREO_SCRIPT_LEN" not in text
    assert "static const choreo_step_t k_choreo_track0_steps[1]" in text
    assert "static const choreo_step_t k_choreo_track1_steps[1]" in text
    assert "static const choreo_step_t k_choreo_track2_steps[1]" in text
    assert ("static const choreo_track_t "
            "k_choreo_tracks[CHOREO_N_TRACKS]") in text


def test_track_filters_are_emitted_as_designated_initializers(tmp_path):
    path = write_script(tmp_path, TRACKS)
    _, text = choreoc.render(path, tmp_path / "choreo_script.h")
    assert ".filter = { .required_caps = CHOREO_CAP_SENSING }" in text
    assert ".filter = { .requires_energy_low = true }" in text
    assert ".filter = { 0 }" in text   # catch-all track


def test_a_tracks_header_compiles_the_way_a_steps_header_does(tmp_path):
    script = write_script(tmp_path, TRACKS)
    out = tmp_path / "choreo_script.h"
    r = run_cli(script, "-o", out)
    assert r.returncode == 0, r.stderr
    assert out.is_file()
    assert "3 track(s)" in r.stdout and "3 step(s) total" in r.stdout


def test_a_tracks_header_round_trips_through_check(tmp_path):
    script = write_script(tmp_path, TRACKS)
    out = tmp_path / "choreo_script.h"
    assert run_cli(script, "-o", out).returncode == 0
    assert run_cli(script, "-o", out, "--check").returncode == 0
    out.write_text(out.read_text() + "\n// drift\n")
    assert run_cli(script, "-o", out, "--check").returncode == 1
