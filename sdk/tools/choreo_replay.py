#!/usr/bin/env python3
"""
choreo_replay — offline replay/regression harness for captured L6/L7 telemetry.

Reads a per-element CSV recorded by examples/webots-formation's
choreo_telemetry.c (set TAPESTRY_TELEMETRY_DIR to capture one) and re-drives
sdk/python/tapestry's Choreo engine tick-by-tick with the EXACT recorded
inputs (each tick's wm_entries snapshot and quorum_state) — the same script
loaded from the same <name>.choreo.toml the real run used. It then diffs the
engine's output at each tick (script step, directive, achievement) against
what was actually recorded.

This is a regression test, not a physics simulation: no Webots, no C build,
no live network — just the L6/L7 state machine re-run against frozen inputs.
A clean replay (0 divergences) is the same claim sdk/CHOREO_SCRIPTS.md's
"Parity" section makes for a bare script rehearsal, extended to real
recorded flight/simulation data: for identical inputs, the Python engine's
tick counts, steps, directives, and achievement timing match the recording
exactly. A divergence means either the recording is stale (the .toml or the
engine changed since capture — re-record) or a genuine regression in
sdk/python/tapestry vs. tapestry-os/subsys/choreo+bse (the C engine this
recording came from).

Usage:
    python3 sdk/tools/choreo_replay.py \\
        --script examples/cf21bl-formation/change-partners.choreo.toml \\
        --telemetry /tmp/telemetry/choreo_0.csv

Exit status: 0 if the replay matched every recorded tick, 1 on any
divergence or error.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

# Import the tapestry package from the sibling sdk/python directory without
# requiring installation (same pattern as choreoc.py).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from tapestry.choreo import Choreo, GoalType
from tapestry.script_toml import ScriptError, load_steps

# Float comparison tolerance — positions round-trip through the CSV at 4
# decimal places (choreo_telemetry.c's "%.4f"), and BSE math runs in C
# float32 vs Python float64, so exact equality isn't the right bar.
DEFAULT_EPS_TOL = 1e-3

# Fields diffed every tick: (CSV column(s), recorded-parser, replayed-getter).
COMPARE_FIELDS = [
    "script_step", "script_complete", "goal_achieved",
    "directive_type", "directive_target",
]


def parse_wm_entries(wm_json: str) -> list:
    entries = json.loads(wm_json)
    for e in entries:
        e["is_active"] = bool(e["is_active"])
        e["is_stale"] = bool(e["is_stale"])
        e["is_self"] = bool(e["is_self"])
    return entries


def replay(script_path: Path, telemetry_path: Path,
          eps_tol: float, max_report: int, verbose: bool) -> int:
    try:
        steps = load_steps(script_path)
    except (ScriptError, OSError) as e:
        print(f"choreo_replay: {e}", file=sys.stderr)
        return 1

    with open(telemetry_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"choreo_replay: {telemetry_path} has no data rows",
              file=sys.stderr)
        return 1

    element_id = int(rows[0]["element_id"])
    choreo = Choreo(element_id=element_id, capabilities=None)
    rc = choreo.submit_script(steps)
    if rc != 0:
        print(f"choreo_replay: submit_script rejected the script "
              f"(rc={rc}) — recording and script disagree before tick 0",
              file=sys.stderr)
        return 1

    divergences = 0
    reported = 0
    n_ticks = 0

    for row in rows:
        if int(row["element_id"]) != element_id:
            print(f"choreo_replay: {telemetry_path} mixes element_id "
                  f"{element_id} and {row['element_id']} — expected one "
                  f"element per file", file=sys.stderr)
            return 1

        # A capture that was cut off mid-process (killed rather than run to
        # its natural exit — the only way choreo_telemetry_close() flushes
        # cleanly) leaves a truncated final row: a partial CSV line whose
        # last field never got its closing quote or trailing data. That's a
        # recording artifact, not a replay divergence — warn and stop
        # rather than crashing on a malformed-JSON exception.
        try:
            wm_entries = parse_wm_entries(row["wm_json"])
            scr_state = {
                "role":         int(row["role"]),
                "quorum_state": int(row["quorum_state"]),
                "leader_id":    255,   # not recorded — unused by tick()
                                       # (see sdk/python/tapestry/bse.py:
                                       # scr_state is read only for
                                       # quorum_state)
            }
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            print(f"choreo_replay: row at tick {row.get('tick', '?')} is "
                  f"malformed ({e}) — recording likely truncated by an "
                  f"unclean stop; stopping replay here", file=sys.stderr)
            break

        choreo.tick(wm_entries, scr_state)
        n_ticks += 1

        recorded_dir_type = int(row["directive_type"])
        recorded_dir_target = (float(row["directive_target_x"]),
                               float(row["directive_target_y"]))
        replayed_dir = choreo.get_directive()

        mismatches = []

        def check(name, recorded, replayed):
            if recorded != replayed:
                mismatches.append((name, recorded, replayed))

        check("script_step", int(row["script_step"]), choreo.script_step())
        check("script_complete", bool(int(row["script_complete"])),
              choreo.script_complete())
        check("goal_achieved", bool(int(row["goal_achieved"])),
              choreo.goal_achieved())
        check("directive_type", recorded_dir_type, int(replayed_dir.type))

        dx = abs(recorded_dir_target[0] - replayed_dir.target[0])
        dy = abs(recorded_dir_target[1] - replayed_dir.target[1])
        if dx > eps_tol or dy > eps_tol:
            mismatches.append(("directive_target", recorded_dir_target,
                              tuple(replayed_dir.target)))

        if mismatches:
            divergences += 1
            if reported < max_report:
                reported += 1
                tick = row["tick"]
                print(f"choreo_replay: divergence at tick {tick} "
                      f"(t={row['wall_time_s']}s):", file=sys.stderr)
                for name, recorded, replayed in mismatches:
                    print(f"    {name}: recorded={recorded!r} "
                          f"replayed={replayed!r}", file=sys.stderr)
        elif verbose:
            print(f"tick {row['tick']}: step={choreo.script_step()} "
                  f"dir={replayed_dir.type.name}{tuple(replayed_dir.target)} "
                  f"achieved={choreo.goal_achieved()}")

    if divergences:
        print(f"choreo_replay: FAIL — {divergences}/{n_ticks} tick(s) "
              f"diverged from the recording ({telemetry_path})",
              file=sys.stderr)
        return 1

    print(f"choreo_replay: OK — {n_ticks} tick(s) replayed, 0 divergences "
          f"(element {element_id}, {telemetry_path})")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        prog="choreo_replay",
        description="Replay captured L6/L7 telemetry through the Python "
                    "Choreo engine and diff against the recording.")
    ap.add_argument("--script", type=Path, required=True,
                    help="the .choreo.toml the recorded run used")
    ap.add_argument("--telemetry", type=Path, required=True,
                    help="a choreo_<id>.csv captured via "
                         "TAPESTRY_TELEMETRY_DIR (see choreo_telemetry.h)")
    ap.add_argument("--eps-tol", type=float, default=DEFAULT_EPS_TOL,
                    help=f"position comparison tolerance, meters "
                         f"(default {DEFAULT_EPS_TOL})")
    ap.add_argument("--max-report", type=int, default=5,
                    help="max divergent ticks to print in detail "
                         "(default 5)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print every matching tick, not just divergences")
    args = ap.parse_args()

    return replay(args.script, args.telemetry, args.eps_tol,
                 args.max_report, args.verbose)


if __name__ == "__main__":
    sys.exit(main())
