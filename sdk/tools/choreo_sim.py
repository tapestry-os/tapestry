#!/usr/bin/env python3
"""
choreo_sim — offline replay AND synthetic simulation harness for the L6/L7
Choreo engine (sdk/python/tapestry). Two modes, one tick loop, one plotter:

  --replay --script <name>.choreo.toml --telemetry <choreo_N.csv>
      Regression mode. Reads a per-element CSV recorded by
      examples/webots-formation's choreo_telemetry.c (set
      TAPESTRY_TELEMETRY_DIR to capture one) and re-drives the Python
      Choreo engine tick-by-tick with the EXACT recorded inputs (each
      tick's wm_entries snapshot and quorum_state), diffing its output
      (script step, directive, achievement) against what was actually
      recorded. A clean replay (0 divergences) means the C engine that
      produced the recording and the current Python engine agree
      tick-for-tick on real flight/simulation data — see
      sdk/CHOREO_SCRIPTS.md's "Parity" section. No Webots, no C build, no
      live network — just the L6/L7 state machine re-run against frozen
      inputs.

  --simulate --script <name>.choreo.toml --elements N
      Script-authoring mode. Instantiates N in-process Choreo objects —
      no C, no Zephyr, no network, no Webots — and drives them through
      the same script with a synthetic multi-element world: each tick,
      every element sees every other element's current (synthetic)
      position with perfect shared visibility (no gossip/staleness/
      quorum-degradation simulation; quorum_state is synthesized
      HEALTHY), then moves toward its directive's target at a capped
      speed. This is deliberately NOT a fidelity simulator — no
      repulsion/leash/arena-clamp physics, that realism is
      examples/webots-formation's job — it exists to give someone
      editing a .choreo.toml sub-second feedback (positions, timing,
      achievement) without a build toolchain or Webots set up, or before
      a physical/simulated substrate exists at all. It is also NOT a
      replacement for tapestry-csm-sim/tapestry-scr-sim, which validate
      partition tolerance and quorum/election under injected network
      faults against the real production C engine — choreo-sim assumes
      away all of that to get a fast script check on the Python mirror.

Both modes share the same per-tick record shape and (with --plot) the
same multi-panel matplotlib figure: an XY trajectory panel plus
script-step and achievement timelines.

matplotlib is a LAZY import used only by --plot — compiling or replaying
a script stays dependency-free, matching choreoc.py's "standard library
only... no venv, nothing to install" commitment.

Usage:
    python3 sdk/tools/choreo_sim.py --replay \\
        --script examples/cf21bl-formation/change-partners.choreo.toml \\
        --telemetry /tmp/telemetry/choreo_0.csv

    python3 sdk/tools/choreo_sim.py --simulate \\
        --script examples/cf21bl-formation/change-partners.choreo.toml \\
        --elements 4 --plot

Exit status (--replay): 0 if the replay matched every recorded tick, 1 on
any divergence or error. Exit status (--simulate): 0 on a clean run, 1 on
error (a scripting bug can't silently "fail" here — choreoc guarantees
every step a hard time bound, so the loop always terminates).
"""

import argparse
import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Import the tapestry package from the sibling sdk/python directory without
# requiring installation (same pattern as choreoc.py).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from tapestry.bse import BSEDirectiveType
from tapestry.choreo import Choreo
from tapestry.script_toml import ScriptError, load_steps, parse_file, to_choreo_steps

# Float comparison tolerance — positions round-trip through the CSV at 4
# decimal places (choreo_telemetry.c's "%.4f"), and BSE math runs in C
# float32 vs Python float64, so exact equality isn't the right bar.
DEFAULT_EPS_TOL = 1e-3

WM_CYCLE_MS = Choreo.WM_CYCLE_MS   # 100 — both modes tick on this period


# ── Shared per-tick record ──────────────────────────────────────────────────

@dataclass
class TickRecord:
    """One element's state at one tick — the common currency between
    --replay and --simulate, and the only thing the plotter needs."""
    tick:              int
    wall_time_s:       float
    x:                 float
    y:                 float
    script_step:       int
    achieved:          bool
    directive_type:    int
    directive_target:  Tuple[float, float]


# ── --replay mode ────────────────────────────────────────────────────────────

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
          eps_tol: float, max_report: int, verbose: bool) -> Tuple[int, List[TickRecord]]:
    """Returns (exit_code, records) — records is this element's tick-by-tick
    trajectory as actually recorded, for optional --plot."""
    try:
        steps = load_steps(script_path)
    except (ScriptError, OSError) as e:
        print(f"choreo_sim: {e}", file=sys.stderr)
        return 1, []

    with open(telemetry_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"choreo_sim: {telemetry_path} has no data rows",
              file=sys.stderr)
        return 1, []

    element_id = int(rows[0]["element_id"])
    choreo = Choreo(element_id=element_id, capabilities=None)
    rc = choreo.submit_script(steps)
    if rc != 0:
        print(f"choreo_sim: submit_script rejected the script "
              f"(rc={rc}) — recording and script disagree before tick 0",
              file=sys.stderr)
        return 1, []

    divergences = 0
    reported = 0
    n_ticks = 0
    records: List[TickRecord] = []

    for row in rows:
        if int(row["element_id"]) != element_id:
            print(f"choreo_sim: {telemetry_path} mixes element_id "
                  f"{element_id} and {row['element_id']} — expected one "
                  f"element per file", file=sys.stderr)
            return 1, records

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
            print(f"choreo_sim: row at tick {row.get('tick', '?')} is "
                  f"malformed ({e}) — recording likely truncated by an "
                  f"unclean stop; stopping replay here", file=sys.stderr)
            break

        choreo.tick(wm_entries, scr_state)
        n_ticks += 1

        recorded_dir_type = int(row["directive_type"])
        recorded_dir_target = (float(row["directive_target_x"]),
                               float(row["directive_target_y"]))
        replayed_dir = choreo.get_directive()

        records.append(TickRecord(
            tick=int(row["tick"]), wall_time_s=float(row["wall_time_s"]),
            x=float(row["pos_x"]), y=float(row["pos_y"]),
            script_step=choreo.script_step(), achieved=choreo.goal_achieved(),
            directive_type=int(replayed_dir.type),
            directive_target=tuple(replayed_dir.target),
        ))

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
                print(f"choreo_sim: divergence at tick {tick} "
                      f"(t={row['wall_time_s']}s):", file=sys.stderr)
                for name, recorded, replayed in mismatches:
                    print(f"    {name}: recorded={recorded!r} "
                          f"replayed={replayed!r}", file=sys.stderr)
        elif verbose:
            print(f"tick {row['tick']}: step={choreo.script_step()} "
                  f"dir={replayed_dir.type.name}{tuple(replayed_dir.target)} "
                  f"achieved={choreo.goal_achieved()}")

    if divergences:
        print(f"choreo_sim: FAIL — {divergences}/{n_ticks} tick(s) "
              f"diverged from the recording ({telemetry_path})",
              file=sys.stderr)
        return 1, records

    print(f"choreo_sim: OK — {n_ticks} tick(s) replayed, 0 divergences "
          f"(element {element_id}, {telemetry_path})")
    return 0, records


# ── --simulate mode ──────────────────────────────────────────────────────────

DEFAULT_SPEED_MPS = 2.0   # capped integrator speed — a generic mid-size
                          # aerial/ground platform figure, not a physics fit


def _initial_layout(n: int) -> Dict[int, Tuple[float, float]]:
    """Spread N synthetic elements on a small circle around the origin so
    they don't all start co-located (which would degenerate e.g. FORM's
    vertex assignment into a simultaneous-departure burst). Purely a
    starting posture for the toy integrator — not meaningful physics."""
    import math
    if n == 1:
        return {0: (0.0, 0.0)}
    radius = max(2.0, n * 0.5)
    return {
        i: (radius * math.cos(2 * math.pi * i / n),
            radius * math.sin(2 * math.pi * i / n))
        for i in range(n)
    }


def simulate(script_path: Path, n_elements: int, speed_mps: float,
            verbose: bool) -> Tuple[int, Dict[int, List[TickRecord]]]:
    """Drives n_elements synthetic Choreo instances through the script with
    perfect shared visibility (no gossip/staleness/quorum simulation) and a
    capped-speed position integrator (no repulsion/leash/arena-clamp — see
    module docstring). Returns (exit_code, records-per-element)."""
    try:
        script = parse_file(script_path)
        steps = to_choreo_steps(script)
    except (ScriptError, OSError) as e:
        print(f"choreo_sim: {e}", file=sys.stderr)
        return 1, {}

    if n_elements < 1:
        print("choreo_sim: --elements must be >= 1", file=sys.stderr)
        return 1, {}

    ids = list(range(n_elements))
    choreos = {i: Choreo(element_id=i, capabilities=None) for i in ids}
    for i, ch in choreos.items():
        rc = ch.submit_script(steps)
        if rc != 0:
            print(f"choreo_sim: submit_script rejected the script for "
                  f"element {i} (rc={rc})", file=sys.stderr)
            return 1, {}

    positions = _initial_layout(n_elements)
    achieved = {i: False for i in ids}
    records: Dict[int, List[TickRecord]] = {i: [] for i in ids}

    max_step_m = speed_mps * (WM_CYCLE_MS / 1000.0)
    # choreoc guarantees every step is time-bounded, so the script total is
    # a hard upper bound on tick count — no separate safety cap needed.
    max_ticks = max(1, -(-script.total_timeout_ms // WM_CYCLE_MS))  # ceil div
    scr_state = {"quorum_state": 2}   # HEALTHY — perfect shared visibility,
                                      # no gossip/staleness/quorum sim at all

    tick = 0
    for tick in range(max_ticks):
        wall_time_s = tick * WM_CYCLE_MS / 1000.0
        # Snapshot BEFORE ticking any element this cycle, so every element
        # sees the same world regardless of dict iteration order.
        snapshot = [
            {"id": i, "x": positions[i][0], "y": positions[i][1],
             "is_active": True, "is_stale": False, "achieved": achieved[i]}
            for i in ids
        ]

        new_positions = {}
        new_achieved = {}
        for i, ch in choreos.items():
            wm_entries = [dict(e, is_self=(e["id"] == i)) for e in snapshot]
            ch.tick(wm_entries, scr_state)
            directive = ch.get_directive()

            records[i].append(TickRecord(
                tick=tick, wall_time_s=wall_time_s,
                x=positions[i][0], y=positions[i][1],
                script_step=ch.script_step(), achieved=ch.goal_achieved(),
                directive_type=int(directive.type),
                directive_target=tuple(directive.target),
            ))

            x, y = positions[i]
            if directive.type == BSEDirectiveType.MOVE_TO_POINT:
                tx, ty = directive.target
                dx, dy = tx - x, ty - y
                dist = (dx * dx + dy * dy) ** 0.5
                if dist > max_step_m:
                    x += dx / dist * max_step_m
                    y += dy / dist * max_step_m
                else:
                    x, y = tx, ty
            # HOLD / MAINTAIN_SPRING / IDLE: no single-point target this
            # toy integrator can move toward — repulsion/spring physics is
            # explicitly out of scope (Webots' job), so the element holds.
            new_positions[i] = (x, y)
            new_achieved[i] = ch.goal_achieved()

            if verbose:
                print(f"tick {tick} elem {i}: pos=({x:.2f},{y:.2f}) "
                      f"step={ch.script_step()} "
                      f"dir={directive.type.name}{tuple(directive.target)} "
                      f"achieved={ch.goal_achieved()}")

        positions, achieved = new_positions, new_achieved
        if all(ch.script_complete() for ch in choreos.values()):
            break

    n_complete = sum(1 for ch in choreos.values() if ch.script_complete())
    print(f"choreo_sim: \"{script.name}\" — {tick + 1} tick(s), "
          f"{n_complete}/{n_elements} element(s) completed the script "
          f"(elements={n_elements}, speed={speed_mps:g} m/s)")
    return 0, records


# ── Shared plot output (lazy matplotlib import) ─────────────────────────────

COLORS = ['#2196F3', '#FF5722', '#4CAF50', '#9C27B0', '#FFC107', '#009688',
         '#E91E63', '#795548']


def plot_records(records_by_element: Dict[int, List[TickRecord]],
                 title: str, out: Optional[str]) -> None:
    """Multi-panel figure in the tapestry-csm-sim/tapestry-scr-sim plot.py
    house style: an XY trajectory panel plus stacked script-step and
    achievement timelines sharing a time axis. Imports matplotlib lazily —
    this is the ONLY code path in this file (or choreoc.py) allowed to
    depend on it."""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 11))
    gs = fig.add_gridspec(3, 1, height_ratios=[2.2, 1, 1], hspace=0.25)
    ax_traj = fig.add_subplot(gs[0])
    ax_step = fig.add_subplot(gs[1])
    ax_ach  = fig.add_subplot(gs[2], sharex=ax_step)

    for idx, (elem_id, records) in enumerate(sorted(records_by_element.items())):
        if not records:
            continue
        color = COLORS[idx % len(COLORS)]
        label = f"element {elem_id}"
        xs = [r.x for r in records]
        ys = [r.y for r in records]
        ts = [r.wall_time_s for r in records]
        steps = [r.script_step for r in records]
        ach = [1 if r.achieved else 0 for r in records]

        ax_traj.plot(xs, ys, color=color, linewidth=1.5, label=label)
        ax_traj.plot(xs[0], ys[0], marker='o', color=color, markersize=6)
        ax_traj.plot(xs[-1], ys[-1], marker='s', color=color, markersize=6)

        ax_step.step(ts, steps, where='post', color=color, linewidth=1.5,
                    label=label)
        ax_ach.step(ts, ach, where='post', color=color, linewidth=1.5,
                   label=label)

    ax_traj.set_xlabel('x', fontsize=9)
    ax_traj.set_ylabel('y', fontsize=9)
    ax_traj.set_title('Trajectory (○ start, □ end)', fontsize=10)
    ax_traj.set_aspect('equal', adjustable='datalim')
    ax_traj.grid(linewidth=0.4, alpha=0.5)
    ax_traj.legend(fontsize=8, loc='best')

    ax_step.set_ylabel('Script step', fontsize=9)
    ax_step.grid(axis='y', linewidth=0.4, alpha=0.5)
    ax_step.spines['top'].set_visible(False)
    ax_step.spines['right'].set_visible(False)

    ax_ach.set_ylabel('Goal achieved', fontsize=9)
    ax_ach.set_ylim(-0.1, 1.1)
    ax_ach.set_yticks([0, 1])
    ax_ach.set_xlabel('Wall time (s)', fontsize=9)
    ax_ach.grid(axis='y', linewidth=0.4, alpha=0.5)
    ax_ach.spines['top'].set_visible(False)
    ax_ach.spines['right'].set_visible(False)

    fig.suptitle(title, fontsize=12, fontweight='bold')

    if out:
        fig.savefig(out, dpi=150, bbox_inches='tight')
        print(f"choreo_sim: saved plot to {out}")
    else:
        plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        prog="choreo_sim",
        description="Replay captured L6/L7 telemetry, or run a synthetic "
                    "multi-element simulation, through the Python Choreo "
                    "engine.")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--replay", action="store_true",
                      help="regression mode: replay a recorded CSV "
                           "(requires --telemetry)")
    mode.add_argument("--simulate", action="store_true",
                      help="script-authoring mode: run N synthetic "
                           "elements through --script (requires "
                           "--elements)")

    ap.add_argument("--script", type=Path, required=True,
                    help="the .choreo.toml to replay or simulate")
    ap.add_argument("--telemetry", type=Path,
                    help="[--replay] a choreo_<id>.csv captured via "
                         "TAPESTRY_TELEMETRY_DIR (see choreo_telemetry.h)")
    ap.add_argument("--eps-tol", type=float, default=DEFAULT_EPS_TOL,
                    help=f"[--replay] position comparison tolerance, "
                         f"meters (default {DEFAULT_EPS_TOL})")
    ap.add_argument("--max-report", type=int, default=5,
                    help="[--replay] max divergent ticks to print in "
                         "detail (default 5)")

    ap.add_argument("--elements", type=int,
                    help="[--simulate] number of synthetic elements")
    ap.add_argument("--speed", type=float, default=DEFAULT_SPEED_MPS,
                    help=f"[--simulate] capped integrator speed, m/s "
                         f"(default {DEFAULT_SPEED_MPS:g})")

    ap.add_argument("--plot", action="store_true",
                    help="show a trajectory/timeline figure "
                         "(requires matplotlib; lazy import)")
    ap.add_argument("--out", type=Path,
                    help="save the --plot figure here instead of "
                         "displaying it")

    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print every tick, not just divergences/summary")
    args = ap.parse_args()

    if args.replay and args.telemetry is None:
        ap.error("--replay requires --telemetry")
    if args.simulate and args.elements is None:
        ap.error("--simulate requires --elements")
    if args.out and not args.plot:
        ap.error("--out has no effect without --plot")

    if args.replay:
        rc, records = replay(args.script, args.telemetry, args.eps_tol,
                            args.max_report, args.verbose)
        if args.plot and records:
            plot_records({0: records}, f"choreo_sim --replay: {args.telemetry.name}",
                        str(args.out) if args.out else None)
        return rc

    rc, records_by_element = simulate(args.script, args.elements, args.speed,
                                      args.verbose)
    if rc == 0 and args.plot:
        plot_records(records_by_element,
                    f"choreo_sim --simulate: {args.script.name} "
                    f"({args.elements} elements)",
                    str(args.out) if args.out else None)
    return rc


if __name__ == "__main__":
    sys.exit(main())
