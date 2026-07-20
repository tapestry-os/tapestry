#!/usr/bin/env python3
"""
choreoc — Tapestry Choreo script compiler: choreo.toml → C header.

Turns a declarative Choreo script (TOML, see sdk/python/tapestry/
script_toml.py for the schema) into a committed C header containing the
choreo_step_t array an embedded target submits via choreo_submit_script().

Standard library only (Python >= 3.11 for tomllib) — no venv, nothing to
install.  The generated header is committed next to the consuming source
(same pattern as examples/lighthouse_cal.h), so firmware builds and CI
never need Python.

Usage:
    python3 sdk/tools/choreoc.py <script.toml> [-o <out.h>]

With no -o, the header is written next to the script as
src/choreo_script.h if src/ exists, else choreo_script.h alongside it.

The Python SDK reads the SAME file directly — no generation step:
    from tapestry.script_toml import load_steps
    choreo.submit_script(load_steps("choreo.toml"))
"""

import argparse
import sys
from pathlib import Path

# Import the tapestry package from the sibling sdk/python directory without
# requiring installation.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from tapestry.script_toml import (ChoreoScript, NormalizedStep, ScriptError,
                                  parse_file)

GOAL_ENUM = {
    "hold":     "CHOREO_GOAL_HOLD",
    "exchange": "CHOREO_GOAL_EXCHANGE",
    "form":     "CHOREO_GOAL_FORM",
    "move":     "CHOREO_GOAL_MOVE",
    "converge": "CHOREO_GOAL_CONVERGE",
    "disperse": "CHOREO_GOAL_DISPERSE",
}

SHAPE_ENUM = {
    "circle": "TAPESTRY_BSE_SHAPE_CIRCLE",
    "line":   "TAPESTRY_BSE_SHAPE_LINE",
    "grid":   "TAPESTRY_BSE_SHAPE_GRID",
}

CAP_FLAGS = [
    (0x01, "CHOREO_CAP_LOCOMOTION"),
    (0x02, "CHOREO_CAP_BONDING"),
    (0x04, "CHOREO_CAP_SENSING"),
    (0x08, "CHOREO_CAP_SIGNALING"),
]


def c_float(v: float) -> str:
    s = f"{v:.6g}"
    if "." not in s and "e" not in s and "inf" not in s and "nan" not in s:
        s += ".0"
    return s + "f"


def caps_expr(mask: int) -> str:
    names = [name for bit, name in CAP_FLAGS if mask & bit]
    return " | ".join(names) if names else "CHOREO_CAP_NONE"


def emit_step(s: NormalizedStep) -> str:
    goal_fields = [f".type = {GOAL_ENUM[s.goal]}"]
    if s.target is not None:
        goal_fields.append(f".target = {{ {c_float(s.target[0])}, "
                           f"{c_float(s.target[1])} }}")
    if s.radius is not None:
        goal_fields.append(f".radius = {c_float(s.radius)}")
    if s.shape is not None:
        goal_fields.append(f".shape = {SHAPE_ENUM[s.shape]}")
    if s.required_caps:
        goal_fields.append(f".required_caps = {caps_expr(s.required_caps)}")
    if s.slot_shift is not None:
        goal_fields.append(f".slot_shift = {s.slot_shift}u")
    if s.direct_path:
        goal_fields.append(".direct_path = true")
    if s.achieve_eps is not None:
        goal_fields.append(f".achieve_eps = {c_float(s.achieve_eps)}")
    if s.achieve_hold_ms is not None:
        goal_fields.append(f".achieve_hold_ms = {s.achieve_hold_ms}u")

    lines = ["    { .goal = { " + goal_fields[0] + ","]
    for gf in goal_fields[1:-1]:
        lines.append("                " + gf + ",")
    if len(goal_fields) > 1:
        lines.append("                " + goal_fields[-1] + " },")
    else:
        lines[0] = "    { .goal = { " + goal_fields[0] + " },"

    lines.append(f"      .max_duration_ms = {s.max_duration_ms}u,")
    lines.append(f"      .advance_on_achieved = "
                 f"{'true' if s.advance_on_achieved else 'false'} }},")
    return "\n".join(lines)


def emit_header(script: ChoreoScript, src_name: str, regen_cmd: str) -> str:
    steps = "\n\n".join(emit_step(s) for s in script.steps)
    return f"""\
/*
 * choreo_script.h — GENERATED from {src_name} — DO NOT EDIT.
 *
 * Choreo: "{script.name}"
 * Regenerate after editing the script file:
 *   {regen_cmd}
 *
 * Every step is time-bounded by construction (choreoc requires it): the
 * script cannot stall in flight, and CHOREO_SCRIPT_TOTAL_TIMEOUT_MS is a
 * hard upper bound on script runtime for mission-backstop math.
 */

#ifndef TAPESTRY_CHOREO_SCRIPT_H
#define TAPESTRY_CHOREO_SCRIPT_H

#include <tapestry/choreo.h>

#define CHOREO_NAME                    "{script.name}"
#define CHOREO_SCRIPT_LEN              {len(script.steps)}u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS {script.total_timeout_ms}u

static const choreo_step_t k_choreo_script[CHOREO_SCRIPT_LEN] = {{
{steps}
}};

#endif /* TAPESTRY_CHOREO_SCRIPT_H */
"""


def main() -> int:
    ap = argparse.ArgumentParser(
        prog="choreoc", description="Compile a Choreo script (TOML) into a "
        "C header of choreo_step_t.")
    ap.add_argument("script", type=Path, help="path to the .toml script")
    ap.add_argument("-o", "--output", type=Path, default=None,
                    help="output header path (default: src/choreo_script.h "
                         "next to the script if src/ exists, else "
                         "choreo_script.h)")
    args = ap.parse_args()

    try:
        script = parse_file(args.script)
    except (ScriptError, OSError) as e:
        print(f"choreoc: {e}", file=sys.stderr)
        return 1

    out = args.output
    if out is None:
        src_dir = args.script.parent / "src"
        out = (src_dir if src_dir.is_dir() else args.script.parent) \
              / "choreo_script.h"

    try:
        repo = Path(__file__).resolve().parent.parent.parent.parent
        rel_script = args.script.resolve().relative_to(repo)
        rel_out = out.resolve().relative_to(repo)
        regen = f"python3 tapestry/sdk/tools/choreoc.py {rel_script} -o {rel_out}"
    except ValueError:
        regen = f"python3 choreoc.py {args.script} -o {out}"

    out.write_text(emit_header(script, args.script.name, regen))
    total_s = script.total_timeout_ms / 1000.0
    print(f"choreoc: \"{script.name}\" — {len(script.steps)} step(s), "
          f"total time bound {total_s:g} s → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
