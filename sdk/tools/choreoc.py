#!/usr/bin/env python3
"""
choreoc — Tapestry Choreo script compiler: <name>.choreo.toml → C header.

Turns a declarative Choreo script (TOML, see sdk/python/tapestry/
script_toml.py for the schema) into a committed C header containing the
choreo_step_t array an embedded target submits via choreo_submit_script().

Standard library only (Python >= 3.11 for tomllib) — no venv, nothing to
install.  The generated header is committed next to the consuming source
(same pattern as examples/lighthouse_cal.h), so firmware builds and CI
never need Python.

Naming convention: a script file is named <name>.choreo.toml, where
<name> matches its own "choreo = " key (e.g. change-partners.choreo.toml
for choreo = "change-partners") — see sdk/CHOREO_SCRIPTS.md.

Usage:
    python3 sdk/tools/choreoc.py <name.choreo.toml> [-o <out.h>]
    python3 sdk/tools/choreoc.py --check [<name.choreo.toml> [-o <out.h>]]

With no -o, the header is written next to the script as
src/choreo_script.h if src/ exists, else choreo_script.h alongside it.

--check writes nothing and exits 1 if a committed header does not match
what its script would generate today.  With no script argument it checks
EVERY generated header in the repository, recovering each one's source
from the regenerate command line embedded in its own banner.  Discovery
rather than a hardcoded list is deliberate: a script compiled into two
consumers drifted once already (examples/webots-formation's copy sat a
release behind examples/cf21bl-formation's), and a list that has to be
edited by hand when a consumer is added would have missed it the same way.

The Python SDK reads the SAME file directly — no generation step:
    from tapestry.script_toml import load_steps
    choreo.submit_script(load_steps("change-partners.choreo.toml"))
"""

import argparse
import re
import sys
from pathlib import Path

# choreoc.py lives at <workspace>/tapestry/sdk/tools/.  Generated banners
# spell paths relative to the WORKSPACE root (west's checkout root, the
# directory holding tapestry/), while discovery scans the repository.
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent.parent
REPO_ROOT      = Path(__file__).resolve().parent.parent.parent

# "python3 tapestry/sdk/tools/choreoc.py <script> -o <header>"
REGEN_RE = re.compile(r"choreoc\.py\s+(\S+)\s+-o\s+(\S+)")

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
    step_fields = [f".advance_on_achieved = "
                  f"{'true' if s.advance_on_achieved else 'false'}"]
    if s.scope:
        step_fields.append(".scope = CHOREO_SCOPE_ALL")
    lines.append("      " + ", ".join(step_fields) + " },")
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


def default_output(script_path: Path) -> Path:
    """Where the header lands when -o is omitted."""
    src_dir = script_path.parent / "src"
    return (src_dir if src_dir.is_dir() else script_path.parent) \
        / "choreo_script.h"


def render(script_path: Path, out_path: Path) -> tuple[ChoreoScript, str]:
    """Parse a script and render the header text it should produce at
    out_path.  The output embeds out_path (in the regenerate banner), so
    the same script rendered for two consumers differs by that line — which
    is why check compares against a render targeting the SAME path."""
    script = parse_file(script_path)
    try:
        rel_script = script_path.resolve().relative_to(WORKSPACE_ROOT)
        rel_out    = out_path.resolve().relative_to(WORKSPACE_ROOT)
        regen = f"python3 tapestry/sdk/tools/choreoc.py {rel_script} -o {rel_out}"
    except ValueError:
        regen = f"python3 choreoc.py {script_path} -o {out_path}"
    return script, emit_header(script, script_path.name, regen)


def discover_pairs() -> list[tuple[Path, Path]]:
    """Find every committed generated header and recover its source script
    from the regenerate command line in its own banner.  Returns
    (script, header) pairs.  Build trees are skipped — they hold copies."""
    pairs: list[tuple[Path, Path]] = []
    for header in sorted(REPO_ROOT.rglob("choreo_script.h")):
        if "build" in header.parts:
            continue
        m = REGEN_RE.search(header.read_text())
        if m is None:
            print(f"choreoc: {header.relative_to(REPO_ROOT)} — no regenerate "
                  f"command in banner; cannot determine its source script",
                  file=sys.stderr)
            continue
        script = (WORKSPACE_ROOT / m.group(1)).resolve()
        if not script.is_file():
            print(f"choreoc: {header.relative_to(REPO_ROOT)} — banner names "
                  f"{m.group(1)}, which does not exist", file=sys.stderr)
            continue
        pairs.append((script, header))
    return pairs


def check_pair(script_path: Path, out_path: Path) -> bool:
    """True if out_path already matches what script_path generates."""
    rel = out_path.resolve()
    try:
        rel = rel.relative_to(REPO_ROOT)
    except ValueError:
        pass
    try:
        _, expected = render(script_path, out_path)
    except (ScriptError, OSError) as e:
        print(f"choreoc: {e}", file=sys.stderr)
        return False
    if not out_path.is_file():
        print(f"choreoc: {rel} — MISSING (never generated)", file=sys.stderr)
        return False
    if out_path.read_text() != expected:
        print(f"choreoc: {rel} — STALE (regeneration needed)", file=sys.stderr)
        return False
    print(f"choreoc: {rel} — already up to date")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(
        prog="choreoc", description="Compile a Choreo script (TOML) into a "
        "C header of choreo_step_t.")
    ap.add_argument("script", type=Path, nargs="?",
                    help="path to the .toml script (omit with --check to "
                         "check every generated header in the repository)")
    ap.add_argument("-o", "--output", type=Path, default=None,
                    help="output header path (default: src/choreo_script.h "
                         "next to the script if src/ exists, else "
                         "choreo_script.h)")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if a committed header does not match what "
                         "its script generates; write nothing")
    args = ap.parse_args()

    if args.script is None:
        if not args.check:
            ap.error("a script is required unless --check is given")
        pairs = discover_pairs()
        if not pairs:
            print("choreoc: no generated headers found", file=sys.stderr)
            return 1
        return 0 if all([check_pair(s, h) for s, h in pairs]) else 1

    out = args.output if args.output is not None \
        else default_output(args.script)

    if args.check:
        return 0 if check_pair(args.script, out) else 1

    try:
        script, text = render(args.script, out)
    except (ScriptError, OSError) as e:
        print(f"choreoc: {e}", file=sys.stderr)
        return 1

    out.write_text(text)
    total_s = script.total_timeout_ms / 1000.0
    print(f"choreoc: \"{script.name}\" — {len(script.steps)} step(s), "
          f"total time bound {total_s:g} s → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
