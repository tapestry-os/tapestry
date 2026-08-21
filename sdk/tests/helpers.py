"""
helpers.py — shared fixtures-as-functions for the Tapestry Python tests.

Two jobs:

  * build the wm_entries / scr_state dicts every tick() call needs, so a
    test says what it means ("three elements in a row, peer 2 stale")
    instead of spelling out dict literals; and

  * write telemetry CSVs in exactly the format
    examples/webots-formation/controllers/cf21bl/choreo_telemetry.c
    records, so choreo_sim's --replay path can be tested without Webots,
    a C build, or a captured flight.  `python3 sdk/tests/helpers.py`
    regenerates the committed fixture in data/.
"""

import csv
import json
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

REPO_ROOT   = Path(__file__).resolve().parent.parent.parent
SCRIPT_TOML = REPO_ROOT / "examples/cf21bl-formation/change-partners.choreo.toml"
DATA_DIR    = Path(__file__).resolve().parent / "data"

# scr_state values (mirrors scr.h quorum_state_t).
QUORUM_LOST     = 0
QUORUM_DEGRADED = 1
QUORUM_HEALTHY  = 2


def scr(quorum: int = QUORUM_HEALTHY, role: int = 0,
        leader_id: int = 0) -> dict:
    """An scr_state dict.  tick() reads only quorum_state (see bse.py)."""
    return {"role": role, "quorum_state": quorum, "leader_id": leader_id}


def wm(positions: Sequence[Tuple[float, ...]],
       self_id: int = 0,
       stale: Iterable[int] = (),
       inactive: Iterable[int] = (),
       achieved: Optional[Sequence[bool]] = None) -> List[dict]:
    """
    wm_entries for len(positions) elements with IDs 0..N-1.

    positions[i] is element i's (x, y) or (x, y, z) — z defaults to 0.0 if
    omitted (a real, valid altitude for a test that doesn't care about it,
    not a "z unset" sentinel — see bse.py's wm_entries docstring).
    `stale` / `inactive` name the IDs whose entries carry those flags, and
    `achieved` supplies the gossiped per-element achievement bit the
    scope="all" predicate reads.
    """
    stale, inactive = set(stale), set(inactive)
    return [
        {
            "id":        i,
            "x":         float(p[0]),
            "y":         float(p[1]),
            "z":         float(p[2]) if len(p) > 2 else 0.0,
            "is_active": i not in inactive,
            "is_stale":  i in stale,
            "is_self":   i == self_id,
            "achieved":  bool(achieved[i]) if achieved is not None else False,
        }
        for i, p in enumerate(positions)
    ]


def solo(x: float = 0.0, y: float = 0.0, z: float = 0.0,
        element_id: int = 0) -> List[dict]:
    """A single-element world model — the element sees only itself."""
    return [{"id": element_id, "x": x, "y": y, "z": z, "is_active": True,
             "is_stale": False, "is_self": True, "achieved": False}]


# ── Telemetry CSV (choreo_telemetry.c format) ────────────────────────────────

TELEMETRY_HEADER = [
    "tick", "wall_time_s", "element_id", "pos_x", "pos_y", "pos_z",
    "quorum_state", "fresh_count", "role",
    "goal_type", "script_step", "script_complete", "goal_achieved",
    "directive_type", "directive_target_x", "directive_target_y",
    "directive_target_z",
    "wm_json",
]


def telemetry_row(tick: int, wall_time_s: float, element_id: int,
                  pos: Tuple[float, float, float], entries: List[dict],
                  goal_type: int, script_step: int, script_complete: bool,
                  goal_achieved: bool, directive_type: int,
                  directive_target: Tuple[float, float, float],
                  quorum: int = QUORUM_HEALTHY, role: int = 0) -> list:
    """One CSV row, field-for-field as choreo_telemetry_write() prints it.

    The C writer emits positions at "%.4f" and wall time at "%.3f"; the
    fixture reproduces that rounding so replay's eps tolerance is being
    exercised against realistic, not exact, values.  wm_json carries the
    same eight keys append_entry_json() records, 'achieved' included —
    without it a scope="all" step has nothing to advance on and replays on
    its timeout instead.
    """
    wm_json = json.dumps(
        [{"id": e["id"], "x": round(e["x"], 4), "y": round(e["y"], 4),
          "z": round(e.get("z", 0.0), 4),
          "is_active": bool(e["is_active"]), "is_stale": bool(e["is_stale"]),
          "is_self": bool(e["is_self"]),
          "achieved": bool(e.get("achieved", False))}
         for e in entries if e["is_active"]],
        separators=(",", ":"))
    return [
        tick, f"{wall_time_s:.3f}", element_id,
        f"{pos[0]:.4f}", f"{pos[1]:.4f}", f"{pos[2]:.4f}",
        quorum, sum(1 for e in entries if e["is_active"] and not e["is_stale"]),
        role, goal_type, script_step,
        1 if script_complete else 0, 1 if goal_achieved else 0,
        directive_type,
        f"{directive_target[0]:.4f}", f"{directive_target[1]:.4f}",
        f"{directive_target[2]:.4f}",
        wm_json,
    ]


def write_telemetry(path: Path, rows: List[list]) -> Path:
    with open(path, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(TELEMETRY_HEADER)
        w.writerows(rows)
    return path


def record_reference_run(script_path: Path, element_id: int = 0,
                         n_elements: int = 2,
                         speed_mps: float = 2.0) -> List[list]:
    """
    Drive the Python engine through a script and return the telemetry rows
    one element would have recorded.

    This is the fixture generator, not a test: it stands in for a Webots
    capture so the --replay code path (CSV reader, wm_json decoding, the
    per-field diff) has something to run against in CI.  Replaying its own
    output can only ever prove self-consistency — a frozen recording that
    stops matching means the Python engine's tick-by-tick behavior changed,
    which is exactly the regression worth catching here.  Cross-language
    parity against the C engine is what a real capture proves; see
    sdk/CHOREO_SCRIPTS.md.
    """
    from tapestry.bse import BSEDirectiveType
    from tapestry.choreo import Choreo
    from tapestry.script_toml import load_steps, parse_file

    script = parse_file(script_path)
    steps  = load_steps(script_path)
    cycle  = Choreo.WM_CYCLE_MS

    ids      = list(range(n_elements))
    choreos  = {i: Choreo(element_id=i, capabilities=None) for i in ids}
    for ch in choreos.values():
        assert ch.submit_script(steps) == 0

    import math
    radius = max(2.0, n_elements * 0.5)
    positions = {i: (radius * math.cos(2 * math.pi * i / n_elements),
                     radius * math.sin(2 * math.pi * i / n_elements), 0.0)
                 for i in ids} if n_elements > 1 else {0: (0.0, 0.0, 0.0)}
    achieved = dict.fromkeys(ids, False)

    max_step_m = speed_mps * (cycle / 1000.0)
    max_ticks  = max(1, -(-script.total_timeout_ms // cycle))
    state      = scr(QUORUM_HEALTHY)
    rows: List[list] = []

    for tick in range(max_ticks):
        snapshot = [{"id": i, "x": positions[i][0], "y": positions[i][1],
                     "z": positions[i][2], "is_active": True, "is_stale": False,
                     "achieved": achieved[i]} for i in ids]
        new_pos, new_ach = {}, {}
        for i, ch in choreos.items():
            entries = [dict(e, is_self=(e["id"] == i)) for e in snapshot]
            ch.tick(entries, state)
            d = ch.get_directive()

            if i == element_id:
                rows.append(telemetry_row(
                    tick=tick, wall_time_s=tick * cycle / 1000.0,
                    element_id=i, pos=positions[i], entries=entries,
                    goal_type=int(ch.current_goal_type()),
                    script_step=ch.script_step(),
                    script_complete=ch.script_complete(),
                    goal_achieved=ch.goal_achieved(),
                    directive_type=int(d.type),
                    directive_target=tuple(d.target)))

            x, y, z = positions[i]
            if d.type == BSEDirectiveType.MOVE_TO_POINT:
                dx, dy, dz = d.target[0] - x, d.target[1] - y, d.target[2] - z
                dist = math.hypot(dx, dy, dz)
                if dist > max_step_m:
                    x += dx / dist * max_step_m
                    y += dy / dist * max_step_m
                    z += dz / dist * max_step_m
                else:
                    x, y, z = d.target
            new_pos[i]  = (x, y, z)
            new_ach[i]  = ch.goal_achieved()
        positions, achieved = new_pos, new_ach
        if all(ch.script_complete() for ch in choreos.values()):
            break
    return rows


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(REPO_ROOT / "sdk" / "python"))
    DATA_DIR.mkdir(exist_ok=True)
    out = write_telemetry(DATA_DIR / "replay_change_partners_0.csv",
                          record_reference_run(SCRIPT_TOML))
    print(f"wrote {out} ({sum(1 for _ in open(out)) - 1} rows)")
