"""
script_toml.py — Tapestry Choreo script file parser (TOML).

The authoring format for Choreo scripts: a small TOML document a domain
expert can edit cold, parsed with the Python standard library (tomllib,
Python >= 3.11) — no third-party dependencies.

    choreo = "change-partners"

    [[steps]]                    # stay at current stations
    hold = { duration = "30s" }

    [[steps]]                    # swap places, advance when achieved
    exchange = { until = "achieved", timeout = "45s",
                 eps = "25cm", settle = "3s" }

    [[steps]]                    # bow: settle on the new stations
    hold = { duration = "8s" }

Each step table has exactly one goal key:

    hold      stay at the current station          (coordinate-free)
    exchange  rotate stations among participants   (coordinate-free)
    form      arrange into a shape                 target, radius, shape
    move      translate formation to a point       target
    converge  gather at a point                    target
    disperse  spread out                           radius

Common parameters:
    duration | timeout   step time bound — "30s", "500ms", or a bare
                         number of seconds.  REQUIRED on every step: this
                         parser is the flight-authoring surface, and the
                         time bound is the robustness net, so it is
                         stricter than the C API (which allows unbounded
                         achievement-only steps).
    until = "achieved"   advance when the L6 achievement predicate fires
                         (before the time bound).
    eps                  achievement radius — "25cm", "0.25m", "250mm",
                         or bare meters.
    settle               achievement sustain time — duration syntax.
    requires             list of capability names:
                         ["locomotion", "bonding", "sensing", "signaling"]

Goal-specific parameters:
    exchange:  shift = N          ring rotation (default 1)
    form:      target = [x, y], radius, shape = "circle"|"line"|"grid"
    move/converge:  target = [x, y]
    disperse:  radius

hold and exchange reject coordinates by design — they reference the
collective's own configuration (see choreo.h).

Consumers:
    - sdk/tools/choreoc.py  emits the C header for embedded targets.
    - load_steps(path)      returns List[ChoreoStep] for the Python SDK:

          from tapestry.script_toml import load_steps
          choreo.submit_script(load_steps("choreo.toml"))
"""

import tomllib
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .choreo import ChoreoStep, Goal, GoalType, GoalShape, ChoreoCapabilities


GOAL_TYPES = {
    "hold":     GoalType.HOLD,
    "exchange": GoalType.EXCHANGE,
    "form":     GoalType.FORM,
    "move":     GoalType.MOVE,
    "converge": GoalType.CONVERGE,
    "disperse": GoalType.DISPERSE,
}

COORDINATE_FREE = ("hold", "exchange")

CAPABILITIES = {
    "locomotion": ChoreoCapabilities.LOCOMOTION,
    "bonding":    ChoreoCapabilities.BONDING,
    "sensing":    ChoreoCapabilities.SENSING,
    "signaling":  ChoreoCapabilities.SIGNALING,
}

SHAPES = {
    "circle": GoalShape.CIRCLE,
    "line":   GoalShape.LINE,
    "grid":   GoalShape.GRID,
}

_KNOWN_PARAMS = {
    "hold":     {"duration", "timeout", "until", "eps", "settle", "requires"},
    "exchange": {"duration", "timeout", "until", "eps", "settle", "requires",
                 "shift"},
    "form":     {"duration", "timeout", "until", "eps", "settle", "requires",
                 "target", "radius", "shape"},
    "move":     {"duration", "timeout", "until", "eps", "settle", "requires",
                 "target"},
    "converge": {"duration", "timeout", "until", "eps", "settle", "requires",
                 "target"},
    "disperse": {"duration", "timeout", "until", "eps", "settle", "requires",
                 "radius"},
}


class ScriptError(ValueError):
    """A schema or validation error in a Choreo script file."""


@dataclass
class NormalizedStep:
    """One parsed step — only explicitly authored fields are non-None."""
    goal:                str
    max_duration_ms:     int
    advance_on_achieved: bool = False
    slot_shift:          Optional[int] = None
    achieve_eps:         Optional[float] = None
    achieve_hold_ms:     Optional[int] = None
    target:              Optional[Tuple[float, float]] = None
    radius:              Optional[float] = None
    shape:               Optional[str] = None
    required_caps:       int = 0


@dataclass
class ChoreoScript:
    name:  str
    steps: List[NormalizedStep] = field(default_factory=list)

    @property
    def total_timeout_ms(self) -> int:
        return sum(s.max_duration_ms for s in self.steps)


def parse_duration_ms(value, where: str) -> int:
    """"30s", "500ms", or a bare number of seconds → milliseconds."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        ms = value * 1000.0
    elif isinstance(value, str):
        v = value.strip().lower()
        try:
            if v.endswith("ms"):
                ms = float(v[:-2])
            elif v.endswith("s"):
                ms = float(v[:-1]) * 1000.0
            else:
                ms = float(v) * 1000.0
        except ValueError:
            raise ScriptError(f"{where}: cannot parse duration {value!r} "
                              f"(use e.g. \"30s\", \"500ms\", or seconds)")
    else:
        raise ScriptError(f"{where}: cannot parse duration {value!r}")
    if ms <= 0:
        raise ScriptError(f"{where}: duration must be positive, got {value!r}")
    return int(round(ms))


def parse_length_m(value, where: str) -> float:
    """"25cm", "250mm", "0.25m", or bare meters → meters."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        m = float(value)
    elif isinstance(value, str):
        v = value.strip().lower()
        try:
            if v.endswith("mm"):
                m = float(v[:-2]) * 0.001
            elif v.endswith("cm"):
                m = float(v[:-2]) * 0.01
            elif v.endswith("m"):
                m = float(v[:-1])
            else:
                m = float(v)
        except ValueError:
            raise ScriptError(f"{where}: cannot parse length {value!r} "
                              f"(use e.g. \"25cm\", \"0.25m\", or meters)")
    else:
        raise ScriptError(f"{where}: cannot parse length {value!r}")
    if m <= 0:
        raise ScriptError(f"{where}: length must be positive, got {value!r}")
    return m


def _parse_step(index: int, table: dict) -> NormalizedStep:
    where = f"steps[{index}]"

    goal_keys = [k for k in table if k in GOAL_TYPES]
    if len(goal_keys) != 1:
        raise ScriptError(
            f"{where}: each step needs exactly one goal key "
            f"({', '.join(sorted(GOAL_TYPES))}); got {sorted(table)}")
    goal = goal_keys[0]
    extra_top = set(table) - {goal}
    if extra_top:
        raise ScriptError(f"{where}: unexpected keys {sorted(extra_top)} "
                          f"beside goal '{goal}'")

    params = table[goal]
    if params is None:
        params = {}
    if not isinstance(params, dict):
        raise ScriptError(f"{where}: '{goal}' must be a table of parameters, "
                          f"e.g. {goal} = {{ duration = \"30s\" }}")
    unknown = set(params) - _KNOWN_PARAMS[goal]
    if unknown:
        raise ScriptError(f"{where}: unknown parameter(s) {sorted(unknown)} "
                          f"for '{goal}' (allowed: "
                          f"{sorted(_KNOWN_PARAMS[goal])})")
    if "duration" in params and "timeout" in params:
        raise ScriptError(f"{where}: give either 'duration' or 'timeout', "
                          f"not both (they are the same time bound)")
    bound = params.get("duration", params.get("timeout"))
    if bound is None:
        raise ScriptError(
            f"{where}: '{goal}' has no time bound — every step needs "
            f"'duration' (or 'timeout'); the bound is the robustness net "
            f"that keeps a script from stalling in flight")

    step = NormalizedStep(goal=goal,
                          max_duration_ms=parse_duration_ms(bound, where))

    until = params.get("until")
    if until is not None:
        if until != "achieved":
            raise ScriptError(f"{where}: until = {until!r} — the only "
                              f"supported value is \"achieved\"")
        step.advance_on_achieved = True

    if "eps" in params:
        step.achieve_eps = parse_length_m(params["eps"], where)
    if "settle" in params:
        step.achieve_hold_ms = parse_duration_ms(params["settle"], where)
    if "shift" in params:
        shift = params["shift"]
        if not isinstance(shift, int) or isinstance(shift, bool) or shift < 1:
            raise ScriptError(f"{where}: shift must be a positive integer")
        step.slot_shift = shift

    if "requires" in params:
        reqs = params["requires"]
        if not isinstance(reqs, list):
            raise ScriptError(f"{where}: requires must be a list of "
                              f"capability names")
        for r in reqs:
            if r not in CAPABILITIES:
                raise ScriptError(f"{where}: unknown capability {r!r} "
                                  f"(known: {sorted(CAPABILITIES)})")
            step.required_caps |= CAPABILITIES[r]

    if goal in COORDINATE_FREE:
        # target/radius/shape are already rejected via _KNOWN_PARAMS —
        # hold and exchange reference the collective's own configuration.
        pass
    else:
        if goal in ("form", "move", "converge"):
            tgt = params.get("target")
            if (not isinstance(tgt, list) or len(tgt) != 2 or
                    not all(isinstance(c, (int, float)) and
                            not isinstance(c, bool) for c in tgt)):
                raise ScriptError(f"{where}: '{goal}' needs "
                                  f"target = [x, y]")
            step.target = (float(tgt[0]), float(tgt[1]))
        if "radius" in params:
            step.radius = parse_length_m(params["radius"], where)
        elif goal == "disperse":
            raise ScriptError(f"{where}: 'disperse' needs a radius "
                              f"(minimum spacing)")
        if "shape" in params:
            if params["shape"] not in SHAPES:
                raise ScriptError(f"{where}: unknown shape "
                                  f"{params['shape']!r} "
                                  f"(known: {sorted(SHAPES)})")
            step.shape = params["shape"]

    return step


def parse_file(path) -> ChoreoScript:
    """Parse and validate a Choreo script file.  Raises ScriptError."""
    with open(path, "rb") as f:
        try:
            doc = tomllib.load(f)
        except tomllib.TOMLDecodeError as e:
            raise ScriptError(f"{path}: not valid TOML: {e}") from e

    name = doc.get("choreo")
    if not isinstance(name, str) or not name:
        raise ScriptError(f"{path}: missing 'choreo = \"<name>\"'")
    unknown = set(doc) - {"choreo", "steps"}
    if unknown:
        raise ScriptError(f"{path}: unexpected top-level keys "
                          f"{sorted(unknown)}")
    raw_steps = doc.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ScriptError(f"{path}: needs at least one [[steps]] entry")

    return ChoreoScript(
        name=name,
        steps=[_parse_step(i, s) for i, s in enumerate(raw_steps)],
    )


def to_choreo_steps(script: ChoreoScript) -> List[ChoreoStep]:
    """Convert a parsed script to SDK ChoreoStep objects."""
    out = []
    for s in script.steps:
        goal = Goal(type=GOAL_TYPES[s.goal])
        if s.target is not None:
            goal.target = s.target
        if s.radius is not None:
            goal.radius = s.radius
        if s.shape is not None:
            goal.shape = SHAPES[s.shape]
        if s.slot_shift is not None:
            goal.slot_shift = s.slot_shift
        if s.achieve_eps is not None:
            goal.achieve_eps = s.achieve_eps
        if s.achieve_hold_ms is not None:
            goal.achieve_hold_ms = s.achieve_hold_ms
        goal.required_caps = s.required_caps
        out.append(ChoreoStep(goal=goal,
                              max_duration_ms=s.max_duration_ms,
                              advance_on_achieved=s.advance_on_achieved))
    return out


def load_steps(path) -> List[ChoreoStep]:
    """One-call convenience: parse a script file into ChoreoStep objects."""
    return to_choreo_steps(parse_file(path))
