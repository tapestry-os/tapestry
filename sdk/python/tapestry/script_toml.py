"""
script_toml.py — Tapestry Choreo script file parser (TOML).

The authoring format for Choreo scripts: a small TOML document a domain
expert can edit cold, parsed with the Python standard library (tomllib,
Python >= 3.11) — no third-party dependencies.  File naming convention:
<name>.choreo.toml, where <name> matches the script's own "choreo ="
key below (e.g. change-partners.choreo.toml) — see sdk/CHOREO_SCRIPTS.md.

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
    duration | timeout   step time bound — "30s", "500ms", "45min", "2h",
                         or a bare number of seconds.  REQUIRED on every
                         step: this parser is the flight-authoring
                         surface, and the time bound is the robustness
                         net, so it is stricter than the C API (which
                         allows unbounded achievement-only steps).
    until = "achieved"   advance when the L6 achievement predicate fires
                         (before the time bound).  Not allowed on hold:
                         hold is trivially achieved in the current
                         runtime, so hold steps are duration-governed
    eps                  achievement radius — "25cm", "0.25m", "250mm",
                         "500um", or bare meters.
    settle               achievement sustain time — duration syntax.
    scope = "self"|"all" whose achievement gates until = "achieved"
                         (default "self"). "all" advances only once this
                         element AND every fresh peer have achieved
                         (aggregated from gossiped state — eventually
                         consistent, not a synchronization barrier; see
                         choreo_collective_achieved() in choreo.h). Only
                         valid alongside until = "achieved"; not allowed
                         on hold (which never carries until either).
    requires             list of capability names:
                         ["locomotion", "bonding", "sensing", "signaling"]
    indicator            §12 Stage 5 effect: "idle"|"active"|"degraded"|
                         "failed" — while this step is active,
                         current_indicator() returns it instead of NONE.
                         Omit for "no override" (the default).
    telemetry_tag        §12 Stage 5 effect: an opaque label surfaced by
                         current_telemetry_tag() for a telemetry capture
                         to record alongside the step — not a wire
                         delivery mechanism (see choreo.h).

Goal-specific parameters:
    exchange:  shift = N          ring rotation (default 1)
    form:      target = [x, y, z], radius (both REQUIRED — radius 0 would
               send every element to the same vertex),
               shape = "circle"|"line"|"grid"
    move/converge:  target = [x, y, z]
    disperse:  radius (REQUIRED — minimum spacing)

hold and exchange reject coordinates by design — they reference the
collective's own configuration (see choreo.h).

Tracks (Choreo SDK Design doc §7) — concurrent, participant-scoped step
sequences; an element runs exactly one track, chosen by the FIRST filter
it matches:

    choreo = "spill-response"

    [[tracks]]                          # perimeter: needs a sensor
    filter = { requires = ["sensing"] }
    [[tracks.steps]]
    hold = { duration = "300s" }

    [[tracks]]                          # everyone else: catch-all (no filter)
    [[tracks.steps]]
    form = { target = [0, 0, 3], radius = 5, duration = "300s" }

A script file gives either [[steps]] (single implicit "all" track — every
script before this feature existed, and still the default) or [[tracks]],
never both.  Each track's `filter` table takes:

    requires    list of capability names (as in a step's `requires`) —
                matches when this element's own capabilities satisfy them
    energy_low  bool — matches when this element's own gossiped
                health_flags has ELEMENT_HEALTH_LOW_BATTERY set

An empty/omitted filter (`filter = {}` or no `filter` key) matches every
element — the "catch-all" a script needs at least one of, in the last
position, to guarantee every element has somewhere to run.  Each track's
`[[tracks.steps]]` is a full step list with its own schema as above
(including `name =` / `on = [...]` transitions — goto targets resolve
only within the SAME track) and its own optional top-level `max_runtime`
for a cyclic track (§8.4); the top-level `max_runtime` key is single-track
([[steps]]) only.

Element departure policy — how a survivor reacts when a participating peer
stops participating (self-declared, or inferred LOST past
WM_EXPIRE_THRESHOLD_MS; see choreo.h's choreo_departure_policy_t):

    mode = "cp"                         # coarse dial: "ap" (default) or "cp"

is shorthand for a plain [on_departure] with default reasons/
min_participants — "ap" -> policy="continue" (do nothing extra beyond
what element_is_participating() already gives every collective
predicate for free); "cp" -> policy="land_in_place" (land immediately —
NOT "recall", which flies home first; a landed-in-place element might
resume the show later, recall abandons it).  For anything more specific,
give the table instead (mutually exclusive with `mode` — pick one):

    [on_departure]
    policy = "hold"                     # continue|hold|land_in_place|recall
    min_participants = 2                # 0 (default) = any departure fires;
                                         # N is itself still fine — only
                                         # dropping STRICTLY BELOW N fires
    reasons = ["fixloss", "geofence"]   # default ["*"] = every reason

A step's own `on_departure = "land_in_place"` replaces `policy` alone for
THAT step only; `reasons`/`min_participants` always stay script-level.
Neither `mode` nor [on_departure] given: policy="continue", every script
written before this feature existed is unaffected.

NOT modeled by sdk/tools/choreo_sim.py --simulate (the Python Choreo
class) — see ChoreoDeparturePolicy's doc in choreo.py. Only choreoc.py's
C-header codegen (the path that actually flies on hardware) acts on
these fields today.

Consumers:
    - sdk/tools/choreoc.py  emits the C header for embedded targets.
    - load_steps(path)      returns List[ChoreoStep] for a [[steps]] script:

          from tapestry.script_toml import load_steps
          choreo.submit_script(load_steps("change-partners.choreo.toml"))

    - load_tracks(path)     returns List[ChoreoTrack] for a [[tracks]] script:

          from tapestry.script_toml import load_tracks
          choreo.submit_tracks(wm_entries, load_tracks("spill-response.choreo.toml"))
"""

import tomllib
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .choreo import (ChoreoStep, ChoreoTrack, ChoreoTrackFilter, Goal,
                     GoalType, GoalShape, ChoreoCapabilities, ChoreoEvent,
                     ChoreoTransition, SubstrateSignal, CHOREO_MAX_TRANSITIONS,
                     CHOREO_MAX_TRACKS, ChoreoDeparturePolicy)
from .bse import BSEFrame, BSEAnchorSelector, BSEMotion


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
    "locomotion":   ChoreoCapabilities.LOCOMOTION,
    "bonding":      ChoreoCapabilities.BONDING,
    "sensing":      ChoreoCapabilities.SENSING,
    "signaling":    ChoreoCapabilities.SIGNALING,
    "abs_position": ChoreoCapabilities.ABS_POSITION,
}

SHAPES = {
    "circle": GoalShape.CIRCLE,
    "line":   GoalShape.LINE,
    "grid":   GoalShape.GRID,
}

# Choreo SDK Design doc §5 frame ladder — form/converge only (stage 1).
# "absolute" (default) is a literal target=[x,y,z] — space is 3D throughout.
FRAMES = {"absolute": 0, "collective": 1, "element": 2}

# §5.2 anchor selectors, frame="element" only. "newest"/"oldest" are named
# here (so an author gets "not yet implemented", not "unknown selector")
# but rejected explicitly — they need L4 join-order tracking that doesn't
# exist yet. "id:N" is parsed separately (embeds the id in the string).
ANCHOR_SELECTORS = {"leader": 0, "self": 2, "lowest-energy": 3}
_ANCHOR_SELECTORS_DEFERRED = {"newest", "oldest"}

# §12 Stage 5 effect: indicator = "<name>" mirrors substrate_signal_t
# (substrate.h) by name, minus "none" — a step that wants no override
# simply omits the key (the runtime default already means "no override").
INDICATORS = {
    "idle":     SubstrateSignal.IDLE,
    "active":   SubstrateSignal.ACTIVE,
    "degraded": SubstrateSignal.DEGRADED,
    "failed":   SubstrateSignal.FAILED,
}

# Common to every goal key — allowed on any step regardless of which goal
# it wraps, same as "requires"/"on" above.
_EFFECT_PARAMS = {"indicator", "telemetry_tag"}

# Element departure policy (see the module docstring's "Element departure
# policy" section) — a per-step override, allowed on any goal key exactly
# like _EFFECT_PARAMS above; a departure can happen during any step type.
_DEPARTURE_PARAMS = {"on_departure"}

_KNOWN_PARAMS = {
    "hold":     {"duration", "timeout", "until", "eps", "settle", "requires", "on"} | _EFFECT_PARAMS | _DEPARTURE_PARAMS,
    "exchange": {"duration", "timeout", "until", "eps", "settle", "requires",
                 "scope", "shift", "path", "on"} | _EFFECT_PARAMS | _DEPARTURE_PARAMS,
    "form":     {"duration", "timeout", "until", "eps", "settle", "requires",
                 "scope", "target", "radius", "shape", "frame", "anchor",
                 "spin", "on"} | _EFFECT_PARAMS | _DEPARTURE_PARAMS,
    "move":     {"duration", "timeout", "until", "eps", "settle", "requires",
                 "scope", "target", "on"} | _EFFECT_PARAMS | _DEPARTURE_PARAMS,
    "converge": {"duration", "timeout", "until", "eps", "settle", "requires",
                 "scope", "target", "frame", "anchor", "on"} | _EFFECT_PARAMS | _DEPARTURE_PARAMS,
    "disperse": {"duration", "timeout", "until", "eps", "settle", "requires",
                 "scope", "radius", "on"} | _EFFECT_PARAMS | _DEPARTURE_PARAMS,
}

# Element departure policy names (script-level `mode`/[on_departure] and
# per-step `on_departure`) — mirrors ChoreoDeparturePolicy (choreo.py) /
# choreo_departure_policy_t (choreo.h) by name.
DEPARTURE_POLICIES = {
    "continue":      ChoreoDeparturePolicy.CONTINUE,
    "hold":          ChoreoDeparturePolicy.HOLD,
    "land_in_place": ChoreoDeparturePolicy.LAND_IN_PLACE,
    "recall":        ChoreoDeparturePolicy.RECALL,
}

# Departure reason names — mirrors csm.h's tapestry_departure_reason_t by
# value (0-3) plus "lost" (bit 4, choreo.h's CHOREO_DEPARTURE_REASON_LOST_BIT
# — never on the wire, only ever inferred locally from expiry). "*" (the
# default) is every bit set — CHOREO_DEPARTURE_REASONS_ALL.
DEPARTURE_REASONS = {
    "complete": 0, "backstop": 1, "fixloss": 2, "geofence": 3, "lost": 4,
}
DEPARTURE_REASONS_ALL_MASK = 0x1F

# §6.1: orbit = { around, radius, rate, ... } is pure TOML-layer sugar,
# desugared (in _desugar_orbit) to a form step before the generic parsing
# logic ever sees it — not a distinct goal type, so not in GOAL_TYPES.
# Listed here only for its own validation/error messages.
_ORBIT_PARAMS = {"duration", "timeout", "until", "eps", "settle", "requires",
                 "scope", "around", "radius", "rate", "on"} | _EFFECT_PARAMS

# §8.2 event vocabulary — see choreo.h's ChoreoEvent for why
# quorum_degraded/quorum_recovered aren't here yet (no concrete use case
# identified for either).
EVENTS = {
    "achieved":       "achieved",
    "element_joined": "element_joined",
    "element_lost":   "element_lost",
    "count_gte":      "count_gte",
    "count_eq":       "count_eq",
    "anchor_lost":    "anchor_lost",
    "quorum_lost":    "quorum_lost",
}
_EVENTS_NEEDING_THRESHOLD = {"count_gte", "count_eq"}

# "end" is reserved — it's the goto sentinel meaning "complete the script",
# not an author-assignable step name.
_RESERVED_STEP_NAMES = {"end"}

# §7 track filter — every key optional; the zero/empty table matches every
# element (the "catch-all" a script needs at least one of).
_KNOWN_TRACK_FILTER_PARAMS = {"requires", "energy_low"}
_KNOWN_TRACK_PARAMS = {"filter", "steps", "max_runtime"}

SCOPES = {"self": 0, "all": 1}

_FRAME_ENUM = {
    "absolute":   BSEFrame.ABSOLUTE,
    "collective": BSEFrame.COLLECTIVE,
    "element":    BSEFrame.ELEMENT,
}
_ANCHOR_ENUM = {
    "leader":         BSEAnchorSelector.LEADER,
    "id":             BSEAnchorSelector.ID,
    "self":           BSEAnchorSelector.SELF,
    "lowest-energy":  BSEAnchorSelector.LOWEST_ENERGY,
}
_MOTION_ENUM = {
    "static": BSEMotion.STATIC,
    "spin":   BSEMotion.SPIN,
}
_EVENT_ENUM = {
    "achieved":       ChoreoEvent.ACHIEVED,
    "element_joined": ChoreoEvent.ELEMENT_JOINED,
    "element_lost":   ChoreoEvent.ELEMENT_LOST,
    "count_gte":      ChoreoEvent.COUNT_GTE,
    "count_eq":       ChoreoEvent.COUNT_EQ,
    "anchor_lost":    ChoreoEvent.ANCHOR_LOST,
    "quorum_lost":    ChoreoEvent.QUORUM_LOST,
}


class ScriptError(ValueError):
    """A schema or validation error in a Choreo script file."""


@dataclass
class NormalizedTransition:
    """One parsed `on = [...]` entry.  event is the TOML string (mapped to
    ChoreoEvent in to_choreo_steps, same pattern as NormalizedStep.frame/
    motion staying strings until conversion).  goto_step_idx is already
    resolved from a step name (or "end") to a literal index — see
    _resolve_goto()."""
    event:         str
    goto_step_idx: int
    threshold:     int = 0


@dataclass
class NormalizedStep:
    """One parsed step — only explicitly authored fields are non-None."""
    goal:                str
    max_duration_ms:     int
    advance_on_achieved: bool = False
    scope:               int = 0   # CHOREO_SCOPE_SELF (0) / CHOREO_SCOPE_ALL (1)
    slot_shift:          Optional[int] = None
    direct_path:         bool = False
    achieve_eps:         Optional[float] = None
    achieve_hold_ms:     Optional[int] = None
    target:              Optional[Tuple[float, float, float]] = None
    radius:              Optional[float] = None
    shape:               Optional[str] = None
    required_caps:       int = 0
    frame:               str = "absolute"          # form/converge only
    anchor_select:       Optional[str] = None       # frame == "element" only
    anchor_id:           Optional[int] = None       # anchor_select == "id" only
    motion:              str = "static"             # form only (§6)
    spin_rate_radps:     Optional[float] = None      # motion == "spin" only
    name:                Optional[str] = None       # §8.3 goto target name
    on:                  List[NormalizedTransition] = field(default_factory=list)
    indicator:           Optional[str] = None       # §12 Stage 5 effect
    telemetry_tag:       Optional[str] = None       # §12 Stage 5 effect
    on_departure:        Optional[str] = None       # per-step policy override


@dataclass
class NormalizedTrack:
    """One parsed [[tracks]] entry — its filter plus its own full step
    list (§7).  Mirrors choreo_track_t / choreo_track_filter_t."""
    required_caps:       int = 0
    requires_energy_low: bool = False
    steps:                List[NormalizedStep] = field(default_factory=list)
    # Same §8.4 cycle-bound rule as ChoreoScript.max_runtime_ms, but
    # per-track: a cycle in one track doesn't bound the others.
    max_runtime_ms:       Optional[int] = None

    @property
    def total_timeout_ms(self) -> int:
        if self.max_runtime_ms is not None:
            return self.max_runtime_ms
        return sum(s.max_duration_ms for s in self.steps)


@dataclass
class ChoreoScript:
    name:  str
    # Exactly one of (steps, tracks) is populated — [[steps]] (single
    # implicit "all" track, every script before §7 existed) or [[tracks]]
    # (§7, concurrent participant-scoped step sequences).  parse_file()
    # enforces this; tracks is None for a [[steps]] script and vice versa.
    steps: List[NormalizedStep] = field(default_factory=list)
    tracks: Optional[List[NormalizedTrack]] = None
    # §8.4: required (and enforced by parse_file) when the step->goto
    # transition graph has a cycle — a cyclic script can run indefinitely,
    # so the sum of step durations no longer bounds its runtime.  None on
    # an acyclic script (the sum is authoritative, as always).  [[steps]]
    # only — see NormalizedTrack.max_runtime_ms for the [[tracks]] case.
    max_runtime_ms: Optional[int] = None
    # Element departure policy — script-level default, from the coarse
    # top-level `mode = "ap"|"cp"` dial OR the more specific [on_departure]
    # table (mutually exclusive — see parse_file()).  "continue"/ALL/0 is
    # the default when neither is given: byte-identical to every script
    # written before this feature existed.
    departure_policy:            str = "continue"
    departure_reasons_mask:      int = DEPARTURE_REASONS_ALL_MASK
    departure_min_participants:  int = 0

    # §11/§8.4 satisfiability warnings (see _derived_capability_warnings())
    # — non-fatal, unlike everything else in this module: the runtime
    # derives and enforces these requirements automatically
    # (derived_caps() in choreo.c) whether or not `requires` lists them,
    # so an unlisted one isn't a script error. It IS worth surfacing to
    # the author, though — deploying to an element that lacks the derived
    # capability still gets a runtime -EPERM, and a `requires` list that
    # doesn't mention it gives no advance warning of that at authoring
    # time. Empty for a script with nothing to flag.
    warnings: List[str] = field(default_factory=list)

    @property
    def total_timeout_ms(self) -> int:
        if self.max_runtime_ms is not None:
            return self.max_runtime_ms
        return sum(s.max_duration_ms for s in self.steps)


def parse_duration_ms(value, where: str) -> int:
    """"30s", "500ms", "45min", "2h", or a bare number of seconds → ms."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        ms = value * 1000.0
    elif isinstance(value, str):
        v = value.strip().lower()
        try:
            if v.endswith("ms"):
                ms = float(v[:-2])
            elif v.endswith("min"):
                ms = float(v[:-3]) * 60_000.0
            elif v.endswith("h"):
                ms = float(v[:-1]) * 3_600_000.0
            elif v.endswith("s"):
                ms = float(v[:-1]) * 1000.0
            else:
                ms = float(v) * 1000.0
        except ValueError as e:
            raise ScriptError(f"{where}: cannot parse duration {value!r} "
                              f"(use e.g. \"30s\", \"500ms\", \"45min\", "
                              f"\"2h\", or seconds)") from e
    else:
        raise ScriptError(f"{where}: cannot parse duration {value!r}")
    if ms <= 0:
        raise ScriptError(f"{where}: duration must be positive, got {value!r}")
    return int(round(ms))


def parse_length_m(value, where: str) -> float:
    """"25cm", "250mm", "500um", "0.25m", or bare meters → meters."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        m = float(value)
    elif isinstance(value, str):
        v = value.strip().lower()
        try:
            if v.endswith("um"):
                m = float(v[:-2]) * 1e-6
            elif v.endswith("mm"):
                m = float(v[:-2]) * 0.001
            elif v.endswith("cm"):
                m = float(v[:-2]) * 0.01
            elif v.endswith("m"):
                m = float(v[:-1])
            else:
                m = float(v)
        except ValueError as e:
            raise ScriptError(f"{where}: cannot parse length {value!r} "
                              f"(use e.g. \"25cm\", \"0.25m\", \"500um\", "
                              f"or meters)") from e
    else:
        raise ScriptError(f"{where}: cannot parse length {value!r}")
    if m <= 0:
        raise ScriptError(f"{where}: length must be positive, got {value!r}")
    return m


def parse_angular_rate_radps(value, where: str) -> float:
    """"0.15rad/s", or a bare number (already rad/s) -> rad/s.  Signed —
    negative is the opposite (CW) turn direction."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    if isinstance(value, str):
        v = value.strip().lower()
        try:
            if v.endswith("rad/s"):
                return float(v[:-5])
            return float(v)
        except ValueError as e:
            raise ScriptError(f"{where}: cannot parse angular rate "
                              f"{value!r} (use e.g. \"0.15rad/s\", or a "
                              f"bare rad/s number)") from e
    raise ScriptError(f"{where}: cannot parse angular rate {value!r}")


def _desugar_orbit(where: str, params) -> dict:
    """orbit = { around, radius, rate, ... } -> form(frame="element",
    anchor={select=around}, shape="circle", spin=rate) — §6.1 preset, pure
    TOML-layer sugar with no new C primitive; the result flows through
    exactly the same form parsing as any hand-written form+spin step."""
    if not isinstance(params, dict):
        raise ScriptError(
            f"{where}: 'orbit' must be a table of parameters, e.g. "
            f"orbit = {{ around = \"leader\", radius = \"1m\", "
            f"rate = \"0.15rad/s\", duration = \"60s\" }}")
    unknown = set(params) - _ORBIT_PARAMS
    if unknown:
        raise ScriptError(f"{where}: unknown parameter(s) {sorted(unknown)} "
                          f"for 'orbit' (allowed: {sorted(_ORBIT_PARAMS)})")
    for required in ("around", "radius", "rate"):
        if required not in params:
            raise ScriptError(f"{where}: 'orbit' needs {required} = ...")

    desugared = dict(params)
    around = desugared.pop("around")
    rate   = desugared.pop("rate")
    desugared["frame"]  = "element"
    desugared["anchor"] = {"select": around}
    desugared["shape"]  = "circle"
    desugared["spin"]   = rate
    return desugared


def _resolve_goto(where: str, goto, name_to_index: dict, n_steps: int) -> int:
    """"end" -> n_steps (the completion sentinel); otherwise a step name
    resolved via name_to_index, built from every step's own name = "..."
    (see parse_file)."""
    if goto == "end":
        return n_steps
    if not isinstance(goto, str) or goto not in name_to_index:
        raise ScriptError(f"{where}: goto {goto!r} does not name a step "
                          f"(use \"end\", or one of the script's own "
                          f"name = \"...\" step names)")
    return name_to_index[goto]


def _parse_step(index: int, table: dict, name_to_index: dict,
                n_steps: int, label: str = "steps") -> NormalizedStep:
    where = f"{label}[{index}]"

    name = table.get("name")
    if name is not None and not isinstance(name, str):
        raise ScriptError(f"{where}: 'name' must be a string")

    if "orbit" in table:
        extra_top = set(table) - {"orbit", "name"}
        if extra_top:
            raise ScriptError(f"{where}: unexpected keys {sorted(extra_top)} "
                              f"beside 'orbit'")
        table = {"form": _desugar_orbit(where, table["orbit"])}

    goal_keys = [k for k in table if k in GOAL_TYPES]
    if len(goal_keys) != 1:
        raise ScriptError(
            f"{where}: each step needs exactly one goal key "
            f"({', '.join(sorted(GOAL_TYPES))}); got {sorted(table)}")
    goal = goal_keys[0]
    extra_top = set(table) - {goal, "name"}
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
    if goal == "hold":
        reserved = {"until", "eps", "settle"} & set(params)
        if reserved:
            raise ScriptError(
                f"{where}: {sorted(reserved)} not allowed on 'hold' — hold "
                f"is trivially achieved in the current runtime (a hold step "
                f"with until = \"achieved\" would advance on the first "
                f"tick), so hold steps are duration-governed; these "
                f"parameters are reserved for scoped achievement")
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
    step.name = name

    if "on" in params:
        on_list = params["on"]
        if not isinstance(on_list, list):
            raise ScriptError(f"{where}: 'on' must be a list of "
                              f"{{ event = ..., goto = ... }} tables")
        if len(on_list) > CHOREO_MAX_TRANSITIONS:
            raise ScriptError(
                f"{where}: 'on' has {len(on_list)} entries — the runtime "
                f"only holds {CHOREO_MAX_TRANSITIONS}")
        for j, t in enumerate(on_list):
            twhere = f"{where}.on[{j}]"
            if not isinstance(t, dict):
                raise ScriptError(f"{twhere}: must be a table, e.g. "
                                  f"{{ event = \"achieved\", goto = \"...\" }}")
            extra = set(t) - {"event", "goto", "threshold"}
            if extra:
                raise ScriptError(f"{twhere}: unknown key(s) {sorted(extra)}")
            event = t.get("event")
            if event not in EVENTS:
                raise ScriptError(f"{twhere}: unknown event {event!r} "
                                  f"(known: {sorted(EVENTS)})")
            if "goto" not in t:
                raise ScriptError(f"{twhere}: needs goto = <step name> "
                                  f"or \"end\"")
            goto_idx = _resolve_goto(twhere, t["goto"], name_to_index, n_steps)
            if event in _EVENTS_NEEDING_THRESHOLD:
                if "threshold" not in t:
                    raise ScriptError(f"{twhere}: event {event!r} needs "
                                      f"a threshold")
            elif "threshold" in t:
                raise ScriptError(f"{twhere}: 'threshold' only applies to "
                                  f"count_gte/count_eq events")
            step.on.append(NormalizedTransition(
                event=event, goto_step_idx=goto_idx,
                threshold=t.get("threshold", 0)))

    until = params.get("until")
    if until is not None:
        if until != "achieved":
            raise ScriptError(f"{where}: until = {until!r} — the only "
                              f"supported value is \"achieved\"")
        step.advance_on_achieved = True

    if "scope" in params:
        if not step.advance_on_achieved:
            raise ScriptError(f"{where}: 'scope' has no effect without "
                              f"until = \"achieved\"")
        scope = params["scope"]
        if scope not in SCOPES:
            raise ScriptError(f"{where}: scope must be \"self\" (default) "
                              f"or \"all\", got {scope!r}")
        step.scope = SCOPES[scope]

    if "eps" in params:
        step.achieve_eps = parse_length_m(params["eps"], where)
    if "settle" in params:
        step.achieve_hold_ms = parse_duration_ms(params["settle"], where)
    if "shift" in params:
        shift = params["shift"]
        if not isinstance(shift, int) or isinstance(shift, bool) or shift < 1:
            raise ScriptError(f"{where}: shift must be a positive integer")
        step.slot_shift = shift
    if "path" in params:
        if params["path"] not in ("arc", "direct"):
            raise ScriptError(f"{where}: path must be \"arc\" (default — "
                              f"preserves XY separation) or \"direct\" "
                              f"(beeline; safe when deconfliction is "
                              f"vertical, e.g. ID-staggered altitudes)")
        step.direct_path = params["path"] == "direct"

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

    if "indicator" in params:
        indicator = params["indicator"]
        if indicator not in INDICATORS:
            raise ScriptError(f"{where}: unknown indicator {indicator!r} "
                              f"(known: {sorted(INDICATORS)})")
        step.indicator = indicator
    if "telemetry_tag" in params:
        tag = params["telemetry_tag"]
        if not isinstance(tag, str) or not tag:
            raise ScriptError(f"{where}: telemetry_tag must be a "
                              f"non-empty string")
        step.telemetry_tag = tag
    if "on_departure" in params:
        policy = params["on_departure"]
        if policy not in DEPARTURE_POLICIES:
            raise ScriptError(f"{where}: unknown on_departure policy "
                              f"{policy!r} (known: "
                              f"{sorted(DEPARTURE_POLICIES)})")
        step.on_departure = policy

    if goal in COORDINATE_FREE:
        # target/radius/shape are already rejected via _KNOWN_PARAMS —
        # hold and exchange reference the collective's own configuration.
        pass
    else:
        if goal == "move":
            # frame/anchor are form/converge only this stage (§12's own
            # staged order) — move keeps its unconditional absolute target.
            tgt = params.get("target")
            if (not isinstance(tgt, list) or len(tgt) != 3 or
                    not all(isinstance(c, (int, float)) and
                            not isinstance(c, bool) for c in tgt)):
                raise ScriptError(f"{where}: '{goal}' needs "
                                  f"target = [x, y, z]")
            step.target = (float(tgt[0]), float(tgt[1]), float(tgt[2]))
        elif goal in ("form", "converge"):
            frame = params.get("frame", "absolute")
            if frame not in FRAMES:
                raise ScriptError(f"{where}: unknown frame {frame!r} "
                                  f"(known: {sorted(FRAMES)})")
            step.frame = frame

            if frame == "absolute":
                if "anchor" in params:
                    raise ScriptError(
                        f"{where}: 'anchor' has no effect with "
                        f"frame = \"absolute\" (the default) — it only "
                        f"applies with frame = \"element\"")
                tgt = params.get("target")
                if (not isinstance(tgt, list) or len(tgt) != 3 or
                        not all(isinstance(c, (int, float)) and
                                not isinstance(c, bool) for c in tgt)):
                    raise ScriptError(
                        f"{where}: '{goal}' needs target = [x, y, z] with "
                        f"frame = \"absolute\" (the default)")
                step.target = (float(tgt[0]), float(tgt[1]), float(tgt[2]))
            else:
                if "target" in params:
                    raise ScriptError(
                        f"{where}: 'target' has no effect with "
                        f"frame = {frame!r} — the target is derived from "
                        f"the frame instead")
                if frame == "collective" and "anchor" in params:
                    raise ScriptError(
                        f"{where}: 'anchor' only applies with "
                        f"frame = \"element\"")
                if frame == "element":
                    anchor = params.get("anchor")
                    if not isinstance(anchor, dict) or "select" not in anchor:
                        raise ScriptError(
                            f"{where}: frame = \"element\" needs "
                            f"anchor = {{ select = ... }}")
                    extra = set(anchor) - {"select"}
                    if extra:
                        raise ScriptError(f"{where}: unexpected anchor "
                                          f"key(s) {sorted(extra)}")
                    sel = anchor["select"]
                    if isinstance(sel, str) and sel.startswith("id:"):
                        try:
                            step.anchor_id = int(sel[3:])
                        except ValueError:
                            raise ScriptError(
                                f"{where}: anchor select {sel!r} — "
                                f"'id:N' needs an integer N") from None
                        step.anchor_select = "id"
                    elif sel in _ANCHOR_SELECTORS_DEFERRED:
                        raise ScriptError(
                            f"{where}: anchor select {sel!r} is not yet "
                            f"implemented — needs L4 join-order tracking "
                            f"that doesn't exist yet; use \"leader\", "
                            f"\"self\", \"lowest-energy\", or \"id:N\"")
                    elif sel not in ANCHOR_SELECTORS:
                        raise ScriptError(
                            f"{where}: unknown anchor select {sel!r} "
                            f"(known: {sorted(ANCHOR_SELECTORS)}, or "
                            f"\"id:N\")")
                    else:
                        step.anchor_select = sel
        if "radius" in params:
            step.radius = parse_length_m(params["radius"], where)
        elif goal == "disperse":
            raise ScriptError(f"{where}: 'disperse' needs a radius "
                              f"(minimum spacing)")
        elif goal == "form":
            raise ScriptError(
                f"{where}: 'form' needs a radius — with radius 0 the BSE "
                f"assigns every element the SAME vertex (target + "
                f"radius·[cos,sin]), sending the whole collective to "
                f"one point with only platform deconfliction between "
                f"airframes")
        if "shape" in params:
            if params["shape"] not in SHAPES:
                raise ScriptError(f"{where}: unknown shape "
                                  f"{params['shape']!r} "
                                  f"(known: {sorted(SHAPES)})")
            step.shape = params["shape"]
        if "spin" in params:
            # form only — _KNOWN_PARAMS["converge"] doesn't list "spin",
            # so it's already rejected there as an unknown parameter;
            # converge's target IS the frame origin, so "rotating the
            # offset" would be a no-op (see bse.h's CONVERGE comment).
            step.motion = "spin"
            step.spin_rate_radps = parse_angular_rate_radps(params["spin"], where)

    return step


def _build_name_map(path, raw_steps: list, label: str = "steps") -> dict:
    """step name -> index, validated for duplicates and the reserved
    "end" name (§8.3's goto sentinel, not an author-assignable name).
    Runs BEFORE _parse_step over the raw (pre-orbit-desugar) tables, so
    every step's on[] can resolve goto targets regardless of parse order.
    `label` scopes error messages to a single track's own step list
    (goto targets resolve only within the same track — see §7's
    module doc) when parsing [[tracks]] rather than [[steps]]."""
    name_to_index = {}
    for i, s in enumerate(raw_steps):
        n = s.get("name") if isinstance(s, dict) else None
        if n is None:
            continue
        if not isinstance(n, str):
            raise ScriptError(f"{path}: {label}[{i}]: 'name' must be a string")
        if n in _RESERVED_STEP_NAMES:
            raise ScriptError(
                f"{path}: {label}[{i}]: name {n!r} is reserved (use it as "
                f"goto = \"end\", not a step name)")
        if n in name_to_index:
            raise ScriptError(
                f"{path}: {label}[{i}]: duplicate step name {n!r} (already "
                f"used by {label}[{name_to_index[n]}])")
        name_to_index[n] = i
    return name_to_index


def _has_cycle(n_steps: int, steps: List[NormalizedStep]) -> bool:
    """DFS cycle detection over the step -> {implicit next, every on[].
    goto_step_idx} graph (§8.4).  Conservative: the implicit next-index
    edge is always included, even when a step's own on[] might cover
    every tick in practice — over-requiring max_runtime occasionally is
    the safe direction; silently missing a real cycle is not."""
    WHITE, GRAY, BLACK = 0, 1, 2
    color = [WHITE] * n_steps

    def edges(i):
        out = set()
        if i + 1 < n_steps:
            out.add(i + 1)
        for t in steps[i].on:
            if t.goto_step_idx < n_steps:
                out.add(t.goto_step_idx)
        return out

    def visit(u):
        color[u] = GRAY
        for v in edges(u):
            if color[v] == GRAY:
                return True
            if color[v] == WHITE and visit(v):
                return True
        color[u] = BLACK
        return False

    return any(color[i] == WHITE and visit(i) for i in range(n_steps))


def _parse_track_filter(where: str, filt) -> Tuple[int, bool]:
    """filter = { requires = [...], energy_low = true } -> (required_caps,
    requires_energy_low).  Missing/empty matches every element."""
    if filt is None:
        return 0, False
    if not isinstance(filt, dict):
        raise ScriptError(f"{where}: 'filter' must be a table, e.g. "
                          f"filter = {{ requires = [\"sensing\"] }}")
    unknown = set(filt) - _KNOWN_TRACK_FILTER_PARAMS
    if unknown:
        raise ScriptError(f"{where}: unknown filter key(s) {sorted(unknown)} "
                          f"(known: {sorted(_KNOWN_TRACK_FILTER_PARAMS)})")

    required_caps = 0
    if "requires" in filt:
        reqs = filt["requires"]
        if not isinstance(reqs, list):
            raise ScriptError(f"{where}: filter.requires must be a list of "
                              f"capability names")
        for r in reqs:
            if r not in CAPABILITIES:
                raise ScriptError(f"{where}: unknown capability {r!r} "
                                  f"(known: {sorted(CAPABILITIES)})")
            required_caps |= CAPABILITIES[r]

    energy_low = filt.get("energy_low", False)
    if not isinstance(energy_low, bool):
        raise ScriptError(f"{where}: filter.energy_low must be true/false")

    return required_caps, energy_low


def _parse_track(index: int, table: dict) -> NormalizedTrack:
    where = f"tracks[{index}]"
    if not isinstance(table, dict):
        raise ScriptError(f"{where}: each track must be a table")
    unknown = set(table) - _KNOWN_TRACK_PARAMS
    if unknown:
        raise ScriptError(f"{where}: unexpected keys {sorted(unknown)} "
                          f"(known: {sorted(_KNOWN_TRACK_PARAMS)})")

    required_caps, energy_low = _parse_track_filter(where, table.get("filter"))

    raw_steps = table.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ScriptError(f"{where}: needs at least one "
                          f"[[tracks.steps]] entry")

    label = f"{where}.steps"
    name_to_index = _build_name_map(where, raw_steps, label=label)
    n_steps = len(raw_steps)
    steps = [_parse_step(i, s, name_to_index, n_steps, label=label)
            for i, s in enumerate(raw_steps)]

    max_runtime_ms = None
    if "max_runtime" in table:
        max_runtime_ms = parse_duration_ms(table["max_runtime"], where)

    if _has_cycle(n_steps, steps) and max_runtime_ms is None:
        raise ScriptError(
            f"{where}: this track's step transitions form a cycle (§8.4) "
            f"— a cyclic track can run indefinitely, so it must declare "
            f"its own max_runtime = \"...\" bound")

    return NormalizedTrack(required_caps=required_caps,
                           requires_energy_low=energy_low,
                           steps=steps, max_runtime_ms=max_runtime_ms)


def _derived_capability_warnings(where: str, step: NormalizedStep) -> List[str]:
    """Choreo SDK Design doc §11's derived floor, as an authoring-time
    warning rather than an error — mirrors derived_caps() in choreo.c
    (see that function's comment for exactly which axes derive which
    capability). Non-fatal: the script is not wrong, and the runtime
    enforces the floor regardless of what `requires` says. This exists so
    an author sees it here rather than discovering it as a surprise
    -EPERM on an element that turns out not to have the capability."""
    out = []
    if step.motion == "spin" and \
            not (step.required_caps & ChoreoCapabilities.LOCOMOTION):
        out.append(
            f"{where}: motion = \"spin\" requires locomotion at runtime "
            f"(Choreo SDK Design doc §11) even though 'requires' doesn't "
            f"list it — add requires = [\"locomotion\", ...] to make it "
            f"explicit")
    if step.goal in ("form", "converge") and step.frame == "absolute" and \
            not (step.required_caps & ChoreoCapabilities.ABS_POSITION):
        out.append(
            f"{where}: frame = \"absolute\" (the default) requires "
            f"abs_position at runtime (Choreo SDK Design doc §11) even "
            f"though 'requires' doesn't list it — add "
            f"requires = [\"abs_position\", ...] to make it explicit, or "
            f"opt into frame = \"collective\"/\"element\" if that's what "
            f"was intended")
    return out


def _track_shadowing_warnings(tracks: List[NormalizedTrack]) -> List[str]:
    """Choreo SDK Design doc §8.4's declaration-order shadowing check, as
    an authoring-time warning.  Track selection is first-match-wins
    (choreo.c's first_matching_track() / choreo.py's
    _first_matching_track()), so a track whose filter only ever matches
    elements an EARLIER track's filter also matches can never be selected
    by any element — its steps are dead weight the author almost
    certainly didn't intend (classic case: the catch-all declared first
    instead of last).

    Filter semantics (track_matches(), both runtimes): element matches
    (required_caps R, requires_energy_low L) iff its caps ⊇ R and
    (¬L or the element is energy-low).  Track j is subsumed by an earlier
    track i iff every (caps, energy) state matching j's filter also
    matches i's:
      - R_i ⊆ R_j   (i demands no capability j doesn't also demand), and
      - L_i ⟹ L_j  (i's energy constraint is no stricter than j's).

    Non-fatal, matching _derived_capability_warnings() above: the script
    still loads and runs — every element just resolves to the earlier
    track — and a warning (not an error) leaves room for a deliberately
    staged file where a later duplicate is kept for a planned filter
    edit.  Each shadowed track reports only its FIRST shadower, mirroring
    which track the runtime would actually pick."""
    out = []
    for j in range(1, len(tracks)):
        for i in range(j):
            subset_caps = (tracks[j].required_caps
                           & tracks[i].required_caps) == tracks[i].required_caps
            energy_ok = tracks[j].requires_energy_low \
                or not tracks[i].requires_energy_low
            if subset_caps and energy_ok:
                out.append(
                    f"tracks[{j}]: unreachable — every element this "
                    f"track's filter matches is already claimed by "
                    f"tracks[{i}]'s filter (§8.4: selection is "
                    f"first-match-wins, and tracks[{i}] requires no "
                    f"capability or energy state tracks[{j}] doesn't "
                    f"also require) — reorder the tracks or tighten "
                    f"tracks[{i}]'s filter")
                break
    return out


def _parse_departure_policy(path, doc: dict):
    """Parse the script-level departure policy from either the coarse
    `mode = "ap"|"cp"` dial or the more specific [on_departure] table —
    mutually exclusive (having both is redundant/ambiguous). Neither
    given: ("continue", ALL, 0) — byte-identical to every script written
    before this feature existed. Returns (policy, reasons_mask,
    min_participants)."""
    has_mode = "mode" in doc
    has_on_departure = "on_departure" in doc
    if has_mode and has_on_departure:
        raise ScriptError(
            f"{path}: give either 'mode' or '[on_departure]', not both — "
            f"'mode' is shorthand for a plain continue/land_in_place "
            f"[on_departure] with default reasons/min_participants")

    if has_mode:
        mode = doc["mode"]
        if mode not in ("ap", "cp"):
            raise ScriptError(f"{path}: mode must be \"ap\" (default) or "
                              f"\"cp\", got {mode!r}")
        policy = "continue" if mode == "ap" else "land_in_place"
        return policy, DEPARTURE_REASONS_ALL_MASK, 0

    if has_on_departure:
        table = doc["on_departure"]
        if not isinstance(table, dict):
            raise ScriptError(f"{path}: [on_departure] must be a table")
        unknown = set(table) - {"policy", "min_participants", "reasons"}
        if unknown:
            raise ScriptError(f"{path}: [on_departure] unexpected key(s) "
                              f"{sorted(unknown)}")
        policy = table.get("policy")
        if policy not in DEPARTURE_POLICIES:
            raise ScriptError(f"{path}: [on_departure].policy must be one "
                              f"of {sorted(DEPARTURE_POLICIES)}, got "
                              f"{policy!r}")
        min_participants = table.get("min_participants", 0)
        if (not isinstance(min_participants, int) or
                isinstance(min_participants, bool) or min_participants < 0):
            raise ScriptError(f"{path}: [on_departure].min_participants "
                              f"must be a non-negative integer")
        reasons = table.get("reasons", ["*"])
        if not isinstance(reasons, list) or not reasons:
            raise ScriptError(f"{path}: [on_departure].reasons must be a "
                              f"non-empty list of reason names, or [\"*\"]")
        if reasons == ["*"]:
            reasons_mask = DEPARTURE_REASONS_ALL_MASK
        else:
            reasons_mask = 0
            for r in reasons:
                if r not in DEPARTURE_REASONS:
                    raise ScriptError(
                        f"{path}: [on_departure].reasons: unknown reason "
                        f"{r!r} (known: {sorted(DEPARTURE_REASONS)}, or "
                        f"[\"*\"] for all)")
                reasons_mask |= 1 << DEPARTURE_REASONS[r]
        return policy, reasons_mask, min_participants

    return "continue", DEPARTURE_REASONS_ALL_MASK, 0


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
    unknown = set(doc) - {"choreo", "steps", "max_runtime", "tracks",
                          "mode", "on_departure"}
    if unknown:
        raise ScriptError(f"{path}: unexpected top-level keys "
                          f"{sorted(unknown)}")

    departure_policy, departure_reasons_mask, departure_min_participants = \
        _parse_departure_policy(path, doc)

    has_steps  = "steps" in doc
    has_tracks = "tracks" in doc
    if has_steps and has_tracks:
        raise ScriptError(f"{path}: give either [[steps]] or [[tracks]], "
                          f"not both")
    if not has_steps and not has_tracks:
        raise ScriptError(f"{path}: needs at least one [[steps]] entry "
                          f"(or [[tracks]])")

    if has_tracks:
        if "max_runtime" in doc:
            raise ScriptError(
                f"{path}: top-level max_runtime has no effect with "
                f"[[tracks]] — give each track its own max_runtime "
                f"instead (a cycle in one track doesn't bound the others)")
        raw_tracks = doc["tracks"]
        if not isinstance(raw_tracks, list) or not raw_tracks:
            raise ScriptError(f"{path}: needs at least one [[tracks]] entry")
        if len(raw_tracks) > CHOREO_MAX_TRACKS:
            raise ScriptError(f"{path}: {len(raw_tracks)} tracks — the "
                              f"runtime only holds {CHOREO_MAX_TRACKS}")
        tracks = [_parse_track(i, t) for i, t in enumerate(raw_tracks)]
        warnings = [w for j, t in enumerate(tracks)
                   for i, s in enumerate(t.steps)
                   for w in _derived_capability_warnings(
                       f"tracks[{j}].steps[{i}]", s)]
        warnings += _track_shadowing_warnings(tracks)
        return ChoreoScript(name=name, tracks=tracks, warnings=warnings,
                           departure_policy=departure_policy,
                           departure_reasons_mask=departure_reasons_mask,
                           departure_min_participants=departure_min_participants)

    raw_steps = doc.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ScriptError(f"{path}: needs at least one [[steps]] entry")

    name_to_index = _build_name_map(path, raw_steps)
    n_steps = len(raw_steps)
    steps = [_parse_step(i, s, name_to_index, n_steps)
            for i, s in enumerate(raw_steps)]

    max_runtime_ms = None
    if "max_runtime" in doc:
        max_runtime_ms = parse_duration_ms(doc["max_runtime"], path)

    if _has_cycle(n_steps, steps) and max_runtime_ms is None:
        raise ScriptError(
            f"{path}: this script's step transitions form a cycle (§8.4) "
            f"— a cyclic script can run indefinitely, so it must declare "
            f"a top-level max_runtime = \"...\" bound")

    warnings = [w for i, s in enumerate(steps)
               for w in _derived_capability_warnings(f"steps[{i}]", s)]
    return ChoreoScript(name=name, steps=steps, max_runtime_ms=max_runtime_ms,
                        warnings=warnings,
                        departure_policy=departure_policy,
                        departure_reasons_mask=departure_reasons_mask,
                        departure_min_participants=departure_min_participants)


def _normalized_to_choreo_steps(steps: List[NormalizedStep]) -> List[ChoreoStep]:
    out = []
    for s in steps:
        goal = Goal(type=GOAL_TYPES[s.goal])
        if s.target is not None:
            goal.target = s.target
        if s.radius is not None:
            goal.radius = s.radius
        if s.shape is not None:
            goal.shape = SHAPES[s.shape]
        if s.slot_shift is not None:
            goal.slot_shift = s.slot_shift
        goal.direct_path = s.direct_path
        goal.frame = _FRAME_ENUM[s.frame]
        if s.anchor_select is not None:
            goal.anchor = _ANCHOR_ENUM[s.anchor_select]
        if s.anchor_id is not None:
            goal.anchor_id = s.anchor_id
        goal.motion = _MOTION_ENUM[s.motion]
        if s.spin_rate_radps is not None:
            goal.spin_rate_radps = s.spin_rate_radps
        if s.achieve_eps is not None:
            goal.achieve_eps = s.achieve_eps
        if s.achieve_hold_ms is not None:
            goal.achieve_hold_ms = s.achieve_hold_ms
        goal.required_caps = s.required_caps
        on = [ChoreoTransition(event=_EVENT_ENUM[t.event],
                               goto_step_idx=t.goto_step_idx,
                               threshold=t.threshold)
             for t in s.on]
        out.append(ChoreoStep(goal=goal,
                              max_duration_ms=s.max_duration_ms,
                              advance_on_achieved=s.advance_on_achieved,
                              scope=s.scope,
                              on=on,
                              indicator=INDICATORS[s.indicator]
                                        if s.indicator is not None
                                        else SubstrateSignal.NONE,
                              telemetry_tag=s.telemetry_tag,
                              on_departure=DEPARTURE_POLICIES[s.on_departure]
                                          if s.on_departure is not None
                                          else None))
    return out


def to_choreo_steps(script: ChoreoScript) -> List[ChoreoStep]:
    """Convert a parsed [[steps]] script to SDK ChoreoStep objects."""
    if script.tracks is not None:
        raise ScriptError(
            f"{script.name}: this is a multi-track script ([[tracks]]) — "
            f"use load_tracks()/to_choreo_tracks(), not "
            f"load_steps()/to_choreo_steps()")
    return _normalized_to_choreo_steps(script.steps)


def to_choreo_tracks(script: ChoreoScript) -> List[ChoreoTrack]:
    """Convert a parsed [[tracks]] script to SDK ChoreoTrack objects."""
    if script.tracks is None:
        raise ScriptError(
            f"{script.name}: this is a single-track script ([[steps]]) — "
            f"use load_steps()/to_choreo_steps(), not "
            f"load_tracks()/to_choreo_tracks()")
    return [
        ChoreoTrack(
            filter=ChoreoTrackFilter(required_caps=t.required_caps,
                                     requires_energy_low=t.requires_energy_low),
            steps=_normalized_to_choreo_steps(t.steps))
        for t in script.tracks
    ]


def load_steps(path) -> List[ChoreoStep]:
    """One-call convenience: parse a [[steps]] script file into
    ChoreoStep objects."""
    return to_choreo_steps(parse_file(path))


def load_tracks(path) -> List[ChoreoTrack]:
    """One-call convenience: parse a [[tracks]] script file into
    ChoreoTrack objects."""
    return to_choreo_tracks(parse_file(path))
