"""
choreo.py — Tapestry Choreographer SDK, L7 (Python)

Python mirror of sdk/include/tapestry/choreo.h +
tapestry-os/subsys/choreo/choreo.c — see those files for the v1.0 feature
scope. Backed by the BSE engine from sdk/python/tapestry/bse.py.
"""

import errno as _errno
from .bse import (BSE, BSEIntent, BSEIntentType, BSEShape, BSEDirective,
                  BSEFrame, BSEAnchorSelector, BSEMotion)
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, List


# ── SCR capability constants (mirrors scr.h SCR_CAP_*) ───────────────────────
_SCR_CAP_RELAY        = 0x01
_SCR_CAP_SENSOR       = 0x02
_SCR_CAP_ACTUATOR     = 0x04
_SCR_CAP_BONDING      = 0x08
_SCR_CAP_ABS_POSITION = 0x10


# ── Public enumerations ───────────────────────────────────────────────────────

class ChoreoState(IntEnum):
    """
    Five-stage lifecycle state (paper §3.9, analogous to Android Activity).

    IDLE        No goal loaded; SDK is quiescent.
    CONFIGURED  Goal validated and stored; BSE not yet ticking.
    RUNNING     BSE ticking; quorum is DEGRADED or HEALTHY.
    SUSPENDED   Quorum dropped to LOST while RUNNING; goal preserved.
                Resumes to RUNNING automatically when quorum recovers.
    TERMINATED  terminate() called; goal cleared.  Transitions immediately
                back to IDLE — callers will not observe this in polling loops.
    """
    IDLE       = 0
    CONFIGURED = 1
    RUNNING    = 2
    SUSPENDED  = 3
    TERMINATED = 4


class ChoreoCapabilities(IntEnum):
    """
    Application-level capability bitmask (paper §3.9).

    Mapped to L5 SCR_CAP_* hardware flags at configure() time:
      LOCOMOTION    → SCR_CAP_ACTUATOR
      SENSING       → SCR_CAP_SENSOR
      SIGNALING     → SCR_CAP_RELAY (best approximation)
      BONDING       → SCR_CAP_BONDING
      ABS_POSITION  → SCR_CAP_ABS_POSITION

    required_caps is a floor the author can raise but not lower — see
    Choreo._derived_caps(): a FORM/CONVERGE goal with frame == ABSOLUTE
    also demands ABS_POSITION, and motion == SPIN also demands LOCOMOTION,
    whether or not the author declared either.
    """
    NONE          = 0x00
    LOCOMOTION    = 0x01
    BONDING       = 0x02
    SENSING       = 0x04
    SIGNALING     = 0x08
    ABS_POSITION  = 0x10


class GoalType(IntEnum):
    NONE     = 0
    FORM     = 1
    MOVE     = 2
    DISPERSE = 3
    CONVERGE = 4
    HOLD     = 5   # stay at current station (coordinate-free)
    EXCHANGE = 6   # rotate stations among participants (coordinate-free)


class GoalShape(IntEnum):
    CIRCLE = 1
    LINE   = 2
    GRID   = 3


class SubstrateSignal(IntEnum):
    """Mirrors substrate_signal_t (substrate.h) — a semantic signal state,
    NOT a physical LED/tone/marker value (that mapping is
    implementation-defined per substrate). Reused directly for
    ChoreoStep.indicator, same as GoalShape/BSEFrame/etc. are reused
    rather than duplicated with a parallel enum. The Python SDK has no
    substrate layer of its own — current_indicator() only returns this
    value for a caller (or a telemetry capture) to act on; it never
    drives real hardware."""
    NONE     = 0
    IDLE     = 1
    ACTIVE   = 2
    DEGRADED = 3
    FAILED   = 4


class ChoreoEvent(IntEnum):
    """Choreo SDK Design doc §8.2's event vocabulary, this subset only
    (the "welcome dance" demo, §8.3's flagship for this feature, uses
    only ELEMENT_JOINED/ELEMENT_LOST).  Mirrors choreo_event_t (choreo.h).

    QUORUM_LOST fires when scr_state's quorum_state == LOST, checked
    before (and composing naturally with, not suppressing) the engine's
    own automatic RUNNING -> SUSPENDED transition — see choreo.h's
    comment for why that no longer needed its own careful design once a
    HOLD step's own timeout stopped freezing while suspended
    (_suspended_hold_timeout()). quorum_degraded/quorum_recovered remain
    absent — no concrete use case identified for either."""
    ACHIEVED       = 0
    ELEMENT_JOINED = 1
    ELEMENT_LOST   = 2
    COUNT_GTE      = 3   # threshold
    COUNT_EQ       = 4   # threshold
    ANCHOR_LOST    = 5
    QUORUM_LOST    = 6


# Bounded to match choreo.h's CHOREO_MAX_TRANSITIONS — enforced in
# submit_script() so a script validated in Python won't silently exceed
# what the C runtime's fixed-size array can hold.
CHOREO_MAX_TRANSITIONS = 4

# Bounded to match choreo.h's CHOREO_MAX_TRACKS.
CHOREO_MAX_TRACKS = 4

# Mirrors csm.h's ELEMENT_HEALTH_LOW_BATTERY bit — the wire-gossiped
# health_flags bitmask, not a locally-recomputed energy_level threshold
# (that derivation is platform-specific; see main.c's cf21bl_pm_battery_low()).
ELEMENT_HEALTH_LOW_BATTERY = 0x01


@dataclass
class ChoreoTransition:
    """One guarded transition (choreo_transition_t, choreo.h).
    goto_step_idx == len(steps) is "end" — completes the script from
    anywhere, exactly like naturally completing the last step."""
    event:         ChoreoEvent
    goto_step_idx: int
    threshold:     int = 0   # COUNT_GTE / COUNT_EQ only


class ChoreoDeparturePolicy(IntEnum):
    """How a survivor reacts when a participating peer stops
    participating (self-declared ELEMENT_HEALTH_DEPARTED, csm.h, or
    inferred LOST past WM_EXPIRE_THRESHOLD_MS).  Mirrors
    choreo_departure_policy_t (choreo.h) — see that type's doc for full
    semantics (each policy's exact behavior, RECALL's fallback to
    LAND_IN_PLACE, HOLD's timeout).

    NOT implemented by this Python Choreo class's tick() — this enum
    exists only so ChoreoStep.on_departure and script_toml.py's TOML
    parsing round-trip correctly for sdk/tools/choreoc.py's C-header
    codegen path (the one that actually flies on hardware). The
    sdk/tools/choreo_sim.py --simulate authoring tool (this class) does
    not model departure policy at all yet — a script's `mode`/
    `[on_departure]`/per-step `on_departure` are silently inert there,
    the same "not a fidelity simulator" disclaimer choreo.h already
    makes for other physics this simulator deliberately omits."""
    CONTINUE      = 0
    HOLD          = 1
    LAND_IN_PLACE = 2
    RECALL        = 3


class ChoreoScope(IntEnum):
    """Whose achievement gates an advance_on_achieved step (design doc
    §8.5). SELF (default) is this element's own achievement only. ALL is
    the collective predicate — this element's own achievement AND every
    active peer's gossiped achieved bit (see Choreo._collective_achieved).
    Eventually consistent, not a synchronization barrier; vacuously true
    only when genuinely solo."""
    SELF = 0
    ALL  = 1


# ── Public data classes ───────────────────────────────────────────────────────

@dataclass
class Goal:
    """Declarative desired world state submitted by the application.

    FORM/MOVE/DISPERSE/CONVERGE reference absolute coordinates; HOLD and
    EXCHANGE reference the collective's own current configuration and carry
    no coordinates at all (see bse.py for the mechanism).
    """
    type:          GoalType
    target:        tuple = (50.0, 50.0, 50.0)   # (x, y, z), full 3D
    radius:        float = 30.0
    shape:         GoalShape = GoalShape.CIRCLE
    required_caps: int = ChoreoCapabilities.NONE  # ChoreoCapabilities bitmask
    slot_shift:      int = 0     # EXCHANGE ring rotation (0 → 1)
    direct_path:     bool = False  # EXCHANGE beeline vs arc (see bse.py)
    achieve_eps:     float = 0.0 # achievement radius (0 → BSE default)
    achieve_hold_ms: int = 0     # sustain time, ms (0 → BSE default)
    frame:           BSEFrame = BSEFrame.ABSOLUTE   # FORM/CONVERGE only (bse.py §5)
    anchor:          BSEAnchorSelector = BSEAnchorSelector.LEADER  # frame==ELEMENT only
    anchor_id:       int = 0     # anchor==ID only
    motion:          BSEMotion = BSEMotion.STATIC   # FORM only (bse.py §6)
    spin_rate_radps: float = 0.0                    # motion==SPIN only
    id:              int = 0     # caller-assigned goal identity; 0 = anonymous.
                                 # Opaque to Tapestry — never generated or
                                 # interpreted here. Mirrors choreo_goal_t::id.
                                 # Used by parked_goal_id() to name which goal
                                 # preempt_goal() displaced.


@dataclass
class ChoreoStep:
    """One step of a linear Choreo script (the minimal Choreo container).

    A step advances on the FIRST matching declared transition (`on`,
    checked in order — §8.2/§8.3), or, absent any match, when its goal is
    achieved (advance_on_achieved) or max_duration_ms elapses (nonzero) —
    the rule every step had before transitions existed.  A step with none
    of these would stall — submit_script() rejects it.  Completing the
    last step (or a transition's goto_step_idx == len(steps), "end")
    terminates the script: directive IDLE, the substrate-neutral
    quiescence signal (each platform maps it to its own inactive posture;
    "take off" and "land" never appear in the goal vocabulary).
    """
    goal:                Goal
    max_duration_ms:     int = 0
    advance_on_achieved: bool = False
    scope:               int = ChoreoScope.SELF   # whose achievement counts
    on: List[ChoreoTransition] = field(default_factory=list)

    # §12 Stage 5 effect annotations — mirrors choreo_step_t's indicator/
    # telemetry_tag (choreo.h). Both default to "no effect", byte-identical
    # to every ChoreoStep written before this feature existed.
    indicator:     SubstrateSignal = SubstrateSignal.NONE
    telemetry_tag: Optional[str]   = None

    # Per-step departure-policy override — mirrors choreo_step_t's
    # on_departure/on_departure_set (choreo.h) as a single Optional field:
    # None means "inherit the script default" (on_departure_set=false in
    # C), byte-identical to every ChoreoStep written before this feature
    # existed. NOT acted on by this Python class's tick() — see
    # ChoreoDeparturePolicy's doc.
    on_departure: Optional[ChoreoDeparturePolicy] = None


@dataclass
class ChoreoTrackFilter:
    """Which elements belong to a track, evaluated by each element
    against its OWN state only (never a peer's) — mirrors
    choreo_track_filter_t (choreo.h §7).  The zero value (both fields
    false/0) matches every element — the "all" default a script with no
    [[tracks]] uses."""
    required_caps:       int  = ChoreoCapabilities.NONE
    requires_energy_low: bool = False


@dataclass
class ChoreoTrack:
    """One participant-scoped step sequence (choreo_track_t, choreo.h
    §7).  An element runs exactly one track at a time — filters are
    evaluated in declaration order, first match wins."""
    filter: ChoreoTrackFilter
    steps:  List[ChoreoStep]


# ── Choreo ────────────────────────────────────────────────────────────────────

class Choreo:
    """
    Tapestry Choreographer SDK entry point (Python).

    One instance per simulated element.

    Usage (explicit lifecycle):
        choreo = Choreo(element_id=0, capabilities=_SCR_CAP_ACTUATOR)
        rc = choreo.configure(Goal(type=GoalType.FORM,
                                   required_caps=ChoreoCapabilities.LOCOMOTION))
        choreo.deploy()

        # each simulation cycle:
        choreo.tick(wm_entries, scr_state)
        d = choreo.get_directive()   # BSEDirective

    Usage (one-shot convenience):
        choreo = Choreo(element_id=0)
        choreo.submit_goal(Goal(type=GoalType.FORM, radius=30.0))

    wm_entries format — list of dicts:
        {'id': int, 'is_active': bool, 'is_stale': bool, 'is_self': bool,
         'achieved': bool}   # peer's gossiped own-goal achievement (scope=all);
                             # optional, defaults to False if absent

    scr_state format — dict:
        {'role': int, 'quorum_state': int, 'leader_id': int}
        quorum_state: 0=LOST, 1=DEGRADED, 2=HEALTHY

    capabilities — SCR_CAP_* hardware bitmask for this element.  Pass None
        (default) to skip the capability check entirely, mirroring the C
        behavior when choreo_register_scr() has not been called.
    """

    QUORUM_LOST = 0

    WM_CYCLE_MS = 100   # step timers integrate time on this tick period

    # Membership debounce hold time (CHOREO_EVENT_ELEMENT_JOINED/LOST/
    # COUNT_*) — mirrors CHOREO_MEMBERSHIP_HOLD_MS (choreo.c).
    MEMBERSHIP_HOLD_MS = 2000

    # Preemption stack depth (preempt_goal()) — mirrors
    # CHOREO_MAX_PREEMPT_DEPTH (choreo.c).
    MAX_PREEMPT_DEPTH = 1

    def __init__(self, element_id: int, capabilities: Optional[int] = None):
        self._bse          = BSE(element_id)
        self._goal: Optional[Goal] = None
        self._state        = ChoreoState.IDLE
        self._capabilities = capabilities   # None ≙ no SCR registered
        self._steps: Optional[List[ChoreoStep]] = None
        self._step_idx     = 0
        self._step_ms      = 0
        self._script_done  = False
        # One debounce timer per Choreo instance, continuous across the
        # whole script's lifetime (not reset per-step) — see
        # _update_membership_debounce().
        self._count_locked: Optional[int] = None
        self._count_candidate: Optional[int] = None
        self._count_candidate_ms = 0
        # Tracks (choreo.h §7).  s_steps/s_step_idx/s_step_ms/s_goal above
        # are reused AS-IS to mean "the ACTIVE track's step state" — an
        # element runs one track at a time.  _tracks is None outside
        # multi-track mode.
        self._tracks: Optional[List[ChoreoTrack]] = None
        self._n_tracks = 0
        self._active_track_idx = 0
        self._track_step_idx: List[int] = [0] * CHOREO_MAX_TRACKS
        # Debounced track-selection state — same shape as the membership
        # debounce above, but the very first determination (inside
        # submit_tracks()) is immediate/undebounced; only subsequent
        # re-evaluations from tick() go through this candidate/hold-timer.
        self._track_candidate_idx: Optional[int] = None
        self._track_candidate_ms = 0
        # Preemption stack: (goal, steps, step_idx, step_ms) tuples, mirrors
        # choreo_parked_goal_t (choreo.c). n_steps/script_active aren't
        # needed separately — len(steps) and "steps is not None" cover them,
        # same encoding _steps already uses outside preemption.
        self._parked: List[tuple] = []

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def configure(self, goal: Goal) -> int:
        """
        Validate and store a goal without starting execution.

        Lifecycle transition: IDLE → CONFIGURED.
        Returns 0 on success.
        Returns -1 if the state is not IDLE, or if goal is None / GoalType.NONE.
        Returns -errno.EPERM if the element's capabilities do not satisfy
        goal.required_caps (a derived floor — see _derived_caps()).
        """
        if goal is None or goal.type == GoalType.NONE:
            return -1
        if self._state != ChoreoState.IDLE:
            return -1
        if not self._caps_satisfied(self._derived_caps(goal)):
            return -_errno.EPERM
        self._goal  = goal
        self._state = ChoreoState.CONFIGURED
        return 0

    def deploy(self) -> int:
        """
        Begin executing the configured goal.

        Lifecycle transition: CONFIGURED → RUNNING.
        Returns 0 on success, -1 if not in CONFIGURED state.
        """
        if self._state != ChoreoState.CONFIGURED:
            return -1
        self._bse.submit_intent(self._goal_to_intent(self._goal))
        self._state = ChoreoState.RUNNING
        return 0

    def _terminate_hard(self) -> None:
        """
        Unconditional full reset — drops any parked goal too.

        Used for the pre-submit reset in submit_goal()/submit_script()/
        submit_tracks(): an ordinary new submission always fully replaces
        everything, whether or not a preemption happens to be active,
        exactly as before preempt_goal() existed. terminate() (below) is
        the pop-aware public path. Mirrors terminate_hard() (choreo.c).
        """
        self._state = ChoreoState.TERMINATED
        self._steps    = None
        self._step_idx = 0
        self._step_ms  = 0
        self._parked.clear()
        self._count_locked = None   # a fresh submission starts fresh
        self._tracks   = None       # drop multi-track mode entirely
        self._n_tracks = 0
        self._bse.set_track_scope(0)   # bse's filter must not stay stuck nonzero
        self._bse.submit_intent(BSEIntent())   # IDLE intent; also drops BSE's parked stack
        self._goal  = None
        self._state = ChoreoState.IDLE

    def terminate(self) -> None:
        """
        Abort the current goal or script.

        If preempt_goal() has a goal parked, this RESUMES it instead of
        going to IDLE: the parked goal (and, if it was a script, its exact
        step/timer position) becomes active again exactly as it was at the
        moment it was preempted, and this returns to RUNNING — repeated
        calls unwind one preemption level at a time. With nothing parked
        (the only case before preempt_goal() existed, and the common case
        today), behavior is unchanged: submits an IDLE intent to the BSE
        (the quiescence signal), clears any active script, and settles in
        IDLE. script_complete() is unaffected either way — it keeps
        reporting whether the most recent script ran to completion.
        Mirrors choreo_terminate() (choreo.c).
        """
        if self._parked:
            self._goal, self._steps, self._step_idx, self._step_ms = \
                self._parked.pop()
            self._bse.resume_intent()
            self._state = ChoreoState.RUNNING
            return
        self._terminate_hard()

    def preempt_goal(self, goal: Goal) -> int:
        """
        Run `goal` immediately, preserving the currently active goal (or
        script, including its exact step/timer position) to resume
        automatically later — via terminate()/cancel_goal() once `goal` is
        done, or naturally when the current step's/track's advance calls
        terminate() on natural completion.

        Unlike submit_goal(), this does NOT discard what was running: it
        is the mechanism side of a goal queue with preemption (v1.0 scope:
        one level deep — a second preempt_goal() call while one is already
        parked returns -errno.EBUSY).

        Requires an active goal (RUNNING or SUSPENDED) to preempt — returns
        -1 from IDLE/CONFIGURED/TERMINATED, since there is nothing to
        preserve.

        Mirrors choreo_preempt_goal() (choreo.c). Returns 0 on success, -1
        if goal is None, GoalType.NONE, or nothing is active to preempt;
        -errno.EBUSY if something is already parked; -errno.EPERM on
        capability mismatch.
        """
        if goal is None or goal.type == GoalType.NONE:
            return -1
        if self._state not in (ChoreoState.RUNNING, ChoreoState.SUSPENDED):
            return -1   # nothing active to preempt
        if len(self._parked) >= self.MAX_PREEMPT_DEPTH:
            return -_errno.EBUSY
        if not self._caps_satisfied(self._derived_caps(goal)):
            return -_errno.EPERM

        rc = self._bse.preempt_intent(self._goal_to_intent(goal))
        if rc != 0:
            return rc   # BSE-side stack full — nothing was pushed here yet

        self._parked.append((self._goal, self._steps, self._step_idx,
                             self._step_ms))
        self._goal  = goal
        self._steps = None   # a preempting goal is a single goal, not a script
        self._state = ChoreoState.RUNNING
        return 0

    def is_preempted(self) -> bool:
        """True if preempt_goal() has a goal parked that terminate()/
        cancel_goal() would resume. Mirrors choreo_is_preempted()."""
        return len(self._parked) > 0

    def parked_goal_id(self) -> int:
        """The Goal.id of the parked goal (see that field's doc for why it
        exists: naming which goal to resume, and reporting which goal
        preempted which). 0 if nothing is parked, or if the parked goal
        never set an id. Mirrors choreo_parked_goal_id()."""
        return self._parked[-1][0].id if self._parked else 0

    def submit_goal(self, goal: Goal) -> int:
        """
        One-shot convenience: configure + deploy.

        Unconditionally fully resets state first if anything is active —
        including discarding any goal parked by preempt_goal(), unlike
        terminate() — then calls configure(goal) followed by deploy(). An
        ordinary new submission always replaces everything; use
        preempt_goal() when the previous goal should survive.
        Returns 0 on success, -1 on invalid goal, -errno.EPERM on
        capability mismatch.
        """
        if goal is None:
            return -1
        if self._state != ChoreoState.IDLE:
            self._terminate_hard()
        self._script_done = False
        rc = self.configure(goal)
        if rc != 0:
            return rc
        return self.deploy()

    def _validate_steps(self, steps: List[ChoreoStep]) -> int:
        """Validate every step up front — a script (or, per-track, a
        track) that would stall or fail a capability check mid-show is
        rejected before anything moves.  Shared by submit_script() and
        submit_tracks() so the two validation paths can't drift (mirrors
        validate_steps() in choreo.c)."""
        if not steps:
            return -1
        for st in steps:
            if st.goal is None or st.goal.type == GoalType.NONE:
                return -1
            if not st.advance_on_achieved and st.max_duration_ms == 0:
                return -1   # no exit condition — stalls by construction
            if st.goal.motion == BSEMotion.SPIN and st.max_duration_ms == 0:
                # A non-terminal motion never "completes" (bse.py §6) —
                # until=achieved alone is not a sufficient exit, only a
                # valid early-advance on top of a real duration bound.
                return -1
            if not self._caps_satisfied(self._derived_caps(st.goal)):
                return -_errno.EPERM
            if len(st.on) > CHOREO_MAX_TRANSITIONS:
                return -1
            for t in st.on:
                # goto_step_idx == len(steps) is "end" (ChoreoTransition) —
                # valid.  Anything past that would index off the list in
                # _script_advance().
                if t.goto_step_idx > len(steps):
                    return -1
        return 0

    def submit_script(self, steps: List[ChoreoStep]) -> int:
        """
        Load and start a goal sequence (mirrors choreo_submit_script).

        Terminates any active goal or script, validates every step up front
        (goal validity, capabilities, and that each step can advance), then
        deploys step 0.
        Returns 0 on success, -1 on invalid arguments or an unadvanceable
        step, -errno.EPERM on capability mismatch.
        """
        rc0 = self._validate_steps(steps)
        if rc0 != 0:
            return rc0

        if self._state != ChoreoState.IDLE:
            self._terminate_hard()
        self._script_done = False

        rc = self.configure(steps[0].goal)
        if rc != 0:
            return rc
        rc = self.deploy()
        if rc != 0:
            return rc

        self._steps    = list(steps)
        self._step_idx = 0
        self._step_ms  = 0
        return 0

    def script_step(self) -> int:
        """Current step index, or -1 if no script is active."""
        return self._step_idx if self._steps is not None else -1

    def script_complete(self) -> bool:
        """True once the most recent script ran all its steps to completion.

        The application's cue to map the IDLE directive to platform
        quiescence.  Reset by the next submit_script / submit_goal.
        """
        return self._script_done

    def goal_achieved(self) -> bool:
        """L6 achievement predicate for the currently executing goal."""
        return self._bse.goal_achieved()

    def collective_achieved(self, wm_entries: List[dict]) -> bool:
        """The scope=ALL achievement predicate — see ChoreoScope and
        _collective_achieved(). Mirrors choreo_collective_achieved() in
        choreo.h/choreo.c."""
        return self._collective_achieved(wm_entries)

    def cancel_goal(self) -> None:
        """Cancel the current goal. Thin wrapper around terminate() — see
        that function for the "resumes a parked goal instead of going to
        IDLE, if one exists" behavior."""
        self.terminate()

    def goal_status(self) -> ChoreoState:
        """Return the current lifecycle state."""
        return self._state

    def current_goal_type(self) -> GoalType:
        """The goal currently executing (RUNNING or SUSPENDED), else NONE.

        Lets the platform layer apply per-goal quorum semantics: a HOLD
        directive may be tracked even with quorum lost (it references only
        this element), while peer-referential directives should be frozen.
        """
        if self._state in (ChoreoState.RUNNING, ChoreoState.SUSPENDED) \
                and self._goal is not None:
            return self._goal.type
        return GoalType.NONE

    # ── Tracks (choreo.h §7) ─────────────────────────────────────────────────

    def _track_matches(self, filt: ChoreoTrackFilter,
                       wm_entries: List[dict]) -> bool:
        """Does `filt` match THIS element's own state?  Never inspects a
        peer — track membership is always self-evaluated (mirrors
        track_matches() in choreo.c)."""
        if not self._caps_satisfied(filt.required_caps):
            return False
        if filt.requires_energy_low:
            low = False
            for e in wm_entries:
                if e.get('is_self', False):
                    low = (e.get('health_flags', 0) & ELEMENT_HEALTH_LOW_BATTERY) != 0
                    break
            if not low:
                return False
        return True

    def _first_matching_track(self, wm_entries: List[dict]) -> Optional[int]:
        """First declared track whose filter matches, or None if none do
        (no catch-all "all" track declared) — mirrors
        first_matching_track() in choreo.c."""
        for i, tr in enumerate(self._tracks):
            if self._track_matches(tr.filter, wm_entries):
                return i
        return None

    def _migrate_to_track(self, new_track: int) -> None:
        """Switch the active track to `new_track`, saving the outgoing
        track's step index and resuming the incoming one's — then
        activate its current step FRESH (new snapshot/timers), not a
        preserved-state resume."""
        self._track_step_idx[self._active_track_idx] = self._step_idx

        self._active_track_idx = new_track
        self._steps    = self._tracks[new_track].steps
        self._step_idx = self._track_step_idx[new_track]
        self._step_ms  = 0

        self._goal = self._steps[self._step_idx].goal
        self._bse.submit_intent(self._goal_to_intent(self._goal))

    def _update_track_selection(self, wm_entries: List[dict]) -> None:
        """Re-evaluate track membership once per RUNNING tick while in
        multi-track mode.  Debounced exactly like
        _update_membership_debounce() — a threshold reading (energy_low)
        can jitter tick-to-tick same as a swarm-size count can."""
        found = self._first_matching_track(wm_entries)
        if found is None:
            # No track currently claims this element — stay on the
            # active track; there's nowhere well-defined to go.
            self._track_candidate_idx = None
            return

        if found == self._active_track_idx:
            self._track_candidate_idx = None
            return
        if self._track_candidate_idx == found:
            self._track_candidate_ms += self.WM_CYCLE_MS
        else:
            self._track_candidate_idx = found
            self._track_candidate_ms  = 0
        if self._track_candidate_ms >= self.MEMBERSHIP_HOLD_MS:
            self._migrate_to_track(found)
            self._track_candidate_idx = None

    def submit_tracks(self, wm_entries: List[dict],
                      tracks: List[ChoreoTrack]) -> int:
        """
        Load and start a multi-track Choreo (mirrors choreo_submit_tracks).

        Validates every track's every step exactly as submit_script() does,
        then determines this element's initial track (first matching
        filter, undebounced, evaluated against wm_entries) and deploys its
        step 0.

        Returns 0 on success, -1 on invalid arguments, an unadvanceable
        step, an out-of-range transition target, too many tracks, or if no
        track's filter matches this element (no catch-all "all" track
        declared); -errno.EPERM on capability mismatch.
        """
        if not tracks or len(tracks) > CHOREO_MAX_TRACKS:
            return -1
        for tr in tracks:
            rc0 = self._validate_steps(tr.steps)
            if rc0 != 0:
                return rc0

        if self._state != ChoreoState.IDLE:
            self._terminate_hard()
        self._script_done = False

        self._tracks             = list(tracks)
        self._n_tracks           = len(tracks)
        self._track_step_idx     = [0] * CHOREO_MAX_TRACKS
        self._track_candidate_idx = None

        found = self._first_matching_track(wm_entries)
        if found is None:
            self._tracks   = None
            self._n_tracks = 0
            return -1   # no track's filter matches this element
        self._active_track_idx = found
        self._steps             = self._tracks[found].steps

        rc = self.configure(self._steps[0].goal)
        if rc != 0:
            self._tracks   = None
            self._n_tracks = 0
            return rc
        rc = self.deploy()
        if rc != 0:
            self._tracks   = None
            self._n_tracks = 0
            return rc

        self._step_idx = 0
        self._step_ms  = 0
        return 0

    def current_track(self) -> int:
        """This element's active track index (0 if not currently in
        multi-track mode, or on track 0).  The application must gossip
        this on element_state_t's current_track before each gossip send —
        it is what lets OTHER elements' bse._participants() filter to
        peers on the SAME track; see wire.h's v4 comment."""
        return self._active_track_idx if self._n_tracks > 0 else 0

    # ── Per-cycle ─────────────────────────────────────────────────────────────

    def tick(self, wm_entries: List[dict], scr_state: dict) -> None:
        """
        Drive L6 decomposition for this cycle (WM_CYCLE_MS period).

        Only drives the BSE in RUNNING state.  Transitions RUNNING →
        SUSPENDED on quorum loss — freezing the BSE and any script timers,
        so a partition pauses the show rather than timing it out — and back
        on recovery.  Advances the active script per the ChoreoStep rules.
        """
        if self._state == ChoreoState.RUNNING:
            if self._n_tracks > 0:
                self._update_track_selection(wm_entries)
            self._bse.set_track_scope(self.current_track())
            self._bse.tick(wm_entries, scr_state)
            self._script_advance(wm_entries, scr_state)
            if (self._state == ChoreoState.RUNNING and
                    scr_state.get('quorum_state', 2) == self.QUORUM_LOST):
                self._state = ChoreoState.SUSPENDED
        elif self._state == ChoreoState.SUSPENDED:
            # Per-goal quorum: a SELF-referential goal (HOLD) still ticks
            # the BSE while suspended — station capture and station-keeping
            # need no peers, and deferring the capture to quorum recovery
            # would capture a drifted position.  Peer-referential goals
            # (EXCHANGE) stay frozen.  Script timers stay frozen too,
            # except a HOLD step's own max_duration_ms
            # (_suspended_hold_timeout()) — permanent isolation needs a
            # way out even for a step that never freezes unsafely.
            if self._goal is not None and self._goal.type == GoalType.HOLD:
                self._bse.tick(wm_entries, scr_state)
                self._suspended_hold_timeout()
            # _suspended_hold_timeout() may have terminated -> IDLE, or
            # advanced to a new step (still SUSPENDED); the recovery
            # check only applies if still actually SUSPENDED.
            if self._state == ChoreoState.SUSPENDED and \
                    scr_state.get('quorum_state', 2) != self.QUORUM_LOST:
                self._state = ChoreoState.RUNNING

    def _collective_achieved(self, wm_entries: List[dict]) -> bool:
        """scope=ALL predicate: own achievement AND every ACTIVE, non-self
        peer's gossiped 'achieved' key (default False if absent).  Vacuously
        true only when genuinely solo — mirrors choreo_collective_achieved
        in choreo.c. Eventually consistent, not a synchronization barrier.

        Active, not fresh: a merely stale peer still votes, from its
        last-received bit.  Skipping stale peers let scope=ALL degrade
        silently into per-element achievement under packet loss — see the
        C implementation's comment for the full mechanism."""
        if not self._bse.goal_achieved():
            return False
        for e in wm_entries:
            if e.get('is_self', False):
                continue
            if not e.get('is_active'):
                continue
            if not e.get('achieved', False):
                return False
        return True

    def _swarm_size(self, wm_entries: List[dict]) -> int:
        """Self + fresh active peer count — Python mirror of
        scr_get_swarm_size() for the membership events below (no BFT
        filtering in the Python SDK; matches bse.py's _participants())."""
        count = 0
        for e in wm_entries:
            if e.get('is_self', False):
                count += 1
            elif e.get('is_active') and not e.get('is_stale'):
                count += 1
        return count

    def _update_membership_debounce(self, wm_entries: List[dict]):
        """Update the membership debounce timer from this tick's swarm
        size and report one-tick (joined, lost) pulses.  See the
        MEMBERSHIP_HOLD_MS comment (__init__) — ELEMENT_JOINED/
        ELEMENT_LOST fire only on the tick a change is CONFIRMED stable;
        COUNT_GTE/COUNT_EQ read the raw live count instead (a threshold
        comparison has no "which direction changed" ambiguity to debounce)."""
        raw = self._swarm_size(wm_entries)
        if self._count_locked is None:
            self._count_locked = raw
            self._count_candidate_ms = 0
            return False, False
        if raw == self._count_locked:
            self._count_candidate_ms = 0   # stable — no pending change
            return False, False
        if raw == self._count_candidate:
            self._count_candidate_ms += self.WM_CYCLE_MS
        else:
            self._count_candidate    = raw
            self._count_candidate_ms = 0
        if self._count_candidate_ms >= self.MEMBERSHIP_HOLD_MS:
            joined = raw > self._count_locked
            lost   = raw < self._count_locked
            self._count_locked = raw
            return joined, lost
        return False, False

    def _event_fires(self, t: ChoreoTransition, st: ChoreoStep,
                     wm_entries: List[dict], scr_state: dict,
                     joined: bool, lost: bool) -> bool:
        if t.event == ChoreoEvent.ACHIEVED:
            return (self._collective_achieved(wm_entries)
                   if st.scope == ChoreoScope.ALL
                   else self._bse.goal_achieved())
        if t.event == ChoreoEvent.ELEMENT_JOINED:
            return joined
        if t.event == ChoreoEvent.ELEMENT_LOST:
            return lost
        if t.event == ChoreoEvent.COUNT_GTE:
            return self._swarm_size(wm_entries) >= t.threshold
        if t.event == ChoreoEvent.COUNT_EQ:
            return self._swarm_size(wm_entries) == t.threshold
        if t.event == ChoreoEvent.ANCHOR_LOST:
            return self._bse.anchor_lost()
        if t.event == ChoreoEvent.QUORUM_LOST:
            return scr_state.get('quorum_state', 2) == self.QUORUM_LOST
        return False

    def _advance_to(self, target_idx: int) -> None:
        """Shared tail of an advance: activate step target_idx, or
        complete the script if target_idx has run off the end.  Mirrors
        advance_to() in choreo.c — factored out so
        _suspended_hold_timeout() below can reach it too."""
        self._step_ms = 0

        if target_idx >= len(self._steps):
            self._script_done = True
            self.terminate()
            return

        self._step_idx = target_idx
        self._goal = self._steps[self._step_idx].goal
        self._bse.submit_intent(self._goal_to_intent(self._goal))

    def _suspended_hold_timeout(self) -> None:
        """Isolated (SUSPENDED) timeout carve-out — HOLD only.  Mirrors
        suspended_hold_timeout() in choreo.c: a HOLD step's own
        max_duration_ms keeps counting down while suspended, so a script
        can give up on permanent isolation instead of station-keeping
        forever.  Deliberately narrow — no on[] transitions, no
        advance_on_achieved (HOLD's achievement is unconditionally true,
        so combining it with SUSPENDED would fire on the first isolated
        tick)."""
        if self._steps is None or self._goal is None or \
                self._goal.type != GoalType.HOLD:
            return
        st = self._steps[self._step_idx]
        self._step_ms += self.WM_CYCLE_MS
        if st.max_duration_ms > 0 and self._step_ms >= st.max_duration_ms:
            self._advance_to(self._step_idx + 1)

    def _script_advance(self, wm_entries: List[dict], scr_state: dict) -> None:
        if self._steps is None:
            return
        st = self._steps[self._step_idx]
        self._step_ms += self.WM_CYCLE_MS

        joined, lost = self._update_membership_debounce(wm_entries)

        target_idx: Optional[int] = None
        for t in st.on:
            if self._event_fires(t, st, wm_entries, scr_state, joined, lost):
                target_idx = t.goto_step_idx
                break   # first match wins

        if target_idx is None:
            if st.advance_on_achieved:
                achieved = (self._collective_achieved(wm_entries)
                           if st.scope == ChoreoScope.ALL
                           else self._bse.goal_achieved())
            else:
                achieved = False
            advance = achieved or \
                      (st.max_duration_ms > 0 and self._step_ms >= st.max_duration_ms)
            if not advance:
                return
            target_idx = self._step_idx + 1

        self._advance_to(target_idx)

    def get_directive(self) -> BSEDirective:
        """Return the directive computed by the last tick."""
        return self._bse.get_directive()

    def current_indicator(self) -> SubstrateSignal:
        """The active step's declared indicator effect (§12 Stage 5), or
        SubstrateSignal.NONE if the current step declared none, or if no
        script is active. Mirrors choreo_current_indicator()."""
        if self._steps is None:
            return SubstrateSignal.NONE
        return self._steps[self._step_idx].indicator

    def current_telemetry_tag(self) -> Optional[str]:
        """The active step's declared telemetry tag (§12 Stage 5), or None
        under the same conditions current_indicator() returns NONE.
        Mirrors choreo_current_telemetry_tag()."""
        if self._steps is None:
            return None
        return self._steps[self._step_idx].telemetry_tag

    # ── Internal ──────────────────────────────────────────────────────────────

    def _caps_satisfied(self, required: int) -> bool:
        """
        Check whether self._capabilities satisfies the required bitmask.

        Mirrors caps_satisfied() in choreo.c:
          - capabilities is None → no SCR registered → always passes.
          - required == NONE (0)  → no requirements  → always passes.
        """
        if self._capabilities is None or not required:
            return True
        hw = self._capabilities
        if (required & ChoreoCapabilities.LOCOMOTION) and not (hw & _SCR_CAP_ACTUATOR):
            return False
        if (required & ChoreoCapabilities.SENSING)    and not (hw & _SCR_CAP_SENSOR):
            return False
        if (required & ChoreoCapabilities.SIGNALING)  and not (hw & _SCR_CAP_RELAY):
            return False
        if (required & ChoreoCapabilities.BONDING)    and not (hw & _SCR_CAP_BONDING):
            return False
        if (required & ChoreoCapabilities.ABS_POSITION) and not (hw & _SCR_CAP_ABS_POSITION):
            return False
        return True

    @staticmethod
    def _derived_caps(goal: Goal) -> int:
        """
        Choreo SDK Design doc §11: capability requirements are a derived
        floor, not solely the author's explicit required_caps — an axis
        value that demands a capability requires it whether or not the
        author declared it.  Mirrors derived_caps() in choreo.c:
          - motion == SPIN -> LOCOMOTION.
          - (type in (FORM, CONVERGE)) and frame == ABSOLUTE -> ABS_POSITION
            — only these two goal types read frame at all (bse.py §5).
        """
        caps = goal.required_caps
        if goal.motion == BSEMotion.SPIN:
            caps |= ChoreoCapabilities.LOCOMOTION
        if goal.type in (GoalType.FORM, GoalType.CONVERGE) and \
                goal.frame == BSEFrame.ABSOLUTE:
            caps |= ChoreoCapabilities.ABS_POSITION
        return caps

    @staticmethod
    def _goal_to_intent(goal: Goal) -> BSEIntent:
        _type_map = {
            GoalType.FORM:     BSEIntentType.FORM,
            GoalType.MOVE:     BSEIntentType.MOVE,
            GoalType.DISPERSE: BSEIntentType.DISPERSE,
            GoalType.CONVERGE: BSEIntentType.CONVERGE,
            GoalType.HOLD:     BSEIntentType.HOLD,
            GoalType.EXCHANGE: BSEIntentType.EXCHANGE,
        }
        return BSEIntent(
            type   = _type_map.get(goal.type, BSEIntentType.IDLE),
            target = goal.target,
            radius = goal.radius,
            shape  = BSEShape(goal.shape),
            frame     = goal.frame,
            anchor    = goal.anchor,
            anchor_id = goal.anchor_id,
            motion          = goal.motion,
            spin_rate_radps = goal.spin_rate_radps,
            slot_shift      = goal.slot_shift,
            direct_path     = goal.direct_path,
            achieve_eps     = goal.achieve_eps,
            achieve_hold_ms = goal.achieve_hold_ms,
            id              = goal.id,   # opaque; see Goal.id
        )
