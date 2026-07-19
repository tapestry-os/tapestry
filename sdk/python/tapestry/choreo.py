"""
choreo.py — Tapestry Choreographer SDK, L7 (Python)

Python mirror of sdk/include/tapestry/choreo.h +
tapestry-os/subsys/choreo/choreo.c.
Backed by the BSE stub from sdk/python/tapestry/bse.py.

NOT FOR PRODUCTION USE.
"""

import errno as _errno
from .bse import (BSE, BSEIntent, BSEIntentType,
                  BSEShape, BSEDirective, BSEDirectiveType)
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List


# ── SCR capability constants (mirrors scr.h SCR_CAP_*) ───────────────────────
_SCR_CAP_RELAY    = 0x01
_SCR_CAP_SENSOR   = 0x02
_SCR_CAP_ACTUATOR = 0x04


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
      LOCOMOTION → SCR_CAP_ACTUATOR
      SENSING    → SCR_CAP_SENSOR
      SIGNALING  → SCR_CAP_RELAY (best approximation)
      BONDING    → (no SCR equivalent; always unsatisfied)
    """
    NONE       = 0x00
    LOCOMOTION = 0x01
    BONDING    = 0x02
    SENSING    = 0x04
    SIGNALING  = 0x08


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


# ── Public data classes ───────────────────────────────────────────────────────

@dataclass
class Goal:
    """Declarative desired world state submitted by the application.

    FORM/MOVE/DISPERSE/CONVERGE reference absolute coordinates; HOLD and
    EXCHANGE reference the collective's own current configuration and carry
    no coordinates at all (see bse.py for the mechanism).
    """
    type:          GoalType
    target:        tuple = (50.0, 50.0)   # (x, y) in logical world coordinates
    radius:        float = 30.0
    shape:         GoalShape = GoalShape.CIRCLE
    required_caps: int = ChoreoCapabilities.NONE  # ChoreoCapabilities bitmask
    slot_shift:      int = 0     # EXCHANGE ring rotation (0 → 1)
    achieve_eps:     float = 0.0 # achievement radius (0 → BSE default)
    achieve_hold_ms: int = 0     # sustain time, ms (0 → BSE default)


@dataclass
class ChoreoStep:
    """One step of a linear Choreo script (the minimal Choreo container).

    A step advances when its goal is achieved (advance_on_achieved) or when
    max_duration_ms elapses (nonzero).  A step with neither would stall —
    submit_script() rejects it.  Completing the last step terminates the
    script: directive IDLE, the substrate-neutral quiescence signal (each
    platform maps it to its own inactive posture; "take off" and "land"
    never appear in the goal vocabulary).
    """
    goal:                Goal
    max_duration_ms:     int = 0
    advance_on_achieved: bool = False


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
        {'id': int, 'is_active': bool, 'is_stale': bool, 'is_self': bool}

    scr_state format — dict:
        {'role': int, 'quorum_state': int, 'leader_id': int}
        quorum_state: 0=LOST, 1=DEGRADED, 2=HEALTHY

    capabilities — SCR_CAP_* hardware bitmask for this element.  Pass None
        (default) to skip the capability check entirely, mirroring the C stub
        behavior when choreo_register_scr() has not been called.
    """

    QUORUM_LOST = 0

    WM_CYCLE_MS = 100   # step timers integrate time on this tick period

    def __init__(self, element_id: int, capabilities: Optional[int] = None):
        self._bse          = BSE(element_id)
        self._goal: Optional[Goal] = None
        self._state        = ChoreoState.IDLE
        self._capabilities = capabilities   # None ≙ no SCR registered
        self._steps: Optional[List[ChoreoStep]] = None
        self._step_idx     = 0
        self._step_ms      = 0
        self._script_done  = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def configure(self, goal: Goal) -> int:
        """
        Validate and store a goal without starting execution.

        Lifecycle transition: IDLE → CONFIGURED.
        Returns 0 on success.
        Returns -1 if the state is not IDLE, or if goal is None / GoalType.NONE.
        Returns -errno.EPERM if the element's capabilities do not satisfy
        goal.required_caps.
        """
        if goal is None or goal.type == GoalType.NONE:
            return -1
        if self._state != ChoreoState.IDLE:
            return -1
        if not self._caps_satisfied(goal.required_caps):
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

    def terminate(self) -> None:
        """
        Abort the current goal or script and return to IDLE.

        Valid from any state.  Submits an IDLE intent to the BSE (the
        quiescence signal) and clears the stored goal and any active script.
        script_complete() is unaffected — it keeps reporting whether the
        most recent script ran to completion.
        """
        self._state = ChoreoState.TERMINATED
        self._steps    = None
        self._step_idx = 0
        self._step_ms  = 0
        self._bse.submit_intent(BSEIntent())   # IDLE intent
        self._goal  = None
        self._state = ChoreoState.IDLE

    def submit_goal(self, goal: Goal) -> int:
        """
        One-shot convenience: configure + deploy.

        Calls terminate() first if a goal is already active, then calls
        configure(goal) followed by deploy().
        Returns 0 on success, -1 on invalid goal, -errno.EPERM on
        capability mismatch.
        """
        if goal is None:
            return -1
        if self._state != ChoreoState.IDLE:
            self.terminate()
        self._script_done = False
        rc = self.configure(goal)
        if rc != 0:
            return rc
        return self.deploy()

    def submit_script(self, steps: List[ChoreoStep]) -> int:
        """
        Load and start a goal sequence (mirrors choreo_submit_script).

        Terminates any active goal or script, validates every step up front
        (goal validity, capabilities, and that each step can advance), then
        deploys step 0.
        Returns 0 on success, -1 on invalid arguments or an unadvanceable
        step, -errno.EPERM on capability mismatch.
        """
        if not steps:
            return -1
        for st in steps:
            if st.goal is None or st.goal.type == GoalType.NONE:
                return -1
            if not st.advance_on_achieved and st.max_duration_ms == 0:
                return -1   # no exit condition — stalls by construction
            if not self._caps_satisfied(st.goal.required_caps):
                return -_errno.EPERM

        if self._state != ChoreoState.IDLE:
            self.terminate()
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

    def cancel_goal(self) -> None:
        """Cancel the current goal and return to IDLE."""
        self.terminate()

    def goal_status(self) -> ChoreoState:
        """Return the current lifecycle state."""
        return self._state

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
            self._bse.tick(wm_entries, scr_state)
            self._script_advance()
            if (self._state == ChoreoState.RUNNING and
                    scr_state.get('quorum_state', 2) == self.QUORUM_LOST):
                self._state = ChoreoState.SUSPENDED
        elif self._state == ChoreoState.SUSPENDED:
            if scr_state.get('quorum_state', 2) != self.QUORUM_LOST:
                self._state = ChoreoState.RUNNING

    def _script_advance(self) -> None:
        if self._steps is None:
            return
        st = self._steps[self._step_idx]
        self._step_ms += self.WM_CYCLE_MS

        advance = (st.advance_on_achieved and self._bse.goal_achieved()) or \
                  (st.max_duration_ms > 0 and self._step_ms >= st.max_duration_ms)
        if not advance:
            return

        self._step_idx += 1
        self._step_ms = 0
        if self._step_idx >= len(self._steps):
            # Script complete → quiescence: terminate submits the IDLE
            # intent; _script_done survives it.
            self._script_done = True
            self.terminate()
            return
        self._goal = self._steps[self._step_idx].goal
        self._bse.submit_intent(self._goal_to_intent(self._goal))

    def get_directive(self) -> BSEDirective:
        """Return the directive computed by the last tick."""
        return self._bse.get_directive()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _caps_satisfied(self, required: int) -> bool:
        """
        Check whether self._capabilities satisfies the required bitmask.

        Mirrors caps_satisfied() in choreo.c:
          - capabilities is None → no SCR registered → always passes.
          - required == NONE (0)  → no requirements  → always passes.
          - BONDING has no SCR mapping → always fails if required.
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
        if required & ChoreoCapabilities.BONDING:
            return False
        return True

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
            slot_shift      = goal.slot_shift,
            achieve_eps     = goal.achieve_eps,
            achieve_hold_ms = goal.achieve_hold_ms,
        )
