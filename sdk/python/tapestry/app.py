"""
app.py — Tapestry Application SDK, L7 (Python)

Python mirror of sdk/include/tapestry/app.h + sdk/src/app_stub.c.
Backed by the BSE stub from tapestry-bse-sim/bse_stub.py.

NOT FOR PRODUCTION USE.
"""

import os
import sys

# Resolve tapestry-bse-sim relative to this file:
#   sdk/python/tapestry/app.py → up 3 → repo root → tapestry-bse-sim/
_REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(_REPO_ROOT, 'tapestry-bse-sim'))

from bse_stub import (BSEStub, BSEIntent, BSEIntentType,
                      BSEShape, BSEDirective, BSEDirectiveType)
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List


# ── Public enumerations ───────────────────────────────────────────────────────

class GoalType(IntEnum):
    NONE     = 0
    FORM     = 1
    MOVE     = 2
    DISPERSE = 3
    CONVERGE = 4


class GoalShape(IntEnum):
    CIRCLE = 1
    LINE   = 2
    GRID   = 3


class GoalStatus(IntEnum):
    IDLE     = 0
    ACTIVE   = 1
    ACHIEVED = 2   # not set by stub; requires BSE feedback controller
    FAILED   = 3


# ── Public data classes ───────────────────────────────────────────────────────

@dataclass
class Goal:
    """Declarative desired world state submitted by the application."""
    type:   GoalType
    target: tuple = (50.0, 50.0)   # (x, y) in logical world coordinates
    radius: float = 30.0
    shape:  GoalShape = GoalShape.CIRCLE


# ── TapestryApp ───────────────────────────────────────────────────────────────

class TapestryApp:
    """
    Tapestry application SDK entry point (Python).

    One instance per simulated element.

    Usage:
        app = TapestryApp(element_id=0)
        app.submit_goal(Goal(type=GoalType.FORM, radius=30.0))

        # each simulation cycle:
        app.tick(wm_entries, scr_state)
        d = app.get_directive()   # BSEDirective

    wm_entries format — list of dicts:
        {'id': int, 'is_active': bool, 'is_stale': bool, 'is_self': bool}

    scr_state format — dict:
        {'role': int, 'quorum_state': int, 'leader_id': int}
        quorum_state: 0=LOST, 1=DEGRADED, 2=HEALTHY
    """

    QUORUM_LOST = 0

    def __init__(self, element_id: int):
        self._bse       = BSEStub(element_id)
        self._goal: Optional[Goal] = None
        self._status    = GoalStatus.IDLE

    # ── Goal management ───────────────────────────────────────────────────────

    def submit_goal(self, goal: Goal) -> None:
        self._goal   = goal
        self._status = GoalStatus.ACTIVE
        self._bse.submit_intent(self._goal_to_intent(goal))

    def cancel_goal(self) -> None:
        self._goal   = None
        self._status = GoalStatus.IDLE
        self._bse.submit_intent(BSEIntent())   # IDLE

    def goal_status(self) -> GoalStatus:
        return self._status

    # ── Per-cycle ─────────────────────────────────────────────────────────────

    def tick(self, wm_entries: List[dict], scr_state: dict) -> None:
        self._bse.tick(wm_entries, scr_state)
        if self._goal is not None and self._status == GoalStatus.ACTIVE:
            if scr_state.get('quorum_state', 2) == self.QUORUM_LOST:
                self._status = GoalStatus.FAILED

    def get_directive(self) -> BSEDirective:
        return self._bse.get_directive()

    # ── Internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _goal_to_intent(goal: Goal) -> BSEIntent:
        _type_map = {
            GoalType.FORM:     BSEIntentType.FORM,
            GoalType.MOVE:     BSEIntentType.MOVE,
            GoalType.DISPERSE: BSEIntentType.DISPERSE,
            GoalType.CONVERGE: BSEIntentType.CONVERGE,
        }
        return BSEIntent(
            type   = _type_map.get(goal.type, BSEIntentType.IDLE),
            target = goal.target,
            radius = goal.radius,
            shape  = BSEShape(goal.shape),
        )
