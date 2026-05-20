"""
bse.py — Tapestry L6 Behavior Synthesis Engine stub (Python)

NOT FOR PRODUCTION USE.

Python mirror of tapestry-os/subsys/bse/bse.c for simulation and
research.  Implements the intent-parser tier of the BSE interface only.
The physics-aware planner, ML inference runtime, simulation bridge, and
feedback controller are absent (commercial BSE).

Intent → directive mapping
--------------------------
  IDLE      → IDLE
  FORM      → MOVE_TO_POINT  — vertex of regular N-gon, slot by element_id rank
  MOVE      → MOVE_TO_POINT  — all elements to same target (stub limitation)
  CONVERGE  → MOVE_TO_POINT  — all elements to target
  DISPERSE  → MAINTAIN_SPRING — spring-field with intent.radius spacing

Usage (one instance per simulated element):

    from tapestry.bse import BSE, BSEIntent, BSEIntentType, BSEShape

    bse = BSE(element_id=0)
    bse.submit_intent(BSEIntent(
        type   = BSEIntentType.FORM,
        target = (50.0, 50.0),
        radius = 30.0,
        shape  = BSEShape.CIRCLE,
    ))

    # each simulation tick:
    bse.tick(wm_entries, scr_state)
    directive = bse.get_directive()   # BSEDirective instance

See sdk/examples/hello_swarm.py for a complete worked example.
"""

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Tuple


# ── Enumerations ──────────────────────────────────────────────────────────────

class BSEIntentType(IntEnum):
    IDLE     = 0
    FORM     = 1
    MOVE     = 2
    DISPERSE = 3
    CONVERGE = 4


class BSEShape(IntEnum):
    CIRCLE = 1
    LINE   = 2
    GRID   = 3


class BSEDirectiveType(IntEnum):
    IDLE            = 0
    HOLD            = 1
    MOVE_TO_POINT   = 2
    MAINTAIN_SPRING = 3


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class BSEIntent:
    type:   BSEIntentType = BSEIntentType.IDLE
    target: Tuple[float, float] = (50.0, 50.0)
    radius: float = 30.0
    shape:  BSEShape = BSEShape.CIRCLE


@dataclass
class BSEDirective:
    type:     BSEDirectiveType = BSEDirectiveType.IDLE
    target:   Tuple[float, float] = (0.0, 0.0)
    spring_k: float = 5.0
    spacing:  float = 30.0


# ── BSE ───────────────────────────────────────────────────────────────────

class BSE:
    """
    Geometry-only intent decomposition stub.

    wm_entries passed to tick() must be a list of dicts with keys:
        id        int   element ID
        is_active bool  entry is alive
        is_stale  bool  entry has not been refreshed within staleness window
        is_self   bool  this entry represents the local element (optional)

    scr_state passed to tick() must be a dict with keys:
        role         int  scr_role_t value
        quorum_state int  quorum_state_t value (0=LOST, 1=DEGRADED, 2=HEALTHY)
        leader_id    int  elected leader element_id
    """

    def __init__(self, element_id: int):
        self.element_id = element_id
        self._intent    = BSEIntent()
        self._directive = BSEDirective()

    def submit_intent(self, intent: BSEIntent) -> None:
        self._intent = intent

    def tick(self, wm_entries: List[dict], scr_state: dict) -> None:
        intent = self._intent

        if intent.type == BSEIntentType.IDLE:
            self._directive = BSEDirective(type=BSEDirectiveType.IDLE)

        elif intent.type == BSEIntentType.FORM:
            self._directive = self._form_directive(wm_entries, intent)

        elif intent.type in (BSEIntentType.MOVE, BSEIntentType.CONVERGE):
            # Stub: move every element to the same target point.
            self._directive = BSEDirective(
                type   = BSEDirectiveType.MOVE_TO_POINT,
                target = intent.target,
            )

        elif intent.type == BSEIntentType.DISPERSE:
            self._directive = BSEDirective(
                type     = BSEDirectiveType.MAINTAIN_SPRING,
                spring_k = 5.0,
                spacing  = intent.radius if intent.radius > 0.0 else 30.0,
            )

        else:
            self._directive = BSEDirective(type=BSEDirectiveType.IDLE)

    def get_directive(self) -> BSEDirective:
        return self._directive

    # ── Internal ─────────────────────────────────────────────────────────────

    def _form_directive(self, wm_entries: List[dict],
                        intent: BSEIntent) -> BSEDirective:
        """
        Assign self a vertex of a regular N-gon.
        N = active + fresh element count (including self).
        Rank = position of self.element_id in the sorted active-ID list.
        """
        active_ids = sorted(
            e['id'] for e in wm_entries
            if e.get('is_active') and not e.get('is_stale')
               and not e.get('is_self', False)
        )
        # Always include self
        if self.element_id not in active_ids:
            active_ids.append(self.element_id)
            active_ids.sort()

        if not active_ids:
            return BSEDirective(type=BSEDirectiveType.HOLD)

        rank  = active_ids.index(self.element_id)
        n     = len(active_ids)
        angle = 2.0 * math.pi * rank / n
        tx    = intent.target[0] + intent.radius * math.cos(angle)
        ty    = intent.target[1] + intent.radius * math.sin(angle)

        return BSEDirective(
            type   = BSEDirectiveType.MOVE_TO_POINT,
            target = (tx, ty),
        )
