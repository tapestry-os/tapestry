"""
bse.py — Tapestry L6 Behavior Synthesis Engine stub (Python)

NOT FOR PRODUCTION USE.

Python mirror of tapestry-os/subsys/bse/bse.c for simulation and
research.  Implements the intent-parser and task-decomposition tiers plus
the minimal feedback controller (achievement predicate).  The physics-aware
planner, ML inference runtime, and simulation bridge are absent (commercial
BSE) — except the EXCHANGE arc, a deliberately minimal deconfliction rule.

Intent → directive mapping
--------------------------
  IDLE      → IDLE            — the substrate-neutral QUIESCENCE signal
  FORM      → MOVE_TO_POINT   — vertex of regular N-gon, slot by element_id rank
  MOVE      → MOVE_TO_POINT   — all elements to same target (stub limitation)
  CONVERGE  → MOVE_TO_POINT   — all elements to target
  DISPERSE  → MAINTAIN_SPRING — spring-field with intent.radius spacing
  HOLD      → MOVE_TO_POINT   — own position captured at activation
                                (coordinate-free station-keeping)
  EXCHANGE  → MOVE_TO_POINT   — rotate stations by slot_shift around the
                                ID-sorted participant ring; targets are a
                                SNAPSHOT of positions at activation and the
                                commanded point travels a CCW arc about the
                                snapshot centroid, preserving separation

Achievement (bse.goal_achieved()): own position within achieve_eps of the
goal point, sustained for achieve_hold_ms.  HOLD is trivially achieved;
IDLE and DISPERSE never are (timeout-only goals).

Usage (one instance per simulated element):

    from tapestry.bse import BSE, BSEIntent, BSEIntentType, BSEShape

    bse = BSE(element_id=0)
    bse.submit_intent(BSEIntent(type=BSEIntentType.EXCHANGE))

    # each simulation tick (WM_CYCLE_MS = 100 ms period):
    bse.tick(wm_entries, scr_state)
    directive = bse.get_directive()   # BSEDirective instance
    done      = bse.goal_achieved()

wm_entries must now carry positions — see the BSE class docstring.
See sdk/examples/hello_swarm.py for a complete worked example.
"""

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple


WM_CYCLE_MS = 100   # mirrors csm.h — tick() integrates time on this period

ACHIEVE_EPS_DEFAULT     = 0.5
ACHIEVE_HOLD_MS_DEFAULT = 3000
EXCHANGE_OMEGA_RADPS    = 0.15


# ── Enumerations ──────────────────────────────────────────────────────────────

class BSEIntentType(IntEnum):
    IDLE     = 0
    FORM     = 1
    MOVE     = 2
    DISPERSE = 3
    CONVERGE = 4
    HOLD     = 5
    EXCHANGE = 6


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
    slot_shift: int = 0            # EXCHANGE ring rotation (0 → 1)
    achieve_eps: float = 0.0       # 0 → ACHIEVE_EPS_DEFAULT
    achieve_hold_ms: int = 0       # 0 → ACHIEVE_HOLD_MS_DEFAULT


@dataclass
class BSEDirective:
    type:     BSEDirectiveType = BSEDirectiveType.IDLE
    target:   Tuple[float, float] = (0.0, 0.0)
    spring_k: float = 5.0
    spacing:  float = 30.0


# ── BSE ───────────────────────────────────────────────────────────────────

class BSE:
    """
    Geometry-only intent decomposition stub + minimal feedback controller.

    wm_entries passed to tick() must be a list of dicts with keys:
        id        int   element ID
        is_active bool  entry is alive
        is_stale  bool  entry has not been refreshed within staleness window
        is_self   bool  this entry represents the local element (optional)
        x, y      float element position (required for FORM rank fallback
                        compatibility it may be omitted, but HOLD, EXCHANGE,
                        and the achievement predicate need real positions —
                        including on the is_self entry)

    scr_state passed to tick() must be a dict with keys:
        role         int  scr_role_t value
        quorum_state int  quorum_state_t value (0=LOST, 1=DEGRADED, 2=HEALTHY)
        leader_id    int  elected leader element_id
    """

    def __init__(self, element_id: int):
        self.element_id = element_id
        self._intent    = BSEIntent()
        self._directive = BSEDirective()
        self._reset_goal_state()

    def _reset_goal_state(self) -> None:
        self._achieved        = False
        self._achieve_accum   = 0
        self._goal_pt: Optional[Tuple[float, float]] = None
        self._hold_station: Optional[Tuple[float, float]] = None
        self._ex: Optional[dict] = None   # exchange snapshot/arc state

    def submit_intent(self, intent: BSEIntent) -> None:
        self._intent = intent
        self._reset_goal_state()
        if intent.type == BSEIntentType.IDLE:
            # Quiescence takes effect immediately (mirrors bse.c — the
            # Choreographer stops ticking after terminate).
            self._directive = BSEDirective(type=BSEDirectiveType.IDLE)

    def tick(self, wm_entries: List[dict], scr_state: dict) -> None:
        intent = self._intent
        self._goal_pt = None

        if intent.type == BSEIntentType.IDLE:
            self._directive = BSEDirective(type=BSEDirectiveType.IDLE)

        elif intent.type == BSEIntentType.HOLD:
            self._tick_hold(wm_entries)

        elif intent.type == BSEIntentType.EXCHANGE:
            self._tick_exchange(wm_entries)

        elif intent.type == BSEIntentType.FORM:
            self._directive = self._form_directive(wm_entries, intent)
            if self._directive.type == BSEDirectiveType.MOVE_TO_POINT:
                self._goal_pt = self._directive.target

        elif intent.type in (BSEIntentType.MOVE, BSEIntentType.CONVERGE):
            # Stub: move every element to the same target point.
            self._directive = BSEDirective(
                type   = BSEDirectiveType.MOVE_TO_POINT,
                target = intent.target,
            )
            self._goal_pt = intent.target

        elif intent.type == BSEIntentType.DISPERSE:
            self._directive = BSEDirective(
                type     = BSEDirectiveType.MAINTAIN_SPRING,
                spring_k = 5.0,
                spacing  = intent.radius if intent.radius > 0.0 else 30.0,
            )

        else:
            self._directive = BSEDirective(type=BSEDirectiveType.IDLE)

        self._tick_achievement(wm_entries)

    def get_directive(self) -> BSEDirective:
        return self._directive

    def goal_achieved(self) -> bool:
        """Minimal L6 feedback controller output — see bse.h."""
        return self._achieved

    # ── Internal ─────────────────────────────────────────────────────────────

    def _own_position(self, wm_entries: List[dict]) -> Optional[Tuple[float, float]]:
        for e in wm_entries:
            if e.get('is_self', False):
                if 'x' in e and 'y' in e:
                    return (float(e['x']), float(e['y']))
                return None
        return None

    def _participants(self, wm_entries: List[dict]):
        """Self + fresh active peers as [(id, (x, y))], ID-sorted."""
        parts = []
        for e in wm_entries:
            if e.get('is_self', False):
                parts.append((self.element_id,
                              (float(e.get('x', 0.0)), float(e.get('y', 0.0)))))
            elif e.get('is_active') and not e.get('is_stale'):
                parts.append((int(e['id']),
                              (float(e.get('x', 0.0)), float(e.get('y', 0.0)))))
        parts.sort(key=lambda p: p[0])
        return parts

    def _tick_hold(self, wm_entries: List[dict]) -> None:
        if self._hold_station is None:
            own = self._own_position(wm_entries)
            if own is None:
                self._directive = BSEDirective(type=BSEDirectiveType.HOLD)
                return
            self._hold_station = own
        self._directive = BSEDirective(
            type   = BSEDirectiveType.MOVE_TO_POINT,
            target = self._hold_station,
        )
        self._goal_pt = self._hold_station

    def _tick_exchange(self, wm_entries: List[dict]) -> None:
        if self._ex is None and not self._exchange_capture(wm_entries):
            # No fresh peer — hold and retry the capture next tick.
            self._directive = BSEDirective(type=BSEDirectiveType.HOLD)
            return
        self._directive = BSEDirective(
            type   = BSEDirectiveType.MOVE_TO_POINT,
            target = self._exchange_arc_target(),
        )
        # Achievement is against the DESTINATION station, and only once the
        # arc itself has completed — otherwise a body tracking the arc
        # perfectly "achieves" while still up to eps short of the station.
        ex = self._ex
        if ex['dtheta'] <= 0.0 or ex['progress'] >= ex['dtheta']:
            self._goal_pt = ex['dest']

    def _exchange_capture(self, wm_entries: List[dict]) -> bool:
        """Freeze the snapshot: stations + own arc about the centroid.

        Each element captures independently from its own world model; the
        snapshots differ by at most one gossip interval of peer motion,
        which the achievement epsilon absorbs.  Frozen targets — never
        live-chasing.
        """
        parts = self._participants(wm_entries)
        ids = [p[0] for p in parts]
        if len(parts) < 2 or self.element_id not in ids:
            return False

        rank  = ids.index(self.element_id)
        shift = self._intent.slot_shift or 1
        dest_i = (rank + shift) % len(parts)

        cx = sum(p[1][0] for p in parts) / len(parts)
        cy = sum(p[1][1] for p in parts) / len(parts)
        own  = parts[rank][1]
        dest = parts[dest_i][1]

        theta0 = math.atan2(own[1] - cy, own[0] - cx)
        theta1 = math.atan2(dest[1] - cy, dest[0] - cx)
        # CCW travel in (0, 2π] — every element rotates the same direction,
        # so pairwise separation is preserved throughout the maneuver.
        dtheta = theta1 - theta0
        while dtheta <= 0.0:
            dtheta += 2.0 * math.pi
        if dest_i == rank:
            dtheta = 0.0

        self._ex = {
            'centroid': (cx, cy),
            'dest':     dest,
            'theta0':   theta0,
            'dtheta':   dtheta,
            'r0':       math.hypot(own[0] - cx, own[1] - cy),
            'r1':       math.hypot(dest[0] - cx, dest[1] - cy),
            'progress': 0.0,
        }
        return True

    def _exchange_arc_target(self) -> Tuple[float, float]:
        ex = self._ex
        ex['progress'] += EXCHANGE_OMEGA_RADPS * (WM_CYCLE_MS * 0.001)
        if ex['dtheta'] <= 0.0 or ex['progress'] >= ex['dtheta']:
            return ex['dest']   # arc complete — exact snapshot station
        frac  = ex['progress'] / ex['dtheta']
        theta = ex['theta0'] + ex['progress']
        r     = ex['r0'] + (ex['r1'] - ex['r0']) * frac
        cx, cy = ex['centroid']
        return (cx + r * math.cos(theta), cy + r * math.sin(theta))

    def _tick_achievement(self, wm_entries: List[dict]) -> None:
        if self._intent.type == BSEIntentType.HOLD and self._hold_station is not None:
            # Staying is the goal — trivially achieved; duration governs.
            self._achieved = True
            return
        if self._goal_pt is None:
            self._achieve_accum = 0
            self._achieved = False
            return

        eps  = self._intent.achieve_eps or ACHIEVE_EPS_DEFAULT
        hold = self._intent.achieve_hold_ms or ACHIEVE_HOLD_MS_DEFAULT
        own  = self._own_position(wm_entries)
        if own is None:
            return
        if math.hypot(own[0] - self._goal_pt[0],
                      own[1] - self._goal_pt[1]) <= eps:
            self._achieve_accum += WM_CYCLE_MS
        else:
            self._achieve_accum = 0
        self._achieved = self._achieve_accum >= hold

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
