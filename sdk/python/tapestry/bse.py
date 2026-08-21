"""
bse.py — Tapestry L6 Behavior Synthesis Engine (Python)

Python mirror of tapestry-os/subsys/bse/bse.c for simulation and research
(kept tick-for-tick equivalent — see the C file's module doc). Implements
the intent-parser and task-decomposition tiers plus the feedback controller
(achievement predicate). Deliberately out of the open-core tier (licensed):
the physics-aware planner, ML inference runtime, and simulation bridge —
except the EXCHANGE arc, a deliberately minimal deconfliction rule.

Intent → directive mapping
--------------------------
  IDLE      → IDLE            — the substrate-neutral QUIESCENCE signal
  FORM      → MOVE_TO_POINT   — vertex assignment by element_id rank; shape
                                CIRCLE (N-gon), LINE (evenly spaced on X),
                                or GRID (near-square rows/cols, radius as
                                cell spacing)
  MOVE      → MOVE_TO_POINT   — offset-preserving translation: own offset
                                from the participant centroid, captured at
                                activation, added to intent.target (a solo
                                element has zero offset and degenerates to
                                CONVERGE)
  CONVERGE  → MOVE_TO_POINT   — all elements to target (collapses formation)
  DISPERSE  → MAINTAIN_SPRING — spring-field with intent.radius spacing;
                                achieved once nearest fresh peer is at
                                least spacing away, sustained (see below)
  HOLD      → MOVE_TO_POINT   — own position captured at activation
                                (coordinate-free station-keeping)
  EXCHANGE  → MOVE_TO_POINT   — rotate stations by slot_shift around the
                                ID-sorted participant ring; targets are a
                                SNAPSHOT of positions at activation and the
                                commanded point travels a CCW arc about the
                                snapshot centroid, preserving separation

Achievement (bse.goal_achieved()): own position within achieve_eps of the
goal point, sustained for achieve_hold_ms.  HOLD is trivially achieved.
DISPERSE has no single goal point — achievement is nearest-fresh-peer
distance >= spacing - achieve_eps, sustained for achieve_hold_ms; vacuously
achieved with no peer visible.  IDLE never achieves (timeout-only goal).

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
EXCHANGE_OCCUPIED_M     = 0.35   # dest occupied while a fresh peer is this close
EXCHANGE_STANDOFF_M     = 0.5    # hold here on the approach line meanwhile


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


class BSEFrame(IntEnum):
    """Choreo SDK Design doc §5 frame ladder, FORM/CONVERGE only. Mirrors
    tapestry_bse_frame_t (bse.h) — see that header for the full rationale
    (ABSOLUTE=0 as the compat-preserving default, why NEWEST/OLDEST aren't
    implemented yet)."""
    ABSOLUTE   = 0
    COLLECTIVE = 1
    ELEMENT    = 2


class BSEAnchorSelector(IntEnum):
    """§5.2 anchor selectors, meaningful only when frame == ELEMENT.
    Mirrors tapestry_bse_anchor_selector_t (bse.h)."""
    LEADER        = 0
    ID            = 1
    SELF          = 2
    LOWEST_ENERGY = 3


ANCHOR_HOLD_MS     = 2000   # mirrors TAPESTRY_BSE_ANCHOR_HOLD_MS (bse.h)
ELEMENT_ID_INVALID = 0xFF   # mirrors csm.h


class BSEMotion(IntEnum):
    """§6 motion modifiers, FORM only.  Mirrors tapestry_bse_motion_t
    (bse.h) — see that header for why CONVERGE never reads this."""
    STATIC = 0
    SPIN   = 1


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class BSEIntent:
    type:   BSEIntentType = BSEIntentType.IDLE
    target: Tuple[float, float, float] = (50.0, 50.0, 50.0)
    radius: float = 30.0
    shape:  BSEShape = BSEShape.CIRCLE
    slot_shift: int = 0            # EXCHANGE ring rotation (0 → 1)
    direct_path: bool = False      # EXCHANGE beeline vs centroid arc
    achieve_eps: float = 0.0       # 0 → ACHIEVE_EPS_DEFAULT
    achieve_hold_ms: int = 0       # 0 → ACHIEVE_HOLD_MS_DEFAULT
    id: int = 0                    # originating goal identity; opaque, unread
                                   # here. Mirrors tapestry_bse_intent_t::id
    frame:     BSEFrame = BSEFrame.ABSOLUTE   # FORM/CONVERGE only (bse.h §5)
    anchor:    BSEAnchorSelector = BSEAnchorSelector.LEADER  # frame==ELEMENT only
    anchor_id: int = 0                                       # anchor==ID only
    motion:          BSEMotion = BSEMotion.STATIC  # FORM only (bse.h §6)
    spin_rate_radps: float = 0.0                   # motion==SPIN only


@dataclass
class BSEDirective:
    type:     BSEDirectiveType = BSEDirectiveType.IDLE
    target:   Tuple[float, float, float] = (0.0, 0.0, 0.0)
    spring_k: float = 5.0
    spacing:  float = 30.0


# ── BSE ───────────────────────────────────────────────────────────────────

class BSE:
    """
    Geometry-only intent decomposition + feedback controller.

    wm_entries passed to tick() must be a list of dicts with keys:
        id        int   element ID
        is_active bool  entry is alive
        is_stale  bool  entry has not been refreshed within staleness window
        is_self   bool  this entry represents the local element (optional)
        x, y, z   float element position, full 3D (required for FORM rank
                        fallback compatibility it may be omitted, but HOLD,
                        EXCHANGE, and the achievement predicate need real
                        positions — including on the is_self entry; z
                        defaults to 0.0 if absent, same as x/y)

    scr_state passed to tick() must be a dict with keys:
        role         int  scr_role_t value
        quorum_state int  quorum_state_t value (0=LOST, 1=DEGRADED, 2=HEALTHY)
        leader_id    int  elected leader element_id
    """

    def __init__(self, element_id: int):
        self.element_id = element_id
        self._intent    = BSEIntent()
        self._directive = BSEDirective()
        self._track_scope: int = 0
        self._reset_goal_state()

    def set_track_scope(self, track: int) -> None:
        """Restrict _participants() to peers gossiping the same
        current_track (Choreo SDK Design doc §7; mirrors
        bse_set_track_scope()).  Defaults to 0, a no-op filter — every
        peer on a script with no [[tracks]] gossips current_track=0."""
        self._track_scope = track

    def _reset_goal_state(self) -> None:
        self._achieved        = False
        self._achieve_accum   = 0
        self._goal_pt: Optional[Tuple[float, float, float]] = None
        self._hold_station: Optional[Tuple[float, float, float]] = None
        self._ex: Optional[dict] = None   # exchange snapshot/arc state
        self._move_offset: Optional[Tuple[float, float, float]] = None
        # FORM/CONVERGE frame == ELEMENT: debounced anchor selector
        # resolution (see ANCHOR_HOLD_MS). None means "unset".
        self._anchor_locked_id: Optional[int]    = None
        self._anchor_candidate_id: Optional[int] = None
        self._anchor_candidate_ms: int           = 0
        # FORM motion == SPIN: accumulated rotation since activation. Pure
        # time integration, reset only on a new activation.
        self._spin_theta: float = 0.0
        # Tick-scoped (also reset at the top of tick(), like _goal_pt) —
        # see anchor_lost().
        self._anchor_lost: bool = False

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
        self._anchor_lost = False

        if intent.type == BSEIntentType.IDLE:
            self._directive = BSEDirective(type=BSEDirectiveType.IDLE)

        elif intent.type == BSEIntentType.HOLD:
            self._tick_hold(wm_entries)

        elif intent.type == BSEIntentType.EXCHANGE:
            self._tick_exchange(wm_entries)

        elif intent.type == BSEIntentType.FORM:
            self._directive = self._form_directive(wm_entries, intent, scr_state)
            if self._directive.type == BSEDirectiveType.MOVE_TO_POINT:
                self._goal_pt = self._directive.target

        elif intent.type == BSEIntentType.MOVE:
            self._directive = self._move_directive(wm_entries, intent)
            if self._directive.type == BSEDirectiveType.MOVE_TO_POINT:
                self._goal_pt = self._directive.target

        elif intent.type == BSEIntentType.CONVERGE:
            # All elements gather at the identical point — deliberately
            # different from MOVE, which preserves formation (see below).
            # Frame resolution (bse.h §5): ABSOLUTE (default) makes eff
            # == intent.target exactly — no-op for every existing caller.
            # intent.motion is deliberately never read here — see bse.h's
            # CONVERGE comment (its target IS the frame origin, so
            # "rotating the offset" is a no-op).
            eff = self._resolve_effective_target(wm_entries, intent, scr_state)
            if eff is None:
                self._directive = BSEDirective(type=BSEDirectiveType.HOLD)
            else:
                self._directive = BSEDirective(
                    type   = BSEDirectiveType.MOVE_TO_POINT,
                    target = eff,
                )
                self._goal_pt = eff

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

    def anchor_lost(self) -> bool:
        """True if the last tick() had a FORM/CONVERGE intent with
        frame == ELEMENT and could not resolve any anchor position —
        never locked one yet, or the previously-locked anchor's peer went
        stale/inactive.  False for every other frame or intent type.
        Tick-scoped, like goal_achieved().  This is the source for
        ChoreoEvent.ANCHOR_LOST (choreo.py §8.2)."""
        return self._anchor_lost

    # ── Internal ─────────────────────────────────────────────────────────────

    def _own_position(self, wm_entries: List[dict]) -> Optional[Tuple[float, float, float]]:
        for e in wm_entries:
            if e.get('is_self', False):
                if 'x' in e and 'y' in e:
                    return (float(e['x']), float(e['y']), float(e.get('z', 0.0)))
                return None
        return None

    def _participants(self, wm_entries: List[dict]):
        """Self + fresh active SAME-TRACK peers as [(id, (x, y))],
        ID-sorted.  Track-filtered via e['current_track'] == self._track_scope
        (§7, wire v4, mirrors collect_participants() in bse.c) — a peer
        gossiping a different track is helping a different collective
        activity (or none) and must not skew centroid/rank here.
        _track_scope defaults to 0, matching every peer's gossiped
        current_track on a script with no tracks, so this is a no-op
        filter for every existing (non-tracked) caller."""
        parts = []
        for e in wm_entries:
            if e.get('is_self', False):
                parts.append((self.element_id,
                              (float(e.get('x', 0.0)), float(e.get('y', 0.0)),
                               float(e.get('z', 0.0)))))
            elif e.get('is_active') and not e.get('is_stale') \
                    and e.get('current_track', 0) == self._track_scope:
                parts.append((int(e['id']),
                              (float(e.get('x', 0.0)), float(e.get('y', 0.0)),
                               float(e.get('z', 0.0)))))
        parts.sort(key=lambda p: p[0])
        return parts

    # ── Frames and anchors (FORM / CONVERGE, bse.h §5) ─────────────────────

    def _lookup_position_by_id(self, wm_entries: List[dict],
                               elem_id: int) -> Optional[Tuple[float, float, float]]:
        """Current position of `elem_id` among self + fresh peers, or None
        if not currently resolvable (gone, stale, or unknown)."""
        for e in wm_entries:
            is_self = e.get('is_self', False)
            eid = self.element_id if is_self else e.get('id')
            if eid != elem_id:
                continue
            if is_self:
                if 'x' in e and 'y' in e:
                    return (float(e['x']), float(e['y']), float(e.get('z', 0.0)))
                return None
            if e.get('is_active') and not e.get('is_stale'):
                return (float(e.get('x', 0.0)), float(e.get('y', 0.0)),
                       float(e.get('z', 0.0)))
        return None

    def _resolve_anchor_selector(self, wm_entries: List[dict], intent: BSEIntent,
                                 scr_state: dict) -> Optional[int]:
        """This tick's raw (undebounced) anchor selector -> an element id,
        or None if no candidate exists right now.  Ties (LOWEST_ENERGY)
        break by lowest id — every element must derive the SAME anchor id
        from the same world-model snapshot."""
        sel = intent.anchor
        if sel == BSEAnchorSelector.SELF:
            return self.element_id
        if sel == BSEAnchorSelector.ID:
            return intent.anchor_id
        if sel == BSEAnchorSelector.LEADER:
            leader = scr_state.get('leader_id', ELEMENT_ID_INVALID)
            return None if leader == ELEMENT_ID_INVALID else leader
        if sel == BSEAnchorSelector.LOWEST_ENERGY:
            best_id: Optional[int] = None
            best_energy = 0
            for e in wm_entries:
                is_self = e.get('is_self', False)
                if not is_self and not (e.get('is_active') and not e.get('is_stale')):
                    continue
                eid    = self.element_id if is_self else e.get('id')
                energy = e.get('energy_level', 0)
                if best_id is None or energy < best_energy or \
                        (energy == best_energy and eid < best_id):
                    best_id, best_energy = eid, energy
            return best_id
        return None

    def _resolve_effective_target(self, wm_entries: List[dict], intent: BSEIntent,
                                  scr_state: dict) -> Optional[Tuple[float, float, float]]:
        """Resolve the effective FORM/CONVERGE target for this tick, per
        intent.frame.  ABSOLUTE returns intent.target unchanged.
        COLLECTIVE returns the live participant centroid.  ELEMENT
        resolves and DEBOUNCES the anchor selector (ANCHOR_HOLD_MS) and
        returns None while no anchor has ever stabilized.

        The selector->id and id->position steps are deliberately split
        (mirrors bse.c): debouncing is about choosing between competing
        VALID anchor candidates, not about masking one that has genuinely
        disappeared — that fails immediately, same as EXCHANGE's own
        can't-compute-this-tick HOLD fallback.
        """
        if intent.frame == BSEFrame.ABSOLUTE:
            return intent.target

        if intent.frame == BSEFrame.COLLECTIVE:
            parts = self._participants(wm_entries)
            if not parts:
                return None
            cx = sum(p[1][0] for p in parts) / len(parts)
            cy = sum(p[1][1] for p in parts) / len(parts)
            cz = sum(p[1][2] for p in parts) / len(parts)
            return (cx, cy, cz)

        # BSEFrame.ELEMENT.  Every path below funnels through `resolved`
        # so anchor_lost() (CHOREO_EVENT_ANCHOR_LOST's source) has one
        # place to be set, instead of repeating it at every return.
        resolved = None
        raw_id = self._resolve_anchor_selector(wm_entries, intent, scr_state)
        if raw_id is None:
            self._anchor_locked_id    = None
            self._anchor_candidate_id = None
        elif self._anchor_locked_id is not None and raw_id == self._anchor_locked_id:
            self._anchor_candidate_id = None   # stable — no pending switch
            resolved = self._lookup_position_by_id(wm_entries, raw_id)
        else:
            # raw_id differs from the locked anchor (or nothing locked yet).
            if self._anchor_candidate_id == raw_id:
                self._anchor_candidate_ms += WM_CYCLE_MS
            else:
                self._anchor_candidate_id = raw_id
                self._anchor_candidate_ms = 0

            if self._anchor_candidate_ms >= ANCHOR_HOLD_MS:
                self._anchor_locked_id    = raw_id
                self._anchor_candidate_id = None
                resolved = self._lookup_position_by_id(wm_entries, raw_id)
            elif self._anchor_locked_id is not None:
                # Candidate not yet confirmed: keep using the still-locked
                # anchor (don't disturb a stable anchor for an unconfirmed
                # switch).
                resolved = self._lookup_position_by_id(wm_entries, self._anchor_locked_id)
            # else: nothing locked yet — first-ever resolution waits out
            # the same hold time as any other switch; resolved stays None.

        if resolved is None:
            self._anchor_lost = True
        return resolved

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

        # Occupied destination (step-skew defense — mirrors bse.c): hold a
        # standoff point on the approach line and defer achievement while a
        # fresh peer still sits on the station.  Real 3D distance.
        dest = ex['dest']
        occupied = any(
            not e.get('is_self', False)
            and e.get('is_active') and not e.get('is_stale')
            and math.hypot(float(e.get('x', 0.0)) - dest[0],
                           float(e.get('y', 0.0)) - dest[1],
                           float(e.get('z', 0.0)) - dest[2])
                < EXCHANGE_OCCUPIED_M
            for e in wm_entries)
        if occupied:
            own = self._own_position(wm_entries)
            if own is not None:
                d = math.hypot(own[0] - dest[0], own[1] - dest[1], own[2] - dest[2])
                if d > 1e-3:
                    self._directive.target = (
                        dest[0] + (own[0] - dest[0]) / d * EXCHANGE_STANDOFF_M,
                        dest[1] + (own[1] - dest[1]) / d * EXCHANGE_STANDOFF_M,
                        dest[2] + (own[2] - dest[2]) / d * EXCHANGE_STANDOFF_M)
            self._goal_pt = None

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

        # Centroid's z averages same as x/y (kept for reference/logging),
        # but the arc's ANGULAR decomposition below stays projected onto
        # the XY plane — a "circle" swap is a planar concept; full
        # spherical rotation is a separate design question, not attempted
        # here.
        #
        # z is NOT part of what's exchanged: this element's own z stays
        # exactly what it already is for the whole maneuver — EXCHANGE
        # only ever reassigns x/y stations. If elements ARE separated in
        # altitude some other way, that is a real safety margin during a
        # horizontal crossing, especially with direct_path's beeline;
        # making z track the destination station's
        # altitude would walk two elements through each other's altitude
        # mid-swap, exactly when horizontal separation is smallest too.
        # `dest`'s z is overwritten with this element's OWN z below (not
        # the peer's), so every z comparison against it is against where
        # this element will actually end up.
        cx = sum(p[1][0] for p in parts) / len(parts)
        cy = sum(p[1][1] for p in parts) / len(parts)
        cz = sum(p[1][2] for p in parts) / len(parts)
        own  = parts[rank][1]
        dest = parts[dest_i][1]
        dest = (dest[0], dest[1], own[2])   # z is not exchanged — see above

        theta0 = math.atan2(own[1] - cy, own[0] - cx)
        theta1 = math.atan2(dest[1] - cy, dest[0] - cx)
        # CCW travel in (0, 2π] — every element rotates the same direction,
        # so pairwise separation is preserved throughout the maneuver.
        dtheta = theta1 - theta0
        while dtheta <= 0.0:
            dtheta += 2.0 * math.pi
        if dest_i == rank:
            dtheta = 0.0
        # Direct path: no arc — target is the destination from tick one
        # (a genuine 3D beeline; safe when something else deconflicts the
        # crossing — see bse.h).
        if self._intent.direct_path:
            dtheta = 0.0

        self._ex = {
            'centroid': (cx, cy, cz),
            'dest':     dest,
            'theta0':   theta0,
            'dtheta':   dtheta,
            'r0':       math.hypot(own[0] - cx, own[1] - cy),
            'r1':       math.hypot(dest[0] - cx, dest[1] - cy),
            'progress': 0.0,
        }
        return True

    def _exchange_arc_target(self) -> Tuple[float, float, float]:
        ex = self._ex
        ex['progress'] += EXCHANGE_OMEGA_RADPS * (WM_CYCLE_MS * 0.001)
        if ex['dtheta'] <= 0.0 or ex['progress'] >= ex['dtheta']:
            return ex['dest']   # arc complete — exact snapshot station
        frac  = ex['progress'] / ex['dtheta']
        theta = ex['theta0'] + ex['progress']
        r     = ex['r0'] + (ex['r1'] - ex['r0']) * frac
        cx, cy, _cz = ex['centroid']
        z = ex['dest'][2]   # own z, held constant — see _exchange_capture()
        return (cx + r * math.cos(theta), cy + r * math.sin(theta), z)

    def _tick_achievement(self, wm_entries: List[dict]) -> None:
        if self._intent.type == BSEIntentType.HOLD and self._hold_station is not None:
            # Staying is the goal — trivially achieved; duration governs.
            self._achieved = True
            return
        if self._intent.type == BSEIntentType.DISPERSE:
            self._tick_disperse_achievement(wm_entries)
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
                      own[1] - self._goal_pt[1],
                      own[2] - self._goal_pt[2]) <= eps:
            self._achieve_accum += WM_CYCLE_MS
        else:
            self._achieve_accum = 0
        self._achieved = self._achieve_accum >= hold

    def _tick_disperse_achievement(self, wm_entries: List[dict]) -> None:
        """DISPERSE has no single goal point — achievement is "spread out":
        this element's nearest fresh active peer at least spacing (less eps
        slack) away, sustained for achieve_hold_ms.  Vacuously achieved with
        no peer visible (nothing to disperse from), matching the collective-
        achievement solo convention.  Without this, goal_achieved() was
        permanently False for DISPERSE, and a script step with
        advance_on_achieved=True and no timeout would never advance.

        Real 3D distance (mirrors formation.c's own position_distance()):
        this is safety-relevant spacing math, not yet flight-validated —
        see examples/cf21bl-formation/README.md's "Known limitations"."""
        eps  = self._intent.achieve_eps or ACHIEVE_EPS_DEFAULT
        hold = self._intent.achieve_hold_ms or ACHIEVE_HOLD_MS_DEFAULT
        own  = self._own_position(wm_entries)
        if own is None:
            return
        min_dist = None
        for e in wm_entries:
            if e.get('is_self', False) or not e.get('is_active') or e.get('is_stale'):
                continue
            d = math.hypot(float(e.get('x', 0.0)) - own[0],
                            float(e.get('y', 0.0)) - own[1],
                            float(e.get('z', 0.0)) - own[2])
            if min_dist is None or d < min_dist:
                min_dist = d
        spread = min_dist is None or min_dist >= (self._directive.spacing - eps)
        if spread:
            self._achieve_accum += WM_CYCLE_MS
        else:
            self._achieve_accum = 0
        self._achieved = self._achieve_accum >= hold

    def _form_directive(self, wm_entries: List[dict],
                        intent: BSEIntent, scr_state: dict) -> BSEDirective:
        """
        Assign self a vertex per intent.shape: CIRCLE (regular N-gon), LINE
        (evenly spaced on the X axis), or GRID (near-square rows/cols,
        radius as cell spacing) — all centered on the resolved frame target
        (bse.h §5; ABSOLUTE/default keeps this exactly intent.target).
        N = active + fresh element count (including self).
        Rank = position of self.element_id in the sorted active-ID list.
        """
        active_ids = sorted(
            e['id'] for e in wm_entries
            if e.get('is_active') and not e.get('is_stale')
               and not e.get('is_self', False)
               and e.get('current_track', 0) == self._track_scope
        )
        # Always include self
        if self.element_id not in active_ids:
            active_ids.append(self.element_id)
            active_ids.sort()

        if not active_ids:
            return BSEDirective(type=BSEDirectiveType.HOLD)

        eff_target = self._resolve_effective_target(wm_entries, intent, scr_state)
        if eff_target is None:
            return BSEDirective(type=BSEDirectiveType.HOLD)

        rank = active_ids.index(self.element_id)
        n    = len(active_ids)

        # Own vertex OFFSET from eff_target (the frame origin) — computed
        # separately so motion == SPIN (below) can rotate just the offset,
        # shape-agnostically, instead of every shape needing its own
        # rotation math.
        if intent.shape == BSEShape.LINE:
            if n > 1:
                step = (2.0 * intent.radius) / (n - 1)
                ox = -intent.radius + step * rank
            else:
                ox = 0.0
            oy = 0.0
        elif intent.shape == BSEShape.GRID:
            cols = math.ceil(math.sqrt(n))
            rows = math.ceil(n / cols)
            col, row = rank % cols, rank // cols
            ox = (col - 0.5 * (cols - 1)) * intent.radius
            oy = (row - 0.5 * (rows - 1)) * intent.radius
        else:
            angle = 2.0 * math.pi * rank / n
            ox = intent.radius * math.cos(angle)
            oy = intent.radius * math.sin(angle)

        # Motion (bse.h §6): SPIN rotates the offset about the frame
        # origin at spin_rate_radps.  Achievement generalizes unchanged —
        # _goal_pt is tick-scoped and simply tracks the rotating vertex.
        if intent.motion == BSEMotion.SPIN:
            self._spin_theta += intent.spin_rate_radps * (WM_CYCLE_MS * 0.001)
            c, s = math.cos(self._spin_theta), math.sin(self._spin_theta)
            ox, oy = ox * c - oy * s, ox * s + oy * c

        # Shapes stay planar — z is the frame origin's, unmodified (see
        # module doc: CIRCLE/LINE/GRID are named 2D patterns; the whole
        # flat shape can sit at any real altitude as a rigid unit).
        return BSEDirective(
            type   = BSEDirectiveType.MOVE_TO_POINT,
            target = (eff_target[0] + ox, eff_target[1] + oy, eff_target[2]),
        )

    def _move_directive(self, wm_entries: List[dict],
                        intent: BSEIntent) -> BSEDirective:
        """
        Offset-preserving translation: snapshot own offset
        from the participant centroid on activation, then command
        intent.target + that offset every tick — the formation translates
        as a rigid body instead of collapsing onto the target.
        """
        if self._move_offset is None:
            parts = self._participants(wm_entries)
            if not parts:
                return BSEDirective(type=BSEDirectiveType.HOLD)
            ids = [p[0] for p in parts]
            if self.element_id not in ids:
                return BSEDirective(type=BSEDirectiveType.HOLD)
            rank = ids.index(self.element_id)
            cx = sum(p[1][0] for p in parts) / len(parts)
            cy = sum(p[1][1] for p in parts) / len(parts)
            cz = sum(p[1][2] for p in parts) / len(parts)
            own = parts[rank][1]
            self._move_offset = (own[0] - cx, own[1] - cy, own[2] - cz)

        ox, oy, oz = self._move_offset
        return BSEDirective(
            type   = BSEDirectiveType.MOVE_TO_POINT,
            target = (intent.target[0] + ox, intent.target[1] + oy,
                     intent.target[2] + oz),
        )
