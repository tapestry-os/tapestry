"""
facts.py — Python mirror of tapestry-os/subsys/csm/facts.c (L4 shared facts).

Same table, merge, rank, eviction, digest and Trickle rules as the C engine,
slot for slot, so a sequence of calls on both gives the same table, the same
digest and the same send decisions (sdk/tests/test_facts.py checks the
shared vectors in sdk/tests/data/fact_vectors.json against both).  See
<tapestry/csm.h>'s "Shared facts" section for the rules themselves.

Choreo and BSE take a FactTable through their tick(..., facts=...) argument;
the landmark anchor and the landmark events read it.
"""

from dataclasses import dataclass, replace
from typing import List, Optional, Tuple
import math

MAX_ELEMENTS = 32

FACT_TYPE_NONE     = 0
FACT_TYPE_TARGET   = 1
FACT_TYPE_ZONE     = 2
FACT_TYPE_HAZARD   = 3
FACT_TYPE_WAYPOINT = 4
FACT_TYPE_APP_MIN  = 0x80

FACT_TYPES = {"target": FACT_TYPE_TARGET, "zone": FACT_TYPE_ZONE,
              "hazard": FACT_TYPE_HAZARD, "waypoint": FACT_TYPE_WAYPOINT}

FACT_RETRACTED  = 0x01
FACT_IMMUTABLE  = 0x02
FACT_PRIO_SHIFT = 2
FACT_PRIO_MASK  = 0x0C

WM_MAX_FACTS             = 16
FACT_MAX_PER_ORIGIN      = WM_MAX_FACTS
FACT_TOMBSTONE_TTL_MS    = 60000
FACT_IMIN_MS             = 250
FACT_IMAX_MS             = 8000
FACT_K                   = 2
FACT_SYNC_TIMEOUT_MS     = 2 * FACT_IMAX_MS
FACT_MERGE_RADIUS_TARGET = 15.0

WM_FACT_CREATED      = 0
WM_FACT_REUSED       = 1
WM_FACT_ERR_INVALID  = -1
WM_FACT_ERR_UNSYNCED = -2
WM_FACT_ERR_FULL     = -3

WM_FACT_EVICT_CAPACITY   = 1
WM_FACT_EVICT_PER_ORIGIN = 2


def fact_prio(p: int) -> int:
    return (p << FACT_PRIO_SHIFT) & FACT_PRIO_MASK


def fact_prio_of(flags: int) -> int:
    return (flags & FACT_PRIO_MASK) >> FACT_PRIO_SHIFT


FactKey = Tuple[int, int, int, int]   # (type, origin, epoch, seq)


@dataclass
class Fact:
    type: int
    origin: int
    epoch: int
    seq: int
    flags: int = 0
    stamp: int = 0
    pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    attr: int = 0

    @property
    def key(self) -> FactKey:
        return (self.type, self.origin, self.epoch, self.seq)

    @property
    def live(self) -> bool:
        return (self.flags & FACT_RETRACTED) == 0

    def to_dict(self) -> dict:
        return {"type": self.type, "origin": self.origin, "epoch": self.epoch,
                "seq": self.seq, "flags": self.flags, "stamp": self.stamp,
                "x": self.pos[0], "y": self.pos[1], "z": self.pos[2],
                "attr": self.attr}

    @staticmethod
    def from_dict(d: dict) -> "Fact":
        return Fact(type=d["type"], origin=d["origin"], epoch=d["epoch"],
                    seq=d["seq"], flags=d.get("flags", 0),
                    stamp=d.get("stamp", 0),
                    pos=(float(d.get("x", 0.0)), float(d.get("y", 0.0)),
                         float(d.get("z", 0.0))),
                    attr=d.get("attr", 0))


def _rank_key(f: Fact) -> Tuple[int, int, int, int]:
    return (f.stamp, f.origin, f.epoch, f.seq)


def _evict_key(f: Fact) -> Tuple[int, int, int, int, int, int]:
    return (1 if f.live else 0, fact_prio_of(f.flags)) + _rank_key(f)


def merge_radius(ftype: int) -> float:
    return FACT_MERGE_RADIUS_TARGET if ftype == FACT_TYPE_TARGET else 0.0


def _dist(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def record_hash(f: Fact) -> int:
    b = bytes([f.type & 0xFF, f.origin & 0xFF, f.epoch & 0xFF,
               (f.epoch >> 8) & 0xFF, f.seq & 0xFF, f.flags & 0xFF,
               f.stamp & 0xFF, (f.stamp >> 8) & 0xFF,
               (f.stamp >> 16) & 0xFF, (f.stamp >> 24) & 0xFF])
    h = 2166136261
    for x in b:
        h ^= x
        h = (h * 16777619) & 0xFFFFFFFF
    return (h ^ (h >> 16)) & 0xFFFF


@dataclass
class _Slot:
    fact: Optional[Fact] = None
    held_ms: int = 0
    tomb_ms: int = 0

    @property
    def used(self) -> bool:
        return self.fact is not None


class FactTable:
    """One element's fact table.  Mirrors wm_facts_t + the wm_fact_* API."""

    def __init__(self, owner_id: int, epoch: int = 0):
        self.owner_id = owner_id
        self.slots: List[_Slot] = [_Slot() for _ in range(WM_MAX_FACTS)]
        self.epoch = epoch & 0xFFFF
        self.next_seq = 0
        self.clock = 0
        self.synced = False
        self.since_init_ms = 0
        self.frames_heard = 0
        self.trickle_i_ms = 0
        self.trickle_t_ms = 0
        self.trickle_elapsed_ms = 0
        self.trickle_c = 0
        self.trickle_fired = False
        self.trickle_started = False
        self.cursor = 0
        self.evictions = 0
        self.last_evicted: Optional[FactKey] = None
        self.last_evict_reason = 0

    # ── internals ──────────────────────────────────────────────────────────

    def _find_slot(self, key: FactKey) -> Optional[_Slot]:
        for s in self.slots:
            if s.used and s.fact.key == key:
                return s
        return None

    def _evict(self, s: _Slot, reason: int) -> None:
        self.evictions += 1
        self.last_evicted = s.fact.key
        self.last_evict_reason = reason
        s.fact = None

    def _admit(self, f: Fact) -> bool:
        if f.live:
            mine = [s for s in self.slots
                    if s.used and s.fact.live and s.fact.origin == f.origin
                    and s.fact.epoch == f.epoch]
            if len(mine) >= FACT_MAX_PER_ORIGIN:
                oldest = min(mine, key=lambda s: _evict_key(s.fact))
                if _evict_key(f) < _evict_key(oldest.fact):
                    return False
                self._evict(oldest, WM_FACT_EVICT_PER_ORIGIN)

        free = None
        lowest = None
        for s in self.slots:
            if not s.used:
                free = s
                break
            if lowest is None or _evict_key(s.fact) < _evict_key(lowest.fact):
                lowest = s
        if free is None:
            if _evict_key(f) < _evict_key(lowest.fact):
                return False
            self._evict(lowest, WM_FACT_EVICT_CAPACITY)
            free = lowest
        free.fact = replace(f)
        free.held_ms = 0
        free.tomb_ms = 0
        return True

    def _nearby_duplicate(self, ftype: int, skip: Optional[FactKey],
                          pos) -> Optional[_Slot]:
        r = merge_radius(ftype)
        if r <= 0.0:
            return None
        best = None
        for s in self.slots:
            if not s.used or not s.fact.live or s.fact.type != ftype:
                continue
            if skip is not None and s.fact.key == skip:
                continue
            if _dist(s.fact.pos, pos) > r:
                continue
            if best is None or _rank_key(s.fact) < _rank_key(best.fact):
                best = s
        return best

    def _trickle_start(self, i_ms: int, rnd: int) -> None:
        half = i_ms // 2
        span = i_ms - half
        self.trickle_i_ms = i_ms
        self.trickle_t_ms = half + (rnd % span if span > 0 else 0)
        self.trickle_elapsed_ms = 0
        self.trickle_c = 0
        self.trickle_fired = False
        self.trickle_started = True

    def _trickle_reset(self) -> None:
        if self.trickle_started and self.trickle_i_ms > FACT_IMIN_MS:
            self.trickle_i_ms = 0

    # ── API ────────────────────────────────────────────────────────────────

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch & 0xFFFF

    def publish(self, ftype: int, pos, attr: int = 0, flags: int = 0
                ) -> Tuple[int, Optional[FactKey]]:
        if ftype == FACT_TYPE_NONE or self.owner_id >= MAX_ELEMENTS:
            return WM_FACT_ERR_INVALID, None
        if not self.synced:
            return WM_FACT_ERR_UNSYNCED, None
        pos = tuple(float(v) for v in pos)
        dup = self._nearby_duplicate(ftype, None, pos)
        if dup is not None:
            return WM_FACT_REUSED, dup.fact.key
        f = Fact(type=ftype, origin=self.owner_id, epoch=self.epoch,
                 seq=self.next_seq,
                 flags=flags & (FACT_IMMUTABLE | FACT_PRIO_MASK),
                 stamp=self.clock + 1, pos=pos, attr=attr)
        if not self._admit(f):
            return WM_FACT_ERR_FULL, None
        self.clock = f.stamp
        self.next_seq = (self.next_seq + 1) & 0xFF
        self._trickle_reset()
        return WM_FACT_CREATED, f.key

    def retract(self, key: FactKey) -> int:
        s = self._find_slot(key)
        if s is None or not s.fact.live:
            return -1
        self.clock += 1
        s.fact.flags |= FACT_RETRACTED
        s.fact.stamp = self.clock
        s.tomb_ms = 0
        self._trickle_reset()
        return 0

    def receive(self, fact: Fact) -> bool:
        if fact.type == FACT_TYPE_NONE or fact.origin >= MAX_ELEMENTS:
            return False
        if fact.stamp > self.clock:
            self.clock = fact.stamp
        held = self._find_slot(fact.key)
        if held is not None:
            h = held.fact
            if not h.live:
                if not fact.live and fact.stamp < h.stamp:
                    held.fact = replace(fact)
                    return True
                return False
            if not fact.live:
                held.fact = replace(fact)
                held.tomb_ms = 0
                return True
            immutable = (h.flags & FACT_IMMUTABLE) != 0
            c = (_rank_key(fact) > _rank_key(h)) - (_rank_key(fact) < _rank_key(h))
            if (immutable and c < 0) or (not immutable and c > 0):
                held.fact = replace(fact)
                return True
            return False

        if fact.live:
            while True:
                dup = self._nearby_duplicate(fact.type, fact.key, fact.pos)
                if dup is None:
                    break
                if _rank_key(fact) > _rank_key(dup.fact):
                    return False
                dup.fact = None
        return self._admit(fact)

    def heard_digest(self, digest: int) -> None:
        self.synced = True
        self.frames_heard += 1
        if digest == self.digest():
            self.trickle_c = min(self.trickle_c + 1, 0xFF)
        else:
            self._trickle_reset()

    def tick(self, elapsed_ms: int) -> None:
        self.since_init_ms += elapsed_ms
        if self.since_init_ms >= FACT_SYNC_TIMEOUT_MS:
            self.synced = True
        for s in self.slots:
            if not s.used:
                continue
            s.held_ms += elapsed_ms
            if not s.fact.live:
                s.tomb_ms += elapsed_ms
                if s.tomb_ms >= FACT_TOMBSTONE_TTL_MS:
                    s.fact = None

    def trickle_tick(self, elapsed_ms: int, rnd: int) -> bool:
        if not self.trickle_started or self.trickle_i_ms == 0:
            self._trickle_start(FACT_IMIN_MS, rnd)
        self.trickle_elapsed_ms += elapsed_ms
        send = False
        if not self.trickle_fired and self.trickle_elapsed_ms >= self.trickle_t_ms:
            self.trickle_fired = True
            send = self.trickle_c < FACT_K
        if self.trickle_elapsed_ms >= self.trickle_i_ms:
            self._trickle_start(min(self.trickle_i_ms * 2, FACT_IMAX_MS), rnd)
        return send

    def pack(self) -> List[Fact]:
        out: List[Fact] = []
        for k in range(WM_MAX_FACTS):
            i = (self.cursor + k) % WM_MAX_FACTS
            if self.slots[i].used:
                out.append(replace(self.slots[i].fact))
                if len(out) == 2:
                    self.cursor = (i + 1) % WM_MAX_FACTS
                    break
        if len(out) == 1:
            self.cursor = 0
        return out

    def digest(self) -> int:
        d = 0
        for s in self.slots:
            if s.used:
                d = (d + record_hash(s.fact)) & 0xFFFF
        return d

    def get(self, key: FactKey) -> Optional[Fact]:
        s = self._find_slot(key)
        return replace(s.fact) if s is not None else None

    def live(self, ftype: int = FACT_TYPE_NONE) -> List[Fact]:
        """Live facts of ftype (NONE: any) in rank order."""
        out = [s.fact for s in self.slots
               if s.used and s.fact.live
               and (ftype == FACT_TYPE_NONE or s.fact.type == ftype)]
        return sorted(out, key=_rank_key)

    def find(self, ftype: int, rank: int) -> Optional[Fact]:
        lv = self.live(ftype)
        if rank < 0 or rank >= len(lv):
            return None
        return replace(lv[rank])

    def count(self, ftype: int = FACT_TYPE_NONE) -> int:
        return len(self.live(ftype))

    def tombstones(self, ftype: int = FACT_TYPE_NONE) -> int:
        return sum(1 for s in self.slots
                   if s.used and not s.fact.live
                   and (ftype == FACT_TYPE_NONE or s.fact.type == ftype))

    def held(self) -> List[Fact]:
        return [replace(s.fact) for s in self.slots if s.used]
