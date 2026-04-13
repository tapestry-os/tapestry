"""
telemetry.py — Combined L4 + L5 per-cycle CSV writer.

The element sends an L4 metric (type=2) and an L5 SCR metric (type=4)
every cycle.  This writer buffers the most recent of each type per
element and emits one combined row when both have arrived.

CSV columns
───────────
L4 columns (from CSM metric):
    wall_time_s, element_id, active_total, active_fresh, active_stale,
    inactive_total, collision_count, fresh_ratio, quorum_held, degraded,
    confidence, cycle_count, mean_age_ms, mean_position_error, min_separation

L5 columns (from SCR metric):
    role           0=NONE, 1=FOLLOWER, 2=LEADER
    leader_id      elected leader element_id; 255 = no leader
    quorum_state   0=LOST, 1=DEGRADED, 2=HEALTHY
    fresh_count    non-self fresh peers
    election_count cumulative leader changes since element startup
"""

import csv
import time

COLUMNS = [
    # L4 — CSM
    'wall_time_s',
    'element_id',
    'active_total',
    'active_fresh',
    'active_stale',
    'inactive_total',
    'collision_count',
    'fresh_ratio',
    'quorum_held',
    'degraded',
    'confidence',
    'cycle_count',
    'mean_age_ms',
    'mean_position_error',
    'min_separation',
    # L5 — SCR
    'role',
    'leader_id',
    'quorum_state',
    'fresh_count',
    'election_count',
]


class TelemetryWriter:
    """
    Buffers L4 and L5 metric dicts per element.  Emits a combined row
    whenever both metrics have arrived for the same element.

    Thread-safety: not required — asyncio single-threaded event loop.
    """

    def __init__(self, path: str):
        self._path    = path
        self._file    = open(path, 'w', newline='', buffering=1)
        self._writer  = csv.DictWriter(self._file, fieldnames=COLUMNS,
                                       extrasaction='ignore')
        self._writer.writeheader()
        self._start   = time.monotonic()
        # Per-element pending buffers
        self._l4_buf: dict[int, dict] = {}
        self._l5_buf: dict[int, dict] = {}

    # ── Incoming metric handlers ──────────────────────────────────────────────

    def on_l4_metric(self, msg: dict):
        """Called when a L4 CSM metric arrives."""
        eid = msg['element_id']
        self._l4_buf[eid] = dict(msg)
        self._maybe_flush(eid)

    def on_scr_metric(self, msg: dict):
        """Called when a L5 SCR metric arrives."""
        eid = msg['element_id']
        self._l5_buf[eid] = dict(msg)
        self._maybe_flush(eid)

    # ── Internal ──────────────────────────────────────────────────────────────

    def _maybe_flush(self, eid: int):
        """Emit a combined row when both L4 and L5 data exist for element eid."""
        if eid not in self._l4_buf or eid not in self._l5_buf:
            return

        row = {}
        row.update(self._l4_buf.pop(eid))
        row.update(self._l5_buf.pop(eid))

        row['wall_time_s']  = round(time.monotonic() - self._start, 3)
        row['quorum_held']  = int(row.get('quorum_held', False))
        row['degraded']     = int(row.get('degraded',    False))

        self._writer.writerow(row)

    def close(self):
        self._file.close()
