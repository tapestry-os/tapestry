"""
telemetry.py — Per-cycle CSV telemetry writer.

Writes one row per metric message received from any element.
The file is line-buffered so it survives an unclean shutdown.

CSV columns
───────────
    wall_time_s      seconds since orchestrator start (float, 3 dp)
    element_id
    active_total     elements currently marked active
    active_fresh     active elements with non-stale world model entries
    active_stale     active elements with stale entries
    inactive_total   elements presumed dead / unreachable
    collision_count  collisions detected this cycle
    fresh_ratio      active_fresh / active_total  [0.0 .. 1.0]
    quorum_held      1 if fresh_ratio >= effective quorum threshold
    degraded         1 if quorum lost (element below its threshold)
    confidence       proximity to quorum [0.0..1.0]; 1.0 = at or above quorum
    cycle_count      total world model cycles run by this element
"""

import csv
import time

COLUMNS = [
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
]


class TelemetryWriter:
    def __init__(self, path: str):
        self._path   = path
        self._file   = open(path, 'w', newline='', buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=COLUMNS,
                                      extrasaction='ignore')
        self._writer.writeheader()
        self._start  = time.monotonic()

    def write(self, metric: dict):
        """Write one metric snapshot row."""
        row = dict(metric)
        row['wall_time_s'] = round(time.monotonic() - self._start, 3)
        row['quorum_held'] = int(row.get('quorum_held', False))
        row['degraded']    = int(row.get('degraded',    False))
        self._writer.writerow(row)

    def close(self):
        self._file.close()
