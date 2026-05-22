"""
collect.py — hardware telemetry collector for Tapestry Phase 2

Listens on UDP for L4 CSM metric and L5 SCR metric packets sent by
physical elements.  Writes a combined CSV (one row per element per cycle)
using the same schema as tapestry-scr-sim so plot.py can visualize it.

Usage:
    python collect.py --out hw_run.csv

Elements send metrics to this machine's IP on port 5100.  Ensure
CONFIG_TAPESTRY_ORCH_IP is set to this machine's LAN IP in the firmware.

Exit with Ctrl+C — the CSV is flushed on every row so partial runs are
always readable.
"""

import asyncio
import argparse
import csv
import logging
import sys
import time

from protocol import decode, COLLECTOR_PORT

log = logging.getLogger(__name__)

# ── CSV schema (matches tapestry-scr-sim telemetry) ──────────────────────────

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
    'role',
    'leader_id',
    'quorum_state',
    'fresh_count',
    'election_count',
]

QUORUM_NAMES = {0: 'LOST', 1: 'DEGRADED', 2: 'HEALTHY'}
ROLE_NAMES   = {0: 'NONE', 1: 'FOLLOWER', 2: 'LEADER'}

# ── Telemetry writer ──────────────────────────────────────────────────────────

class TelemetryWriter:
    """
    Buffers L4 and L5 metric dicts per element.  Emits one combined CSV
    row when both metrics have arrived for the same element_id.
    """

    def __init__(self, path: str):
        self._file   = open(path, 'w', newline='', buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=COLUMNS,
                                      extrasaction='ignore')
        self._writer.writeheader()
        self._start  = time.monotonic()
        self._l4: dict[int, dict] = {}
        self._l5: dict[int, dict] = {}

    def on_metric(self, msg: dict):
        eid = msg['element_id']
        self._l4[eid] = msg
        self._flush(eid)

    def on_scr_metric(self, msg: dict):
        eid = msg['element_id']
        self._l5[eid] = msg
        self._flush(eid)

    def _flush(self, eid: int):
        if eid not in self._l4 or eid not in self._l5:
            return
        row = {}
        row.update(self._l4.pop(eid))
        row.update(self._l5.pop(eid))
        row['wall_time_s'] = round(time.monotonic() - self._start, 3)
        row['quorum_held'] = int(row.get('quorum_held', False))
        row['degraded']    = int(row.get('degraded', False))
        self._writer.writerow(row)

    def close(self):
        self._file.close()

# ── Metric collector ──────────────────────────────────────────────────────────

class Collector(asyncio.DatagramProtocol):

    def __init__(self, writer: TelemetryWriter):
        self._writer = writer

    def datagram_received(self, data: bytes, addr):
        msg = decode(data)
        if msg is None:
            return

        t = msg['type']
        if t == 'metric':
            self._writer.on_metric(msg)
            log.debug("L4  elem=%d fresh=%.2f quorum=%s",
                      msg['element_id'], msg['fresh_ratio'],
                      'held' if msg['quorum_held'] else 'lost')
        elif t == 'scr_metric':
            self._writer.on_scr_metric(msg)
            log.debug("L5  elem=%d %s leader=%d quorum=%s",
                      msg['element_id'],
                      ROLE_NAMES.get(msg['role'], '?'),
                      msg['leader_id'],
                      QUORUM_NAMES.get(msg['quorum_state'], '?'))

    def error_received(self, exc):
        log.warning("UDP error: %s", exc)

# ── Entry point ───────────────────────────────────────────────────────────────

async def run(host: str, port: int, out: str):
    writer = TelemetryWriter(out)
    loop   = asyncio.get_running_loop()

    transport, _ = await loop.create_datagram_endpoint(
        lambda: Collector(writer),
        local_addr=(host, port),
    )

    log.info("collecting on %s:%d  →  %s", host, port, out)
    log.info("waiting for elements — Ctrl+C to stop and save")

    try:
        await asyncio.Event().wait()   # run until interrupted
    finally:
        transport.close()
        writer.close()
        log.info("saved: %s", out)


def main():
    parser = argparse.ArgumentParser(
        description='Tapestry Phase 2 telemetry collector')
    parser.add_argument('--host', default='0.0.0.0',
                        help='listen address (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=COLLECTOR_PORT,
                        help=f'listen port (default: {COLLECTOR_PORT})')
    parser.add_argument('--out', default='hw_run.csv',
                        help='output CSV path (default: hw_run.csv)')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s',
        stream=sys.stderr,
    )

    try:
        asyncio.run(run(args.host, args.port, args.out))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
