"""
collect_serial.py — USB serial telemetry collector for Tapestry micro:bit swarm

Reads METRIC lines from one or two micro:bit USB serial ports and writes a
combined CSV compatible with tapestry-scr-hw/telemetry/plot.py.

Each micro:bit emits lines like:
    METRIC,<uptime_ms>,<elem_id>,<fresh_ratio>,<quorum_state>,<role>,
           <leader_id>,<election_count>,<mean_age_ms>

The host attaches its own wall-clock timestamp when each line arrives.
Columns not available from the serial protocol (active_total, etc.) are
filled with 0 / sentinel values so the same plot.py works for both
the UDP (ESP32/RA8D1) and BLE (micro:bit) data sets.

Usage (single board):
    python collect_serial.py --ports /dev/ttyACM0 --out mb_run.csv

Usage (two boards at once):
    python collect_serial.py --ports /dev/ttyACM0 /dev/ttyACM1 --out mb_run.csv

Dependencies:
    pip install pyserial
"""

import argparse
import csv
import logging
import sys
import time
import threading

import serial

log = logging.getLogger(__name__)

# ── CSV schema (subset of collect.py schema; plot.py reads only these) ────────

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

# ── METRIC line parser ────────────────────────────────────────────────────────

def parse_metric_line(line: str) -> dict | None:
    """
    Parse a METRIC CSV line from the firmware.
    Returns a dict with the known fields, or None if the line is malformed.
    """
    parts = line.strip().split(',')
    if len(parts) != 9 or parts[0] != 'METRIC':
        return None
    try:
        _, uptime_ms, elem_id, fresh_ratio, quorum_state, role, leader_id, \
            election_count, mean_age_ms = parts
        quorum_int = int(quorum_state)
        return {
            'element_id':          int(elem_id),
            'fresh_ratio':         float(fresh_ratio),
            'quorum_state':        quorum_int,
            'quorum_held':         1 if quorum_int >= 1 else 0,
            'degraded':            1 if quorum_int == 1 else 0,
            'role':                int(role),
            'leader_id':           int(leader_id),
            'election_count':      int(election_count),
            'mean_age_ms':         float(mean_age_ms),
            # Fields not available from BLE transport — filled with sentinels
            'active_total':        0,
            'active_fresh':        0,
            'active_stale':        0,
            'inactive_total':      0,
            'collision_count':     0,
            'confidence':          1.0,
            'cycle_count':         0,
            'mean_position_error': 0.0,
            'min_separation':      0,
            'fresh_count':         0,
        }
    except (ValueError, IndexError):
        return None

# ── Writer ─────────────────────────────────────────────────────────────────────

class TelemetryWriter:
    """Thread-safe CSV writer — called from per-port reader threads."""

    def __init__(self, path: str):
        self._file   = open(path, 'w', newline='', buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=COLUMNS,
                                      extrasaction='ignore')
        self._writer.writeheader()
        self._start  = time.monotonic()
        self._lock   = threading.Lock()

    def write_row(self, row: dict):
        row['wall_time_s'] = round(time.monotonic() - self._start, 3)
        with self._lock:
            self._writer.writerow(row)

    def close(self):
        self._file.close()

# ── Per-port reader thread ─────────────────────────────────────────────────────

def port_reader(port: str, baud: int, writer: TelemetryWriter,
                stop_event: threading.Event):
    """
    Opens a serial port and reads METRIC lines until stop_event is set.
    Designed to run in its own thread so two boards can be read in parallel.
    """
    log.info("opening %s at %d baud", port, baud)
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        log.error("cannot open %s: %s", port, exc)
        return

    try:
        while not stop_event.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                log.warning("%s read error: %s", port, exc)
                break

            if not raw:
                continue

            try:
                line = raw.decode('utf-8', errors='replace')
            except Exception:
                continue

            row = parse_metric_line(line)
            if row is not None:
                log.debug("%s: elem=%d fresh=%.2f quorum=%s role=%s",
                          port,
                          row['element_id'],
                          row['fresh_ratio'],
                          QUORUM_NAMES.get(row['quorum_state'], '?'),
                          ROLE_NAMES.get(row['role'], '?'))
                writer.write_row(row)
    finally:
        ser.close()
        log.info("%s closed", port)

# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Tapestry micro:bit serial telemetry collector')
    parser.add_argument('--ports', nargs='+', required=True,
                        help='serial port(s), e.g. /dev/ttyACM0 /dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=115200,
                        help='baud rate (default: 115200)')
    parser.add_argument('--out', default='mb_run.csv',
                        help='output CSV path (default: mb_run.csv)')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s',
        stream=sys.stderr,
    )

    writer     = TelemetryWriter(args.out)
    stop_event = threading.Event()

    threads = [
        threading.Thread(
            target=port_reader,
            args=(port, args.baud, writer, stop_event),
            daemon=True,
        )
        for port in args.ports
    ]

    for t in threads:
        t.start()

    log.info("collecting from %s  →  %s", ', '.join(args.ports), args.out)
    log.info("Ctrl+C to stop and save")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=3)
        writer.close()
        log.info("saved: %s", args.out)


if __name__ == '__main__':
    main()
