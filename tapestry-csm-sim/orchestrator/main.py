"""
main.py — Tapestry CSM simulation orchestrator entry point.

Usage
─────
    python main.py [options]

    --elements N        number of element processes          (default: 5)
    --scenario NAME     scenario to run                      (default: default)
    --duration S        total simulation duration in seconds (default: 30)
    --element-bin PATH  path to element zephyr.exe
                        (default: ../build/element/zephyr/zephyr.exe)
    --out FILE          telemetry CSV output path            (default: telemetry.csv)

Available scenarios: default, flapping, sleep, asymmetric
    (see orchestrator/scenarios.py for definitions)

The orchestrator:
    1. Starts the gossip broker (UDP server on ORCH_PORT).
    2. Spawns N element processes, each passed ELEMENT_ID and ORCH_PORT
       via environment variables.
    3. Runs the selected scenario (timed partition / power events).
    4. Writes per-cycle metrics from every element to a CSV.
    5. Terminates all element processes and closes the CSV on exit.
"""

import argparse
import asyncio
import logging
import os
import subprocess
import sys

from broker    import GossipBroker
from telemetry import TelemetryWriter
from scenarios import run as run_scenario, available as available_scenarios
from protocol  import ORCH_PORT, ELEMENT_BASE_PORT

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(name)s: %(message)s',
)
log = logging.getLogger(__name__)

DEFAULT_ELEMENT_BIN = '../build/element/zephyr/zephyr.exe'


# ── Argument parsing ──────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Tapestry CSM simulation orchestrator',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--elements',     type=int,   default=5,
                   help='Number of element processes')
    p.add_argument('--scenario',     type=str,   default='default',
                   choices=available_scenarios(),
                   help='Scenario to run')
    p.add_argument('--duration',     type=float, default=30.0,
                   help='Total simulation duration in seconds')
    p.add_argument('--element-bin',  type=str,   default=DEFAULT_ELEMENT_BIN,
                   dest='element_bin',
                   help='Path to element zephyr.exe')
    p.add_argument('--out',          type=str,   default='telemetry.csv',
                   help='Telemetry CSV output path')
    return p.parse_args()


# ── Process management ────────────────────────────────────────────────────────

def spawn_elements(n: int, element_bin: str) -> list:
    procs = []
    for i in range(n):
        env = {
            **os.environ,
            'ELEMENT_ID': str(i),
            'ORCH_PORT':  str(ORCH_PORT),
        }
        log.info("spawning element %d (port %d)", i, ELEMENT_BASE_PORT + i)
        p = subprocess.Popen(
            [element_bin],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        procs.append(p)
    return procs


def terminate_elements(procs: list):
    for p in procs:
        if p.poll() is None:
            p.terminate()
    for p in procs:
        try:
            p.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            log.warning("element pid %d did not exit — killing", p.pid)
            p.kill()


# ── Async main ────────────────────────────────────────────────────────────────

async def async_main(args):
    telemetry = TelemetryWriter(args.out)

    async def on_metric(msg):
        telemetry.write(msg)

    broker = GossipBroker(n_elements=args.elements, metric_cb=on_metric)
    await broker.start()

    # Give the broker socket time to bind before elements try to connect
    await asyncio.sleep(0.3)

    procs = spawn_elements(args.elements, args.element_bin)

    # Give elements time to initialize their sockets before the scenario fires
    await asyncio.sleep(1.0)

    log.info(
        "simulation started — %d elements, scenario '%s', %.0f s",
        args.elements, args.scenario, args.duration,
    )

    try:
        await run_scenario(args.scenario, broker, args.elements, args.duration)
    except asyncio.CancelledError:
        pass
    finally:
        log.info("shutting down")
        terminate_elements(procs)
        telemetry.close()
        broker.stop()

    log.info("telemetry written to %s", args.out)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    if not os.path.exists(args.element_bin):
        log.error("element binary not found: %s", args.element_bin)
        log.error("build it with:  west build -b native_sim/native/64 zephyr/element")
        sys.exit(1)

    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        log.info("interrupted by user")


if __name__ == '__main__':
    main()
