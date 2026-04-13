"""
main.py — Tapestry SCR simulation orchestrator entry point.

Usage
─────
    python main.py [options]

    --elements N        number of element processes          (default: 5)
    --scenario NAME     scenario to run                      (default: leader_loss)
    --duration S        total simulation duration in seconds (default: 30)
    --element-bin PATH  path to scr element zephyr.exe
                        (default: ../build/element/zephyr/zephyr.exe)
    --out FILE          telemetry CSV output path            (default: telemetry.csv)
    --quorum-min N      minimum fresh peers for DEGRADED     (default: 1)
    --quorum-target N   minimum fresh peers for HEALTHY      (default: 2)
    --bias FLOAT        L4 consistency_bias [0.0–1.0]        (default: 0.0)

Available scenarios: leader_loss, cascade, default, flapping, sleep, asymmetric

The orchestrator:
    1. Starts the gossip broker (UDP server on ORCH_PORT).
    2. Spawns N element processes, each given ELEMENT_ID, ORCH_PORT,
       CONSISTENCY_BIAS, QUORUM_MIN, QUORUM_TARGET via environment.
    3. Runs the selected scenario (timed partition / power events).
    4. Writes per-cycle combined L4+L5 telemetry to a CSV.
    5. Terminates all element processes on exit.
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


# ── Argument parsing ─────────────────────────────────────��────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Tapestry SCR simulation orchestrator',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--elements',      type=int,   default=5,
                   help='Number of element processes')
    p.add_argument('--scenario',      type=str,   default='leader_loss',
                   choices=available_scenarios(),
                   help='Scenario to run')
    p.add_argument('--duration',      type=float, default=30.0,
                   help='Total simulation duration in seconds')
    p.add_argument('--element-bin',   type=str,   default=DEFAULT_ELEMENT_BIN,
                   dest='element_bin',
                   help='Path to SCR element zephyr.exe')
    p.add_argument('--out',           type=str,   default='telemetry.csv',
                   help='Combined L4+L5 telemetry CSV output path')
    p.add_argument('--quorum-min',    type=int,   default=1,
                   dest='quorum_min',
                   help='Minimum fresh peers for DEGRADED quorum state')
    p.add_argument('--quorum-target', type=int,   default=2,
                   dest='quorum_target',
                   help='Minimum fresh peers for HEALTHY quorum state')
    p.add_argument('--bias',          type=float, default=0.0,
                   help='L4 consistency_bias [0.0=AP .. 1.0=CP]')
    return p.parse_args()


# ── Process management ──────────────────────────────────────────────────��─────

def spawn_elements(n: int, element_bin: str,
                   quorum_min: int, quorum_target: int,
                   bias: float) -> list:
    procs = []
    for i in range(n):
        env = {
            **os.environ,
            'ELEMENT_ID':       str(i),
            'ORCH_PORT':        str(ORCH_PORT),
            'CONSISTENCY_BIAS': str(bias),
            'QUORUM_MIN':       str(quorum_min),
            'QUORUM_TARGET':    str(quorum_target),
        }
        log.info("spawning element %d (port %d, qmin=%d qtgt=%d bias=%.2f)",
                 i, ELEMENT_BASE_PORT + i, quorum_min, quorum_target, bias)
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

    async def on_l4_metric(msg):
        telemetry.on_l4_metric(msg)

    async def on_scr_metric(msg):
        telemetry.on_scr_metric(msg)

    broker = GossipBroker(
        n_elements    = args.elements,
        metric_cb     = on_l4_metric,
        scr_metric_cb = on_scr_metric,
    )
    await broker.start()

    await asyncio.sleep(0.3)

    procs = spawn_elements(
        args.elements, args.element_bin,
        args.quorum_min, args.quorum_target, args.bias,
    )

    await asyncio.sleep(1.0)

    log.info(
        "simulation started — %d elements, scenario '%s', %.0f s, "
        "qmin=%d qtgt=%d bias=%.2f",
        args.elements, args.scenario, args.duration,
        args.quorum_min, args.quorum_target, args.bias,
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
        log.error("build it with:")
        log.error("  west build -b native_sim/native/64 "
                  "--build-dir ../build/element "
                  "tapestry-scr-sim/zephyr/element")
        sys.exit(1)

    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        log.info("interrupted by user")


if __name__ == '__main__':
    main()
