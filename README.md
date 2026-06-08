# Tapestry

An open-source operating system framework for physically reconfigurable matter.
For full documentation, see [tapestry-os.com/docs](https://tapestry-os.com/docs/).

## Vision

Building a swarm of physical elements — robots, drones, exoskeletons, or whatever comes next —
means solving the same coordination problems every time. Gossip protocols, partition tolerance,
leader election, shared world state: every project rebuilds this from scratch. There's no
agreed-upon line between where the hardware ends and the software begins.

Tapestry draws that line by applying separation of concerns to a domain that has never had it.
When the physical substrate is encapsulated below a stable API, developers can write applications
that coordinate across many elements — formations, distributed sensing, adaptive structures —
without caring whether those elements are drones, microrobots, or something that hasn't been built
yet. That's portability, and something more: a new class of collective software, with Tapestry
as the platform to write it.

## Architecture — Seven Layers

```
  L7  Choreographer                High-level intent programming; developer-facing API
  L6  Behavior Synthesis Engine    Translates intent into collective behavioral plans
  L5  Swarm Coordination Runtime   Quorum-based coordination, role assignment, lightweight BFT
  L4  Collective State Manager     Distributed world model; aggregated shared state
  L3  Inter-Element Communication  Mesh networking, routing, encryption; substrate-agnostic
  L2  Element Runtime              Per-element OS: scheduling, power, actuation, local sensing
  L1  Physical Substrate Interface HAL motor drivers, sensor buses, communication transceivers
```

L1–L5 execute on each physical element; L6–L7 execute on external compute (developer workstation, edge node, or cloud) or a sufficiently capable element within the collective. 
The public API (L6–L7) is designed to remain stable as
physical scale decreases from centimeters to nanometers.

The current L1–L2 runtime is built on the [Zephyr RTOS](https://docs.zephyrproject.org), used in the same
way Android uses Linux. Everything from L3 upward is Tapestry's sole responsibility with no Zephyr
dependency.

## Quickstart — L5 swarm sim in under 5 minutes

See [CONTRIBUTING.md](CONTRIBUTING.md) for prerequisites and workspace setup, then run from the
workspace root (`tapestry-workspace/`):

```bash
# Build the L5 simulation element
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/element \
    tapestry/tapestry-scr-sim/zephyr/element

# Run a 5-element swarm through the leader_loss scenario
cd tapestry/tapestry-scr-sim/orchestrator
python main.py --elements 5 --scenario leader_loss --duration 30 --out run.csv

# Plot the results
python plot.py run.csv --out run.png
```

`run.png` shows five panels: fleet-mean quorum state (LOST/DEGRADED/HEALTHY), fraction of
elements agreeing on the same leader, elements with LEADER role, fleet-mean fresh peer count,
and minimum element separation. You should see all elements elect element 0 as leader, then
re-elect element 1 at t≈7.5 s when the partition fires at t=5 s, then recover back to element 0
at t≈16.5 s when the partition heals at t=15 s.

To explore the L4 consistency dial (AP vs. CP tradeoff):

```bash
# Build the L4 simulation element
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-csm-sim/build/element \
    tapestry/tapestry-csm-sim/zephyr/element

cd tapestry/tapestry-csm-sim/orchestrator

# AP mode: elements keep moving through the partition (bias=0.0, default)
python main.py --elements 5 --scenario default --duration 60 --out ap.csv

# CP mode: elements freeze on quorum loss (bias=1.0)
CONSISTENCY_BIAS=1.0 python main.py --elements 5 --scenario default --duration 60 --out cp.csv

python plot.py ap.csv cp.csv --labels AP CP --out ap_vs_cp.png
```

In AP mode, no element ever sets the degraded flag; the fleet keeps moving and mean position
error peaks at 2.4 units during the partition. In CP mode, the minority island freezes in place —
40% of the fleet degrades for ~3.8 s — but position error peaks at only 1.9 units (20% lower).
Both modes fully reconverge by t≈20 s.

## Repository layout

```
tapestry/
├── sdk/                           L7 Choreographer SDK (stable developer surface)
│   ├── include/tapestry/
│   │   └── choreo.h               L7 SDK API header (Goal / lifecycle / directive)
│   ├── python/tapestry/
│   │   ├── choreo.py              L7 Python mirror
│   │   └── bse.py                 L6 Python stub
│   └── examples/
│       └── hello_swarm.py         Minimal worked example (no sim required)
│
├── tapestry-os/                   Tapestry OS framework
│   ├── include/tapestry/
│   │   ├── wire.h                 L3 on-wire frame format (pure C99, no OS deps)
│   │   ├── csm.h                  L4 public API boundary (pure C99, no OS deps)
│   │   ├── scr.h                  L5 public API boundary (includes csm.h)
│   │   ├── bse.h                  L6 interface contract (intent → directive)
│   │   ├── runtime.h              L2 element runtime (full-stack main loop helper)
│   │   ├── substrate.h            L1 HAL (motor, power, signal abstractions)
│   │   ├── transport.h            L3 transport API (send / drain / telemetry)
│   │   └── transceiver.h          L3 transceiver plugin interface
│   ├── boards/
│   │   └── bbc_microbit_v2/       micro:bit V2 board support (Cutebot HAL, BLE overlay)
│   ├── subsys/csm/
│   │   └── world_model.c          L4 CSM implementation (pure C99)
│   ├── subsys/scr/
│   │   └── scr.c                  L5 SCR implementation (pure C99)
│   ├── subsys/bse/
│   │   └── bse.c                  L6 BSE stub (geometry task decomposition)
│   ├── subsys/choreo/
│   │   └── choreo.c               L7 Choreographer stub (lifecycle + goal dispatch)
│   ├── subsys/runtime/
│   │   ├── runtime.c              L2 full-stack tick sequencer
│   │   └── power.c/.h             L2 power state machine (active/idle/sleep/harvest)
│   └── subsys/transport/
│       ├── transport.c            Transceiver registry and multiplexer
│       ├── gossip.c/.h            Wire framing, relay, HMAC auth
│       ├── transceiver_ble.c/.h   BLE advertising backend (Zephyr bt_* API)
│       ├── transceiver_udp.c/.h   UDP broadcast backend (Zephyr zsock_* API)
│       └── net_init.c/.h          WiFi / Ethernet bring-up (Zephyr net_mgmt)
│
├── tapestry-csm-sim/              L4 simulation harness
│   ├── sim_protocol.h             Sim-only additions: ports, control protocol,
│   │                              compat aliases over <tapestry/wire.h>
│   ├── tests/                     ztest unit tests for L4
│   ├── zephyr/element/            Zephyr native_sim element application
│   │   └── src/
│   │       ├── main.c             Main loop: gossip, tick, movement, metrics
│   │       ├── comms.c/h          Sim-specific UDP transport (loopback → orchestrator)
│   │       └── movement.c/h       Random walk + peer repulsion
│   └── orchestrator/              Python asyncio gossip broker + telemetry
│       ├── main.py                Entry point: launches elements, runs scenario
│       ├── broker.py              Routes gossip by partition island, injects ground-truth error
│       ├── protocol.py            Python mirror of wire.h + sim_protocol.h structs
│       ├── telemetry.py           Per-cycle CSV writer
│       ├── scenarios.py           Timed partition/power injection scripts
│       └── plot.py                5-panel matplotlib efficacy visualizer
│
├── tapestry-scr-sim/              L5 simulation harness
│   ├── tests/                     ztest unit tests for L5
│   ├── zephyr/element/            Zephyr native_sim element (L4 + L5)
│   │   └── src/
│   │       ├── main.c             Main loop: gossip, L4 tick, L5 SCR tick, metrics
│   │       └── comms_scr.c/h      SCR metric send (extends csm-sim comms)
│   └── orchestrator/              Python asyncio orchestrator for L5
│       ├── main.py                Entry point; --quorum-min, --quorum-target, --bias
│       ├── broker.py              Gossip routing + SCR metric dispatch
│       ├── protocol.py            Python mirror of wire.h + scr_protocol.h structs
│       ├── telemetry.py           Combined L4+L5 CSV writer (one row per element/cycle)
│       ├── scenarios.py           L4 scenarios + leader_loss, cascade
│       └── plot.py                5-panel SCR visualizer: quorum, agreement, roles
│
└── tapestry-scr-hw/               Hardware swarm firmware (Phase 1 + Phase 2)
    ├── README.md                  Build, flash, and telemetry instructions
    ├── Kconfig                    Element configuration (ID, ports, WiFi, ORCH_IP)
    ├── src/
    │   └── main.c                 Element main loop (L4 + L5); wires OS transport
    │                              and adaptation layers together
    └── telemetry/
        ├── collect.py             UDP metric collector → CSV
        ├── protocol.py            Python mirror of wire.h structs
        └── plot.py                5-panel hardware telemetry visualizer
```

## Building

All `west build` commands run from the workspace root (`tapestry-workspace/`).

### Unit tests

```bash
# L4 — gossip, Lamport clocks, staleness, quorum, reconciliation, spatial queries
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-csm-sim/build/test-csm \
    tapestry/tapestry-csm-sim/tests
./tapestry/tapestry-csm-sim/build/test-csm/zephyr/zephyr.exe

# L5 — quorum classification, leader election, partition/heal, re-election on leader loss
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/tests \
    tapestry/tapestry-scr-sim/tests
./tapestry/tapestry-scr-sim/build/tests/zephyr/zephyr.exe
```

### Simulation elements

```bash
# L4 element (consumed by the L4 Python orchestrator)
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-csm-sim/build/element \
    tapestry/tapestry-csm-sim/zephyr/element

# L5 element (runs L4 gossip + L5 SCR tick; consumed by the L5 Python orchestrator)
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/element \
    tapestry/tapestry-scr-sim/zephyr/element
```

## Simulation reference

### L4 — Collective State Manager

```bash
cd tapestry-csm-sim/orchestrator

python main.py --elements 5 --scenario default --duration 60 --out ap_run.csv
CONSISTENCY_BIAS=1.0 python main.py --elements 5 --scenario default --duration 60 --out cp_run.csv
python plot.py ap_run.csv cp_run.csv --labels AP CP --out ap_vs_cp.png
```

Available scenarios: `default`, `flapping`, `asymmetric`, `sleep`.

**Telemetry columns**

| Column | Description |
|---|---|
| `fresh_ratio` | Fraction of active peers with non-stale world model entries |
| `degraded` | 1 when element is below its quorum threshold |
| `confidence` | Proximity to quorum threshold [0.0, 1.0] |
| `mean_age_ms` | Average age of active peer entries (ms) |
| `mean_position_error` | Mean distance between believed and actual peer positions (broker ground truth) |
| `min_separation` | Closest peer distance this cycle (safety proxy) |
| `collision_count` | Peers within MIN\_SEPARATION (3.0 units) this cycle |

### L5 — Swarm Coordination Runtime

```bash
cd tapestry-scr-sim/orchestrator

python main.py --elements 5 --scenario leader_loss --duration 30 --out leader_loss.csv
python main.py --elements 5 --scenario cascade --duration 30 --out cascade.csv
python plot.py leader_loss.csv cascade.csv --labels "Leader loss" Cascade --out scr_compare.png
```

Available scenarios: `leader_loss`, `cascade`, `default`, `flapping`, `asymmetric`, `sleep`.
`--quorum-min` and `--quorum-target` set peer-count thresholds; `--bias` sets the L4 consistency dial.

**Telemetry columns** (in addition to all L4 columns above)

| Column | Description |
|---|---|
| `role` | 0=NONE, 1=FOLLOWER, 2=LEADER, 3=RELAY, 4=SENSOR, 5=ACTUATOR |
| `leader_id` | Elected leader element ID; 255 = no leader (quorum LOST) |
| `quorum_state` | 0=LOST, 1=DEGRADED, 2=HEALTHY |
| `fresh_count` | Non-self trusted fresh peers visible this cycle |
| `task_slot` | Ordinal in sorted fresh peer list (0 = leader); valid when quorum ≥ DEGRADED |
| `election_count` | Cumulative leader changes since element startup |

## Hardware validation

The L4/L5 stack has been validated on physical hardware in two phases:

- **Phase 1** — ztest suites run unchanged on real MCUs, confirming that `world_model.c` and
  `scr.c` are pure C99 and compile for any Zephyr-supported target.
- **Phase 2** — a two-element swarm gossiping via UDP broadcast over a shared LAN, demonstrating
  partition detection (~1.5 s), autonomous leader election, and recovery with no simulation broker.

See [`tapestry-scr-hw/README.md`](tapestry-scr-hw/README.md) for build, flash, and telemetry instructions.

## License

Tapestry is released under the [Apache 2.0 License](LICENSE).

Copyright 2026 James V Steele.

## Acknowledgments

Tapestry was designed and developed by James V Steele.
[Claude](https://claude.ai) (Anthropic) was used as a development tool during initial implementation phases.
