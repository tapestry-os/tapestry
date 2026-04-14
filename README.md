# Tapestry

An open source operating system framework for physically reconfigurable matter.

## Vision

Tapestry is a scale-invariant system software framework that provides
programmability over physically reconfigurable matter: distributed collections
of elements that sense, move, and coordinate without central control.

Its domain spans a new class of system where the compute substrate is not
silicon but physical reality itself — from drone swarms and warehouse automation
in the near term, to microrobotics and precision agriculture in the mid term, to
smart materials, surgical nanobots, and molecular machines on a 15–20 year
horizon.

The landscape of programmable physical systems is profoundly fragmented. Every
research group reinvents the control plane from scratch. There is no Android, no
Linux, no POSIX, indeed no agreed-upon abstraction boundary between physical elements
and the software that programs them. That gap is where Tapestry lives.

Just as Android abstracted over diverse mobile hardware so that developers could
target a single API regardless of the underlying silicon, Tapestry abstracts over
diverse element hardware, communication substrates, and physical actuation
mechanisms: enabling an entirely new class of developer to directly program
physical reality.

## The Tapestry Stack — Seven Layers

Tapestry is organized as a seven-layer stack, deliberately mirroring the
separation of concerns in traditional OS design while incorporating the unique
requirements of physical element systems.

```
  L7  Matter Application Layer     High-level intent programming; developer-facing API
  L6  Behavior Synthesis Engine    Translates intent into collective behavioral plans
  L5  Swarm Coordination Runtime   Distributed consensus, role assignment, fault tolerance
  L4  Collective State Manager     Distributed world model; aggregated shared state
  L3  Inter-Element Communication  Mesh networking, routing, encryption; substrate-agnostic
  L2  Element Runtime              Per-element OS: scheduling, power, actuation, local sensing
  L1  Physical Substrate Interface HAL motor drivers, sensor buses, communication transceivers
```

Layers L1–L2 run on individual elements. L3–L5 are collective distributed
functions. L6–L7 are the developer-facing programming surface.

A key design invariant: L6–L7 remain stable as physical scale decreases from
centimeters to nanometers, while L1–L5 implementations evolve. Application code
written today should run on nanoscale elements in 20 years — for the same reason
that swapping L1 from copper to fiber does not change how L7 applications work in
OSI networking.

## L1–L2 — Current Implementation (Zephyr RTOS)

Tapestry currently uses [Zephyr RTOS](https://docs.zephyrproject.org) as the
L1–L2 substrate, in the same way that Android uses Linux as its kernel: the
layers above it are unaware of and independent from the choice below.

**What Zephyr provides:**

- **L1 (PSI)** — devicetree-based hardware abstraction, peripheral drivers,
  communication transceiver support (BLE, UWB, UART). Physical capabilities are
  declared, not hard-coded.
- **L2 (Element Runtime)** — priority-based task scheduler with deterministic
  interrupt latency, power state management (active/idle/sleep), and a POSIX
  socket API (`zsock_*`) used as the L3 transport primitive in simulation.

**What Zephyr does not provide** — and where Tapestry begins:

Zephyr is a single-element OS. It has no concept of other elements, a shared
physical world state, collective objectives, quorum, or partition tolerance.
Everything from L3 upward is Tapestry's responsibility. The boundary is made
explicit in code: `#include <tapestry/csm.h>` pulls in no Zephyr types and
compiles cleanly against any C99 toolchain.

**Portability:** replacing Zephyr with FreeRTOS, a bare-metal HAL, or any other OS requires
only rewriting the adaptation layer (`comms.c`, `main.c`). The CSM logic in
`tapestry-os/subsys/csm/` is unchanged. As physical scale decreases toward
micro and nanoscale targets, L1–L2 implementations will evolve — potentially
to custom ASICs or biochemical state machines — while L3–L7 remain stable.

## L3 - Inter-Element Communication

The mechanism for elements to communicate with each other is currently a simulation scaffold.
It will eventually need to be replaced with a real mesh implementation before running on physical hardware. 

## L4 — Collective State Manager

The Collective State Manager (CSM) is the most technically novel layer in Tapestry. It maintains the
distributed world model — the shared understanding of the system's physical state
that no single element could maintain alone.

Each element maintains a local world model: a fixed-size table of peer states
propagated by gossip and kept consistent via Lamport logical clocks. Key
properties:

- Gossip-based state propagation with configurable staleness thresholds
- Lamport clock ordering and partition-aware reconciliation
- A continuous `consistency_bias` dial from pure AP (always available, keep
  moving during partition) to pure CP (always consistent, freeze on quorum loss), replacing the
  traditional binary mode switch
- Per-cycle efficacy metrics: fresh ratio, confidence, mean age, collision
  detection, min separation

The public include is `#include <tapestry/csm.h>`. The implementation in
`tapestry-os/subsys/csm/` is pure C99 with no OS dependencies.

### Consistency bias

The `CONSISTENCY_BIAS` environment variable (float, 0.0–1.0) sets each element's
position on the AP/CP spectrum at startup. At 0.0 the element never freezes. At
1.0 it freezes when fewer than `WM_QUORUM_FRACTION` of known peers are fresh.
Intermediate values scale the quorum threshold linearly, producing a degraded
state rather than a hard switch.

### Telemetry columns

| Column | Description |
|---|---|
| `fresh_ratio` | Fraction of active peers with non-stale world model entries |
| `degraded` | 1 when element is below its quorum threshold |
| `confidence` | Proximity to quorum threshold [0.0, 1.0] |
| `mean_age_ms` | Average age of active peer entries (ms) |
| `mean_position_error` | Mean distance between believed and actual peer positions (broker ground truth) |
| `min_separation` | Closest peer distance this cycle (safety proxy) |
| `collision_count` | Peers within MIN\_SEPARATION (3.0 units) this cycle |

## L5 — Swarm Coordination Runtime

The Swarm Coordination Runtime (SCR) sits above the CSM and provides two
collective services that L4 deliberately does not attempt:

**Quorum management** — classifies swarm health into three levels based on
the number of fresh (non-stale) peers visible in the current world model
snapshot:

| State | Condition | Meaning |
|---|---|---|
| `HEALTHY` | fresh peers ≥ `quorum_target` | Full consensus available |
| `DEGRADED` | fresh peers ≥ `quorum_min` | Proceed with reduced confidence |
| `LOST` | fresh peers < `quorum_min` | Cannot form reliable consensus |

`quorum_min` and `quorum_target` are peer counts (not fractions) supplied
at init time. This separates domain knowledge about expected swarm size from
the L4 protocol mechanics.

**Role election** — every element independently elects the same leader
without extra messages. The fresh peer (including self) with the lowest
`element_id` is the leader. All elements compute this identically from their
converged world model; convergence time is bounded by `WM_STALE_THRESHOLD_MS`
(1500 ms). When a leader's entry goes stale the next `scr_tick()` automatically
elects from the remaining fresh set.

Roles: `SCR_ROLE_LEADER`, `SCR_ROLE_FOLLOWER`, `SCR_ROLE_NONE` (quorum lost).

The public include is `#include <tapestry/scr.h>`, which transitively
includes `<tapestry/csm.h>`. The implementation in `tapestry-os/subsys/scr/`
is pure C99 with no OS dependencies.

## Repository layout

```
tapestry/
├── tapestry-os/                   Tapestry OS framework (pure C99, no OS dependencies)
│   ├── include/tapestry/
│   │   ├── csm.h                  L4 public API boundary
│   │   └── scr.h                  L5 public API boundary (includes csm.h)
│   ├── subsys/csm/
│   │   ├── state.h                Core types: element_state_t, position_t, …
│   │   ├── world_model.h          CSM internal API
│   │   └── world_model.c          CSM implementation (pure C99, no OS deps)
│   └── subsys/scr/
│       ├── scr.h                  SCR internal API: scr_state_t, roles, quorum
│       └── scr.c                  SCR implementation (pure C99, no OS deps)
│
├── tapestry-csm-sim/              L4 simulation harness
│   ├── sim_protocol.h             Wire format shared by C elements and Python
│   ├── tests/                     ztest unit tests for L4
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   └── src/main.c
│   ├── zephyr/element/            Zephyr native_sim element application
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   └── src/
│   │       ├── main.c             Main loop: gossip, tick, movement, metrics
│   │       ├── comms.c/h          UDP transport (zsock_* API)
│   │       └── movement.c/h       Random walk + peer repulsion
│   └── orchestrator/              Python asyncio gossip broker + telemetry
│       ├── main.py                Entry point: launches elements, runs scenario
│       ├── broker.py              Routes gossip by partition island, injects
│       │                          ground-truth position error into metrics
│       ├── protocol.py            Python mirror of sim_protocol.h wire structs
│       ├── telemetry.py           Per-cycle CSV writer
│       ├── scenarios.py           Timed partition/power injection scripts
│       └── plot.py                5-panel matplotlib efficacy visualiser
│
└── tapestry-scr-sim/              L5 simulation harness
    ├── scr_protocol.h             Wire format additions for L5 SCR metric
    ├── tests/                     ztest unit tests for L5
    │   ├── CMakeLists.txt
    │   ├── prj.conf
    │   └── src/main.c
    ├── zephyr/element/            Zephyr native_sim element (L4 + L5)
    │   ├── CMakeLists.txt
    │   ├── prj.conf
    │   └── src/
    │       ├── main.c             Main loop: gossip, L4 tick, L5 SCR tick, metrics
    │       └── comms_scr.c/h      SCR metric send (extends csm-sim comms)
    └── orchestrator/              Python asyncio orchestrator for L5
        ├── main.py                Entry point; --quorum-min, --quorum-target, --bias
        ├── broker.py              Gossip routing + SCR metric dispatch
        ├── protocol.py            Python mirror of sim_protocol.h + scr_protocol.h
        ├── telemetry.py           Combined L4+L5 CSV writer (one row per element/cycle)
        ├── scenarios.py           L4 scenarios + leader_loss, cascade
        └── plot.py                5-panel SCR visualiser: quorum, agreement, roles
```

## Architectural boundary

`tapestry/csm.h` and `tapestry/scr.h` are the framework's public includes.
Neither contains Zephyr or OS-specific types. The include hierarchy is:

```
<tapestry/scr.h>          L4 + L5 surface (element firmware that runs both layers)
  └── <tapestry/csm.h>    L4 surface only (adaptation layer, movement, comms)
        ├── subsys/csm/state.h
        └── subsys/csm/world_model.h
```

The adaptation layer (Zephyr main loop, zsock transport) lives exclusively in
`tapestry-csm-sim/zephyr/element/src/` and is replaceable per platform without
touching the CSM or SCR logic.

This boundary is intentional and load-bearing. Future ports to other RTOS targets
(FreeRTOS, bare-metal) require only a new adaptation layer, while `tapestry-os/`
compiles unchanged.

The monorepo maintains a hard directory boundary between `tapestry-os/` (OS
logic) and the simulation harnesses. The path to independent repositories is
preserved via `git subtree split` and `west.yml`.

## Getting started

**Prerequisites:** [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html) 0.17.0+, Python ≥ 3.11. Tested on Raspberry Pi aarch64 and Ubuntu 22.04 (Zephyr 4.4.0-rc1).

```bash
# 1. Initialise west workspace
west init -m https://github.com/tapestry-os/tapestry --mr main tapestry-workspace
cd tapestry-workspace
west update
west zephyr-export

# 2. Python dependencies for the simulation orchestrators
cd tapestry
python3 -m venv .venv && source .venv/bin/activate
pip install pandas matplotlib
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for full setup details.

## Building

All `west build` commands run from the workspace root (`tapestry-workspace/`).

**L4 unit tests** — gossip, Lamport clocks, staleness, quorum, reconciliation,
spatial queries:

```bash
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-csm-sim/build/test-csm \
    tapestry/tapestry-csm-sim/tests
./tapestry/tapestry-csm-sim/build/test-csm/zephyr/zephyr.exe
```

**L5 unit tests** — quorum classification, leader election, partition/heal,
stale and expired peer exclusion, re-election on leader loss:

```bash
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/tests \
    tapestry/tapestry-scr-sim/tests
./tapestry/tapestry-scr-sim/build/tests/zephyr/zephyr.exe
```

**L5 simulation element** — Zephyr native_sim binary that runs L4 gossip and
L5 SCR tick, consumed by the SCR Python orchestrator:

```bash
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/element \
    tapestry/tapestry-scr-sim/zephyr/element
```

**L4 simulation element** — Zephyr native_sim binary consumed by the L4 Python
orchestrator:

```bash
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-csm-sim/build/element \
    tapestry/tapestry-csm-sim/zephyr/element
```

## Running the L5 simulation

```bash
cd tapestry-scr-sim/orchestrator

# Leader loss — partition element 0 (initial leader) at t=5s, heal at t=15s
python main.py --elements 5 --scenario leader_loss --duration 30 --out leader_loss.csv

# Cascade — sequential leader elimination
python main.py --elements 5 --scenario cascade --duration 30 --out cascade.csv

# Compare the two
python plot.py leader_loss.csv cascade.csv --labels "Leader loss" Cascade --out scr_compare.png
```

Available scenarios: `leader_loss`, `cascade`, `default`, `flapping`, `asymmetric`, `sleep`.

The `--quorum-min` and `--quorum-target` flags control the SCR quorum thresholds
(peer counts, not fractions). The `--bias` flag sets the L4 consistency dial.

### SCR telemetry columns

| Column | Description |
|---|---|
| `role` | 0=NONE, 1=FOLLOWER, 2=LEADER |
| `leader_id` | Elected leader element ID; 255 = no leader (quorum LOST) |
| `quorum_state` | 0=LOST, 1=DEGRADED, 2=HEALTHY |
| `fresh_count` | Non-self fresh peers visible this cycle |
| `election_count` | Cumulative leader changes since element startup |

Plus all L4 telemetry columns (see L4 section above).

## Running the L4 simulation

```bash
cd tapestry-csm-sim/orchestrator

# AP mode (bias=0.0, default) — elements keep moving through partitions
python main.py --elements 5 --scenario default --duration 60 --out ap_run.csv

# CP mode (bias=1.0) — elements freeze on quorum loss
CONSISTENCY_BIAS=1.0 python main.py --elements 5 --scenario default --duration 60 --out cp_run.csv

# Compare
python plot.py ap_run.csv cp_run.csv --labels AP CP --out ap_vs_cp.png
```

Available scenarios: `default`, `flapping`, `asymmetric`, `sleep`.
