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

## Repository layout

```
tapestry/
├── tapestry-os/                   Tapestry OS framework (no sim dependencies)
│   ├── include/tapestry/
│   │   └── csm.h                  Public API boundary — consumers include this
│   ├── subsys/csm/
│   │   ├── state.h                Core types: element_state_t, position_t, …
│   │   ├── world_model.h          CSM internal API
│   │   └── world_model.c          CSM implementation (pure C99, no OS deps)
│   └── tests/csm/
│       └── src/main.c             ztest unit tests
│
└── tapestry-csm-sim/              Simulation harness (consumes tapestry-os)
    ├── sim_protocol.h             Wire format shared by C elements and Python
    ├── zephyr/element/            Zephyr native_sim element application
    │   └── src/
    │       ├── main.c             Main loop: gossip, tick, movement, metrics
    │       ├── comms.c/h          UDP transport (zsock_* API)
    │       └── movement.c/h      Random walk + peer repulsion
    └── orchestrator/              Python asyncio gossip broker + telemetry
        ├── main.py                Entry point: launches elements, runs scenario
        ├── broker.py              Routes gossip by partition island, injects
        │                          ground-truth position error into metrics
        ├── protocol.py            Python mirror of sim_protocol.h wire structs
        ├── telemetry.py           Per-cycle CSV writer
        ├── scenarios.py           Timed partition/power injection scripts
        └── plot.py                5-panel matplotlib efficacy visualiser
```

## Architectural boundary

`tapestry/csm.h` is the framework's public include. It contains no Zephyr or
OS-specific types. The adaptation layer (Zephyr main loop, zsock transport) lives
exclusively in `tapestry-csm-sim/zephyr/element/src/` and is replaceable per
platform without touching the CSM logic.

This boundary is intentional and load-bearing. Future ports to other RTOS targets
(FreeRTOS, bare-metal) require only a new adaptation layer, while `tapestry-os/` compiles unchanged.

The monorepo maintains a hard directory boundary between `tapestry-os/` (OS
logic) and `tapestry-csm-sim/` (test harness). The path to independent
repositories is preserved via `git subtree split` and `west.yml`.

## Building

Requires [Zephyr RTOS](https://docs.zephyrproject.org) with `west` and a Python
≥ 3.11 virtual environment. Tested on Raspberry Pi (aarch64, Zephyr 4.4.0-rc1).

```bash
# Unit tests
west build -b native_sim/native/64 \
    --build-dir tapestry-csm-sim/build/test-csm \
    tapestry-os/tests/csm
./tapestry-csm-sim/build/test-csm/zephyr/zephyr.exe

# Simulation element
west build -b native_sim/native/64 \
    --build-dir tapestry-csm-sim/build/element \
    tapestry-csm-sim/zephyr/element
```

## Running the simulation

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

### Consistency bias

The `CONSISTENCY_BIAS` environment variable (float, 0.0–1.0) sets each element's
position on the AP/CP spectrum at startup. At 0.0 the element never freezes. At
1.0 it freezes when fewer than `WM_QUORUM_FRACTION` of known peers are fresh. Intermediate values
scale the quorum threshold linearly, producing a degraded state rather than a
hard switch.

## Telemetry columns

| Column | Description |
|---|---|
| `fresh_ratio` | Fraction of active peers with non-stale world model entries |
| `degraded` | 1 when element is below its quorum threshold |
| `confidence` | Proximity to quorum threshold [0.0, 1.0] |
| `mean_age_ms` | Average age of active peer entries (ms) |
| `mean_position_error` | Mean distance between believed and actual peer positions (broker ground truth) |
| `min_separation` | Closest peer distance this cycle (safety proxy) |
| `collision_count` | Peers within MIN\_SEPARATION (3.0 units) this cycle |
