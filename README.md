# Tapestry

An open source operating system framework for physically reconfigurable matter.
For more details, refer to the [project documentation](https://tapestry-os.com/documentation).

## Vision

Tapestry is a scale-invariant system software framework that provides
programmability over physically reconfigurable matter: distributed collections
of elements that sense, move, and coordinate without central control.

Its architecture draws from modern operating systems and edge compute platforms to address a spectrum of heterogeneous collective systems from drone swarms and warehouse automation
in the near term, to microrobotics and precision agriculture in the mid term, to
smart materials, surgical nanobots, and molecular machines on a 15–20 year
horizon.

The landscape of programmable physical systems is fragmented. Every
research group reinvents the control plane from scratch. There is
no agreed-upon abstraction boundary between physical elements
and the software that programs them. That gap is where Tapestry lives.

Just as Android abstracted over diverse mobile hardware so that developers could
target a single API regardless of the underlying silicon, Tapestry abstracts over
diverse element hardware, communication substrates, and physical actuation
mechanisms: enabling a new class of developer to directly program
physical reality.

## The Tapestry Stack — Seven Layers

Tapestry is organized as a seven-layer stack, deliberately mirroring the
separation of concerns in traditional OS design while incorporating the unique
requirements of physical element systems.

```
  L7  Choreographer                 High-level intent programming; developer-facing API
  L6  Behavior Synthesis Engine    Translates intent into collective behavioral plans
  L5  Swarm Coordination Runtime   Quorum-based coordination, role assignment, lightweight peer-filtering BFT mitigation (whitelist + anomaly exclusion)
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
  socket API (`zsock_*`) used as the L3 transport primitive in simulation and on hardware.

**What Zephyr does not provide** — and where Tapestry begins:

Zephyr is a single-element OS. It has no concept of other elements, a shared
physical world state, collective objectives, quorum, or partition tolerance.
Everything from L3 upward is Tapestry's responsibility. The boundary is made
explicit in code: `#include <tapestry/csm.h>` does not pull in any Zephyr types and
compiles cleanly against any C99 toolchain.

**Portability:** The pure C99 core (`tapestry-os/subsys/csm/`, `tapestry-os/subsys/scr/`)
and wire protocol (`tapestry/wire.h`) compile against any C99 toolchain with no Zephyr
dependency. The transport implementations in `tapestry-os/subsys/transport/` use Zephyr's
`zsock_*` and `bt_*` APIs and require porting alongside the board adaptation layer
(`net_init.c`, `main.c`) when targeting a different RTOS. As physical scale decreases toward
micro and nanoscale targets, L1–L2 implementations will evolve — potentially to custom ASICs
or biochemical state machines — while L3–L7 remain stable.

## L3 — Inter-Element Communication

L3 is implemented by two transport backends, both in `tapestry-os/subsys/transport/`:

- **UDP broadcast** (`udp/udp_gossip.c`) — each element broadcasts its state to the subnet
  on a fixed port; any element within the same L2 segment receives it with no addressing
  or routing configuration. Used by the simulation harness and the WiFi/Ethernet hardware
  elements (ESP32, RA8D1).
- **BLE advertising** (`ble/ble_gossip.c`) — each element advertises its state in a
  non-connectable BLE manufacturer-specific record; any element within radio range (~10 m)
  receives it passively. Used by micro:bit elements and as a secondary transport on the
  ESP32 bridge element.

The on-wire frame format for both transports is defined in `tapestry/wire.h`
(`tapestry_gossip_frame_t`, 19 bytes packed). Both transports feed received frames
directly into `wm_receive_gossip()` — L4 and above see no difference.

Both are single-hop transports. A future L3 evolution would replace them with a true
multi-hop mesh protocol (e.g. 802.15.4, BLE mesh, or custom RF) to remove range and
infrastructure dependencies — without touching L4 or above.

## L4 — Collective State Manager

The Collective State Manager (CSM) maintains the
distributed world model — the shared understanding of the system's physical state
that no single element could maintain alone.

Each element maintains a local world model: a fixed-size table of peer states
propagated by gossip and kept consistent via Lamport logical clocks. Key
properties:

- Gossip-based state propagation with configurable staleness thresholds
- Lamport clock ordering and partition-aware reconciliation
- A continuous `consistency_bias` dial from pure AP (always available, keep
  moving during partition) to pure CP (always consistent, freeze on quorum loss), 
  extending the traditional binary mode switch
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
| `HEALTHY` | fresh peers ≥ `quorum_target` | Full quorum consensus available (quorum-based coordination; not formal BFT) |
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
├── tapestry-os/                   Tapestry OS framework
│   ├── include/tapestry/
│   │   ├── csm.h                  L4 public API boundary (pure C99, no OS deps)
│   │   ├── scr.h                  L5 public API boundary (includes csm.h)
│   │   └── wire.h                 L3 on-wire frame format (pure C99, no OS deps)
│   ├── subsys/csm/
│   │   ├── state.h                Core types: element_state_t, position_t, …
│   │   ├── world_model.h          CSM internal API
│   │   └── world_model.c          CSM implementation (pure C99)
│   ├── subsys/scr/
│   │   ├── scr.h                  SCR internal API: scr_state_t, roles, quorum
│   │   └── scr.c                  SCR implementation (pure C99)
│   └── subsys/transport/
│       ├── ble/
│       │   ├── ble_gossip.h       BLE advertising gossip transport API
│       │   └── ble_gossip.c       BLE implementation (Zephyr bt_* API)
│       └── udp/
│           ├── udp_gossip.h       UDP broadcast gossip transport API
│           └── udp_gossip.c       UDP implementation (Zephyr zsock_* API)
│
├── tapestry-csm-sim/              L4 simulation harness
│   ├── sim_protocol.h             Sim-only additions: ports, control protocol,
│   │                              compat aliases over <tapestry/wire.h>
│   ├── tests/                     ztest unit tests for L4
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   └── src/main.c
│   ├── zephyr/element/            Zephyr native_sim element application
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   └── src/
│   │       ├── main.c             Main loop: gossip, tick, movement, metrics
│   │       ├── comms.c/h          Sim-specific UDP transport (loopback → orchestrator)
│   │       └── movement.c/h       Random walk + peer repulsion
│   └── orchestrator/              Python asyncio gossip broker + telemetry
│       ├── main.py                Entry point: launches elements, runs scenario
│       ├── broker.py              Routes gossip by partition island, injects
│       │                          ground-truth position error into metrics
│       ├── protocol.py            Python mirror of wire.h + sim_protocol.h structs
│       ├── telemetry.py           Per-cycle CSV writer
│       ├── scenarios.py           Timed partition/power injection scripts
│       └── plot.py                5-panel matplotlib efficacy visualiser
│
├── tapestry-scr-sim/              L5 simulation harness
│   ├── scr_protocol.h             Sim-only compat aliases over <tapestry/wire.h>
│   ├── tests/                     ztest unit tests for L5
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   └── src/main.c
│   ├── zephyr/element/            Zephyr native_sim element (L4 + L5)
│   │   ├── CMakeLists.txt
│   │   ├── prj.conf
│   │   └── src/
│   │       ├── main.c             Main loop: gossip, L4 tick, L5 SCR tick, metrics
│   │       └── comms_scr.c/h      SCR metric send (extends csm-sim comms)
│   └── orchestrator/              Python asyncio orchestrator for L5
│       ├── main.py                Entry point; --quorum-min, --quorum-target, --bias
│       ├── broker.py              Gossip routing + SCR metric dispatch
│       ├── protocol.py            Python mirror of wire.h + scr_protocol.h structs
│       ├── telemetry.py           Combined L4+L5 CSV writer (one row per element/cycle)
│       ├── scenarios.py           L4 scenarios + leader_loss, cascade
│       └── plot.py                5-panel SCR visualiser: quorum, agreement, roles
│
└── tapestry-scr-hw/               Hardware swarm firmware (Phase 1 + Phase 2)
    ├── README.md                  Build, flash, and telemetry instructions
    ├── Kconfig                    Element configuration (ID, ports, WiFi, ORCH_IP)
    ├── src/
    │   ├── main.c                 Element main loop (L4 + L5); wires OS transport
    │   │                          and adaptation layers together
    │   └── net_init.c/h           WiFi / Ethernet bring-up (Zephyr net_mgmt)
    └── telemetry/
        ├── collect.py             UDP metric collector → CSV
        ├── protocol.py            Python mirror of wire.h structs
        └── plot.py                5-panel hardware telemetry visualiser
```

## Architectural boundary

The public includes are `tapestry/csm.h`, `tapestry/scr.h`, and `tapestry/wire.h`.
None contains Zephyr or OS-specific types. The include hierarchy is:

```
<tapestry/scr.h>          L4 + L5 surface (firmware that runs both layers)
  └── <tapestry/csm.h>    L4 surface only
        ├── subsys/csm/state.h
        └── subsys/csm/world_model.h

<tapestry/wire.h>         L3 on-wire frame format (pure C99; no OS deps)
  ↑ included by:
    subsys/transport/ble/ble_gossip.h   (Zephyr bt_* dependent)
    subsys/transport/udp/udp_gossip.h   (Zephyr zsock_* dependent)
```

`tapestry-os/subsys/csm/` and `tapestry-os/subsys/scr/` are pure C99 and
compile against any toolchain. `tapestry-os/subsys/transport/` contains the
Zephyr-dependent transport implementations; porting to a different RTOS requires
replacing those files and the board adaptation layer (`net_init.c`, `main.c`),
while the CSM, SCR, and wire protocol are unchanged.

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

## Quickstart — L5 swarm sim in under 5 minutes

After completing setup above, run these commands from the workspace root
(`tapestry-workspace/`). They build the simulation element binary, launch a
5-element swarm through the `leader_loss` scenario, and produce a telemetry
plot.

```bash
# 1. Build the L5 simulation element
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/element \
    tapestry/tapestry-scr-sim/zephyr/element

# 2. Run the L5 orchestrator — 5 elements, 30 s, leader_loss scenario
cd tapestry/tapestry-scr-sim/orchestrator
python main.py --elements 5 --scenario leader_loss --duration 30 --out run.csv

# 3. Plot the results
python plot.py run.csv --out run.png
```

`run.png` shows five panels: quorum agreement across elements, elected leader
ID over time, per-element role transitions, fresh peer count, and L4 consistency
confidence. You should see all elements elect element 0 as leader, then
re-elect element 1 automatically when element 0 is partitioned at t≈5 s and
recover back to element 0 at t≈15 s.

To try the L4 consistency dial:

```bash
# AP mode: elements keep moving through the partition (bias=0.0, default)
python main.py --elements 5 --scenario leader_loss --duration 30 \
    --bias 0.0 --out ap.csv

# CP mode: elements freeze on quorum loss (bias=1.0)
python main.py --elements 5 --scenario leader_loss --duration 30 \
    --bias 1.0 --out cp.csv

python plot.py ap.csv cp.csv --labels AP CP --out ap_vs_cp.png
```

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

**L4 simulation element** — Zephyr native_sim binary consumed by the L4 Python
orchestrator:

```bash
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-csm-sim/build/element \
    tapestry/tapestry-csm-sim/zephyr/element
```

**L5 simulation element** — Zephyr native_sim binary that runs L4 gossip and
L5 SCR tick, consumed by the SCR Python orchestrator:

```bash
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/element \
    tapestry/tapestry-scr-sim/zephyr/element
```

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

## Hardware validation

The L4/L5 stack has been validated on physical hardware in two phases:

- **Phase 1** — the ztest suites run unchanged on real MCUs, confirming that
  `world_model.c` and `scr.c` are pure C99 and compile correctly for any
  Zephyr-supported target.
- **Phase 2** — a two-element swarm runs on real boards gossiping via UDP
  broadcast over a shared LAN, demonstrating partition detection (~1.5 s),
  autonomous leader election, and recovery with no simulation broker.

See [`tapestry-scr-hw/README.md`](tapestry-scr-hw/README.md) for specific hardware build,
flash, and telemetry instructions.

## License

Tapestry is released under the [Apache 2.0 License](LICENSE).

Copyright 2026 James V Steele.

## Acknowledgments

Tapestry was designed and developed by James V Steele.
[Claude](https://claude.ai) (Anthropic) was used as a development 
tool during initial implementation phases.
