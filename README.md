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
  socket API (`zsock_*`) used as the L3 transport primitive in simulation and on hardware.

**What Zephyr does not provide** — and where Tapestry begins:

Zephyr is a single-element OS. It has no concept of other elements, a shared
physical world state, collective objectives, quorum, or partition tolerance.
Everything from L3 upward is Tapestry's responsibility. The boundary is made
explicit in code: `#include <tapestry/csm.h>` does not pull in any Zephyr types and
compiles cleanly against any C99 toolchain.

**Portability:** replacing Zephyr with FreeRTOS, a bare-metal HAL, or any other OS requires
only rewriting the adaptation layer (`comms.c`, `main.c`). The CSM logic in
`tapestry-os/subsys/csm/` is unchanged. As physical scale decreases toward
micro and nanoscale targets, L1–L2 implementations will evolve — potentially
to custom ASICs or biochemical state machines — while L3–L7 remain stable.

## L3 — Inter-Element Communication

L3 is implemented as **UDP broadcast gossip** over the local network, running
on both the simulation harness and physical hardware.  Each element
broadcasts its state to the subnet on a fixed port; any element within the
same L2 segment receives it with no addressing or routing configuration.

This is a single-hop, infrastructure-dependent transport — elements must share
a broadcast domain.  It correctly validates the L4/L5 protocol stack on real
hardware and models the semantics of short-range mesh radio (any element within
range receives the transmission).  A future L3 evolution would replace UDP
broadcast with a true multi-hop mesh protocol (e.g. 802.15.4, BLE mesh, or
custom RF) to remove the infrastructure dependency and extend range beyond a
single L2 segment — without touching L4 or above.

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

## Phase 1 — Hardware validation

The L4 and L5 test suites run unchanged on physical hardware. Board-specific
`prj.conf` overlays in each `tests/boards/` directory supply the FPU and
stack-size settings needed; the test source and CMakeLists.txt are untouched.

This validates the core architectural claim: `world_model.c` and `scr.c` are
pure C99 and compile correctly for any Zephyr-supported MCU.

**Different and representative compute boards have been used to validate the franework.** Additional SDK toolchains required (install via `zephyr-sdk-setup.sh`):
- `xtensa-espressif_esp32_zephyr-elf` — for ESP-WROVER-KIT
- `arm-zephyr-eabi` — for EK-RA8D1

### ESP-WROVER-KIT (ESP32 / Xtensa LX6)

```bash
# Build L4 tests
west build -b esp_wrover_kit/esp32/procpu \
    --build-dir tapestry/tapestry-csm-sim/build/hw-esp-test-csm \
    tapestry/tapestry-csm-sim/tests

# Flash and monitor
west flash --build-dir tapestry/tapestry-csm-sim/build/hw-esp-test-csm
west espressif monitor --build-dir tapestry/tapestry-csm-sim/build/hw-esp-test-csm

# Build L5 tests
west build -b esp_wrover_kit/esp32/procpu \
    --build-dir tapestry/tapestry-scr-sim/build/hw-esp-test-scr \
    tapestry/tapestry-scr-sim/tests

west flash --build-dir tapestry/tapestry-scr-sim/build/hw-esp-test-scr
west espressif monitor --build-dir tapestry/tapestry-scr-sim/build/hw-esp-test-scr
```

### Renesas EK-RA8D1 (RA8D1 / Cortex-M85)

The EK-RA8D1 has a J-Link OB debugger on board. `west flash` uses it
automatically; connect the USB debug port before running.

```bash
# Build L4 tests
west build -b ek_ra8d1 \
    --build-dir tapestry/tapestry-csm-sim/build/hw-ra8d1-test-csm \
    tapestry/tapestry-csm-sim/tests

# Flash and open serial terminal (115200 8N1)
west flash --build-dir tapestry/tapestry-csm-sim/build/hw-ra8d1-test-csm
west debug --build-dir tapestry/tapestry-csm-sim/build/hw-ra8d1-test-csm

# Build L5 tests
west build -b ek_ra8d1 \
    --build-dir tapestry/tapestry-scr-sim/build/hw-ra8d1-test-scr \
    tapestry/tapestry-scr-sim/tests

west flash --build-dir tapestry/tapestry-scr-sim/build/hw-ra8d1-test-scr
```

Ztest output appears on the board's UART console (USB CDC-ACM or the
J-Link virtual COM port at 115200 baud). A passing run ends with:
`PROJECT EXECUTION SUCCESSFUL`.

## Phase 2 — Hardware swarm validation

Two physical boards gossip **directly** over the LAN via UDP broadcast
with no broker in the path.  The ESP32 uses WiFi; the RA8D1 uses Ethernet;
both must connect to the same router so they share a single L2 segment
(standard home routers bridge WiFi and Ethernet ports correctly; mesh
systems vary).

```
[ESP32] ──UDP broadcast :5000──► [RA8D1]
[RA8D1] ──UDP broadcast :5000──► [ESP32]

[ESP32] ──UDP unicast──► collector :5100   (L4+L5 metrics → collect.py)
[RA8D1] ──UDP unicast──► collector :5100
```

This models real mesh-radio broadcast semantics: an element transmits its
state and any element within range receives it — no relay, no addressing.

The firmware automatically disables WiFi power save after connecting.
Without this, the AP buffers broadcast frames until the DTIM beacon
interval (potentially 10–20 s on some routers), which exceeds the
1500 ms world-model stale threshold and causes spurious LOST/HEALTHY
oscillation.

### Build — ESP-WROVER-KIT (element 0, WiFi)

Create `tapestry/tapestry-scr-hw/wifi.conf` from the example template and
fill in your network credentials:

```bash
cp tapestry/tapestry-scr-hw/wifi.conf.example \
   tapestry/tapestry-scr-hw/wifi.conf
# edit wifi.conf — wifi.conf is gitignored
```

Set `CONFIG_TAPESTRY_ORCH_IP` to the collector machine's LAN IP in `wifi.conf` to gather telemetry (not required in an actual deployment):

```
CONFIG_TAPESTRY_WIFI_SSID="your_ssid"
CONFIG_TAPESTRY_WIFI_PSK="your_password"
CONFIG_TAPESTRY_ORCH_IP="192.168.x.x"   # machine running collect.py
```

Build and flash:

```bash
west build -b esp_wrover_kit/esp32/procpu \
    --build-dir tapestry/tapestry-scr-hw/build/hw-esp \
    -s tapestry/tapestry-scr-hw \
    -- -DEXTRA_CONF_FILE="$(pwd)/tapestry/tapestry-scr-hw/wifi.conf"

west flash \
    --build-dir tapestry/tapestry-scr-hw/build/hw-esp \
    --esp-device /dev/ttyUSB1
```

### Build — EK-RA8D1 (element 1, Ethernet)

Set `CONFIG_TAPESTRY_ORCH_IP` to gather telemetry for the RA8D1 build on the command line:

```bash
west build -b ek_ra8d1 \
    --build-dir tapestry/tapestry-scr-hw/build/hw-ra8d1 \
    -s tapestry/tapestry-scr-hw \
    -- -DCONFIG_TAPESTRY_ORCH_IP='"192.168.x.x"'

west flash --build-dir tapestry/tapestry-scr-hw/build/hw-ra8d1
```

### Run the telemetry collector

On the laptop (must be on the same LAN as both boards):

```bash
cd tapestry/tapestry-scr-hw/telemetry
pip install pandas matplotlib   # one-time
python collect.py --out hw_run.csv
```

### Proof point — physical partition

1. Both boards running: `fresh_ratio = 1.0`, `quorum_state = HEALTHY`,
   leader = element 0 (lowest ID).
2. **Reset (POR) the ESP32** to simulate a partition (or move it out of communication range):
   - RA8D1 ages the ESP32 entry: fresh → stale → inactive (~1500 ms)
   - `quorum_state` drops HEALTHY → DEGRADED → LOST
   - RA8D1 assigns NONE role, `election_count` increments
3. **ESP32 reconnects** (after WiFi association + DHCP, ~5–10 s):
   - First gossip broadcast triggers `wm_receive_gossip()`
   - Lamport clock merge reconciles both world models
   - Quorum recovers, leader re-elected, `election_count` increments again

### Plot results

```bash
python plot.py hw_run.csv --out hw_run.png
```

The plot shows fresh ratio, quorum state, role, elected leader, and mean
peer age across both elements — the partition event appears as a cliff in
all five panels, and the recovery as a step back up.

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
