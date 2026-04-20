# Tapestry Hardware Validation

This directory contains the firmware and telemetry tooling for running the
Tapestry L4/L5 stack on physical hardware. Two phases are documented:

- **Phase 1** — run the L4 and L5 ztest suites on real MCUs to confirm the
  pure C99 core compiles and passes on non-native targets.
- **Phase 2** — run a two-element swarm where the boards gossip directly over
  the LAN, demonstrating partition detection, leader election, and recovery
  on real hardware with no simulation broker in the path.

All `west build` commands are run from the **workspace root**
(`tapestry-workspace/`).

---

## Phase 1 — Unit tests on hardware

The L4 and L5 ztest suites compile and run unchanged on physical boards.
Board-specific `prj.conf` overlays in each `tests/boards/` directory supply
FPU and stack-size settings; the test source and CMakeLists.txt are untouched.

This validates the core architectural claim: `world_model.c` and `scr.c` are
pure C99 and compile correctly for any Zephyr-supported MCU.

Additional SDK toolchains required (install via `zephyr-sdk-setup.sh`):
- `xtensa-espressif_esp32_zephyr-elf`
- `arm-zephyr-eabi`

### WiFi-capable board (e.g., Espressif ESP32-WROVER-KIT with Xtensa LX6)

```bash
# Build L4 tests
west build -b esp_wrover_kit/esp32/procpu \
    --build-dir tapestry/tapestry-csm-sim/build/hw-esp-test-csm \
    tapestry/tapestry-csm-sim/tests

west flash --build-dir tapestry/tapestry-csm-sim/build/hw-esp-test-csm
west espressif monitor --build-dir tapestry/tapestry-csm-sim/build/hw-esp-test-csm

# Build L5 tests
west build -b esp_wrover_kit/esp32/procpu \
    --build-dir tapestry/tapestry-scr-sim/build/hw-esp-test-scr \
    tapestry/tapestry-scr-sim/tests

west flash --build-dir tapestry/tapestry-scr-sim/build/hw-esp-test-scr
west espressif monitor --build-dir tapestry/tapestry-scr-sim/build/hw-esp-test-scr
```

### Ethernet-capable board (e.g., Renesas EK-RA8D1 with ARM Cortex-M85)

```bash
# Build L4 tests
west build -b ek_ra8d1 \
    --build-dir tapestry/tapestry-csm-sim/build/hw-ra8d1-test-csm \
    tapestry/tapestry-csm-sim/tests

west flash --build-dir tapestry/tapestry-csm-sim/build/hw-ra8d1-test-csm

# Build L5 tests
west build -b ek_ra8d1 \
    --build-dir tapestry/tapestry-scr-sim/build/hw-ra8d1-test-scr \
    tapestry/tapestry-scr-sim/tests

west flash --build-dir tapestry/tapestry-scr-sim/build/hw-ra8d1-test-scr
```

Ztest output appears on the board's UART console at 115200 baud.
A passing run ends with: `PROJECT EXECUTION SUCCESSFUL`.

---

## Phase 2 — Hardware swarm

Two boards gossip **directly** over the LAN via UDP broadcast with no broker
in the path. One board uses WiFi; the other uses Ethernet. Both must connect
to the same router so they share a single L2 segment (standard commercial routers
bridge WiFi and Ethernet ports correctly; mesh systems vary).

```
[WiFi board]     ──UDP broadcast :5000──► [Ethernet board]
[Ethernet board] ──UDP broadcast :5000──► [WiFi board]

[WiFi board]     ──UDP unicast──► collector :5100   (metrics → collect.py)
[Ethernet board] ──UDP unicast──► collector :5100
```

This models real mesh-radio broadcast semantics: an element transmits its
state and any element within range receives it — no relay, no addressing.

The firmware automatically disables WiFi power save after connecting.
Without this, some APs buffer broadcast frames until the DTIM beacon
interval (potentially 10–20 s), which exceeds the 1500 ms world-model
stale threshold and causes spurious LOST/HEALTHY oscillation.

### Build — WiFi element (element 0)

Create `tapestry/tapestry-scr-hw/wifi.conf` from the example template:

```bash
cp tapestry/tapestry-scr-hw/wifi.conf.example \
   tapestry/tapestry-scr-hw/wifi.conf
# edit wifi.conf — it is gitignored
```

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

### Build — Ethernet element (element 1)

```bash
west build -b ek_ra8d1 \
    --build-dir tapestry/tapestry-scr-hw/build/hw-ra8d1 \
    -s tapestry/tapestry-scr-hw \
    -- -DCONFIG_TAPESTRY_ORCH_IP='"192.168.x.x"'

west flash --build-dir tapestry/tapestry-scr-hw/build/hw-ra8d1
```

### Run the telemetry collector

On a machine on the same LAN as both boards:

```bash
cd tapestry/tapestry-scr-hw/telemetry
pip install pandas matplotlib   # one-time
python collect.py --out hw_run.csv
```

### Proof point — physical partition

1. Both boards running: `fresh_ratio = 1.0`, `quorum_state = HEALTHY`,
   leader = element 0 (lowest ID).
2. **Reset the WiFi element** to simulate a partition:
   - Ethernet element ages the WiFi entry: fresh → stale → inactive (~1500 ms)
   - `quorum_state` drops HEALTHY → DEGRADED → LOST
   - Ethernet element assigns NONE role, `election_count` increments
3. **WiFi element reconnects** (after association + DHCP, ~5–10 s):
   - First gossip broadcast triggers `wm_receive_gossip()`
   - Lamport clock merge reconciles both world models
   - Quorum recovers, leader re-elected, `election_count` increments again

### Plot results

```bash
python plot.py hw_run.csv           # interactive window
python plot.py hw_run.csv --out hw_run.png
```

The plot shows fresh ratio, quorum state, role, elected leader, and mean
peer age across both elements — the partition event appears as a cliff in
all five panels, and the recovery as a step back up.
