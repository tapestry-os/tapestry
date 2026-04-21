# Tapestry Hardware Validation

This directory contains firmware and telemetry tooling for running the
Tapestry L4/L5 stack on physical hardware.  A single CMake project targets
all three supported board families; the board overlay selects the transport:

| Board | Transport | Telemetry |
|---|---|---|
| ESP-WROVER-KIT (ESP32) | WiFi UDP + BLE advertising | UDP → `collect.py` |
| EK-RA8D1 (RA8D1) | Ethernet UDP | UDP → `collect.py` |
| BBC micro:bit V2 (nRF52833) | BLE advertising | USB serial → `collect_serial.py` |

Two phases are documented:

- **Phase 1** — run the L4/L5 ztest suites on each MCU to confirm the pure
  C99 core compiles and passes on non-native targets.
- **Phase 2** — live swarm: boards gossip directly with no simulation broker.
  Two-board (LAN) and two-board (BLE) variants; combine all three for a
  heterogeneous swarm.

All `west build` commands are run from the **workspace root**
(`tapestry-workspace/`).

---

## Phase 1 — Unit tests on hardware

Board-specific `prj.conf` overlays in each `tests/boards/` directory supply
FPU and stack-size settings; the test source and CMakeLists.txt are untouched.

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

### BLE-capable board (e.g., BBC micro:bit V2 with nRF52833 / Cortex-M4F)

```bash
# Build L4 tests
west build -b bbc_microbit_v2 \
    --build-dir tapestry/tapestry-csm-sim/build/hw-mb-test-csm \
    tapestry/tapestry-csm-sim/tests

west flash --build-dir tapestry/tapestry-csm-sim/build/hw-mb-test-csm

# Build L5 tests
west build -b bbc_microbit_v2 \
    --build-dir tapestry/tapestry-scr-sim/build/hw-mb-test-scr \
    tapestry/tapestry-scr-sim/tests

west flash --build-dir tapestry/tapestry-scr-sim/build/hw-mb-test-scr
```

Ztest output appears on the board's UART console (or USB serial bridge) at
115200 baud.  A passing run ends with: `PROJECT EXECUTION SUCCESSFUL`.

---

## Phase 2 — Hardware swarm (LAN variant: ESP32 + RA8D1)

Two boards gossip **directly** over the LAN via UDP broadcast with no broker
in the path.  One board uses WiFi; the other uses Ethernet.  Both must connect
to the same router so they share a single L2 segment (standard commercial routers
bridge WiFi and Ethernet ports correctly; mesh systems vary).

```
[WiFi board]     ──UDP broadcast :5000──► [Ethernet board]
[Ethernet board] ──UDP broadcast :5000──► [WiFi board]

[WiFi board]     ──UDP unicast──► collector :5100   (metrics → collect.py)
[Ethernet board] ──UDP unicast──► collector :5100
```

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

---

## Phase 2 — Hardware swarm (BLE variant: two micro:bit Cutebots)

Two micro:bit V2 boards each mounted on a Cutebot Mini robot.  Gossip travels
over BLE non-connectable advertising — each element advertises its state and
any element within radio range (~10 m open air) receives it.

```
[micro:bit elem 0] ──BLE ADV──► [micro:bit elem 1]
[micro:bit elem 1] ──BLE ADV──► [micro:bit elem 0]

[micro:bit elem 0] ──USB serial──► /dev/ttyACM0  (metrics → collect_serial.py)
[micro:bit elem 1] ──USB serial──► /dev/ttyACM1
```

The Cutebot motors and LEDs reflect the current L5 quorum and role state:

| Quorum state | Role | LEDs | Motors |
|---|---|---|---|
| HEALTHY | LEADER | Green | Forward 70% |
| HEALTHY | FOLLOWER | Blue | Forward 50% |
| DEGRADED | any | Orange | Stopped |
| LOST | any | Red | Stopped |

### Build and flash

Build both elements first, then flash by copying the `.hex` to the `MICROBIT`
USB drive that appears when each board is connected.  `west flash` is not used
here — the micro:bit's built-in DAPLink bootloader makes drag-and-drop more
reliable than pyocd.

```bash
# Build element 0
west build -b bbc_microbit_v2 \
    --build-dir tapestry/tapestry-scr-hw/build/hw-mb0 \
    tapestry/tapestry-scr-hw

# Build element 1
west build -b bbc_microbit_v2 \
    --build-dir tapestry/tapestry-scr-hw/build/hw-mb1 \
    tapestry/tapestry-scr-hw \
    -- -DCONFIG_TAPESTRY_ELEMENT_ID=1
```

Flash each board (connect one at a time; the drive is always named `MICROBIT`):

```bash
# Flash element 0
cp tapestry/tapestry-scr-hw/build/hw-mb0/zephyr/zephyr.hex /media/$USER/MICROBIT/

# Swap USB cable to second board, then flash element 1
cp tapestry/tapestry-scr-hw/build/hw-mb1/zephyr/zephyr.hex /media/$USER/MICROBIT/
```

The board resets automatically when the copy completes.

### Run the telemetry collector

Connect both micro:bits via USB.  On Linux they appear as `/dev/ttyACM0` and
`/dev/ttyACM1`; on macOS as `/dev/tty.usbmodem*`.

```bash
cd tapestry/tapestry-scr-hw/telemetry
pip install pyserial   # one-time
python collect_serial.py --ports /dev/ttyACM0 /dev/ttyACM1 --out mb_run.csv
```

### Proof point — BLE partition

1. Both boards powered on and within range: LEDs go from red (LOST) to
   green + blue (HEALTHY) within ~2 s of first BLE advertisement.
   Element 0 (lowest ID) is always elected leader.
2. **Carry element 0 out of BLE range** (into another room):
   - Element 1 ages the peer entry: fresh → stale → inactive (~1500 ms)
   - `quorum_state` drops HEALTHY → DEGRADED → LOST
   - Element 1 LED turns red, motors stop
3. **Bring element 0 back into range**:
   - First received advertisement triggers `wm_receive_gossip()`
   - Lamport clock merge reconciles both world models
   - Quorum recovers within ~2 s, leader re-elected, motors restart

Note: BLE range indoors is ~10–30 m — driving apart in the same room will
not cause a partition.  Move one board into another room or power it off
to demonstrate the partition/recovery sequence.

### Plot results

```bash
python plot.py mb_run.csv           # interactive window
python plot.py mb_run.csv --out mb_run.png
```

---

## Phase 2 — Three-element heterogeneous swarm (ESP32 + RA8D1 + micro:bit)

The ESP32 bridges the LAN and BLE worlds simultaneously — it runs WiFi and BLE
at the same time using the ESP32's hardware coexistence arbitration.

```
[ESP32 elem 0] ──UDP broadcast──► [RA8D1 elem 1]
[RA8D1 elem 1] ──UDP broadcast──► [ESP32 elem 0]

[ESP32 elem 0] ──BLE ADV──► [micro:bit elem 2]
[micro:bit elem 2] ──BLE ADV──► [ESP32 elem 0]

[ESP32 + RA8D1] ──UDP unicast──► collect.py :5100
[micro:bit]     ──USB serial──► collect_serial.py
```

### Build — micro:bit element (element 2)

```bash
west build -b bbc_microbit_v2 \
    --build-dir tapestry/tapestry-scr-hw/build/hw-mb2 \
    tapestry/tapestry-scr-hw \
    -- -DCONFIG_TAPESTRY_ELEMENT_ID=2

west flash --build-dir tapestry/tapestry-scr-hw/build/hw-mb2
```

### Quorum settings for three elements

With three elements, raising `QUORUM_TARGET` to 2 makes HEALTHY require both
peers visible.  Pass this at build time for all three elements:

```
-DCONFIG_TAPESTRY_QUORUM_MIN=1 -DCONFIG_TAPESTRY_QUORUM_TARGET=2
```

### Collect telemetry from all three elements

```bash
# Terminal 1 — UDP metrics from ESP32 and RA8D1
cd tapestry/tapestry-scr-hw/telemetry
python collect.py --out hw_lan.csv

# Terminal 2 — serial metrics from micro:bit
python collect_serial.py --ports /dev/ttyACM0 --out hw_ble.csv
```

Overlay both runs in one plot:

```bash
python plot.py hw_lan.csv hw_ble.csv --labels "LAN" "BLE" --out hw_three.png
```
