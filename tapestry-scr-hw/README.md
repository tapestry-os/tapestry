# Tapestry Hardware Validation

This directory contains firmware and telemetry tooling for running the
Tapestry L4/L5 stack on physical hardware.  A single CMake project targets
all supported board families; the board overlay selects the transport:

| Board | Transport | Telemetry |
|---|---|---|
| ESP-WROVER-KIT (ESP32) | WiFi UDP + BLE advertising | UDP → `collect.py` |
| EK-RA8D1 (RA8D1) | Ethernet UDP | UDP → `collect.py` |
| BBC micro:bit V2 (nRF52833) | BLE advertising | USB serial → `collect_serial.py` |
| Ambiq Apollo510 EVB (Cortex-M55) | Wired UART gossip bus | USB serial → `collect_serial.py` |

The Apollo510 has no radio on-die, so it gossips over a shared multi-drop
serial bus instead.  Bring-up, bus wiring, and triage are documented in
[`tapestry-os/boards/apollo510_evb/README.md`](../tapestry-os/boards/apollo510_evb/README.md).

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
interval (potentially 10–20 s), which exceeds the 3000 ms world-model
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
   - Ethernet element ages the WiFi entry: fresh → stale (3 s) → inactive (5 s)
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

## Phase 2 — Hardware swarm (wired variant: Apollo510 EVB + EK-RA8D1)

Two elements, two vendors, two wires.  No host, no LAN, no telemetry
collector — the Ambiq Cortex-M55 and the Renesas Cortex-M85 gossip directly
over a shared serial bus using the same `transceiver_uart.c` backend.  This is
the cheapest way to prove the L3 transport is genuinely substrate-agnostic
rather than tuned to one part.

### Hardware setup

Each board takes **one USB cable to its on-board debugger**, which supplies
both power and SWD:

| Board | Port | Provides |
|---|---|---|
| Apollo510 EVB | J-Link USB | power + SWD flashing + UART0 console (VCOM) |
| EK-RA8D1 | J-Link OB USB | power + SWD flashing; console — see below |

Leave the EK-RA8D1's debug-mode jumper in its default "Debug On-Board"
position (per the Renesas *EK-RA8D1 User's Manual*).  Ethernet is not used in
this variant, so the SW1-5 setting the LAN variant needs is irrelevant here.

> ### ⚠ Remove the Apollo510 loopback jumper first
>
> If you ran the single-board loopback test, GP1 and GP23 on J8 are shorted
> together.  **Take that jumper off before wiring the second board.**  Leaving
> it in place ties the Apollo's own TX to the RA8D1's TX — two push-pull 3.3 V
> outputs driving the same node, which is a dead short whenever they disagree.

Gossip travels **directly between the two boards** over three jumper leads.
The host is not a bridge and never sees a gossip frame — its USB cables carry
power and SWD only, and (in Option A below) its Ethernet carries telemetry
only.  Once both boards are running and powered, the host can be unplugged
entirely and they keep gossiping.

| From | Signal | To |
|---|---|---|
| Apollo510 J8 **GP1** | TX → RX | EK-RA8D1 **P408** |
| EK-RA8D1 **P409** | TX → RX | Apollo510 J8 **GP23** |
| Apollo510 J8 **GND** | ground | EK-RA8D1 **GND** |

```
   Apollo510 J8 GP1  (TX) ──────────► P408 (RX)  EK-RA8D1
   EK-RA8D1  P409   (TX) ──────────► GP23 (RX)  Apollo510 J8
   Apollo510 J8 GND ───────────────── GND        EK-RA8D1
```

TX crosses to RX in both directions — a straight-through TX↔TX pairing gets
you nothing but two transmitters shouting at each other.

Both parts are 3.3 V, so no level shifting.  Female-to-female jumper leads
suit the male pin headers on both boards.

Signal integrity is not a constraint at this speed — 115200 baud has an 8.7 us
bit period, so any reasonable lead carries it across a desk.

**Ground.**  If both boards are powered from USB ports on the *same* host they
already share a ground through it, so the third lead is belt-and-braces.  On
separate supplies it is mandatory — without a shared reference the two UARTs
have no agreed idea of what a logic level is.  Run it either way; connect it
first.

**Finding P408 / P409 on the EK-RA8D1.**  Renesas silkscreens its expansion
headers with the port name directly, so look for `P408` and `P409`.  They come
from the board's own `sci3_default` pinctrl group, which the stock DTS enables
at 115200 — matching the Apollo bus exactly.  Which physical connector they
land on is a board detail not recorded in Zephyr's files; the *EK-RA8D1 User's
Manual* pinout is the authority.  If they turn out not to be reachable, any
other SCI channel can be used instead — it needs a DTS overlay pointing
`tapestry,gossip-uart` at that UART node, the same one-line change the
Apollo510 overlay makes.

### Two logging options

The Apollo510 has no Ethernet, so its telemetry is always METRIC CSV on its
USB console.  The RA8D1 can go either way, and that choice decides how it is
built.

**Option A — UDP logging (recommended if you already have the LAN set up).**
Keep the board's stock networking config and simply add the wired transceiver.
No `--extra-conf`, no Kconfig warnings, and it sidesteps the console question
below entirely.  Needs an Ethernet cable, a DHCP server on that LAN, and
`collect.py` running on the collector host.

**Option B — no LAN.**  Use the `ek_ra8d1_uartbus.conf` fragment, which turns
networking off so telemetry falls back to serial CSV.  Nothing but the two
boards and three wires — but it depends on the RA8D1 console being reachable.

Either way the gossip path is identical: the wire between the two boards.
Only telemetry differs.

> **If DHCP does not lease**, an Option A element waits
> `CONFIG_TAPESTRY_NET_CONNECT_TIMEOUT_MS` (30 s when a second transport is
> configured, as it is here), logs
> `no IPv4 address after 30000 ms — continuing without the network`, and then
> comes up on the wired bus alone with no UDP telemetry.  The console sits at
> "DHCP started — waiting for IPv4 address" for that half minute first.
>
> Before this behaviour existed the wait was unbounded, and because network
> bring-up runs ahead of every other transceiver the element never initialised
> the wired bus at all — it hung with the bus dead and never logged why.  If
> you are running older firmware, that is the symptom.  Option B has no
> network dependency either way.

### A note on the EK-RA8D1 console

The LAN variant sends RA8D1 telemetry over UDP to `collect.py`, so its serial
console was never needed.  This variant has no network, so telemetry falls
back to METRIC CSV lines on the console — and whether the J-Link OB exposes a
VCOM for it is a board detail not recorded in Zephyr's board files.

Check after plugging in:

```bash
ls /dev/ttyACM*
```

If a second device appears alongside the Apollo510's, that is the RA8D1
console at 115200.  If not, either wire a 3.3 V USB-serial adapter to the
console pins (UART9 / SCI9, P10_14 TX / P10_15 RX), or simply skip it — **the
Apollo510's console alone is sufficient to prove the test**, since its
`rx_frames` counter and quorum state both reflect whether the RA8D1 is
gossiping.

### Build and flash

The Apollo510 side is the same either way:

```bash
# Apollo510 EVB — element 0
west build -b apollo510_evb -p always \
    --build-dir tapestry/tapestry-scr-hw/build/ap510-e0 \
    tapestry/tapestry-scr-hw -- -DCONFIG_TAPESTRY_ELEMENT_ID=0
west flash --build-dir tapestry/tapestry-scr-hw/build/ap510-e0
```

**Option A — RA8D1 with UDP logging.**  One extra Kconfig on the normal LAN
build; the Tapestry overlay already supplies the `tapestry,gossip-uart` chosen
node, so both transceivers register and the element gossips over UDP *and* the
wire:

```bash
west build -b ek_ra8d1 -p always \
    --build-dir tapestry/tapestry-scr-hw/build/ra8d1-e1 \
    tapestry/tapestry-scr-hw \
    -- -DCONFIG_TAPESTRY_TRANSCEIVER_UART=y \
       -DCONFIG_TAPESTRY_ORCH_IP='"192.168.x.x"'
west flash -r jlink --build-dir tapestry/tapestry-scr-hw/build/ra8d1-e1
```

**Option B — RA8D1 with no LAN.**  Serial telemetry instead.  Note the bare
filename: `--extra-conf` paths resolve relative to the **application**
directory (`tapestry-scr-hw`), not your shell's working directory, so a
workspace-relative path here silently doubles into
`tapestry-scr-hw/tapestry/tapestry-scr-hw/...` and fails to configure.

```bash
west build -b ek_ra8d1 -p always \
    --build-dir tapestry/tapestry-scr-hw/build/ra8d1-e1 \
    tapestry/tapestry-scr-hw \
    --extra-conf ek_ra8d1_uartbus.conf
west flash -r jlink --build-dir tapestry/tapestry-scr-hw/build/ra8d1-e1
```

Option B prints Kconfig warnings about `NET_*` symbols having unmet
dependencies.  Expected: the networking block lives in `boards/ek_ra8d1.conf`,
which Zephyr merges before the `--extra-conf` fragment and which cannot be
made conditional.  The fragment's values win.  Option A produces no warnings.

Element IDs: the Apollo510 defaults to 0 and `boards/ek_ra8d1.conf` already
sets 1, so they pair without further configuration.

### Proof point — quorum across two vendors

On the Apollo510 console, three things change once the RA8D1 joins:

| Signal | Alone | With peer |
|---|---|---|
| `uart_bus: rx_frames` | 0 | climbing |
| `METRIC` quorum_state | `0` (`SCR_QUORUM_LOST`) | `2` (`SCR_QUORUM_HEALTHY`) |
| LEDs | LED4 only | LED2 + LED3 |

`rx_frames` is the one to watch.  During single-board loopback testing it
stays at 0 by design — echo suppression discards the element's own frames —
so a non-zero `rx_frames` is unambiguous evidence of a *peer*.

### Confirmed result (2026-08-25)

```
Apollo510  METRIC,...,0,1.0000,2,2,0,1,300.0    quorum HEALTHY, role LEADER,  leader 0
EK-RA8D1   METRIC,...,1,1.0000,2,1,0,3,400.0    quorum HEALTHY, role FOLLOWER, leader 0
```

Both elements converged on element 0 as leader (lowest ID wins), `fresh_ratio`
1.0 on both, `ck_fail = 0` on both.  Ambiq Cortex-M55 and Renesas Cortex-M85
sharing state over two wires with no host in the path.

**Signal-quality check: `rx_bytes / rx_frames` should be exactly 27** — the
framed gossip frame size.  A higher ratio means bytes are arriving that never
form a frame; they are discarded harmlessly during preamble hunting, but they
indicate a noisy line worth investigating before trusting the link at longer
runs or higher rates.

Full bring-up, bus wiring and triage for the Apollo510 side:
[`tapestry-os/boards/apollo510_evb/README.md`](../tapestry-os/boards/apollo510_evb/README.md).

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

One binary runs on all boards — element IDs are negotiated at boot via auto-ID:

```bash
west build -b bbc_microbit_v2 \
    --build-dir tapestry/tapestry-scr-hw/build/hw-mb \
    tapestry/tapestry-scr-hw
```

Flash each board with the same `.hex` (connect one at a time; the drive is always named `MICROBIT`):

```bash
cp tapestry/tapestry-scr-hw/build/hw-mb/zephyr/zephyr.hex /media/$USER/MICROBIT/
# Swap USB cable to second board and copy again
cp tapestry/tapestry-scr-hw/build/hw-mb/zephyr/zephyr.hex /media/$USER/MICROBIT/
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
   The element assigned the lowest ID at boot is elected leader.
2. **Carry element 0 out of BLE range** (into another room):
   - Element 1 ages the peer entry: fresh → stale (3 s) → inactive (5 s)
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

### Build — micro:bit element

Use the same binary built in the BLE variant section above (no per-robot flags needed):

```bash
west flash --build-dir tapestry/tapestry-scr-hw/build/hw-mb
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
