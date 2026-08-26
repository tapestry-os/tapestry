# Ambiq Apollo510 EVB — Tapestry board support

Board support for running a Tapestry element on the **AP510EVB** (Ambiq
Apollo510, Cortex-M55 @ up to 250 MHz, 4 MB MRAM, 3 MB SRAM).  Zephyr calls
the board `apollo510_evb`.

| Layer | On this board |
|---|---|
| L1 substrate | 3 on-board LEDs (status signal), BTN0 (manual proximity injection), no actuators |
| L1 transceiver | Wired UART gossip bus on UART2 (`transceiver_uart.c`) |
| L2–L5 | Unmodified — the pure-C99 core builds as-is |
| Telemetry | METRIC CSV lines on the UART0 console, read by `telemetry/collect_serial.py` |

## Why a wired gossip bus

The Apollo510 has no radio on-die (unlike the Apollo4 Blue parts), and the EVB
carries no companion transceiver.  Every other Tapestry hardware target gossips
over BLE, WiFi/Ethernet UDP, or the Crazyflie's nRF51 co-processor; this one has
none of those.  So the port adds a board-agnostic backend —
`tapestry-os/subsys/transport/transceiver_uart.c` — that carries gossip frames
over a shared multi-drop serial bus.  It is not Apollo-specific: any Zephyr
target with a spare interrupt-driven UART can select it with
`CONFIG_TAPESTRY_TRANSCEIVER_UART=y` and a `tapestry,gossip-uart` chosen node.

## Bus pins

Confirmed against the EVB schematic (2026-08-25).  The gossip bus runs on
**UART2**, on two pins of header **J8**:

| Role | Pin | J8 net name |
|---|---|---|
| TX | **GP1** | `TRC0_SSDA_SMOSI_TX23_VC0_GP1` |
| RX | **GP23** | `M7SDA_M7MOSI_SWO_RX23_GP23` |
| GND | — | J8 ground, needed once a second element is attached |

The two sit opposite each other on J8, so a single jumper lead loops TX back
to RX for the one-board bring-up test below.

UART0 stays untouched — console, shell, and telemetry.

### Reading the EVB net names

The suffix on each J8/J9 net name says which UART instances that pin can
serve, which is what makes picking a pair straightforward:

- `TX01` / `RX01` → UART0 **or** UART1
- `TX23` / `RX23` → UART2 **or** UART3
- `CTS*` / `RTS*` → flow control; unused, this transceiver does not use it

**TX and RX must come from the same instance.** `GP0` (`TX01`) paired with
`GP22` (`TX23`) is two transmitters and receives nothing — `GP22` is never an
RX option for any UART.

The authoritative table is
`modules/hal/ambiq/dts/apollo510/ambiq-apollo510-pinmux.h`; the EVB net-name
suffixes agree with it on every J8 pin.

Two pins are unavailable on this board: **P55** (UART0 console RX) and **P53**
(`mspi1` chip enable, enabled in the board DTS).

### Moving the bus

Change the two `pinmux` macros in `apollo510_evb.overlay`, the matching
`GPIO_Pn` entries in `uart2_sleep`, and repoint the `&uartN` node and the
`chosen` node.  Nothing else in the port refers to these pins.

`GP1`/`GP23` also carry trace data (TRC0) and an alternate SWO respectively,
but the board DTS puts SWO on P28 and never enables ETM trace, and IOM7 is
disabled.  Do not enable ETM trace while the bus is live.

## Wiring the bus

**Two elements** — wire straight across, no bus at all.  This is the
recommended bring-up topology:

```
   element 0 GP1 (TX) ─────────► GP23 (RX) element 1
   element 1 GP1 (TX) ─────────► GP23 (RX) element 0
   element 0 J8 GND ───────────── J8 GND element 1
```

**Three or more elements** — one shared line, every TX on it and every RX on
it, plus a common ground:

```
   E0.TX ─┬─────┬─────┬──────────  bus  ──┐
   E1.TX ─┘     │     │                   │
   E2.TX ───────┘     │                   │
   E3.TX ─────────────┘                   │
   E0.RX  E1.RX  E2.RX  E3.RX ────────────┘
   all GND ── common
```

TX pins are push-pull, so tying them together directly means two elements
transmitting at once fight over the line.  Put a **~1 kΩ series resistor in
each TX leg**, or give each node a real open-drain / RS-485 transceiver.  A
collision corrupts the frame, the receiver's Fletcher-8 checksum rejects it,
and the loss is absorbed — every element re-sends its full state every
`GOSSIP_INTERVAL_MS` (500 ms).  A common ground is not optional.

Bus baud is the UART2 `current-speed` in the overlay (115200).  All elements
must agree on it.

## Build and flash

From the workspace root (`tapestry-workspace/`), one build per element with a
distinct ID:

```bash
west build -b apollo510_evb \
    --build-dir tapestry/tapestry-scr-hw/build/ap510-e0 \
    tapestry/tapestry-scr-hw -- -DCONFIG_TAPESTRY_ELEMENT_ID=0
west flash --build-dir tapestry/tapestry-scr-hw/build/ap510-e0

west build -b apollo510_evb \
    --build-dir tapestry/tapestry-scr-hw/build/ap510-e1 \
    tapestry/tapestry-scr-hw -- -DCONFIG_TAPESTRY_ELEMENT_ID=1
west flash --build-dir tapestry/tapestry-scr-hw/build/ap510-e1
```

`west flash` uses the on-board J-Link (`AP510NFA-CBR`, SWD) and needs SEGGER
J-Link software plus the `pylink` Python module on the host.

### The Apollo510 is a secure part — mind the reset

J-Link reports this chip as `Secure Part` / `Secure Chip`, and its secure
bootloader **must run after reset** to bring up clocks and power domains.  A
debugger reset that does not let the bootloader complete leaves the core
running against uninitialised peripherals, and the first register access to a
UART or GPIO takes a precise data bus fault:

```
<err> os: ***** HARD FAULT *****
<err> os: >>> ZEPHYR FATAL ERROR 25: Unknown error on CPU 0
```

Fatal error 25 is `K_ERR_ARM_BUS_PRECISE_DATA_BUS`.  Seeing it right after a
flash means the boot sequence, not the firmware — the same image runs fine
once started properly.

`west flash` is the normal path and works:

```bash
west flash --build-dir <build-dir>
```

This works out of the box: `tapestry-scr-hw/CMakeLists.txt` defines an
`app_set_runner_args()` macro for this board that adds `--reset-type=2`, so no
flag is needed on the command line.

**Why that matters.** With J-Link's default reset, `west flash` fails roughly
half the time on this part (measured 2026-08-25: 2 failures in 4 runs).  The
Apollo510 is a secure chip whose bootloader must run to completion after
reset, and the script west generates goes `r` straight into `loadfile` with no
`halt` between — so the flash-loader download can begin while the bootloader
is still running:

```
Programming flash [100%] Done.
****** Error: Timeout while programming sector @0x00410000 ... RAMCode did not respond in time!
Error while programming flash: Programming failed.
```

Because that script starts with `ExitOnError 1`, the run aborts and leaves
flash **half-written** — which boots and then hard-faults with
`ZEPHYR FATAL ERROR 25` (`K_ERR_ARM_BUS_PRECISE_DATA_BUS`, a peripheral access
against uninitialised clocks).  It reads convincingly as a firmware bug; it
sent one debugging session chasing a nonexistent power-management fault.

`--reset-type=2` fixes this, and is pinned for this board in
`tapestry-scr-hw/CMakeLists.txt` via the `BOARD_RUNNER_ARGS_jlink` cache
variable — no flag needed on the command line.  (Zephyr's documented
`app_set_runner_args()` macro hook was tried first and silently does nothing on
4.4.0-rc1; see the comment in CMakeLists.txt.)  With it,
5 consecutive clean flashes; without it, 2 failures in 4.  Five trials cannot
prove a race never fires, but under the old rate a run of 5 had about a 3%
chance, so the improvement is real.  `--speed 500` was tried and did not help.

If a flash ever does fail, west exits non-zero, so it is safe to simply retry —
a failed attempt is fully overwritten by the next erase-and-program:

```bash
until west flash --build-dir <build-dir>; do echo "retrying…"; done
```

### west hides the real error

The runner swallows JLinkExe's output, so **every** failure looks identical at
the shell:

```
FATAL ERROR: command exited with status 1: /opt/SEGGER/JLink_V936/JLinkExe ...
```

A powered-off board, an unplugged probe, and the bootloader race are
indistinguishable from that line alone.  Before theorising, re-run with
`west -v flash` — that surfaces JLink's own output and the generated commander
script.  (And check the board is actually powered.)

If you ever do hit the timeout, flashing by hand always works; the explicit
`halt` is the whole difference:

```bash
JLinkExe -nogui 1 -if swd -speed 1000 -device AP510NFA-CBR
J-Link> connect
J-Link> r
J-Link> halt
J-Link> loadfile <build-dir>/zephyr/zephyr.bin 0x410000
J-Link> r
J-Link> g
```

A flash only counts if you see **both** of these:

```
Verifying flash   [100%] Done.
...
O.K.
```

### Always check which image booted

A failed flash leaves the previous (or a partial) image running, and reading
test results off a stale image has cost real debugging time here twice.  The
transceiver's boot line is an unambiguous fingerprint:

```
uart gossip bus ready on uart@4003b000
```

UART0 is `0x40039000`, UART1 `0x4003a000`, UART2 `0x4003b000`, UART3
`0x4003c000` — so that one field tells you exactly which overlay is live.
Confirm it before interpreting anything else.

Note the load address: Apollo510 MRAM starts at **`0x410000`**, not 0.  The
build produces no `.hex`, so a raw `.bin` load needs that address explicitly.

Console — UART0 on the J-Link USB VCOM port, 115200 8N1:

```bash
minicom -D /dev/ttyACM0 -b 115200
```

## Reading the LEDs

### Silkscreen mapping

The EVB silkscreen numbers its LEDs independently of Zephyr's alias
numbering, and the board carries indicator LEDs that software cannot drive at
all.  Confirmed on hardware 2026-08-25 with `CONFIG_TAPESTRY_AP510_LED_WALK`:

| Silkscreen | Zephyr alias | SoC pin | Driven by |
|---|---|---|---|
| **LED2** | `led0` | P165 | Tapestry substrate |
| **LED3** | `led1` | P89 | Tapestry substrate |
| **LED4** | `led2` | P92 | Tapestry substrate |
| LED1, LED5 | — | — | board indicators (power / J-Link), always on |

So a booted element normally shows LED1 and LED5 lit no matter what — ignore
them.  Only LED2/LED3/LED4 carry Tapestry state.

### Status word

The three driven LEDs read as one status word: `led0` = element is up,
`led1` = quorum healthy, `led2` = something is wrong.

| Signal | led0 (LED2) | led1 (LED3) | led2 (LED4) | Meaning |
|---|---|---|---|---|
| `NONE` | ○ | ○ | ○ | output suppressed (sleep / harvest) |
| `IDLE` | ● | ○ | ○ | element present, no active goal |
| `ACTIVE` | ● | ● | ○ | goal in progress, quorum healthy |
| `DEGRADED` | ● | ○ | ● | goal in progress, quorum reduced |
| `FAILED` | ○ | ○ | ● | quorum lost or goal unachievable |

A lone element with no peers has no quorum, so **LED4 alone** is the correct
steady state for a single board — not a fault.  Two boards wired together
should settle on `ACTIVE` (LED2 + LED3) within a few gossip intervals.  If
they stay at `FAILED`, the bus is not carrying frames — see the triage table
below.

To re-derive the mapping on a different board revision, rebuild with
`-DCONFIG_TAPESTRY_AP510_LED_WALK=y`: each alias is lit alone for 1.5 s at
boot with a matching console line.

## BTN0 is not a sensor

`substrate_sense(SUBSTRATE_SENSOR_PROXIMITY, …)` returns 0.0 (obstacle at
contact range) while BTN0 is held and 1.0 (clear) when it is released.  The
EVB has no range sensor; this is a deliberate human-in-the-loop injection
point so obstacle-reactive behaviour can be exercised by hand.  Nothing in the
current L2–L5 stack calls `substrate_sense()` yet.

`SUBSTRATE_SENSOR_BATTERY` returns -1.  The EVB is bench-powered — reporting a
full cell would be a lie about a board that cannot run off a battery at all.

## Triage: the bus is not carrying frames

### Known-good loopback baseline

With GP1 jumpered straight to GP23 on J8 and no peer attached, a healthy
element looks like this (confirmed on hardware 2026-08-25):

```
uart_bus: tx=4  rx_frames=0 rx_bytes=108 ck_fail=0 len_bad=0 q_drop=0 echo=4
uart_bus: tx=8  rx_frames=0 rx_bytes=216 ck_fail=0 len_bad=0 q_drop=0 echo=8
uart_bus: tx=12 rx_frames=0 rx_bytes=324 ck_fail=0 len_bad=0 q_drop=0 echo=12
```

Three invariants to check, in order of usefulness:

- **`rx_bytes / tx == 27`** — the framed gossip frame is exactly 27 bytes
  (2 magic + 1 length + 22 payload + 2 checksum).  Anything less means bytes
  are being lost on the wire; anything more means the parser is resyncing on
  garbage.
- **`echo == tx`** — every frame came back and was recognised as our own.
- **`ck_fail == len_bad == q_drop == 0`** — nothing corrupt, nothing dropped.

`rx_frames == 0` is correct here: echo suppression discards our own frames
before they reach the queue.  `rx_frames` only climbs once a real peer is on
the bus.

Pulling the jumper is the negative control — `echo` stops climbing while `tx`
carries on.

### Counter reference

`transceiver_uart.c` prints one counter line every 2 s, but only when a
counter has moved:

```
uart_bus: tx=42 rx_frames=41 rx_bytes=1107 ck_fail=0 len_bad=0 q_drop=0 echo=42
```

Read it on both elements at once:

| Symptom | Cause |
|---|---|
| `tx` climbs, `rx_bytes` stays 0 on both | bus not connected, or TX/RX not crossed, or no common ground |
| `rx_bytes` climbs, `ck_fail` climbs, `rx_frames` stays 0 | connected but garbled — baud mismatch, or contention with no series resistors |
| `rx_frames` climbs but quorum stays LOST | frames arrive but are rejected upstream — check that both elements have distinct `CONFIG_TAPESTRY_ELEMENT_ID` |
| `q_drop` climbs | the element is not draining gossip fast enough; RX queue is 8 slots |
| `echo` tracks `tx` exactly | normal on a shared bus — that is this element hearing itself, suppressed before it reaches the queue |

`echo` staying at 0 on a shared-bus wiring means RX is not hearing the local
TX, which is itself a wiring fault worth chasing.

If gossip RX is intermittent rather than absent, the next thing to try is
`CONFIG_PM=n`.  The transceiver takes a `pm_policy` lock on
`PM_STATE_SUSPEND_TO_RAM` at init specifically so the UART keeps its clock
between cycles; turning PM off entirely rules that mechanism in or out.

## Running the ztest suites on hardware

Board `.conf` fragments are supplied for all three suites, so Phase 1
(pure-C99 core on a real MCU) works the same as on the other targets:

```bash
west build -b apollo510_evb \
    --build-dir tapestry/tapestry-csm-sim/build/hw-ap510-test-csm \
    tapestry/tapestry-csm-sim/tests
west flash --build-dir tapestry/tapestry-csm-sim/build/hw-ap510-test-csm

west build -b apollo510_evb \
    --build-dir tapestry/tapestry-scr-sim/build/hw-ap510-test-scr \
    tapestry/tapestry-scr-sim/tests
west flash --build-dir tapestry/tapestry-scr-sim/build/hw-ap510-test-scr
```

Results print on the UART0 console.

## Files

```
tapestry-os/boards/apollo510_evb/
├── apollo510_evb.overlay          UART2 gossip bus + tapestry,gossip-uart chosen node
├── substrate_apollo510_evb.c      L1 substrate: LEDs, BTN0, no actuators
└── README.md                      this file

tapestry-os/subsys/transport/
├── transceiver_uart.c             wired serial gossip backend (board-agnostic)
└── transceiver_uart.h

tapestry-scr-hw/boards/apollo510_evb.conf         element build config
tapestry-scr-hw/tests/boards/apollo510_evb.conf   ztest config
tapestry-csm-sim/tests/boards/apollo510_evb.conf  ztest config
tapestry-scr-sim/tests/boards/apollo510_evb.conf  ztest config
```
