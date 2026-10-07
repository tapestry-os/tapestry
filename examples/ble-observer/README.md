# BLE Observer — record a Tapestry swarm on one clock

A passive BLE observer for an **nRF5340 DK**. It hears every Tapestry
advertisement in range and streams it to a Linux host over the DK's USB
serial, so an untethered robot run can be debugged afterwards. It never
transmits and never joins the collective.

It records three kinds of traffic:

- **Gossip**: every element's state frame (`tapestry-os/include/tapestry/wire.h`),
  sent as non-connectable *extended* advertising. This includes boot-time
  ID-negotiation beacons, which use the same frame with id 255.
- **Shared facts** (wire v7): each element's fact frames — up to two fact
  records plus a digest of its fact table — sent in short bursts on their
  own extended advertising set, paced by Trickle.
- **Debug telemetry**: the legacy advertising set that
  `examples/cutebot-formation` adds when built with
  `-DEXTRA_CONF_FILE=ble_debug.conf`. It carries status packets and a log text
  stream (`examples/cutebot-formation/src/ble_debug.h`).

The DK only filters and timestamps. All decoding happens on the host in
`tools/observer_log.py`, so a wire format change needs a script update,
not a reflash.

## Layout

```
src/main.c                    app core: scan, filter, one line per packet
prj.conf                      app core Kconfig
boards/…cpuapp.overlay        console UART at 1 Mbaud
sysbuild.cmake, Kconfig.sysbuild
sysbuild/hci_ipc/prj.conf     network core: the BLE controller
tools/observer_log.py         host recorder (Python standard library)
tools/test_observer_log.py    its tests: pytest examples/ble-observer/tools
```

## Controller settings that matter

Several defaults silently lose Tapestry packets. Each setting below is
commented where it is set:

| Setting | Image | Default → here | Without it |
|---|---|---|---|
| `BT_CTLR_SCAN_DATA_LEN_MAX` | net core | 31 → 255 | gossip PDUs (56 B of manufacturer data) truncated or dropped |
| `BT_CTLR_SCAN_AUX_SET` (+ `BT_CTLR_ADVANCED_FEATURES`) | net core | 1 → 8 | a robot's extended PDU is lost while another's is being followed; the value is silently ignored unless the advanced-features menu is enabled |
| `BT_BUF_EVT_DISCARDABLE_SIZE` / `_COUNT` | app core | 58 / 3 → 255 / 16 | extended reports carrying a gossip frame (~84 B) don't fit the host's advertising-report buffers |

Scanning is passive and continuous (window == interval == 60 ms), with no
duplicate filtering.

## Build and flash

From the workspace root (`~/code/tapestry`):

```sh
source .venv/bin/activate
west build -b nrf5340dk/nrf5340/cpuapp --sysbuild -d build-observer tapestry/examples/ble-observer
west flash -d build-observer -r jlink --domain hci_ipc
west flash -d build-observer -r jlink --domain ble-observer
```

Flash the **network core first** the first time, and whenever the DK has
run other firmware. The nRF5340 relocks a core's debug port at every
reset unless firmware on that core reopens it. When J-Link meets a locked
network core it prints `Device will be unsecured now.` and erases the
whole chip, including an app core flashed a moment earlier. A plain
`west flash` flashes the app core first, so it ends with an erased app
core and a silent DK. Once both cores run this firmware, a plain
`west flash -d build-observer -r jlink` is fine.

`-r jlink` uses the SEGGER J-Link tools. The board's default runner,
`nrfutil`, works too (add `--recover` for the first flash) if it is
installed.

## Output

The DK's application-core UART runs at **1 000 000 baud**. It appears on
the host as the J-Link's **second** USB serial interface
(`/dev/serial/by-id/usb-SEGGER_J-Link_<serial>-if02`, usually
`/dev/ttyACM1`), not `-if00`. It writes one line per Tapestry
advertisement:

```
t_us,rssi,addr,adv,kind,id,seq,hex
```

| Field | Meaning |
|---|---|
| `t_us` | DK uptime (µs) when the report reached the host stack: one clock for every robot |
| `rssi` | dBm |
| `addr` | advertiser address: random, per boot; a robot's gossip and debug sets have different ones |
| `adv` | `L` legacy / `E` extended; a trailing `!` marks an incomplete report |
| `kind` | `G` gossip, `B` negotiation beacon, `F` fact frame, `S` debug status, `T` debug text, `U` other Tapestry data |
| `id` | element id (255 while negotiating); `F`: the sender |
| `seq` | `G`: update_seq, `B`: nonce, `F`: table digest, `S`/`T`: debug packet seq |
| `hex` | manufacturer data after the company id, starting with the type/kind byte |

Lines starting with `#` come from the DK itself: a banner at boot, then a
stats line every second (`reports`, `tapestry`, `incomplete`, and a running
`dropped` count of lines lost to a full output buffer). The LED toggles
once per stats line.

Measured with one robot at close range: 14–17 Tapestry lines/s, about 10
debug and 4–5 gossip/beacon. Four robots come to about 60 lines/s, roughly
8 kB/s, well under the link's ~100 kB/s.

## Recording a run

```sh
python3 tapestry/examples/ble-observer/tools/observer_log.py /dev/serial/by-id/usb-SEGGER_J-Link_*-if02
```

Ctrl-C ends the run. Each run writes a directory under `logs/ble-<time>/`
relative to where you start the script (`--out DIR` overrides it):

| File | Contents |
|---|---|
| `raw.log` | every serial line with host time; `--replay raw.log` re-decodes it |
| `gossip.csv` | one row per new `(id, update_seq)`, so relays collapse |
| `facts.csv` | one row per fact record in a new fact frame (a burst's copies collapse); a digest-only frame (an empty table, e.g. a rebooted robot) is one row with no record |
| `status.csv` | one row per new debug status packet |
| `stats.csv` | the DK's per-second counters |
| `events.log` | one time-ordered log of robot text lines, beacons, id assignment, reboots, lost text, DK messages, and the first time each fact (or its retraction) is heard |
| `summary.txt` | per robot: unique packets / sequence span (= loss), mean RSSI |

Debug packets are de-duplicated by `(id, seq)`, or by `(address, seq)`
while a robot is still at id 255. The text stream is reassembled per robot:

- **A lost chunk** flushes the partial line with ` [text lost]`, logs a
  `gap` event, and prefixes the next fragment with `[...]`.
- **A partial line** older than 3 s is flushed with ` [partial]` rather
  than held.
- **A reboot** (debug seq or status uptime going backwards) resets that
  robot's state.

A text line is stamped with the DK time of its *first* chunk, which is
closest to when the robot printed it. It is written once its last chunk
arrives, so it can appear just after events stamped slightly later.

`disc_x`/`disc_y` are decoded from gossip's `reserved0..3` bytes when
`health_flags` bit `0x0100` is set. Firmware that doesn't use those bytes
sends zeros, which leaves the columns empty.

The script carries a generated copy of the wire structs
(`tapestry-os/tools/gen_wire_protocol.py` lists it), so CI's
`gen_wire_protocol.py --check` fails if `wire.h` changes and the copy
isn't regenerated.
