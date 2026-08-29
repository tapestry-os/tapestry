# Demo — Cutebot Collective Formation

BBC micro:bit V2 + Cutebot Mini robots self-organize using the Tapestry
world model and BLE gossip. No central controller. This is the L1-L7
Choreo reference example on a differential-drive ground rover — a
different substrate class from `cf21bl-formation`'s lighthouse-tracked
quadrotor, and the first consumer of Choreo's FORM goal to run on real
hardware anywhere in this repo.

Two build modes (Kconfig choice `DEMO_MODE`, see `Kconfig`):

- **`DEMO_MODE_CHOREO` (default)** — a real L5 SCR + a declarative L7
  Choreo script (`form-grid.choreo.toml`) driven through the L6 BSE: hold
  the boot-time station, arrange into a near-square grid, hold the
  finished grid. **Run on real Cutebot hardware, which reached and held
  the grid.** Read "Known limitations" below for what that run does and
  does not establish.
- **`DEMO_MODE_SHOWCASE`** — the original, hardware-validated L4-only
  emergent spring field (no SCR/L6/L7). Kept as a fallback.

## How it works (both modes)

1. Each robot advertises its dead-reckoning position over BLE.
2. Peer positions are received into the local L4 world model.
3. The micro:bit 5×5 LED matrix displays the robot's dead-reckoning position
   in real time (one lit pixel = estimated location in the 100×100 logical world).

### `DEMO_MODE_SHOWCASE` — spring field

A spring-field algorithm computes a differential drive command:
- repulsion when a peer is closer than `DEMO_TARGET_SPACING`
- attraction when farther

Hysteresis thresholds (`FORCE_START` / `FORCE_STOP`) prevent oscillation
near equilibrium and absorb gossip-propagated micro-corrections.

Cutebot LEDs reflect **formation completeness** — whether all currently-known
active peers are fresh:
- **Red** — no known peers (isolated or booting)
- **Yellow** — some peers stale; movement halted until all active peers are fresh
- **Green** — all known peers fresh (stable formation at any size)

This means the LEDs return to green after a removal and redistribution,
regardless of how many robots remain.

### `DEMO_MODE_CHOREO` — L5/L6/L7 script

A real L5 SCR (`scr_init()`/`scr_tick()`) tracks quorum from the actual
world model, and a declarative L7 Choreo script
(`form-grid.choreo.toml`, compiled to `src/choreo_script.h` by
`sdk/tools/choreoc.py`) drives the robots through the L6 BSE:

1. **hold** — station-keep at the boot-time position (coordinate-free).
2. **form** (shape=grid) — arrange into a near-square grid centered on
   the arena, via `demo_track_target()`'s differential-drive go-to-point
   controller (`formation.c`) — a proportional turn-then-drive law (turn
   toward the commanded point, drive forward, decelerate on approach,
   with an emergency-repulsion backstop against too-close peers), the
   *first* real-hardware-targeted consumer of Choreo's FORM goal in this
   repo.
3. **hold** — settle on the finished grid.

Script completion (quiescence) maps to simply holding still — there is no
takeoff/landing concept for a ground rover.

LEDs follow the active step's declared indicator effect
(`choreo_current_indicator()`) when one is set, falling back to the same
peer-freshness heuristic as showcase mode otherwise.

To change the script: edit `form-grid.choreo.toml`, then regenerate:

```sh
python3 sdk/tools/choreoc.py tapestry/examples/cutebot-formation/form-grid.choreo.toml
```

## Hardware

| Item | Details |
|---|---|
| Board | BBC micro:bit V2 (nRF52833, Cortex-M4F) |
| Robot chassis | Elecfreaks Cutebot Mini |
| Arena | 800 mm × 800 mm |
| Equilibrium spacing | ~341 mm (43 logical units, 4-robot square) |

## Build

One binary runs on all robots — no per-robot build flags needed.
Run from the workspace root (`tapestry-workspace/`):

```sh
west build -b bbc_microbit_v2 tapestry/examples/cutebot-formation
```

Builds `DEMO_MODE_CHOREO` by default. For the original, hardware-validated
spring field instead:

```sh
west build -b bbc_microbit_v2 tapestry/examples/cutebot-formation \
  -- -DCONFIG_DEMO_MODE_SHOWCASE=y
```

The hex is written to `build/zephyr/zephyr.hex`.

## Flash

The micro:bit appears as a USB mass storage device when plugged in.
Copy the same hex to every robot:

```sh
cp build/zephyr/zephyr.hex /media/$USER/MICROBIT/
cp build/zephyr/zephyr.hex /media/$USER/MICROBIT1/
cp build/zephyr/zephyr.hex /media/$USER/MICROBIT2/
cp build/zephyr/zephyr.hex /media/$USER/MICROBIT3/
```

Each board flashes itself and reboots automatically on receipt.

**Boot synchronization:** the auto-ID window is 8 seconds and all robots must
power on within that window to negotiate IDs correctly. If the sequential copies
stagger the reboots, unplug and replug all robots simultaneously after flashing
to align their boot times.

## Demo procedure

This section describes `DEMO_MODE_SHOWCASE`'s spring-field behavior
(cold start → scatter → stable equilibrium). `DEMO_MODE_CHOREO`'s boot,
auto-ID, and placement steps are identical; its runtime behavior is the
hold → form(grid) → hold sequence described above instead of continuous
emergent spacing.

**Physical placement before power-on**
IDs are assigned by FICR nonce rank at boot — you won't know which physical
robot gets which ID until after the first run. Connect one robot to serial,
note its ID in the boot log (`element %u`), and put a numbered sticker on it.
Do this once per robot; the FICR nonce is fixed hardware.

Orient each robot facing outward from the center point of the arena:

| ID | Heading | Face toward |
|---|---|---|
| 0 | 0° | right (east) |
| 1 | 90° | away from you (north) |
| 2 | 180° | left (west) |
| 3 | 270° | toward you (south) |

For three robots the angles are 0°, 120°, 240°.

This matters for dead-reckoning accuracy: the code initializes each robot's
heading to its outward angle so spring forces project fully forward on the
first tick. If a robot's physical orientation does not match its assigned
heading, its dead-reckoning position will drift in the wrong direction from
the start.

**Cold start (robots clustered)**
Power all robots on within 8 seconds of each other. During the boot window
LEDs are red (no peers yet). After the window the robots detect each other and
LEDs go green as gossip propagates. Because all robots seed their world-model
position near arena center, spring repulsion immediately drives them outward
into an equidistant distribution.

**Formation stable**
Once robots have spread out and settled, all LEDs hold solid green. The micro:bit
matrix shows each robot's estimated position as a single lit pixel.

**Remove a robot**
Power off the robot first (gossip stops immediately), then pick it up.
Moving it without powering it off has no effect — the robot continues advertising
its last position over BLE regardless of physical location.

After ~1.5 s the missing robot's world-model entry goes stale on the remaining
robots. LEDs shift to yellow and movement halts — robots hold position while the
world model is incomplete. After ~5 s the entry expires and is removed; LEDs
return to green and the survivors begin redistributing to fill the gap.

**Rejoin (optional)**
Place the powered-off robot anywhere in the arena and power it on. During its
8 s boot window it reads claimed IDs from the running robots' gossip and takes
the lowest unclaimed slot. When it exits the window it starts gossiping near
arena center and all robots reorganize to include it. The rejoining robot's LED
goes from red (booting) to green (formation) within one gossip cycle of joining.

## Auto-ID

During an 8-second boot window each robot advertises its FICR hardware nonce
and listens for peers. After the window:

1. **Nonce rank** among co-booting robots → candidate rank (lower nonce = lower rank).
2. **Claimed IDs** from already-running robots (live gossip) are avoided.
3. **element_id** = rank-th unclaimed ID.

The window duration is set in `prj.conf` via
`CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS` (overrides the 4 s default in `transport.c`).

## Starting positions

All robots seed their world-model position on a 3-unit-radius circle around
arena center (50, 50). Adjacent robots are ~4 logical units apart — far below
the 50-unit target spacing — so spring repulsion immediately drives them outward.
Physically, robots should start clustered in the center of the arena.

The tiny radius (24 mm in an 800 mm arena) is deliberate: robots that seed at
exactly the same position produce zero spring force and never move. The angular
offset per ID breaks that degeneracy.

## Calibration constants

Defined in [src/formation.h](src/formation.h). Override at build time with
`-- -DDEMO_<CONSTANT>=<value>`.

| Constant | Default | Meaning |
|---|---|---|
| `DEMO_MAX_SPEED` | 135.0 | Odometry linearisation constant (see below) |
| `DEMO_WHEEL_TRACK` | 10.6 | Wheel-center to wheel-center, logical units (800 mm arena) |
| `DEMO_TARGET_SPACING` | 50.0 | Showcase mode: desired peer spacing → equilibrium ≈ 43 units = 341 mm |
| `DEMO_TRACK_MAX_FORCE` | 40.0 | Choreo mode: full-range attraction toward the commanded FORM/HOLD point |
| `DEMO_TRACK_SLOW_RADIUS` | 15.0 | Choreo mode: attraction ramps down inside this range of the target |
| `DEMO_TRACK_ARRIVE_EPS` | 2.0 | Choreo mode: command zero motion inside this range (arrival snap) |
| `DEMO_TRACK_MIN_SEP` / `DEMO_TRACK_EMERGENCY_K` | 20.0 / 3.0 | Choreo mode: emergency repulsion backstop against too-close peers |

**Recalibrating for a different arena:**

The Cutebot motor curve is highly non-linear — speed barely increases above 75%
throttle. Calibrate `DEMO_MAX_SPEED` at the speed actually commanded by the
formation (22%), not at 100%:

```
DEMO_MAX_SPEED   = speed_mm_per_s_at_22pct / 0.22 × 100 / arena_width_mm
DEMO_WHEEL_TRACK = cutebot_track_mm / arena_width_mm × 100
```

Measured fleet average: 238 mm/s at 22% throttle. Cutebot Mini wheel track: ~85 mm.
Use `examples/motor-test` to measure per-robot stiction threshold and speed before
running the formation demo. Per-robot `DEMO_MAX_SPEED` values can be passed at
build time with `-- -DDEMO_MAX_SPEED=N`.

Use `examples/motor-test` to measure actual stiction threshold and speed
per robot before running the formation demo.

## Tuning tips

- **Robots don't scatter on cold start** — check that robots are seeding near
  the same position (all starting within a 3-unit cluster). If `n_total` was
  wrong at boot (ID negotiation failed), initial positions may be degenerate.
- **Robots won't move after a peer is removed** — `FORCE_START` too high. The
  redistribution force after removing one robot from an N-bot equilibrium is
  approximately `2 × SPRING_K × equilibrium_offset`. Default `FORCE_START = 50`
  is tuned for the 50-unit target spacing with up to 3 active peers.
- **Oscillation / chatter near equilibrium** — `FORCE_START` too low. Raise it
  above the force produced by a one-cycle positional overshoot.
- **Formation too tight / too spread** — adjust `DEMO_TARGET_SPACING`.

## Testing

`tests/` is a host-buildable ztest suite (no BLE, no hardware) covering
`demo_track_target()`'s go-to-point law (arrival snap, trapezoidal
approach, the exact-180°-behind reverse case, the emergency repulsion
backstop) and the form-grid script end-to-end, driven through real
`demo_odometry_t` + `demo_track_target` + `demo_odometry_update`
integration (not a teleporting perfect tracker) — matches
`cf21bl-formation/tests`'s pattern.

```sh
west build -p always -b native_sim/native/64 tapestry/examples/cutebot-formation/tests
./build/zephyr/zephyr.exe
```

Not run against real Zephyr/native_sim as part of writing this suite (no
`west`/`ZEPHYR_BASE` toolchain available in that session) — the same
scenarios were instead verified with a standalone host build (plain
`clang`, no Zephyr, linking `formation.c` directly against the real
`scr.c`/`bse.c`/`choreo.c`/`world_model.c` sources) and passed, including
the 4-element form-grid end-to-end run. Run this suite for real the first
time `native_sim` is available, before trusting it as a regression gate.

## Known limitations

Dead-reckoning drifts. World-model positions are computed entirely from motor
commands, not physical sensing. Picking up a robot and repositioning it
physically has no effect on its self-reported position until it moves under
motor power again. The planned fix is to replace dead-reckoning distance with
RSSI-based proximity from the BLE scan callbacks (`transceiver_ble.c` already
receives `rssi` per peer).

**FORM's `abs_position` capability claim is weaker than its usual meaning.**
`SCR_CAP_ABS_POSITION` (declared in `main.c`'s `scr_init()` call, and
implicitly required by FORM's default `frame = "absolute"`) normally means
a real absolute-position sensor (lighthouse, GPS, mocap). Cutebot has none
— it satisfies the capability with dead reckoning from each robot's shared
`compute_start_pos()` seed, the same assumption the original spring-field
demo already made implicitly. This is honest (every robot's frame agrees by
construction, same as before), but the grid's absolute placement will drift
off-center over a long mission in a way a real abs-position sensor would
not. See `form-grid.choreo.toml`'s own comment for the full rationale.
