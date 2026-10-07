# Demo — Cutebot Collective Formation

BBC micro:bit V2 + Cutebot robots (Mini or Pro) self-organize using the Tapestry
world model and BLE gossip. No central controller. This is the L1-L7
Choreo reference example on a differential-drive ground rover — a
different substrate class from `cf21bl-formation`'s lighthouse-tracked
quadrotor, and the first consumer of Choreo's FORM goal to run on real
hardware anywhere in this repo.

One normal build mode (Kconfig choice `DEMO_MODE`, see `Kconfig`), plus a
handful of single-robot diagnostic modes documented there
(`DEMO_MODE_STRAIGHT_LINE`, `DEMO_MODE_CONVERGE_TEST`,
`DEMO_MODE_WHEEL_CHARACTERIZE`, `DEMO_MODE_LINE_SENSOR_BENCH`):

- **`DEMO_MODE_CHOREO` (default)** — a real L5 SCR + a declarative L7
  Choreo (`ring.choreo.toml`, "choreo-1") driven through the L6
  BSE: hold the boot-time station, form an equidistant ring, hold the
  settled ring. **Run on real Cutebot hardware** — four-robot runs on
  2026-08-24 (grid script) and 2026-09-12 (ring script, after the
  FORM/HOLD-debounce and sync-barrier fixes) both reached and held their
  target formation cleanly. Read "Known limitations" below for what that
  establishes and does not.

## How it works

1. Each robot advertises its dead-reckoning position over BLE.
2. Peer positions are received into the local L4 world model.
3. The micro:bit 5×5 LED matrix displays the robot's dead-reckoning position
   in real time (one lit pixel = estimated location in the 100×100 logical world).

A real L5 SCR (`scr_init()`/`scr_tick()`) tracks quorum from the actual
world model, and a declarative L7 Choreo
(`ring.choreo.toml`, compiled to `src/choreo_script.h` by
`sdk/tools/choreoc.py`) drives the robots through the L6 BSE:

1. **hold** — station-keep at the boot-time position (coordinate-free).
2. **form** (shape=circle, frame=absolute) — arrange into an equidistant
   ring centered on the arena, via `demo_track_target()`'s
   differential-drive go-to-point controller (`formation.c`) — a
   proportional turn-then-drive law (turn toward the commanded point,
   drive forward, decelerate on approach, with an emergency-repulsion
   backstop against too-close peers), the *first* real-hardware-targeted
   consumer of Choreo's FORM goal in this repo.
3. **hold** ("settled") — freezes the achieved ring position. Only
   re-forms on a *debounced* `element_lost`/`element_joined` (a real,
   sustained membership change), not on ordinary BLE gossip jitter — see
   `ring.choreo.toml`'s own comments for why this replaced an earlier
   design that recomputed the ring every tick and visibly hunted on
   noise.

Choreo completion (quiescence) maps to simply holding still — there is no
takeoff/landing concept for a ground rover.

LEDs follow the active step's declared indicator effect
(`choreo_current_indicator()`) when one is set, falling back to
`demo_set_leds()`'s own peer-freshness heuristic otherwise:
- **Red** — no fresh peers (isolated or booting)
- **Yellow** — some peers stale
- **Green** — all known peers fresh

To change the Choreo: edit `ring.choreo.toml`, then regenerate:

```sh
python3 sdk/tools/choreoc.py tapestry/examples/cutebot-formation/ring.choreo.toml \
    -o tapestry/examples/cutebot-formation/src/choreo_script.h
```

Other scripts in this directory compile the same way (swap the `.toml` name;
`src/choreo_script.h` stays the ring unless you deliberately regenerate it):
`form-grid.choreo.toml` (the earlier near-square grid), and the two
discovery demos below.

### Discovery demos: `spill.choreo.toml`, `leak.choreo.toml`

A hidden **IR-dark target** is printed on the board (ink or IR tape).  
A **120 mm printed disc** is what leak was validated with.  
Each robot patrols the board, and when a
ground IR sensor confirms a dark hit it latches its own (local) `discovered`
state and **stops where it stands**.

- **spill** — no ring, no tracks: each robot that crosses the (larger, odd)
  target stops on its own edge, green; robots that never touch it hold
  orange.  The finished picture is the target's cordon.
- **leak** — disc symbolizes a leak, all search and the finder rejoins: the
  finder publishes where it found it as a **shared fact** (L4, wire v7) that
  every robot holds and re-sends; everyone parks for 4 s (`found`), then ALL
  robots, finder included, form a radius-20 circle around that point
  (`anchor = { select = "landmark", type = "target" }`).  Switch one robot
  OFF (lifting it is not removal: it keeps gossiping) and after ~6 s the rest
  re-form around the same spot — four robots make a square, three an
  equilateral triangle.  Switch it back ON and it relearns the fact from
  whichever peer it hears first, so it rings the same point (before wire v7
  it took a peer's ring slot for the leak).  Leak searches with a symmetric outward spiral (FORM circles
  about the centre, stepping `rotation` 20° and the radius 22 → 37 per leg,
  on a shared clock), so place the marker **2 to 3½ squares from the
  centre, in any direction**: (30,30), (70,30), (70,70) and (30,70) are all
  found on the first leg out.

Build and flash (from the workspace root; the committed ring header is
untouched if you write the header to a scratch copy, or regenerate it
back afterwards):

```sh
python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cutebot-formation/leak.choreo.toml \
    -o tapestry/examples/cutebot-formation/src/choreo_script.h
west build -b bbc_microbit_v2 tapestry/examples/cutebot-formation -p always \
    -- -DCONFIG_CUTEBOT_VARIANT_PRO=y -DEXTRA_CONF_FILE=ble_debug.conf
```

(Drop `-DCONFIG_CUTEBOT_VARIANT_PRO=y` for the Mini, and `ble_debug.conf`
for demo firmware without wireless telemetry.) Grid correction
(`DEMO_GRID_CORRECTION`) defaults to `n`: on the Mini it would snap the
estimate to the nearest gridline on ANY dark hit, so the marker itself
would shift the finder's estimate by up to ~5 units, and on this printed
board neither robot's sensors see the squares reliably anyway.

**Detection rules** (`demo_discovery_detect()` and its gates in `main.c`):
- Only a robot driving **forward** can discover; a robot that is stationary,
  lifted by hand, or within **0.5 s of a backup** (the front rocks up and reads
  dark) cannot.
- **leak** (filled disc): two consecutive dark polls, and once a target fact
  is held the robot ignores its own dark hits (one finder).  A find within 15
  units of a known target reuses it rather than publishing a second one.
- **spill** (boundary outline of any width): a single sensor entry edge is
  enough, and every robot that crosses stops. The Mini's line sensors are
  interrupt-driven, so even a ~20 ms crossing between two polls counts; the
  Pro polls its four sensors from a background thread
  (`cutebot_pro_line_watch_*()`, ~20 samples/s measured) for the same
  effect.
- Armed only for scripts that react to a find (`CHOREO_USES_DISCOVERY`, emitted
  by `choreoc`); the ring is untouched.  Scripts that read landmarks
  (`CHOREO_USES_LANDMARKS`) also publish the find as a target fact; a
  rebooted robot publishes only after it has heard its peers' fact table.

**Bring-up:** `-- -DCONFIG_DEMO_MODE_DISCOVERY_BENCH=y` drives one robot
straight and logs every dark hit; lift the robot and the sensors see "no
surface", which reads dark too, so detection is gated to the trusted playing
area `[10, 90]` (`demo_discovery_in_safe_zone()`).  The blue lift LEDs are
deliberately not used.

**Placement constraints** (spill, from host simulation of 4
robots with wheel mismatch and 20% gossip loss — dead reckoning only, grid
correction off; leak's spiral has its own rule, above):
- Robots start on a cross about the centre (ids 0/1/2/3 at E/N/W/S, ±10).  A
  target within ~15 units of the centre is "found" at t = 0.
- The patrol is 9 circle legs of radius 20 (a translated copy of the boot
  formation), ideal coverage ~71% of marker positions; the outer band and
  the inner corners between lines can be missed.  A marker is only found if
  some robot's path crosses it.
- Two robots may cross the target before either hears the other. In leak
  their two target facts merge (within 15 units) into the earlier-stamped
  one on every robot, so all ring one centre; before wire v7 they could ring
  two (seen on hardware, 2026-10-05).
- Accumulated drift over the ~2 min lap is uncorrected: expect final
  positions off by several units.

## Hardware

| Item | Details |
|---|---|
| Board | BBC micro:bit V2 (nRF52833, Cortex-M4F) |
| Robot chassis | Elecfreaks Cutebot Mini (default) or Cutebot Pro (see below) |
| Arena | 1266.25 mm × 1266.25 mm (printed chessboard: 8 squares + 1-square border) |
| Ring diameter | ~760 mm (radius 30 logical units, `ring.choreo.toml`) |

### Hardware variants (Kconfig choice `CUTEBOT_VARIANT`)

- **`CUTEBOT_VARIANT_MINI` (default)** — the original hardware every mode in
  this example was built and hardware-validated against. Open-loop drive
  (`cutebot.c`, raw I2C percentage commands), 2 interrupt-driven GPIO line
  sensors (`cutebot_line.c`).
- **`CUTEBOT_VARIANT_PRO`** — ELECFREAKS Cutebot Pro (V2.1.0), which has
  replaced the Mini fleet and runs ring and leak on hardware. Closed-loop
  encoder motors (measured wheel distance, driven via the controller's own
  firmware PID) and a 4-way I2C line sensor read as raw gray levels. Its
  constants are bench-measured (`CUTEBOT_PRO_WHEEL_TRACK_MM` 92 mm,
  `CUTEBOT_PRO_DIST_COUNTS_PER_CM` 22.56, `formation.h`), and each robot
  has its own turn trim (below). `DEMO_MODE_ENCODER_BENCH`,
  `DEMO_MODE_CIRCLE_BENCH` and the Pro branch of
  `DEMO_MODE_LINE_SENSOR_BENCH` remain for checking a unit;
  `DEMO_MODE_STRAIGHT_LINE` and `DEMO_MODE_CONVERGE_TEST` are Mini-only
  (built around the Mini's line-crossing timing). See `Kconfig`'s
  `CUTEBOT_VARIANT_PRO` help text for the protocol, verified against
  ELECFREAKS' own `pxt-Cutebot-Pro` source.

  Build with:
  ```sh
  west build -b bbc_microbit_v2 tapestry/examples/cutebot-formation \
    -- -DCONFIG_CUTEBOT_VARIANT_PRO=y
  ```

#### Cutebot Pro: turn calibration

Heading error is what makes the Pro's dead-reckoning estimate drift, and
each robot's encoders mis-measure rotation by a few percent, CCW and CW
differently. Each robot's two scales live in `src/turn_calibration.h`,
keyed by its micro:bit's hardware id, and are applied to every turn the
odometry measures; a robot without an entry logs `no turn cal for hw 0x…`
at boot and uses 1.0. Compiled in, because flashing a micro:bit erases its
whole flash; re-measure if a micro:bit moves to another chassis.

To measure a robot:
```sh
west build -p always -b bbc_microbit_v2 -d build-cal tapestry/examples/cutebot-formation \
  -- -DCONFIG_CUTEBOT_VARIANT_PRO=y -DCONFIG_DEMO_MODE_TURN_CALIBRATE=y \
     -DEXTRA_CONF_FILE=ble_debug.conf
```
Flash it and switch on; with the observer logging, it announces itself
every 15 s:
```
turn cal hw 0xdd4e88f3: A = CCW, B = CW, 5 turns (table 1.0000/1.0000)
```
Line the robot up with a reference, then press **A** (CCW) three times in
a row **without re-aligning**, stepping back each time: each press spins
five turns by the encoder (counted with the robot's current trim) and logs
```
turn cal CCW run 1: trim 1.0000, expect +2.6 deg (raw 1802.6, 7.3 s)
```
Read how far the robot ended past (+) or short of (−) its starting heading
after all three presses, then do the same with **B** (CW). For each
direction:

    scale = (5400 + offset_deg) / (sum of the three "raw" values)

Measuring 15 turns at once makes a 10° reading error only 0.2%, where
re-aligning each run proved unreliable. With the trims in place, `expect`
should match what you see to within a few degrees.


## Build

One binary runs on all robots — no per-robot build flags needed.
Run from the workspace root (`tapestry-workspace/`):

```sh
west build -b bbc_microbit_v2 tapestry/examples/cutebot-formation
```

Builds `DEMO_MODE_CHOREO` by default — this is the only normal build mode
now (see Kconfig for the diagnostic single-robot modes).

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

**Physical placement — power on first, then place**
IDs are assigned by FICR nonce rank at boot, so you can't know which physical
robot gets which ID before power-on. Rather than pre-labelling hardware via a
one-time serial connection, each robot announces its own negotiated ID by
blinking its LEDs white — `id + 1` short flashes, repeated a few times —
immediately after auto-ID completes and before it starts moving. Power on all
robots together, then place/orient each one while it's blinking, reading its
ID off the flash count.

Orient each robot facing outward from the center point of the arena, one
chessboard square out **along its own heading axis** — cardinal, not
diagonal. Find the point where the four central squares of the board meet
(the board's true center falls on a grid line, not inside a square, since 8
squares per side is even); each robot's own center then goes on the next
grid-line intersection straight out from there:

| ID | Heading | Face toward |
|---|---|---|
| 0 | 0° | right (east) — one square right of center, on the horizontal centerline |
| 1 | 90° | away from you (north) — one square away, on the vertical centerline |
| 2 | 180° | left (west) — one square left of center, on the horizontal centerline |
| 3 | 270° | toward you (south) — one square toward you, on the vertical centerline |

For three robots the angles are 0°, 120°, 240°.

This matters for dead-reckoning accuracy: the code initializes each robot's
heading to its outward angle to match this placement. If a robot's physical
orientation does not match its assigned heading, its dead-reckoning position
will drift in the wrong direction from the start.

**Cold start (robots clustered)**
Power all robots on within 8 seconds of each other. During the boot window
LEDs are red (no peers yet). After the window the sync barrier holds all
robots stationary (motors never commanded) until every expected peer is
confirmed fresh — so a late power-on can't put an early-booted robot at an
advantage. Once the barrier releases, the script executes: **hold** the
boot-time station, then **form** — the ring re-divides itself as the FORM
step runs until each robot reaches its equidistant vertex.

**Formation stable ("settled")**
Once the ring is achieved, FORM transitions to a named **hold** and freezes
there — LEDs hold solid green (or whatever the step's own indicator sets).
Unlike the FORM step itself, this hold does *not* recompute on ordinary
BLE gossip jitter; it only reacts to a debounced, sustained membership
change.

**Remove a robot**
Power off the robot first (gossip stops immediately), then pick it up.
Moving it without powering it off has no effect — the robot continues
advertising its last position over BLE regardless of physical location.

The missing robot's world-model entry goes stale on the remaining robots
after `WM_STALE_THRESHOLD_MS` (app-scoped to 3000 ms here — see
`CMakeLists.txt`'s comment for why this is looser than the 1500 ms core
default), then expires and is removed. Once that departure has held for
`CHOREO_MEMBERSHIP_HOLD_MS` (3000 ms, debounced so a single dropped gossip
frame can't fire it), the settled hold's `element_lost` transition fires
and the survivors re-run FORM to redistribute into a smaller ring.

**Rejoin (optional)**
A robot that is powered back on remembers neither its id nor its position.
It negotiates an id during its 8 s boot window (the rank-th id no running
robot claims — see Auto-ID), then seeds its dead-reckoning position from the
boot slot for that id, so **where you place it decides where it thinks it
is**. Placing it "anywhere" does not work.

- **One robot at a time, all the others still running** (the simple way):
  the only free id is the one it had, and it counts the same number of
  robots as at the start, so its boot slot is its original one. Put it back
  on its starting square, facing outward, and power it on.
- **Otherwise** (several robots off, or switched on together): the id can
  change, and so can the slot, which sits at angle 360° × id / n, one square
  from center, where n is the robot itself plus those it hears. Count its
  blinks (id + 1 flashes, repeated 4 times) and place it on that slot facing
  outward before the blinking ends.

Once its arrival has been fresh for `CHOREO_MEMBERSHIP_HOLD_MS`, the settled
hold's `element_joined` transition fires on the other robots and the ring
re-forms to include it.

## Wireless debug telemetry (untethered runs)

Build with `-DEXTRA_CONF_FILE=ble_debug.conf` (adds `CONFIG_DEMO_BLE_DEBUG`
and a second BLE advertising set; leave it out for demo firmware).  Each
robot then broadcasts, in LEGACY advertising format so any BLE scanner can
hear it:

- a status packet, ~2 Hz: step, quorum, fresh peers, estimate and heading,
  directive target, latched discovery point, darkest line-sensor reading;
- the key log events as text: step changes, discovery, `sees peer N
  DISCOVERED`, `latched discovery point`, `fresh peers a -> b`, quorum
  lost/recovered, auto-ID and barrier progress.

Every packet carries a sequence number, and each text chunk is sent twice,
so a receiver can drop duplicates and mark gaps.  Robots ignore these
packets (too short to be gossip).  See `src/ble_debug.h` for the packet
format.  To receive and log them, use `examples/ble-observer` (an nRF5340
DK plus `tools/observer_log.py`).

Debug builds also log, over serial every 5 s, how many debug packets the
robot queued and how many of each peer's it heard (`dbg tx: ... | heard
peers' dbg: ...`) — a quick check that the radio is not congested.

## Auto-ID

During an 8-second boot window each robot advertises its FICR hardware nonce
and listens for peers. After the window:

1. **Nonce rank** among co-booting robots → candidate rank (lower nonce = lower rank).
2. **Claimed IDs** from already-running robots (live gossip) are avoided.
3. **element_id** = rank-th unclaimed ID.

The window duration is set in `prj.conf` via
`CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS` (overrides the 4 s default in `transport.c`).

## Starting positions

All robots seed their world-model position on a `DEMO_START_RADIUS`-radius
circle around arena center (50, 50) — one chessboard square (`DEMO_SQUARE_
UNITS`, 10.0 logical units = 126.6 mm), placing 4 robots 179 mm apart
center-to-center at boot (see `compute_start_pos()` in `main.c`). This
clears the ~100 mm Cutebot width and lands each seed exactly on a grid
intersection, matching the placement table above.

The nonzero radius (rather than all four at exact arena center) is
deliberate: FORM's differential-drive controller needs a real, distinct
starting position and heading per robot to compute a sensible initial
turn — identical starting positions/headings would be a degenerate case.

## Calibration constants

Defined in [src/formation.h](src/formation.h). Override at build time with
`-- -DDEMO_<CONSTANT>=<value>`.

| Constant | Default | Meaning |
|---|---|---|
| `DEMO_MAX_SPEED` | 85.4 | Odometry linearisation constant (see below) |
| `DEMO_WHEEL_TRACK` | 6.71 | Wheel-center to wheel-center, logical units (chessboard arena) |
| `DEMO_TARGET_SPACING` | 39.5 | Turn-gain normalization only (see `formation.h`) — not a real spacing target since Choreo removed the spring field |
| `DEMO_TRACK_MAX_FORCE` | 40.0 | Full-range attraction toward the commanded FORM/HOLD point |
| `DEMO_TRACK_SLOW_RADIUS` | 9.5 | Attraction ramps down inside this range of the target |
| `DEMO_TRACK_ARRIVE_EPS` | 2.0 | Command zero motion inside this range (arrival snap) |
| `DEMO_TRACK_MIN_SEP` / `DEMO_TRACK_EMERGENCY_K` | 25.3 / 4.0 | Emergency repulsion backstop against too-close peers |
| `DEMO_TRACK_REPEL_DEADBAND` | 5.0 | Holds (zero output) instead of chattering when repulsion nearly cancels attraction |

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

- **A robot never starts moving** — check `n_total` in the boot log; if ID
  negotiation heard the wrong peer count, the sync barrier
  (`DEMO_SYNC_SETTLE_MS`, `main.c`) never releases. See "Auto-ID" below.
- **Settles, then keeps re-adjusting a little ("hunting")** — this was the
  2026-09-11 bug fixed by the FORM→named-HOLD transition in
  `ring.choreo.toml`; if it reappears, check that the script still
  transitions to a genuine `hold` step on `achieved` rather than looping
  back into `form` unconditionally.
- **Ring re-forms too eagerly / too sluggishly on membership change** —
  adjust `CHOREO_MEMBERSHIP_HOLD_MS` / `CHOREO_REMOTE_ADOPT_HOLD_MS`
  (`CMakeLists.txt`, currently 3000 ms) — the debounce window before a
  peer loss/gain is treated as real rather than transient BLE gossip
  jitter.
- **Formation too tight / too spread** — adjust `ring.choreo.toml`'s
  `radius`, keeping `DEMO_TRACK_MIN_SEP` (formation.h) below the
  resulting inter-robot spacing so repulsion doesn't fight the formation
  at equilibrium.
- **Two robots collide or oscillate near each other** — see
  `DEMO_TRACK_MIN_SEP` / `DEMO_TRACK_EMERGENCY_K` / `DEMO_TRACK_REPEL_
  DEADBAND`'s docs (`formation.h`) for the emergency-repulsion backstop's
  tuning history and known limits (it settles a head-on approach to a
  safe standoff, not a completed pass-by — see `bse.h`'s EXCHANGE arc for
  why a real swap avoids this case by construction).

## Testing

`tests/` is a host-buildable ztest suite (no BLE, no hardware) covering
`demo_track_target()`'s go-to-point law (arrival snap, trapezoidal
approach, the exact-180°-behind reverse case, the emergency repulsion
backstop) and the ring Choreo end-to-end (hold → form circle → settled
hold), driven through real `demo_odometry_t` + `demo_track_target` +
`demo_odometry_update` integration (not a teleporting perfect tracker) —
matches `cf21bl-formation/tests`'s pattern.

```sh
west build -p always -b native_sim/native/64 tapestry/examples/cutebot-formation/tests
./build/zephyr/zephyr.exe
```

Not run against real Zephyr/native_sim as part of writing this suite (no
`west`/`ZEPHYR_BASE` toolchain available in that session) — the same
scenarios were instead verified with a standalone host build (plain
`clang`, no Zephyr, linking `formation.c` directly against the real
`scr.c`/`bse.c`/`choreo.c`/`world_model.c` sources) and passed, including
the 4-element ring end-to-end run. Run this suite for real the first
time `native_sim` is available, before trusting it as a regression gate.

## Known limitations

Dead-reckoning drifts. Positions come from wheel motion alone (motor
commands on the Mini, encoders on the Pro); nothing re-anchors them to the
board. Grid correction exists (`demo_grid_correct()` /
`demo_grid_heading_correct()` on the Mini, `demo_junction_correct()` on the
Pro) but is off by default: this printed board's ink is nearly
transparent to the robots' IR sensors (Pro: black and white squares 4-14
counts apart per channel out of 255), and the Pro's junction snap fires on
black squares. Heading error dominates, so the Pro applies a measured
per-robot turn trim (`turn_calibration.h`); even so, a robot was ~10-15
units off after ~20 s on 2026-10-05, so keep runs short and finds early.
`demo_arena_fence()` (`formation.c`) only bounds how far a robot can be
COMMANDED to travel relative to its own estimate; it does not correct the
estimate.

**Operating conditions that break detection.**
- **Sunlight** (and likely halogen or incandescent spotlights) saturates
  the IR line sensors to 255, which reads as "marker": a run in a sunny
  room produced a false find and then every robot reading 255 wherever it
  stood. Shade the board.
- **A failing battery or chassis.** The Pro's I2C replies carry no
  checksum; a chassis dropping off the bus returned nonsense encoder
  counters that teleported the estimate 50-66 units. Such reads are now
  rejected and logged as `encoder read implausible` (BLE text and serial):
  a robot logging that needs its battery or connections checked. The four
  battery LEDs on the chassis are its own and cannot be read; they can show
  full on a battery that sags under motor load.

**FORM's `abs_position` capability claim is weaker than its usual meaning.**
`SCR_CAP_ABS_POSITION` (declared in `main.c`'s `scr_init()` call, and
implicitly required by FORM's default `frame = "absolute"`) normally means
a real absolute-position sensor (lighthouse, GPS, mocap). Cutebot has none
— it satisfies the capability with dead reckoning from each robot's shared
`compute_start_pos()` seed. This is honest (every robot's frame agrees by
construction), but the ring's absolute placement will drift off-center over
a long mission in a way a real abs-position sensor would not. See
`ring.choreo.toml`'s own comment for the full rationale.
