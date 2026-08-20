# Demo — CF2.1 Brushless Collective Formation

Crazyflie 2.1 Brushless quadrotors coordinating via the Tapestry stack,
REAL lighthouse position (not dead reckoning), and gossip over the
nRF51822's syslink P2P radio channel.  Two build modes (Kconfig choice
`DEMO_MODE`):

- **Choreo mode (default)** — the first flight of the L6/L7 layers: a
  declarative L7 Choreo script ("hold stations → exchange places → rest")
  drives the drones through the L6 Behavior Synthesis Engine.  **One
  binary for every drone** — element IDs are negotiated at boot over the
  radio (same auto-ID protocol as `examples/cutebot-formation`).  See
  "Choreo demo" below.
- **Showcase mode** (`-DCONFIG_DEMO_MODE_SHOWCASE=y`) — the
  flight-validated 2026-07 L4 spring-field showcase (line → rotating
  triangle → member departs → line), per-drone builds.  See "Showcase
  demo" below.

This is the multi-drone follow-on to the flight-validated single-drone
stack (lighthouse XY position hold + baro altitude hold in
`cf21bl_stabilizer.c`). For a gentler on-ramp, `examples/altitude-hold-tether`
exercises the altitude loop alone and `examples/lighthouse-test` the
positioning alone.

## Choreo demo (default mode) — swap places

The entire application-level "program" is
**`change-partners.choreo.toml`** in this directory — a three-step,
coordinate-free script a non-programmer can edit cold. (The
`<name>.choreo.toml` naming — name matching the script's own
`choreo = "<name>"` key — is the project-wide convention for Choreo
scripts; see `sdk/CHOREO_SCRIPTS.md`.)

```toml
choreo = "change-partners"

[[steps]]
hold = { duration = "10s", requires = ["locomotion"] }

[[steps]]
[steps.exchange]
until    = "achieved"
timeout  = "30s"
path     = "direct"    # beeline — safe here: deconfliction is vertical
eps      = "25cm"
settle   = "3s"
requires = ["locomotion"]

[[steps]]
hold = { duration = "8s", requires = ["locomotion"] }   # bow
```

`path = "direct"` beelines each drone straight to its destination (~4 s
for a 1 m swap at the tracker speed limit) — safe on this platform
because the ID-staggered altitudes deconflict the crossing.  Omit it (or
`path = "arc"`) to get the default centroid-arc maneuver, which preserves
XY separation for platforms with no vertical dimension (ground robots).

The firmware consumes a committed generated header
(`src/choreo_script.h`, same pattern as `examples/lighthouse_cal.h`).
After editing the TOML, regenerate and rebuild:

```sh
python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cf21bl-formation/change-partners.choreo.toml
```

`choreoc` is standard-library-only Python (>= 3.11, system python3 — no
venv, nothing to install) and validates the script before emitting
anything: every step must be time-bounded (the timeout is the robustness
net that keeps a script from stalling in flight — stricter than the C
API on purpose), coordinate goals must have coordinates, and hold /
exchange must NOT (they reference the collective's own configuration).
The Python SDK reads the same file directly, no generation step:
`choreo.submit_script(tapestry.script_toml.load_steps("change-partners.choreo.toml"))`.

What the steps mean:

1. `hold` (10 s) — each drone station-keeps at its own position.
2. `exchange` — swap places: each drone takes its partner's station.
   Stations are frozen snapshots of peer positions at step activation
   (from the L4 world model — nothing is prescribed); with `path =
   "direct"` the commanded target beelines straight to the destination
   (~4 s for a 1 m swap), safe here because altitude staggering
   deconflicts the crossing vertically.  Advances on the **collective**
   achievement predicate (`scope = "all"` — within `eps` of the
   destination for `settle`, 25 cm / 3 s by default, AND its gossiped peer
   reports the same), with a 30 s timeout as the robustness net — if
   lighthouse jitter keeps achievement from ever firing, the show still
   ends cleanly. `scope = "all"` is also what keeps the first finisher
   airborne and gossiping until its partner catches up: its own step
   won't advance until the collective predicate is true.
3. `hold` (8 s, the "bow") — a deliberate settle beat on the new stations
   before landing, not a sync mechanism (that's `scope = "all"` above).

The script never says "take off", "land", or any altitude: script
completion → directive IDLE → **quiescence**, which this platform maps to
landing in place and disarming.  Takeoff is the same mapping in reverse —
a parked drone holding a MOVE directive activates.  Altitude staggering
(0.30/0.50/... m by negotiated ID) is a platform deconfliction rule the
Choreo never sees, and it is also what makes the swap trivially safe in
the vertical dimension.

Safety layers are unchanged from the showcase (see "Safety layer" below);
the mission-duration backstop in choreo mode is derived from the script
(hold + exchange timeout + bow + 40 s margin) and only fires if the
script stalls, e.g. the partner is lost mid-show (quorum loss suspends
the script — frozen timers — and freezes the target).

### Build + fly (2 drones, ONE build for both)

```sh
west build -p always -b crazyflie21bl tapestry/examples/cf21bl-formation
cfloader flash build/zephyr/zephyr.bin stm32-dfu     # same binary, both drones
```

Before the first flight, bench-test auto-ID over the radio with motors
disabled (`-- -DCONFIG_PWM=n`, or props off): power both drones on
together and confirm unique `auto_id:` ids and `n_total=2` on both
consoles.  This is new radio behavior (discovery beacons over syslink
P2P) — validate it before anything spins.

### Radio triage (when auto-ID hears nobody)

A drone whose window heard no peers refuses to arm and becomes a
**self-healing** grounded diagnostic station (override:
`-DCONFIG_DEMO_ALLOW_SOLO=y`): it keeps gossiping and listening, and
recovers WITHOUT a power-cycle — either by spotting a distinct-ID peer
directly, or by renegotiating at a jittered 15–45 s interval (a
duplicate-ID peer's gossip claims the contested id during the window,
so the renegotiator takes the next free one).  Both drones then proceed
to flight on their own.  The diagnostic log streams, per drone:

| Log line | Meaning |
|---|---|
| `auto_id: t=... beacons_tx=N nonces_heard=0` | transmitting beacons but hearing none (live, 1 Hz during the window) |
| `GROUNDED-DIAG: ... window(beacons= nonces= running=)` | the retained outcome of the (possibly console-less) boot window, re-logged every 2 s |
| `GROUNDED-DIAG: DUPLICATE ID — N frames ...` | another element also holds this id; its gossip is arriving but was invisible to the world model — renegotiation will resolve it |
| `recovered: peer VISIBLE` / `recovered via renegotiation` | self-heal succeeded; the drone continues to flight prep |
| `p2p: tx=N rx_frames=M ...` (every 2 s, cumulative) | N = frames handed to the nRF51; M = valid P2P frames received.  `tx>0, rx_frames=0` here with `rx_frames>0` on the OTHER console = one dead link direction — suspect that TX antenna/nRF51 or this RX |
| `ck_fail` / `short` / `q_drop` climbing | syslink frames dying in the STM32 parser (byte loss under USART6 load) — the P2P gossip embedded in the same stream dies with them |

Counters and the window summary are cumulative, so a console attached
minutes after a console-less boot still tells the whole story.

1. Place both drones **at least 1 m apart** (e.g. on the (0,0) and (1,0)
   marks), noses along lighthouse world +X, **well inside** base-station
   coverage.  1 m = 2× `DEMO_MIN_SEP_M` — closer than that and
   station-keeping fights the separation repulsion for the whole flight
   (the firmware logs a warning at first contact if the peers are closer
   than 1 m in the shared frame; that warning also fires if a biased
   lighthouse frame merely *believes* they are close — either way, do not
   fly the script through it).
2. Power both on **within ~4 s of each other** — the 6 s auto-ID windows
   must overlap.  Watch the consoles for the `auto_id:` lines: each drone
   must report a **unique id** and `n_total=2` before flight.  A drone
   that heard nobody claims id=0 and will fly a solo script — if
   `n_total` is wrong, power-cycle both and retry.
3. Each drone waits for its lighthouse fix, counts down 5 s, arms, ramps
   to its ID-staggered altitude, and the script runs: 10 s of station
   hold, ~4 s direct-path swap, 8 s bow, then both land in place — each on
   its partner's original mark — and disarm.

This build carries a real L5 SCR (`scr_init()`/`scr_tick()` — quorum,
role, task slot, and the abort protocol, computed from the actual world
model, not a stand-in), with one flight-tested filter on top: the
quorum the Choreo sees is **debounced upward** — a peer only counts as
contact after ~2 s of sustained freshness (≥ 2 consecutive gossip
frames).  A single lucky packet through a bad radio window used to wake
the Choreo for a second at a time, letting the tracker's leash ratchet
the target toward whatever the (possibly corrupt) position estimate said
— sustained contact is required before anything moves.  Loss is still
immediate.  Per-goal quorum: `hold` steps station-keep even with zero
peers (they reference only the drone itself); `exchange` freezes without
quorum.

Console log markers: `choreo "change-partners" loaded`,
`choreo step 0/1/2`, `achieved`, `q=H` / `q=L(susp)` (debounced quorum /
suspended), and `choreo complete — resting`.

Tuning lives in `change-partners.choreo.toml` (edit → re-run `choreoc` →
rebuild), not in build flags.  Three or more drones work unmodified —
`exchange` becomes
"everyone move one seat around the ring" (stations rotate by one
position CCW).

### Choreo unit tests (native_sim)

The `tests/` suite now also covers the script engine end-to-end: the
hold→exchange→bow script with a perfect-tracking mirrored partner (swap
completes in ~26 s, separation never below 0.9 m, final station exact,
IDLE directive at completion), suspension freezing step timers,
exchange's hold-until-snapshot behavior, and rejection of unadvanceable
steps.

```sh
west build -p always -b native_sim tapestry/examples/cf21bl-formation/tests
./build/zephyr/zephyr.exe
```

## How it works (both modes)

1. Each drone reads its own absolute position from the lighthouse deck
   (meters, home-relative to the shared calibration origin) and gossips it.
2. Peer positions arrive into the local L4 world model — no odometry, no
   drift; every drone's reported position is ground truth.
3. `formation.c`'s holonomic spring field computes a target X/Y point
   directly (no heading/turn-rate — the stabilizer's position-hold loop is
   already absolute and holonomic).
4. Each drone converts that absolute target into its own home-relative,
   normalized setpoint via `cf21bl_stabilizer_get_pos_home()` and commands
   it through `substrate_move()`.
5. Altitude is fixed per drone (staggered by `element_id` — see below) and
   held closed-loop by `CONFIG_CF21BL_ALTITUDE_HOLD` (baro), independent of
   the lighthouse.
6. A per-drone safety layer (see `main.c`'s `flight_state_t` machine) lands
   a drone independently on fix loss, geofence breach, or mission timeout —
   see "Safety layer" below. Battery-critical landing is handled inside
   `cf21bl_stabilizer.c` itself, also independently per drone.

**Units**: positions in this example are meters in the lighthouse world
frame, NOT the abstract 0–100 logical-unit space `cutebot-formation`
uses — see the comment block at the top of `src/formation.h`.

## Requirements

- All three drones must be flashed with the **same** lighthouse base-station
  poses and OOTX calibration (`src/main.c`'s `BS0`/`BS1`/`BS0_CALIB`/
  `BS1_CALIB`) — gossiped positions are only comparable in a shared frame.
  Current values come from the single shared header
  `examples/lighthouse_cal.h` (generated from the YAML next
  to it) — if the room is recalibrated, update that ONE file.
- Each drone needs its own lighthouse deck (USART3, PC10/PC11 — see
  `cf21bl_lighthouse.c`) and a working BMP390 baro for altitude hold.

## Build — showcase mode (one build per drone)

Choreo mode (default) needs no per-drone flags — see "Choreo demo" above.
Showcase mode keeps the per-drone build workflow:

```sh
west build -p always -b crazyflie21bl tapestry/examples/cf21bl-formation \
  -- -DCONFIG_DEMO_MODE_SHOWCASE=y \
     -DCONFIG_TAPESTRY_ELEMENT_ID=0   # 1, 2 for the other two drones
```

Flash: `cfloader flash build/zephyr/zephyr.bin stm32-dfu`

Console: CRTP radio only — the lighthouse deck takes USART3.
`python3 ~/code/tapestry/read_console.py`. With 3 drones transmitting on the
same nRF51 radio config note the id of each drone in the log and 
treat concurrent consoles as best-effort, or build using the compiler
flag `-DCONFIG_LOG=n` to quiet all but one drone.

Gossip-only bench test (no motors): add `-DCONFIG_PWM=n` — runs the L4
gossip/transport/lighthouse stack against `substrate_null.c`, useful for
verifying multi-drone position exchange before arming anyone.

## Showcase demo (line → rotating triangle → member departs → line)

Legacy L4-only mode — build every command below with
`-DCONFIG_DEMO_MODE_SHOWCASE=y` (the per-drone flags `DEMO_START_DELAY_S`,
`DEMO_MISSION_DURATION_S`, and `DEMO_HOLD_STATION` only exist in this
mode).  The formation phase is a pure function of how many fresh peers
each drone sees — no leader, no phase timers, no uplink:

- **1 fresh peer** → pair phase: springs hold 1 m spacing, the alignment
  term (`DEMO_ALIGN_ROT_RADPS`) rotates the pair about its centroid until
  its axis lies along world X (a torque — it cannot fight the springs).
- **2 fresh peers** → triangle phase: springs relax into the equilateral
  triangle and the rotation term (`DEMO_ROT_OMEGA_RADPS`) orbits it slowly
  about the shared centroid.
- A drone that reaches its mission duration lands, **goes gossip-silent**,
  and is expired by its peers (~5 s) — the survivors pause briefly
  (hold-on-stale), then re-form the line. Same code path as a real failure.
- A weak **centroid anchor** (`DEMO_ANCHOR_*`, identical for every drone —
  pure translation, still leaderless) parks the formation over the ground
  marks; without it the field controls only shape and orientation, and the
  show drifts wherever disturbances push it. Set `DEMO_ANCHOR_K` to 0 for
  pure-emergence mode.

Builds (power all three on together; id=1 stages itself):

```sh
# id=0 and id=2 — fly the whole show (~75 s airborne)
west build ... -- -DCONFIG_TAPESTRY_ELEMENT_ID=0 -DCONFIG_DEMO_MISSION_DURATION_S=75
west build ... -- -DCONFIG_TAPESTRY_ELEMENT_ID=2 -DCONFIG_DEMO_MISSION_DURATION_S=75

# id=1 — joins 15 s late, leaves 30 s after arming (~34 s airborne)
west build ... -- -DCONFIG_TAPESTRY_ELEMENT_ID=1 \
  -DCONFIG_DEMO_START_DELAY_S=15 -DCONFIG_DEMO_MISSION_DURATION_S=30
```

Ground placement (world frame, all noses along +X, tape-mark these):

| Drone | Spot (x, y) | Note |
|-------|-------------|------|
| id=0  | (0.00, 0.00) | On the calibration origin — its first `fix` line should read ≈(0,0,0), which doubles as the pre-show calibration sanity check |
| id=1  | (0.50, 0.67) | Near the triangle's north apex (true apex is y=0.87; the springs nudge it the last 0.2 m on climb) |
| id=2  | (1.00, 0.00) | East end of the line |

Approximate timeline from simultaneous power-on: line by ~25 s; id=1 airborne
~32 s and triangle forms; rotation through ~60 s; id=1 lands and goes silent
~62 s; line re-forms ~67 s; id=0/id=2 land ~88 s.

## Safety layer

| Trigger | Detected by | Response |
|---|---|---|
| Own battery critical | `cf21bl_stabilizer.c` (`CONFIG_CF21BL_PM`) | Independent forced landing — pre-existing, not formation-specific |
| Own lighthouse fix lost | `main.c` | Zero X/Y immediately (avoids the stabilizer's fix-lost velocity-feedforward fallback inheriting a stale position value); land after `FIX_LOSS_GRACE_MS` (2 s) sustained loss |
| Geofence breach | `main.c`, checked against the lighthouse origin (not this drone's home) | Independent landing |
| Mission duration elapsed | `main.c`, per-drone timer from its own arm time | Independent landing — the closest thing to a "coordinated" land without a wireless uplink, since all drones arm within the same sync window |
| Minimum separation violated | `formation.c` (extra repulsion below `DEMO_MIN_SEP_M`) | Stronger repulsion force + a logged warning in `main.c`; not a landing trigger |

Every trigger above is per-drone local state — one drone landing never
affects another's flight.

Separately, on a real quorum-loss edge (a peer's gossip actually going stale, not one
of the local triggers above) this drone gossips its own state immediately at
`TAPESTRY_QOS_HARD_RT` instead of waiting for the next scheduled cycle, so a peer
losing quorum is heard about as fast as the radio allows — watch flight logs for
`quorum LOST — sending HARD_RT gossip now`. See `examples/webots-formation/README.md`'s
"Testing HARD_RT quorum-loss gossip" for how to exercise this without flying.

## Flight checklist

1. Flash all three drones with IDs 0, 1, 2 (same BS calibration on all three).
2. Place each drone in the arena, **nose along lighthouse world +X**
   (yaw hold locks heading at boot, and world-frame corrections assume
   that boot heading — see the note in `prj.conf`), spaced roughly
   `DEMO_TARGET_SPACING_M` apart. Exact placement doesn't matter — the
   stabilizer captures each drone's own position as its home the instant
   this file starts commanding a non-idle altitude, and the spring field
   converges regardless of starting layout.
3. Power on all three within the sync window (a few seconds of slack).
4. Each drone independently: gyro cal, waits for its own lighthouse fix
   (aborts to sleep if none within 30 s), 5 s countdown, arms, ramps
   gently to its ID-staggered cruise altitude, then joins the formation.
5. After `MISSION_DURATION_S` (60 s default), every drone lands and
   disarms independently.

## Fleet bring-up (do this once per new drone before flying it here)

0. **ESC firmware** (once per new airframe): this project's motor path
   (RC PWM 400 Hz or OneShot125 through the STM32 timer, PC15 reset
   sequencing in `crazyflie21bl.c`) requires the ESCs to run **BLHeli_S
   with input-protocol auto-detect**. Validated configuration: BLHeli_S
   16.7, flashed/configured via Betaflight passthrough + ESC Configurator
   (esc-configurator.com — the target reads "O-H-10 - BLHeli_S, 16.7").
   While connected, verify the **PPM min/max throttle endpoints are
   identical on all four ESCs** (1000/2000).
1. Flash + confirm: ESC RC-PWM (or OneShot125) auto-detect arms cleanly,
   gyro calibration completes without warnings, lighthouse deck attached
   and reading (`examples/lighthouse-test` is a good bench check in
   isolation), battery telemetry flowing (`examples/altitude-hold-tether`
   with `CONFIG_CF21BL_PM=y` is a good bench check).
2. Confirm this drone's `CONFIG_TAPESTRY_ELEMENT_ID` is unique in the fleet.
3. Bench-test the syslink P2P transport alone first: build two units with
   `-DCONFIG_PWM=n` (gossip-only) and confirm each sees the other's
   `element_id` in the world model over radio before ever arming motors.

## Staged flight test campaign

Run in order; do not advance until the current stage passes.

1. **2 drones, ground/gossip only** (`-DCONFIG_PWM=n` on both, or props
   off) — confirm both drones' world models show each other fresh, no
   transport drops, no USART6 coexistence issues (console + PM + syslink
   P2P all active).
2. **2 drones hovering, staggered heights, no coordination check** — fly
   both independently (e.g. `altitude-hold-tether` or this example with
   `DEMO_TARGET_SPACING_M` set very large so the spring field is
   effectively inactive) to confirm two lighthouse-tracked drones don't
   occlude each other's base-station line of sight badly enough to cause
   simultaneous fix loss.
3. **2 drones, this example, static formation hold** — full spring field
   with `CONFIG_TAPESTRY_ELEMENT_COUNT=2`.
4. **3 drones, static formation hold** — `CONFIG_TAPESTRY_ELEMENT_COUNT=3`.
5. **3 drones, formation maneuver** — perturb one drone (nudge or briefly
   override its target) and confirm the others react, analogous to
   `examples/cutebot-formation`'s remove/rejoin test.

**Pass criteria per stage**: all drones hold their assigned spacing within
the single-drone error envelope (~±0.5 m, self-recovering — see project
memory's single-drone flight history), no separation violations logged, and
every drone reaches a clean scheduled or triggered landing.

## Tuning constants

Defined in `src/formation.h` (spring field) and `src/main.c` (mission
parameters). Override at build time with `-- -D<CONSTANT>=<value>`.

| Constant | Default | Meaning |
|---|---|---|
| `DEMO_TARGET_SPACING_M` | 1.0 | Desired peer spacing at equilibrium, meters |
| `DEMO_MAX_SPEED_MPS` | 0.3 | Max commanded approach speed |
| `DEMO_MIN_SEP_M` | 0.5 | Hard-floor separation — extra repulsion below this |
| `GEOFENCE_RADIUS_M` (main.c) | 2.0 | Distance from lighthouse origin before individual landing |
| `MISSION_DURATION_S` (main.c) | 60 | Per-drone flight duration before landing |
| `ALT_BASE_M` / `ALT_STEP_PER_ID_M` (main.c) | 0.30 / 0.20 | Per-ID cruise altitude stagger |

## Known limitations

- No wireless "land now" command — mission duration is the only
  coordinated stop; a real abort still means power-cycling or physically
  intervening (same as every single-drone example in this project).
- **3D separation math is unvalidated in flight.** `formation.c`'s peer
  distance (spring field, `DEMO_MIN_SEP_M`/`EMERGENCY_K` emergency
  repulsion) now folds altitude (z) into the distance metric, not just
  x/y — an explicit, requested change from the previous flight-tested 2D
  behavior, not an incidental one. Because altitude is staggered per
  element ID (`ALT_BASE_M`/`ALT_STEP_PER_ID_M`) specifically so horizontal
  proximity is what matters for downwash/visual clearance, this
  measurably weakens the emergency-repulsion trigger for drones that are
  horizontally close but at their normal staggered altitudes — the fixed
  altitude gap inflates the 3D distance past the threshold. Needs a real
  flight-test pass (start in `examples/webots-formation`) before trusting
  it in a multi-drone flight; nothing in this repo has validated it yet.
  `GEOFENCE_RADIUS_M`'s origin-distance check was deliberately left
  horizontal-only (see the comment at its call site in `main.c`) rather
  than folded into the same change.
- **No attitude-estimate accessor.** `own_state.orientation` gossips
  `orientation_identity()` (no rotation) rather than a real IMU-derived
  estimate — `cf21bl_stabilizer.c` runs a complementary filter internally
  for its own angle-mode control but exposes no accessor for the result.
  Wiring one up is a real follow-up, not done here. `own_state.position.z`
  gossips the per-ID staggered *cruise altitude* (a constant), not a live
  baro reading — same reason. Compare `examples/webots-formation`, which
  gossips genuine ground-truth 6DoF pose (GPS + InertialUnit) since Webots
  exposes both directly. If an attitude-estimate accessor is added later,
  its zero-reference must be calibrated to the shared lighthouse world
  frame (see `tapestry/tapestry-os/include/tapestry/csm.h`'s
  `orientation_t` comment for the full convention) — not reported as raw
  gyro output relative to wherever the drone happened to be pointed at
  boot. The existing "nose along lighthouse world +X" boot placement
  ("Flight checklist" above) is the calibration hook that would make
  this correct, provided the eventual accessor takes its zero from that
  same moment.
