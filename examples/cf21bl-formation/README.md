# Demo — CF2.1 Brushless Collective Formation

Three Crazyflie 2.1 Brushless quadrotors self-organize into a spring-field
formation using the Tapestry L4 world model, REAL lighthouse position (not
dead reckoning), and gossip over the nRF51822's syslink P2P radio channel.
No central controller, no L5 SCR — formation is a pure L4 emergent behavior,
same philosophy as `examples/collective-formation` for Cutebots.

This is the multi-drone follow-on to the single-drone work in
`examples/lh2-hover` (lighthouse position hold) and `examples/altitude-hold-tether`
(baro altitude hold) — both flight-validated prerequisites. Read those first
if the stabilizer/lighthouse architecture is unfamiliar.

## How it works

1. Each drone reads its own absolute position from the lighthouse deck
   (metres, home-relative to the shared calibration origin) and gossips it.
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

**Units**: positions in this example are metres in the lighthouse world
frame, NOT the abstract 0–100 logical-unit space `collective-formation`
uses — see the comment block at the top of `src/formation.h`.

## Requirements

- All three drones must be flashed with the **same** lighthouse base-station
  poses and OOTX calibration (`src/main.c`'s `BS0`/`BS1`/`BS0_CALIB`/
  `BS1_CALIB`) — gossiped positions are only comparable in a shared frame.
  Current values are from `examples/lighthouse_cal_office_260706.yaml`
  (2026-07-06); keep this in sync with `examples/lh2-hover/src/main.c` if
  the room is recalibrated again.
- Same lighthouse base stations, same room, same physical setup used for
  the single-drone lh2-hover validation.
- Each drone needs its own lighthouse deck (USART3, PC10/PC11 — see
  `cf21bl_lighthouse.c`) and a working BMP390 baro for altitude hold.

## Build (one per drone)

```sh
west build -p always -b crazyflie21bl tapestry/examples/cf21bl-formation \
  -- -DCONFIG_TAPESTRY_ELEMENT_ID=0   # 1, 2 for the other two drones
```

Flash: `cfloader flash build/zephyr/zephyr.bin stm32-dfu`

Console: CRTP radio only — the lighthouse deck takes USART3.
`python3 ~/code/tapestry/read_console.py`. With 3 drones transmitting on the
same nRF51 radio config there is no per-drone channel plan yet (see Phase D
below); treat concurrent consoles as best-effort, or build one drone at a
time with `-DCONFIG_LOG=n` to quiet the others.

Gossip-only bench test (no motors): add `-DCONFIG_PWM=n` — runs the L4
gossip/transport/lighthouse stack against `substrate_null.c`, useful for
verifying multi-drone position exchange before arming anyone.

## Safety layer

| Trigger | Detected by | Response |
|---|---|---|
| Own battery critical | `cf21bl_stabilizer.c` (`CONFIG_CF21BL_PM`) | Independent forced landing — pre-existing, not formation-specific |
| Own lighthouse fix lost | `main.c` | Zero X/Y immediately (avoids the stabilizer's fix-lost velocity-feedforward fallback inheriting a stale position value); land after `FIX_LOSS_GRACE_MS` (2 s) sustained loss |
| Geofence breach | `main.c`, checked against the lighthouse origin (not this drone's home) | Independent landing |
| Mission duration elapsed | `main.c`, per-drone timer from its own arm time | Independent landing — the closest thing to a "coordinated" land without a wireless uplink, since all drones arm within the same sync window |
| Minimum separation violated | `formation.c` (extra repulsion below `DEMO_MIN_SEP_M`) | Stronger repulsion force + a logged warning in `main.c`; not a landing trigger |

Every trigger above is per-drone local state — one drone landing never
affects another's flight, per the project's Phase E requirement.

## Flight checklist

1. Flash all three drones with IDs 0, 1, 2 (same BS calibration on all three).
2. Place each drone in the arena, **nose along lighthouse world +X**
   (same placement requirement as lh2-hover), spaced roughly
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

## Fleet bring-up (Phase D — do this once per new drone before flying it here)

1. Flash + confirm: ESC RC-PWM (or OneShot125) auto-detect arms cleanly,
   gyro calibration completes without warnings, lighthouse deck attached
   and reading (`examples/lighthouse-test` is a good bench check in
   isolation), battery telemetry flowing (`examples/altitude-hold-tether`
   with `CONFIG_CF21BL_PM=y` is a good bench check).
2. BLHeli_S "PWM Frequency" check via ESC Configurator on **every** motor
   of the new drone — avoid 24 kHz (Bitcraze warning, flagged since
   2026-06-09, never actually confirmed on any unit including drone 1; do
   all three while ESCs are on the bench for this campaign).
3. Confirm this drone's `CONFIG_TAPESTRY_ELEMENT_ID` is unique in the fleet.
4. Bench-test the syslink P2P transport alone first: build two units with
   `-DCONFIG_PWM=n` (gossip-only) and confirm each sees the other's
   `element_id` in the world model over radio before ever arming motors.

## Staged flight test campaign (Phase F)

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
   `examples/collective-formation`'s remove/rejoin test.

**Pass criteria per stage**: all drones hold their assigned spacing within
the single-drone error envelope (~±0.5 m, self-recovering — see project
memory's single-drone flight history), no separation violations logged, and
every drone reaches a clean scheduled or triggered landing.

**Known risk carried into this campaign**: the single-drone lighthouse work
found recurring phantom/reflection fixes in this room, all successfully
gated (see `cf21bl_lighthouse.c`). With multiple drones the base stations
can now also be occluded by another DRONE (not just furniture) — watch for
correlated fix loss between drones that are physically between each other
and a base station.

## Tuning constants

Defined in `src/formation.h` (spring field) and `src/main.c` (mission
parameters). Override at build time with `-- -D<CONSTANT>=<value>`.

| Constant | Default | Meaning |
|---|---|---|
| `DEMO_TARGET_SPACING_M` | 1.0 | Desired peer spacing at equilibrium, metres |
| `DEMO_MAX_SPEED_MPS` | 0.3 | Max commanded approach speed |
| `DEMO_MIN_SEP_M` | 0.5 | Hard-floor separation — extra repulsion below this |
| `GEOFENCE_RADIUS_M` (main.c) | 2.0 | Distance from lighthouse origin before individual landing |
| `MISSION_DURATION_S` (main.c) | 60 | Per-drone flight duration before landing |
| `ALT_BASE_M` / `ALT_STEP_PER_ID_M` (main.c) | 0.30 / 0.25 | Per-ID cruise altitude stagger |

None of the mission-parameter defaults above are hardware-validated yet —
they're conservative starting points for the Phase F campaign, same
convention as the single-drone gains before their own tether validation.

## Known limitations

- No wireless "land now" command — mission duration is the only
  coordinated stop; a real abort still means power-cycling or physically
  intervening (same as every single-drone example in this project).
- Altitude stagger (`ALT_STEP_PER_ID_M`) is a guess, not validated against
  real downwash interaction at this arena's spacing.
- No 3-drone CRTP console channel plan (Phase D item, deferred) — expect
  console output from multiple drones to interleave or drop on one radio.
