# Webots simulation — Tapestry's real stack against simulated physics

A reusable pattern for compiling Tapestry's real, unmodified L3-L7 stack into a Webots
controller and running it against simulated physics instead of real hardware. `controllers/common/` holds everything that's
substrate-agnostic; `controllers/cf21bl/` is the one element this example currently ships
a substrate for. See "Porting to a different element" below for adding another.

The demonstrated script is [`examples/cf21bl-formation`](../cf21bl-formation/README.md)'s
`change-partners.choreo.toml`, read directly from that example (no forked copy) — the
exact same L4-L7 stack and L3 gossip framing, compiled unmodified into a plain Webots C
controller instead of onto real cf21bl hardware. N simulated drones (the default
world places 4 on a ring) hold station, rotate one seat, bow, and land.

Why this exists: real-hardware iteration means charging batteries, re-flashing, and
risking crashes for every tuning change, and caps out at whatever elements are physically
on hand. This gets the same L3-L7 behavior running cheaply and repeatably before (or
alongside) real flight testing, and generalizes toward larger swarms Webots can simulate
but a real fleet can't easily match.

## Architecture — why this isn't a Zephyr build

`tapestry-scr-hw` (real hardware) and the `tapestry-csm-sim`/`tapestry-scr-sim`
harnesses all build through Zephyr/`west`. This example does not — it's a plain host C
build using Webots' own `Makefile.include`, because:

- `world_model.c` (L4), `scr.c` (L5), `bse.c` (L6), `choreo.c` (L7) are already pure C99
  with zero Zephyr dependency — they compile unchanged into any C toolchain.
- `gossip.c` (L3 framing) depends on Zephyr for exactly a clock call, a random call,
  and logging macros — all satisfied by fake `zephyr/kernel.h` /
  `zephyr/logging/log.h` headers in `controllers/common/zephyr_shim/`
  (on the include path, so `gossip.c` itself is compiled byte-for-byte unmodified from
  `tapestry-os/subsys/transport/gossip.c`). HMAC wire auth
  (`CONFIG_TAPESTRY_WIRE_AUTH_ENABLED`) stays disabled, same as hardware's own
  bench-test no-auth mode.
- `transceiver_udp.c` (the real UDP transceiver) uses Zephyr's `zsock_*` socket API,
  unavailable outside a Zephyr build. `controllers/common/transceiver_udp_posix.c`
  implements the same 4-function `tapestry_transceiver_t` vtable over plain BSD sockets
  instead. This is a **new, additive** file — `transceiver_udp.c` is untouched and
  remains what hardware and the Zephyr `native_sim` harnesses build against.

Everything here is substrate-agnostic — none of it is specific to `cf21bl`
or to flying elements at all, which is why it lives in `controllers/common/` rather than
`controllers/cf21bl/`.

One controller process per element (Webots' standard multi-robot model — each `Robot`
node in the `.wbt` world gets its own OS process), mirroring how `tapestry-scr-hw` runs
one process per physical element.

## The `cf21bl` substrate

`substrate_webots.c` implements `tapestry/substrate.h` against Webots' Crazyflie device
API (`m1..m4_motor`, `gps`, `gyro`, `inertial_unit` — same devices as Webots' bundled
`crazyflie.c` reference controller) using the vendored Bitcraze PID cascade
(`pid_controller.c`, same file and gains as that reference controller, Apache-2.0) for
attitude/velocity/altitude control. This, `pid_controller.c` and the
flight-state-machine half of `main.c` (altitude ramp/land), is the entire
cf21bl-specific surface of this example.

Unlike the real cf21bl hardware's substrate (which reinterprets `linear.x/y` as an
absolute home-relative position for its own stabilizer's reasons), this backend uses
`substrate_twist_t`'s documented convention directly: a body-frame **rate** command.
`linear.z` integrates into an altitude setpoint over time rather than being read as an
absolute altitude — no "home" concept is needed since Webots' GPS is already absolute
ground truth.

Two rates: `substrate_move()` (called at the L4-L7 coordination cadence, 100 ms)
only latches the desired twist; `substrate_webots_step()` (called every Webots physics
step, ~8 ms) reads sensors, runs the PID cascade, and writes the four motors. This
mirrors the physical decoupling real hardware has between `cf21bl_stabilizer.c`'s fast
attitude loop and the 100 ms main loop that only updates its target.

## Target tracking

`controllers/common/tracker.c` is `demo_choreo_track` from
`cf21bl-formation/src/formation.c`, unmodified — same target-leash, emergency-repulsion,
and arena-clamp logic already flight-validated on hardware. It's substrate-agnostic
(2D position math only, no altitude or motor concept), which is why it lives in
`common/` rather than `cf21bl/`. The only change from the hardware version is
`DEMO_ARENA_LIMIT_M`: hardware derives it from a Zephyr Kconfig value
(`CONFIG_CF21BL_POS_MAX_M`); here it's a plain constant sized for a Webots arena.
`demo_compute_drive` (the showcase-mode leaderless spring field, unused by this
Choreo-only example) was dropped rather than carried as dead code.

## Build

```sh
export WEBOTS_HOME=/Applications/Webots.app                    # the .app bundle itself
export WEBOTS_HOME_PATH=/Applications/Webots.app/Contents       # where resources/Makefile.include actually lives
cd tapestry/examples/webots-formation/controllers/cf21bl
make
```

(Adjust paths for your platform/install location — Webots' own Makefile system
expects `WEBOTS_HOME` to be the bundle root for library/include paths but
`WEBOTS_HOME_PATH` to be where `resources/Makefile.include` lives; on macOS those
differ by `/Contents`, which is easy to get wrong — if `make` fails with
`webots/robot.h file not found`, this is almost always why.)

## Porting to a different element

Reusable as-is, no changes needed:
- `controllers/common/` (transceiver, Zephyr shim, target tracker)
- The Makefile pattern — copy `controllers/cf21bl/Makefile`, keep `COMMON = ../common`
  and the `TAPESTRY_ROOT`-relative core sources, keep the `choreo_script.h`
  dependency-tracking fix (see its comment — Webots' own `.d` tracking doesn't handle
  header-only changes correctly on a macOS fat-binary build).
- The core L3-L7 sources themselves (`tapestry-os/subsys/{csm,scr,bse,choreo}/*.c`,
  `tapestry-os/subsys/transport/gossip.c`).

What a new element needs, in a new `controllers/<substrate>/`:
- `substrate_<name>.c/h` implementing `tapestry/substrate.h`
  (`tapestry-os/include/tapestry/substrate.h`) against that element's Webots device API —
  the only hard requirement. A wheeled/ground element's version is arguably *simpler* than
  `cf21bl`'s: no altitude, no attitude PID cascade, just `substrate_move()` converting
  `twist.linear.x`/`twist.angular.z` into wheel velocities.
- Its own `main.c` — but only the flight-specific parts of `cf21bl/main.c` need
  rewriting (the altitude ramp/land state machine has no ground-element equivalent). The
  coordination-tick shape — `gossip_drain` → `wm_tick` → `wm_update_self` → `choreo_tick`
  → `demo_choreo_track` → `substrate_move` → `gossip_send`, once per `WM_CYCLE_MS` — is
  the part worth copying directly.
- A `.wbt` world instantiating that element's proto instead of `Crazyflie {}`.

## Running a different choreo script

To fly a different script: write your own `<name>.choreo.toml` (goal/parameter reference:
[`sdk/CHOREO_SCRIPTS.md`](../../sdk/CHOREO_SCRIPTS.md)), then compile it to the
**same** header path — the generated symbol names (`k_choreo_script`, `CHOREO_NAME`,
etc.) are fixed regardless of the script's own name, since `main.c` always
`#include`s `"choreo_script.h"`. For example, Running this choreo with the 
`cf21bl` substrate:

```sh
python3 tapestry/sdk/tools/choreoc.py path/to/<name>.choreo.toml \
    -o tapestry/examples/webots-formation/controllers/cf21bl/choreo_script.h
cd tapestry/examples/webots-formation/controllers/cf21bl && make
```

Then re-run the world — no `.wbt` changes needed unless you're also changing drone
count.

## Run

```sh
cd tapestry/examples/webots-formation
open -a Webots worlds/change_partners.wbt      # macOS GUI
```

`EXTERNPROTO` fetches the Crazyflie proto from GitHub on first run (cached afterward,
needs network once). Press play.

To watch the console without the GUI (what was used to validate this example):

```sh
/Applications/Webots.app/Contents/MacOS/webots --mode=fast --batch --minimize \
  --stdout --stderr worlds/change_partners.wbt
```

All drones should: ramp to their ID-staggered cruise altitude, hold station for 10 s,
rotate one seat around the ring via a direct beeline to the next station, bow for 8 s,
then land in place and go idle — identical console markers to the hardware version
(`choreo "change-partners" loaded`, `choreo step 0/1/2`, `choreo complete — resting`),
confirming L4-L7 behavior is unchanged from real flight. Watch the periodic 1 Hz status
line (`... pos=(x,y) tgt=(x,y) goal=(x,y) ... step=N q=H`) — `pos` should visibly
approach `goal` each second, and `q` should read `H` (healthy quorum) once every drone
is gossiping (`peers N-1/N-1`).

## Telemetry capture / offline replay

Set `TAPESTRY_TELEMETRY_DIR` before launching Webots to record each drone's
per-tick L6/L7 inputs and outputs (position, wm_entries snapshot, quorum,
script step, directive, achievement) to `<dir>/choreo_<element_id>.csv` —
see `controllers/cf21bl/choreo_telemetry.h`. Unset (the default), this is a
complete no-op — no file, no overhead.

```sh
mkdir -p /tmp/telemetry
TAPESTRY_TELEMETRY_DIR=/tmp/telemetry \
  /Applications/Webots.app/Contents/MacOS/webots --mode=fast --batch --minimize \
  --stdout --stderr worlds/change_partners.wbt
```

Replay a captured CSV through the Python L6/L7 engine offline (no Webots, no
C build) and diff every tick against the recording — a regression test for
`sdk/python/tapestry` against real captured flight data, not just a bare
script rehearsal:

```sh
python3 ../../sdk/tools/choreo_sim.py --replay \
    --script ../cf21bl-formation/change-partners.choreo.toml \
    --telemetry /tmp/telemetry/choreo_0.csv
```

See sdk/CHOREO_SCRIPTS.md's "Parity" section for what a clean replay (0
divergences) actually establishes.

`choreo_sim.py` also has a `--simulate` mode — no CSV, no Webots, no C
build at all: it drives a synthetic multi-element run of a `.choreo.toml`
directly through `sdk/python/tapestry` for sub-second script feedback
while authoring. See sdk/CHOREO_SCRIPTS.md's "Script-authoring
simulation" section.

## Known limitations

Deliberate simplifications versus the hardware version:

| Hardware (`cf21bl-formation`) | This example | Why |
|---|---|---|
| `transport_negotiate_id()` radio auto-ID at boot | Element ID from `.wbt` `controllerArgs` | Webots starts every controller deterministically together — no discovery problem to solve. Auto-ID is a fine stretch goal if it's ever worth exercising in sim too. |
| UDP broadcast to `255.255.255.255` | Unicast fan-out to a small fixed peer-port list (`controllers/common/transceiver_udp_posix.c`) | Loopback broadcast between sibling processes is unreliable on macOS. Wire format is untouched — this is a transport-only simplification, not a protocol change. |
| Lighthouse fix-loss handling, battery-critical landing | Not ported | No equivalent failure mode exists against Webots' GPS ground truth in this scope. |
| `GEOFENCE_RADIUS_M` = 2.0 m (room-scaled) | `GEOFENCE_RADIUS_M` = `DEMO_ARENA_LIMIT_M` (5.0 m) | Same backstop (lands if the drone's actual measured position strays past this radius from the origin — independent of the target-leash/arena-clamp above), sized to this example's own arena instead of the hardware room's. |

