# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

## [0.9.0] — 2026-08-16

### Added
- **Real L5 SCR wired into `examples/cf21bl-formation` and
  `examples/webots-formation`'s cf21bl controller** — both now call
  `scr_init()`/`scr_tick()`, so role, `task_slot`, leader election and the
  abort protocol are computed from the actual world model instead of being
  synthesized from raw L4 freshness. Note that `SUSPENDED` is additionally
  gated by an application-level debounce: both apps overwrite
  `scr_state_t::quorum_state` after `scr_tick()` with a sustained-freshness signal (`QUORUM_UP_MS`, 2 s) before handing it to `choreo_tick()`. At the thresholds both apps configure (`quorum_min` = `quorum_target` = 1) this yields the same HEALTHY/LOST verdict L5 computes, plus a 2 s confirmation before an up-transition is believed — quorum loss is still immediate. The debounce is applied to a copy handed to `choreo_tick()`, never written back into the live `scr_state_t`, so L5's own accessors stay mutually consistent. Hysteresis on quorum acquisition belongs inside L5 rather than being repeated per application — deciding whether the collective has quorum is L5's responsibility, and an obligation carried by every element main loop is one every new element main loop can forget (TODO). Enforced real-world geofence and mission-duration landing backstops.
- **Collective achievement (`scope = "all"`)** — each element now gossips
  an `achieved` bit every cycle (`tapestry_gossip_frame_t` gains an
  `achieved` field, a wire-compatible append — no `TAPESTRY_WIRE_VERSION`
  bump), aggregated via `choreo_collective_achieved()`. `choreo_step_t`
  and Choreo-script TOML gain `scope = "self"|"all"` (default `"self"`,
  existing scripts unaffected). `examples/cf21bl-formation`'s exchange
  step now uses `scope = "all"`, replacing the bow step's former role as
  an implicit per-drone-achievement workaround
- ztest coverage (native_sim, `examples/cf21bl-formation/tests`) for
  `FORM` shapes, `MOVE` translation, and `scope = "all"` collective
  achievement (including solo/vacuous-truth and stale-peer cases)
- **Webots simulation pattern (`examples/webots-formation/`)** — compiles
  the real, unmodified L3-L7 stack (`world_model.c`/`scr.c`/`bse.c`/
  `choreo.c`/`gossip.c`) into a plain host C Webots controller against
  simulated physics: a hardware-in-the-loop bridge. Structured as a
  reusable pattern rather than a single-robot demo: `controllers/common/`
  holds everything substrate-agnostic (POSIX UDP transceiver, Zephyr
  header shim, target tracker)
- **Offline capture/replay harness for the Choreo engine** — 
  `sdk/tools/choreo_sim.py
  --simulate` (renamed from `choreo_replay.py`, which is now its
  `--replay` mode; both share one tick loop and plot output) drives N
  in-process `Choreo` objects through a `.choreo.toml` with no C, Zephyr,
  or network involved. For someone editing a script who has no build 
  toolchain or Webots set up yet, or is designing for a substrate 
  that doesn't exist as code
- **`choreoc.py --check`, enforced in CI** — generated `choreo_script.h`
  headers are committed so firmware builds never need Python, which means
  a script edit that is not recompiled ships silently. `--check` writes
  nothing and exits non-zero if a committed header does not match what its
  script generates today, the same guarantee
  `gen_wire_protocol.py --check` already gave the wire mirrors. With no
  arguments it discovers every generated header in the repository and
  recovers each one's source from the regenerate command line in its own
  banner, so an added consumer is covered without editing CI — the drift
  it is meant to catch happened precisely because a second consumer was
  added and nothing knew to keep it in sync

### Changed
- **Status banners rewritten across `sdk/README.md`, `choreo.h`, `bse.h`,
  `bse.c`, `choreo.c` and their Python mirrors** — feature ready for v1.0 release. They now itemize the feature scope: what L6/L7
  implement today, and which capabilities (physics planner, ML runtime,
  goal queue with preemption and arbitration, hi-fi simulation bridge,
  monitor telemetry export) are deliberately out of scope rather than
  merely unfinished

### Fixed
- **`FORM` shape** — `shape = "line"` and `"grid"` were accepted and
  silently rendered a circle regardless; both now produce the requested
  layout (`bse.c`/`bse.py`)
- **`MOVE`** — previously identical to `CONVERGE` (collapsed every
  element onto the same point); now offset-preserving formation
  translation with each element keeping its position relative to 
  the formation centroid. A solo element still degenerates to
  `CONVERGE`, correctly, since there is no formation to preserve
- **`tapestry-scr-hw`'s movement loop** — `substrate_move()` is now
  directive-driven reading `choreo_get_directive()`
- **`examples/webots-formation` ran a stale Choreo script** — its generated
  `choreo_script.h` predated collective achievement and was missing
  `scope = "all"` on the exchange step, so the simulation did not run the
  same script as the hardware demo despite the README saying it did.
  Regenerated from `change-partners.choreo.toml`
- **Out-of-range element ID corrupted memory in L4/L5** — `wm_init()` and
  `wm_update_self()` indexed `world_model_t::entries[]` with an unvalidated
  `owner_id` (an out-of-bounds *write* for any ID ≥ `MAX_ELEMENTS`),
  `wm_check_collisions()` read the same way, and `scr_tick()` could write
  one entry past its `candidates[]` array because its peer loop skips only
  `own_id`. Peer IDs arriving over the wire were already validated; the
  owner's ID, which comes from the application, was not. It is now checked
  once in `wm_init()` — an out-of-range owner leaves the model inert rather
  than corrupting adjacent memory — and `scr_tick()`'s array is sized for
  the worst case. Reachable through a hand-configured element ID or an
  `ELEMENT_ID_INVALID` propagated from a failed identity negotiation
- **`transport.h` contradicted `wire.h` on QoS** — `transport_send()`'s
  documentation claimed `qos_tier` was "embedded in the gossip frame so
  peers can prioritize accordingly". It is not: `gossip_send()` ignores the
  parameter, and `wire.h` correctly documents the tiers as reserved
  placeholders. `transport.h` now says the same

## [0.8.0] — 2026-08-01

### Added
- **L6/L7: first flight-ready Choreo** — coordinate-free `HOLD` (station
  captured at activation) and `EXCHANGE` (snapshot stations, rotate by
  `slot_shift` around the ID-sorted ring, commanded target travels a CCW
  arc about the snapshot centroid so mutual separation is preserved by
  construction) goals, plus a minimal feedback controller (achievement
  predicate). L7 gains linear goal scripts (`choreo_submit_script`) with
  per-step timeout/advance-on-achieved and quiescence on completion
  (`IDLE` directive — platforms map it to their own inactive posture;
  "take off"/"land" never appear at L7). `SUSPENDED` now freezes the BSE
  and script timers, except `HOLD`, which keeps ticking since it
  references no peer
- **Choreo scripts authored in TOML** — `sdk/tools/choreoc.py` compiles a
  `.toml` script to a committed C header (stdlib-only, no dependencies);
  the Python SDK loads the identical file directly via
  `tapestry/script_toml.py`. New `sdk/CHOREO_SCRIPTS.md` authoring guide.
  Script files follow an `<name>.choreo.toml` naming convention
- **`examples/cf21bl-formation` Choreo mode** (new default) — one binary
  for every drone (IDs negotiated at boot over syslink P2P) running a
  hold → exchange → bow script through the full safety envelope
  (min-separation repulsion, target leash, arena clamp); script
  completion lands in place. The original spring-field showcase is
  preserved behind `DEMO_MODE_SHOWCASE`
- **Medium-agnostic auto-ID** — discovery beacons are now plain gossip
  frames over any transceiver, not BLE-only; new diagnostics
  (per-transceiver drop counters, live progress log, duplicate-ID frame
  counter) and self-healing (a drone hearing no peers grounds and
  retries instead of arming solo)
- **Wire schema version** — a one-byte `TAPESTRY_WIRE_VERSION` in every
  L3 message header, and in the gossip frame itself so BLE and syslink
  P2P (which carry it with no header wrapper at all) are covered too; a
  frame from a mismatched schema is rejected and logged at every layer
  that carries it, instead of silently misinterpreted
- **`tapestry-os/tools/gen_wire_protocol.py`** — generates the three
  previously hand-maintained Python wire-protocol mirrors
  (`tapestry-csm-sim`, `tapestry-scr-sim`, `tapestry-scr-hw`) directly
  from `wire.h`; CI now fails if a mirror drifts from source
- Choreo script ztest suite (native_sim) covering the hold → exchange →
  bow script end to end

### Changed
- Choreo-mode gossip cadence raised to 5 Hz (was 2 Hz) — the old cadence
  let peer entries go stale in roughly half of all windows; landed
  elements now keep gossiping in Choreo mode so a finisher's departure
  can't strip its still-flying partner's only peer
- Auto-ID window now opens after a 2.5 s radio settle delay, with
  discovery beacons at 150 ms during the window (from 2 Hz) to
  compensate for ~20–35% measured syslink P2P delivery under load
- TOML script validation tightened: `hold` rejects `until`/`eps`/`settle`
  (trivially achieved — would advance on the first tick), `form`
  requires a `radius` (radius 0 sent every element to the same vertex),
  `move` warns that it currently behaves like `converge`; duration and
  length units gained `"min"`, `"h"`, and `"um"`

### Fixed
- Landing touchdown could cut motors mid-air when the walked-down target
  reached zero before the airframe had actually descended
- A two-drone script's only "member departs" path (landing) could
  silently strip a still-flying partner's last peer, suspending its
  Choreo and freezing `EXCHANGE`'s step timeout — recoverable only by
  the outer mission-duration backstop

## [0.7.0] — 2026-07-14

### Added
- **Crazyflie 2.1 Brushless board support** (`crazyflie21bl`) — full Zephyr
  bring-up: ESC control (RC PWM 400 Hz and OneShot125 with BLHeli_S
  auto-detect), BMI088 IMU at 1 kHz with Mahony attitude filter, BMP390
  barometric altitude hold (cascaded position→velocity→thrust), watchdog,
  CRTP radio console, and battery monitoring/compensation with forced
  landing via nRF51 syslink
- **Lighthouse V2 positioning driver** (`cf21bl_lighthouse.c`) — SteamVR
  2.0 base-station triangulation from the Bitcraze Lighthouse deck: sweep
  pairing, OOTX sweep calibration, optical-FOV angle gating,
  miss-distance and speed-limit outlier rejection, and long-baseline
  velocity estimation
- **Lighthouse XY position hold** in the stabilizer — P+I+D with a
  rate-clamped body-frame level-trim integrator and yaw heading hold
- **Syslink P2P gossip transport** — drone-to-drone L4 gossip over the
  nRF51's 2.4 GHz radio, no co-processor reflash required
- **cf21bl-formation demo** — three drones self-organize (line → rotating
  triangle → member departs → line) with no leader, phase timers, or
  uplink: the formation is a pure function of fresh-peer count.  Includes
  per-drone staging (late join / early departure), centroid anchor,
  target leash, and a per-drone safety layer (fix-loss, geofence,
  mission-duration, land-in-place, gossip-silence after landing)
- **Formation-field unit tests** (ztest, native_sim) — regression tests
  for the flight-found spring-field bugs — plus CI coverage: both ztest
  suites run on native_sim and the demo + runtime element compile-check
  for `crazyflie21bl`
- **Shared lighthouse calibration** (`examples/lighthouse_cal.h`/`.yaml`)
  — single in-code copy of base-station geometry consumed by every
  position-aware example
- **`patches/`** — two Zephyr I2C v1 RTIO driver fixes required by the
  BMI088 on this board (pending upstream submission; reapply after
  `west update`)

### Changed
- **`examples/collective-formation` renamed to `examples/cutebot-formation`**
  (including the BLE advertised name) — the generic name now belongs to no
  single platform; the Crazyflie counterpart is `examples/cf21bl-formation`
- Retired superseded bring-up examples (`altitude-hold-test`, `lh2-hover`);
  their roles are covered by `altitude-hold-tether`, `lighthouse-test`, and
  `cf21bl-formation`
- Normalized all documentation and comments to American English spellings

### Fixed
- **Lighthouse position-hold sign inversion** — the XY correction was added
  to the angle setpoint unnegated (positive feedback bounded only by
  saturation); explains a long history of "under-damped orbit" that gain
  tuning never resolved
- **Stale elevation gate after base-station recalibration** — replaced
  room-specific empirical angle bounds with the LH2 optical FOV (a hardware
  constant), ending gate-induced single-station dropouts
- **Formation target detachment** — virtual targets could decouple
  unboundedly from their drones under emergency repulsion; now leashed to
  the stabilizer's error-saturation radius
- **Altitude-ramp integrator windup** — higher-cruise drones overshot and
  kept climbing after long takeoff ramps

## [0.6.1] — 2026-06-08

### Fixed
- **L4 sim element** — fixed uninitialized `comms.shutdown` flag in
  `comms_init`; could cause the element's main loop to exit immediately
  on startup if stack memory happened to be non-zero

## [0.6.0] — 2026-05-21

### Added
- **L2 Element Runtime** — added power state machine, main-loop ownership,
  devicetree energy harvester integration
- **L6 Behavior Synthesis Engine** (C) and **L7 SDK** (Python) — initial
  developer-facing API surface; wired into hardware and sim elements
- **L5 SCR** — roles, task slots, abort protocol, and BFT anomaly filtering
- **HMAC-SHA256 frame authentication** — optional per-deployment shared key,
  4-byte truncated tag appended to gossip frames
- **QoS tiers** and **energy/health gossip fields** in the wire format
- **Two-hop opportunistic relay** (BATMAN-inspired mesh) behind
  `CONFIG_TAPESTRY_MESH_RELAY`; prevents broadcast storms via hop_count cap
- **Auto-ID** — boot-time nonce exchange eliminates per-robot build flags;
  one binary flashes to all elements in a deployment
- **Collective-formation demo** — 4 Cutebot Mini robots self-organize into
  a regular formation using L4 gossip; validated on hardware

### Changed
- **L7 layer renamed** to "Choreographer" (`choreo`); lifecycle states and
  per-goal capability permissions added
- **L1 layer** refactored from actuation HAL to Physical Substrate Interface (PSI)
- **Transport subsystem** flattened into `tapestry-os/subsys/transport/` with
  a transceiver vtable
- Sim and runtime main-loop cadence improved

### Fixed
- Three architectural misattributions in comments and docs corrected
  to match the architecture paper

## [0.5.0] — 2026-04-23

### Added
- **L4 Collective State Manager** — distributed world model with gossip-based
  state propagation, Lamport clock ordering, partition-aware reconciliation,
  spatial queries, and a continuous `consistency_bias` AP/CP dial
- **L5 Swarm Coordination Runtime** — quorum classification (LOST / DEGRADED /
  HEALTHY) and deterministic lowest-ID leader election over the L4 world model
- **L4 + L5 ztest suites** — 8 L4 and 15 L5 unit tests; run on native_sim and
  validated on physical hardware
- **L4 + L5 simulation harnesses** — Python asyncio orchestrators with gossip
  broker, scenario injection, telemetry CSV writer, and matplotlib visualizer
- **Hardware validation Phase 1** — L4/L5 ztests pass on ESP-WROVER-KIT
  (ESP32), EK-RA8D1 (RA8D1 / Cortex-M85), and BBC micro:bit V2 (nRF52833)
- **Hardware validation Phase 2** — live two-element swarms over UDP/LAN
  (ESP32 + RA8D1) and BLE advertising (two micro:bit V2 Cutebots); motor and
  LED state driven by quorum and role
- **CI** — GitHub Actions workflow: L4/L5 ztests on native_sim, compile checks
  for all three hardware targets and Phase 2 firmware
- `CODE_OF_CONDUCT.md`, `SECURITY.md`

[Unreleased]: https://github.com/tapestry-os/tapestry/compare/v0.9.0...HEAD
[0.9.0]: https://github.com/tapestry-os/tapestry/compare/v0.8.0...v0.9.0[0.8.0]: https://github.com/tapestry-os/tapestry/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/tapestry-os/tapestry/compare/v0.6.1...v0.7.0
[0.6.1]: https://github.com/tapestry-os/tapestry/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/tapestry-os/tapestry/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/tapestry-os/tapestry/releases/tag/v0.5.0
