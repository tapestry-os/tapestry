# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

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

[Unreleased]: https://github.com/tapestry-os/tapestry/compare/v0.7.0...HEAD
[0.7.0]: https://github.com/tapestry-os/tapestry/compare/v0.6.1...v0.7.0
[0.6.1]: https://github.com/tapestry-os/tapestry/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/tapestry-os/tapestry/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/tapestry-os/tapestry/releases/tag/v0.5.0
