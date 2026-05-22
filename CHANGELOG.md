# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

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

[Unreleased]: https://github.com/tapestry-os/tapestry/compare/v0.5.0...HEAD
[0.5.0]: https://github.com/tapestry-os/tapestry/releases/tag/v0.5.0
