# Demo — Collective Formation

BBC micro:bit V2 + Cutebot Mini robots self-organise into a regular formation
using the Tapestry L4 world model and BLE gossip. No central controller.
No L5 SCR — formation is a pure L4 emergent behaviour.

## How it works

1. Each robot advertises its dead-reckoning position over BLE.
2. Peer positions are received into the local L4 world model.
3. A spring-field algorithm computes a differential drive command:
   - attraction when a peer is farther than `TARGET_SPACING`
   - repulsion when closer
4. Hysteresis thresholds (`FORCE_START` / `FORCE_STOP`) prevent
   oscillation near equilibrium and absorb gossip-propagated micro-corrections.
5. The micro:bit 5×5 LED matrix displays the robot's dead-reckoning position
   in real time (one lit pixel = estimated location in the 100×100 logical world).
6. Cutebot LEDs reflect how many fresh peers are currently visible:
   - Red — isolated (0 peers)
   - Yellow — 1 peer
   - Green — 2 or more peers

## Hardware

| Item | Details |
|---|---|
| Board | BBC micro:bit V2 (nRF52833, Cortex-M4F) |
| Robot chassis | Elecfreaks Cutebot Mini |
| Transport | BLE advertising / passive scan (no pairing) |
| Arena | 300 mm × 300 mm |
| Formation | ~150 mm square side (50 logical units) |

## Build and flash

One binary runs on all robots — no per-robot build flags needed:

```sh
west build -b bbc_microbit_v2 tapestry/examples/collective-formation
# Flash to every robot with the same .hex
west flash
```

## Auto-ID

During a 4-second boot window each robot advertises its FICR hardware nonce
and listens for peers.  After the window:

1. **Nonce rank** among co-booting robots → candidate rank (lower nonce = lower rank).
2. **Claimed IDs** from already-running robots (live gossip) are avoided.
3. **element_id** = rank-th unclaimed ID.

A rebooting robot sees which IDs are already claimed and reclaims the lowest
free slot — so it rejoins without conflicting with live peers.

## Starting positions

Each robot starts at its vertex of a regular N-gon whose side length equals
`DEMO_TARGET_SPACING`.  Robots therefore boot at their spring equilibrium
positions, minimising initial forces.

## Calibration constants

Defined in [src/formation.h](src/formation.h). Override at build time with
`-- -DDEMO_<CONSTANT>=<value>`.

| Constant | Default | Meaning |
|---|---|---|
| `DEMO_SPEED_SCALE` | 250.0 | Logical units/s at 100% motor speed |
| `DEMO_WHEEL_TRACK` | 28.0 | Wheel-centre to wheel-centre, logical units |
| `DEMO_TARGET_SPACING` | 59.0 | Desired peer spacing (→ ~150 mm side) |

**Recalibrating for a different arena:**
```
SPEED_SCALE = (measured_speed_mm_per_s_at_100pct) * 100 / arena_width_mm
WHEEL_TRACK = cutebot_track_mm / arena_width_mm * 100
```
Measured Cutebot speed: ~150 mm/s at 20% → 750 mm/s at 100%.
Cutebot Mini wheel track: ~85 mm.

## Tuning tips

- **Robots won't move** — `FORCE_START` too high relative to `SPRING_K`.
  Edit the values directly in `src/formation.c` (these are hardcoded `#define`s
  without `#ifndef` guards and cannot be overridden at build time).
- **Oscillation / overshoot** — `FORCE_START` too low. Raise it above the
  force produced by a one-cycle overshoot: `overshoot_units × SPRING_K × peer_count`.
- **Formation too tight / too spread** — adjust `DEMO_TARGET_SPACING`.
