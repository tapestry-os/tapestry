# Demo — Collective Formation

Four BBC micro:bit V2 + Cutebot Mini robots self-organise into a square formation
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
6. Cutebot LEDs reflect how many fresh peers are visible:
   - Red — isolated (0 peers)
   - Yellow — 2-robot pair
   - Green — 3-robot triangle
   - White — full 4-robot square

## Hardware

| Item | Details |
|---|---|
| Board | BBC micro:bit V2 (nRF52833, Cortex-M4F) |
| Robot chassis | Elecfreaks Cutebot Mini |
| Transport | BLE advertising / passive scan (no pairing) |
| Arena | 300 mm × 300 mm |
| Formation | ~150 mm square side (50 logical units) |

## Build and flash

Build one binary per robot, varying `TAPESTRY_ELEMENT_ID` (0–3):

```sh
# Element 0
west build -b bbc_microbit_v2 tapestry/examples/collective-formation
west flash

# Element 1
west build -b bbc_microbit_v2 tapestry/examples/collective-formation \
    -- -DCONFIG_TAPESTRY_ELEMENT_ID=1
west flash

# Elements 2 and 3 — repeat with IDs 2 and 3
```

## Starting positions

Robots boot at the four corners of the equilibrium square (logical 100×100 world),
so spring forces are near-zero at startup and there is no violent initial spread.

| Element | Logical start |
|---|---|
| 0 | (25, 25) |
| 1 | (75, 25) |
| 2 | (25, 75) |
| 3 | (75, 75) |

## Calibration constants

Defined in [src/formation.h](src/formation.h). Override at build time with
`-- -DDEMO1_<CONSTANT>=<value>`.

| Constant | Default | Meaning |
|---|---|---|
| `DEMO1_SPEED_SCALE` | 250.0 | Logical units/s at 100% motor speed |
| `DEMO1_WHEEL_TRACK` | 28.0 | Wheel-centre to wheel-centre, logical units |
| `DEMO1_TARGET_SPACING` | 59.0 | Desired peer spacing (→ ~150 mm side) |

**Recalibrating for a different arena:**
```
SPEED_SCALE = (measured_speed_mm_per_s_at_100pct) * 100 / arena_width_mm
WHEEL_TRACK = cutebot_track_mm / arena_width_mm * 100
```
Measured Cutebot speed: ~150 mm/s at 20% → 750 mm/s at 100%.
Cutebot Mini wheel track: ~85 mm.

## Tuning tips

- **Robots won't move** — `FORCE_START` too high relative to `SPRING_K`.
  Reduce `FORCE_START` or increase `SPRING_K`.
- **Oscillation / overshoot** — `FORCE_START` too low. Raise it above the
  force produced by a one-cycle overshoot: `overshoot_units × SPRING_K × peer_count`.
- **Formation too tight / too spread** — adjust `DEMO1_TARGET_SPACING`.

## Motor test helper

[`../motor-test/`](../motor-test/) is a standalone binary that ramps motor speed
from 20% to 100% in steps for speed characterisation. Build it the same way:

```sh
west build -b bbc_microbit_v2 tapestry/examples/motor-test
```
