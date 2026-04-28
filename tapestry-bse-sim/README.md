# tapestry-bse-sim — L6 BSE Python stub

Python implementation of the Tapestry L6 Behavior Synthesis Engine for
simulation and research.  Mirrors `tapestry-os/subsys/bse/bse_stub.c`.

## What is implemented

| BSE tier | Status |
|---|---|
| Intent parser | ✓ Stub (geometry-only) |
| Physics-aware planner | ✗ Commercial BSE |
| ML inference runtime | ✗ Commercial BSE |
| Simulation bridge | ✗ Commercial BSE |
| Feedback controller | ✗ Commercial BSE |

## Intent types

| Intent | Directive produced |
|---|---|
| `IDLE` | `IDLE` |
| `FORM` | `MOVE_TO_POINT` — vertex of regular N-gon, slot by element_id rank |
| `MOVE` | `MOVE_TO_POINT` — all elements to same target (stub limitation) |
| `CONVERGE` | `MOVE_TO_POINT` — all elements to target |
| `DISPERSE` | `MAINTAIN_SPRING` — spring-field with `intent.radius` spacing |

## Usage

```python
from bse_stub import BSEStub, BSEIntent, BSEIntentType, BSEShape

bse = BSEStub(element_id=0)
bse.submit_intent(BSEIntent(
    type   = BSEIntentType.FORM,
    target = (50.0, 50.0),
    radius = 30.0,
))

# Each simulation cycle (wm_entries is a list of world-model peer dicts):
bse.tick(wm_entries, scr_state)
d = bse.get_directive()
print(d.type, d.target)
```

See `sdk/examples/hello_swarm.py` for a complete worked example.
