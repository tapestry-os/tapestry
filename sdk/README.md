# Tapestry SDK

The application layer (L7) of the Tapestry OS stack.  This is the
stable interface that application developers code against.  The stack
below it is managed by Tapestry; application code calls only into this SDK.

```
┌─────────────────────────────────────────────────┐
│  Your application                               │  ← codes against sdk/
│  L6  BSE — Behavior Synthesis Engine (stub)     │  ← tapestry-os/subsys/bse/
│  L5  SCR — Swarm Coordination Runtime           │
│  L4  CSM — Coherent Swarm Memory                │
│  L3  Transport — UDP / BLE gossip               │
│  L2  Element Runtime — Zephyr RTOS              │
│  L1  Physical Substrate Interface               │
└─────────────────────────────────────────────────┘
```

> **Stub implementation — not for production use.**
> The BSE backing this SDK implements intent parsing only (geometry-based
> directive synthesis).  The physics-aware planner, ML inference runtime,
> simulation bridge, and feedback controller are commercial BSE components
> not present in this release.

## Quick start — Python (simulation / research)

```sh
# From the repo root:
python sdk/examples/hello_swarm.py
```

```python
import sys
sys.path.insert(0, 'sdk/python')
sys.path.insert(0, 'tapestry-bse-sim')

from tapestry.app import TapestryApp, Goal, GoalType

app = TapestryApp(element_id=0)
app.submit_goal(Goal(type=GoalType.FORM, target=(50.0, 50.0), radius=30.0))

# each simulation cycle:
app.tick(wm_entries, scr_state)
directive = app.get_directive()
```

## Quick start — C (embedded / Zephyr)

Include `sdk/include` and `tapestry-os/include` in your build, and add the
stub sources to your `CMakeLists.txt`:

```cmake
set(TAPESTRY_SDK      ${CMAKE_CURRENT_SOURCE_DIR}/../sdk)
set(TAPESTRY_OS_BSE   ${TAPESTRY_OS_ROOT}/subsys/bse)

target_sources(app PRIVATE
    ${TAPESTRY_OS_BSE}/bse_stub.c
    ${TAPESTRY_SDK}/src/app_stub.c
)
target_include_directories(app PRIVATE
    ${TAPESTRY_SDK}/include
)
```

Then in your main loop:

```c
#include <tapestry/app.h>

// startup:
tapestry_init(element_id);
tapestry_goal_t goal = {
    .type   = TAPESTRY_GOAL_FORM,
    .target = { .x = 50.0f, .y = 50.0f },
    .radius = 30.0f,
    .shape  = TAPESTRY_BSE_SHAPE_CIRCLE,
};
tapestry_submit_goal(&goal);

// each cycle, after wm_tick() and scr_tick():
tapestry_tick(&wm, &scr);
const tapestry_bse_directive_t *d = tapestry_get_directive();
```

## Goal types

| Goal | Directive produced by stub |
|---|---|
| `FORM` | `MOVE_TO_POINT` — regular N-gon vertex, slot by element_id rank |
| `MOVE` | `MOVE_TO_POINT` — all elements to target (no formation offset) |
| `CONVERGE` | `MOVE_TO_POINT` — all elements to target |
| `DISPERSE` | `MAINTAIN_SPRING` — spring-field with `radius` spacing |

## Goal status

| Status | Meaning |
|---|---|
| `IDLE` | No goal submitted |
| `ACTIVE` | Goal in progress |
| `ACHIEVED` | Not set by stub — requires BSE feedback controller |
| `FAILED` | Quorum lost while goal was active |

## Directory layout

```
sdk/
  include/tapestry/app.h   L7 API header (stable interface)
  src/app_stub.c           L7 C stub — delegates to bse_stub.c
  python/tapestry/         Python package (app.py, __init__.py)
  examples/hello_swarm.py  Minimal worked example (no sim required)
```

L6 files live in `tapestry-os/` (they are OS layer, not SDK):
```
tapestry-os/include/tapestry/bse.h   L6 interface contract
tapestry-os/subsys/bse/bse_stub.c    L6 C stub
tapestry-bse-sim/bse_stub.py         L6 Python stub
```
