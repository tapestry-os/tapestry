# Tapestry SDK — Choreographer (L7)

The Choreographer (L7) of the Tapestry OS stack.  This is the
stable interface that application developers code against.  The stack
below it is managed by Tapestry; application code calls only into this SDK.

```
┌─────────────────────────────────────────────────┐
│  L7  Choreographer (your code)                  │  ← codes against sdk/
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

from tapestry.choreo import Choreo, Goal, GoalType

choreo = Choreo(element_id=0)
choreo.submit_goal(Goal(type=GoalType.FORM, target=(50.0, 50.0), radius=30.0))

# each simulation cycle:
choreo.tick(wm_entries, scr_state)
directive = choreo.get_directive()
```

## Quick start — C (embedded / Zephyr)

Include `sdk/include` and `tapestry-os/include` in your build, and add the
stub sources to your `CMakeLists.txt`:

```cmake
set(TAPESTRY_OS_BSE    ${TAPESTRY_OS_ROOT}/subsys/bse)
set(TAPESTRY_OS_CHOREO ${TAPESTRY_OS_ROOT}/subsys/choreo)

target_sources(app PRIVATE
    ${TAPESTRY_OS_BSE}/bse.c
    ${TAPESTRY_OS_CHOREO}/choreo.c
)
target_include_directories(app PRIVATE
    ${TAPESTRY_SDK}/include
)
```

Then in your main loop:

```c
#include <tapestry/choreo.h>

// startup:
choreo_init(element_id);
choreo_goal_t goal = {
    .type   = CHOREO_GOAL_FORM,
    .target = { .x = 50.0f, .y = 50.0f },
    .radius = 30.0f,
    .shape  = TAPESTRY_BSE_SHAPE_CIRCLE,
};
choreo_submit_goal(&goal);

// each cycle, after wm_tick() and scr_tick():
choreo_tick(&wm, &scr);
const tapestry_bse_directive_t *d = choreo_get_directive();
```

## Goal types

| Goal | Directive produced by stub |
|---|---|
| `CHOREO_GOAL_FORM` | `MOVE_TO_POINT` — regular N-gon vertex, slot by element_id rank |
| `CHOREO_GOAL_MOVE` | `MOVE_TO_POINT` — all elements to target (no formation offset) |
| `CHOREO_GOAL_CONVERGE` | `MOVE_TO_POINT` — all elements to target |
| `CHOREO_GOAL_DISPERSE` | `MAINTAIN_SPRING` — spring-field with `radius` spacing |

## Lifecycle states

| State | Meaning |
|---|---|
| `CHOREO_STATE_IDLE` | No goal loaded |
| `CHOREO_STATE_CONFIGURED` | Goal validated; BSE not yet ticking |
| `CHOREO_STATE_RUNNING` | BSE ticking; quorum DEGRADED or HEALTHY |
| `CHOREO_STATE_SUSPENDED` | Quorum LOST; goal preserved, resumes on recovery |
| `CHOREO_STATE_TERMINATED` | Transitional; settles immediately to IDLE |

## Directory layout

The SDK contains only interface artefacts; implementations live in `tapestry-os/`:

```
sdk/
  include/tapestry/choreo.h        L7 API header (stable interface)
  python/tapestry/choreo.py        L7 Python mirror
  python/tapestry/bse.py           L6 Python stub
  examples/hello_swarm.py          Minimal worked example (no sim required)

tapestry-os/
  include/tapestry/bse.h           L6 interface contract
  subsys/bse/bse.c                 L6 C stub
  subsys/choreo/choreo.c           L7 C stub
```
