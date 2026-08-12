# Tapestry SDK — Choreographer (L7)

The Choreographer (L7) of the Tapestry OS stack.  This is the
stable interface that application developers code against.  The stack
below it is managed by Tapestry; application code calls only into this SDK.

```
┌─────────────────────────────────────────────────┐
│  L7  Choreographer (your code)                  │  ← codes against sdk/
│  L6  BSE — Behavior Synthesis Engine            │  ← tapestry-os/subsys/bse/
│  L5  SCR — Swarm Coordination Runtime           │
│  L4  CSM — Collective State Manager             │
│  L3  Transport — UDP / BLE gossip               │
│  L2  Element Runtime — Zephyr RTOS              │
│  L1  Physical Substrate Interface               │
└─────────────────────────────────────────────────┘
```

> **v1.0 feature scope.** The BSE backing this SDK implements intent
> parsing, geometry-based task decomposition (FORM vertex assignment across
> CIRCLE/LINE/GRID, MOVE offset-preserving formation translation, CONVERGE,
> EXCHANGE station rotation over a snapshot with arc or direct path), and a
> feedback controller — the per-element achievement predicate
> (`choreo_goal_achieved` / `bse_goal_achieved`) plus its collective
> aggregation (`choreo_collective_achieved`, `scope = "all"` in TOML — see
> [`CHOREO_SCRIPTS.md`](CHOREO_SCRIPTS.md)). A TOML script authoring/compiler
> toolchain also exists (`sdk/tools/choreoc.py`,
> `sdk/python/tapestry/script_toml.py`).
>
> Deliberately out of the open-core tier (a licensing boundary, not a gap):
> optimization across the swarm (a physics-aware planner, ML inference — the
> EXCHANGE arc is a fixed geometric rule, not a planner), priority/preemption
> across goals, a simulation/replay bridge, the monitor stage's telemetry
> export, and the `scope = "all"` barrier/lockstep upgrade (collective
> achievement here is eventually consistent, bounded by gossip latency — not
> a synchronization guarantee). See `tapestry-os/include/tapestry/bse.h` and
> `sdk/include/tapestry/choreo.h` for the itemized contract this
> implementation satisfies.

> **Writing a multi-step show?** Scripts (ordered, time-bounded goal
> sequences) are authored once in TOML and either compiled to a C header or
> loaded directly in Python — see [`CHOREO_SCRIPTS.md`](CHOREO_SCRIPTS.md).
> The quick starts below cover the single-goal API scripts are built from.

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
sources to your `CMakeLists.txt`:

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

| Goal | Directive produced |
|---|---|
| `CHOREO_GOAL_FORM` | `MOVE_TO_POINT` — vertex by element_id rank; `shape` CIRCLE (N-gon), LINE (evenly spaced), or GRID (near-square rows/cols) |
| `CHOREO_GOAL_MOVE` | `MOVE_TO_POINT` — target displaced by this element's own offset from the participant centroid (preserves formation; a solo element degenerates to CONVERGE) |
| `CHOREO_GOAL_CONVERGE` | `MOVE_TO_POINT` — all elements to target (collapses formation) |
| `CHOREO_GOAL_EXCHANGE` | `MOVE_TO_POINT` — rotate stations among participants, arc or direct path |
| `CHOREO_GOAL_HOLD` | `MOVE_TO_POINT` — station-keep at the position captured on activation |
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

The SDK contains only interface artifacts; implementations live in `tapestry-os/`:

```
sdk/
  include/tapestry/choreo.h        L7 SDK API header (Goal / lifecycle / directive)
  python/tapestry/choreo.py        L7 Python mirror
  python/tapestry/bse.py           L6 Python mirror
  python/tapestry/script_toml.py   Choreo script (TOML) parser/validator
  tools/choreoc.py                 Choreo script compiler: TOML -> C header
  examples/hello_swarm.py          Minimal worked example (no sim required)
  CHOREO_SCRIPTS.md                Script authoring + compilation guide

tapestry-os/
  include/tapestry/choreo.h        L7 C header (choreo_init, choreo_tick, etc.)
  include/tapestry/bse.h           L6 interface contract (intent → directive)
  subsys/choreo/choreo.c           L7 Choreographer implementation
  subsys/bse/bse.c                 L6 BSE implementation
```
