# Choreo Scripts — authoring and compiling (L7)

This is the platform-agnostic guide to writing a **Choreo script**: an
ordered, time-bounded sequence of goals that drives a collective through the
L7 Choreographer / L6 BSE (see [`README.md`](README.md) for the full layer
stack and the single-goal C/Python API). If you're looking for a worked,
flying example, see `examples/cf21bl-formation/` (aerial swap) — this
document is the general reference every Choreo script is written against,
independent of any one example or platform.

A script is authored **once**, in TOML, and consumed by whichever runtime(s)
you target:

- **Embedded / Zephyr (C)** — compiled ahead-of-time into a committed C
  header by `sdk/tools/choreoc.py`. Firmware builds and CI never need Python.
- **Python (simulation / research)** — read directly at runtime by
  `tapestry.script_toml.load_steps()`. No generation step.

Both consume the identical `.toml` file and produce identical
`choreo_step_t` / `ChoreoStep` sequences — there is one script, two runtimes.

## Naming convention

A Choreo script file is named `<name>.choreo.toml`, where `<name>` matches
the script's own `choreo = "<name>"` key (e.g. `change-partners.choreo.toml`
for `choreo = "change-partners"`). The double extension makes the file kind
self-identifying wherever it appears (a directory listing, a diff, a CI
glob) and scales cleanly once a project holds more than one script.

## The file format

```toml
choreo = "change-partners"

[[steps]]                          # stay at current stations
hold = { duration = "10s", requires = ["locomotion"] }

[[steps]]                          # swap places, advance when achieved
[steps.exchange]
until    = "achieved"
timeout  = "30s"
path     = "direct"
eps      = "25cm"
settle   = "3s"
requires = ["locomotion"]

[[steps]]                          # bow: settle on the new stations
hold = { duration = "8s", requires = ["locomotion"] }
```

- The top-level `choreo = "<name>"` key is required — it becomes
  `CHOREO_NAME` in the generated header.
- Each `[[steps]]` entry is one step, executed in order. Multi-parameter
  steps need the `[steps.<goal>]` sub-table form (TOML inline tables are
  single-line only); single-parameter steps can use the inline
  `hold = { duration = "10s" }` shorthand.
- Every step needs **exactly one** goal key (below) and a time bound. There
  is no implicit "last step" behavior: when the final step completes, the
  Choreo emits the `IDLE` directive — **quiescence**. Each platform maps
  that to its own inactive posture (an aerial element lands and disarms, a
  ground robot stops). Take-off and landing are never named in a script.

## Goal keys

| Goal | References | Extra parameters |
|---|---|---|
| `hold` | the element's own current station (coordinate-free) | — |
| `exchange` | participants' own stations, rotated (coordinate-free) | `shift` (ring rotation, default 1), `path` |
| `form` | an absolute point + shape | `target = [x, y]`, `radius` (both required — radius 0 would send every element to the same vertex), `shape` |
| `move` | an absolute point | `target = [x, y]` |
| `converge` | an absolute point | `target = [x, y]` |
| `disperse` | current positions, spread apart | `radius` (required — minimum spacing) |

`hold` and `exchange` are **coordinate-free by design** — they reference the
collective's own configuration, not application-supplied coordinates, and
the parser rejects `target`/`radius`/`shape` on them. This is what lets the
same script fly regardless of where the elements actually start.

`exchange` rotates stations by `shift` around the ID-sorted ring of fresh
participants (frozen snapshots taken at step activation, never live-chasing
a moving peer); `shift = 1` with two elements is a swap. `path = "arc"`
(the default) travels an arc about the formation centroid, preserving XY
separation by construction — use this on platforms with no vertical
dimension. `path = "direct"` beelines straight to the destination — only
safe if something else deconflicts the crossing (e.g. altitude staggering
on an aerial platform); see
`examples/cf21bl-formation/change-partners.choreo.toml` for a worked case.

## Common parameters (every goal)

| Key | Meaning |
|---|---|
| `duration` / `timeout` | step time bound. **Required on every step** — this is the robustness net that keeps a script from stalling; give exactly one of the two names (they're the same field). |
| `until = "achieved"` | advance as soon as the achievement predicate fires (scope decides whose — see `scope` below), instead of waiting out the full duration. The timeout still applies as a fallback. Not allowed on `hold` — hold is trivially achieved, so hold steps are duration-governed (`until`/`eps`/`settle` on hold are rejected; reserved for future scoped-achievement semantics). |
| `eps` | achievement radius (default: BSE default if omitted). |
| `settle` | how long the error must stay within `eps` before achievement fires (default: BSE default). |
| `scope = "self"` \| `"all"` | whose achievement `until = "achieved"` waits for (default `"self"`). `"all"` advances only once this element **and** every fresh peer have achieved — aggregated from an `achieved` bit each element gossips every cycle ("achieved-bit" item). Eventually consistent, bounded by gossip latency — not a synchronization barrier (that's the doc's separate `barrier = true`, not implemented). A lone element with no fresh peers is vacuously "all achieved", so it can't deadlock alone. Only valid alongside `until = "achieved"`; not allowed on `hold`. |
| `requires` | list of capability names the executing element must have: `["locomotion", "bonding", "sensing", "signaling"]`. A step whose requirements the registered element can't satisfy is rejected at submit time. |

**Duration syntax**: `"30s"`, `"500ms"`, `"45min"`, `"2h"`, or a bare
number (seconds).
**Length syntax**: `"25cm"`, `"250mm"`, `"500um"`, `"0.25m"`, or a bare
number (meters).

> **Unit footgun (TOML vs. C):** bare numbers mean different things on
> the two authoring surfaces. In TOML, `duration = 2` is **2 seconds**;
> in a hand-written `choreo_step_t`, `.max_duration_ms = 2` is
> **2 milliseconds** (the field names carry the unit: `max_duration_ms`,
> `achieve_hold_ms`). `choreoc` converts between them — one more reason
> to author in TOML and never edit the generated header.

Note on `move` vs. `converge`: `move` translates the formation to
`target`, preserving each element's offset from the participant centroid
(a rigid-body translation) — it does not collapse the formation. Use
`converge` when gathering everyone at the same point is what you mean.

## Validation

The TOML parser (`sdk/python/tapestry/script_toml.py`) is intentionally
**stricter** than the raw C/Python API, because it's the flight-authoring
surface:

- Every step must carry a time bound — the C API alone permits an
  achievement-only step with no timeout; the parser refuses to author one.
- `hold` must not carry `until`/`eps`/`settle` — trivially-achieved
  semantics would make such a step advance on the first tick, and the
  parameters are reserved for future scoped achievement.
- `hold`/`exchange` must not carry coordinates; `form`/`move`/`converge`
  must; `disperse` must carry a `radius`.
- Unknown goal keys, unknown parameters, and unknown capability/shape names
  are rejected with a message naming the offending step index and the
  allowed set.

Errors look like:

```
choreoc: <name>.choreo.toml: steps[1]: 'exchange' has no time bound — every
step needs 'duration' (or 'timeout'); the bound is the robustness net that
keeps a script from stalling in flight
```

If it parses, it is guaranteed compilable and (for the reasons above)
guaranteed not to stall a flight by construction.

## Building for C / Zephyr (embedded targets)

Compile the TOML into a committed C header — same pattern as
`examples/lighthouse_cal.h`, so the firmware build and CI never invoke
Python:

```sh
python3 sdk/tools/choreoc.py path/to/<name>.choreo.toml
```

Standard-library-only (Python ≥ 3.11 for `tomllib`) — no venv, nothing to
install; use the system `python3`.

With no `-o`, the header is written to `src/choreo_script.h` next to the
script if a `src/` directory exists there, else `choreo_script.h` alongside
it. Override with `-o <path>`:

```sh
python3 sdk/tools/choreoc.py path/to/<name>.choreo.toml -o path/to/src/choreo_script.h
```

The generated header carries its own regeneration command in a banner
comment and is marked **DO NOT EDIT** — always edit the `.toml`, never the
header, and re-run `choreoc` after every edit. It defines:

```c
#define CHOREO_NAME                    "change-partners"
#define CHOREO_SCRIPT_LEN              3u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 48000u   /* sum of every step's bound */

static const choreo_step_t k_choreo_script[CHOREO_SCRIPT_LEN] = { ... };
```

`CHOREO_SCRIPT_TOTAL_TIMEOUT_MS` is a hard upper bound on script runtime
(every step is time-bounded by construction) — use it to size an outer
mission-duration backstop, e.g.
`MISSION_DURATION_S = CHOREO_SCRIPT_TOTAL_TIMEOUT_MS/1000 + <margin>`.

Wire the header and the L6/L7 sources into your Zephyr app
(`CMakeLists.txt`, alongside the base SDK wiring from
[`README.md`](README.md#quick-start--c-embedded--zephyr)):

```cmake
target_sources(app PRIVATE
    ${TAPESTRY_OS_ROOT}/subsys/bse/bse.c
    ${TAPESTRY_OS_ROOT}/subsys/choreo/choreo.c
)
target_include_directories(app PRIVATE
    ${TAPESTRY_SDK}/include        # for <tapestry/choreo.h>
    ${CMAKE_CURRENT_SOURCE_DIR}/src # for the generated choreo_script.h
)
```

Then in `main.c`:

```c
#include <tapestry/choreo.h>
#include "choreo_script.h"   /* GENERATED — see banner for the regen command */

choreo_init(element_id);
if (choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN) != 0) {
    /* a step was rejected (bad goal, unsatisfiable capability, ...) —
     * this should only happen if the header is stale relative to a
     * choreo.h change; choreoc already validated the script itself. */
}

/* each main-loop cycle, after wm_tick() / scr_tick(): */
choreo_tick(&wm, &scr);
const tapestry_bse_directive_t *d = choreo_get_directive();
if (choreo_script_complete()) {
    /* directive is IDLE — map to this platform's quiescent posture */
}
```

## Building for Python (simulation / research)

No generation step — the same `.toml` is parsed at runtime:

```python
from tapestry.choreo import Choreo
from tapestry.script_toml import load_steps

choreo = Choreo(element_id=0)
choreo.submit_script(load_steps("path/to/<name>.choreo.toml"))

# each simulation cycle:
choreo.tick(wm_entries, scr_state)
directive = choreo.get_directive()
if choreo.script_complete():
    ...
```

`load_steps()` raises `tapestry.script_toml.ScriptError` (a `ValueError`)
on anything the validation rules above reject — catch it if you're loading
a script from outside your own repo.

## Parity

The C engine (fed by the generated header) and the Python engine (fed by
`load_steps()` on the same file) are the same state machine ported twice —
identical step sequencing, identical achievement predicate, identical
timeout math. For a given script and identical inputs, tick counts and
final positions match exactly between the two, which makes the Python SDK
a legitimate way to rehearse a script (including multi-agent parity checks)
before ever compiling it for hardware.

That parity claim doesn't have to stay theoretical — `sdk/tools/choreo_sim.py
--replay` checks it against real recorded runs. Capture a Webots run's
per-tick inputs and outputs to CSV (set `TAPESTRY_TELEMETRY_DIR`; see
`examples/webots-formation/controllers/cf21bl/choreo_telemetry.h`), then
replay that CSV through the Python engine and diff every tick:

```sh
python3 sdk/tools/choreo_sim.py --replay \
    --script examples/cf21bl-formation/change-partners.choreo.toml \
    --telemetry /path/to/choreo_0.csv
```

A clean replay (0 divergences) means the C engine that produced the
recording and the current Python engine agree tick-for-tick on real
flight/simulation data, not just a bare script rehearsal. A divergence
means either the recording is stale (script or engine changed since
capture — re-record) or a genuine regression in `sdk/python/tapestry` vs.
`tapestry-os/subsys/choreo`+`bse`. This is offline capture-and-replay
infrastructure for regression testing, not ML training — see
`tapestry/choreo.h`'s status banner for that distinction.

## Script-authoring simulation

`sdk/tools/choreo_sim.py --simulate` is a lightweight, dependency-free
sanity check for a script you're still editing — sub-second feedback
without a build toolchain or Webots set up, and without a substrate
existing at all yet. It instantiates N in-process `Choreo` objects (no C,
no Zephyr, no network) and ticks them with perfect shared visibility
(every element sees every other element's current position; no gossip,
staleness, or quorum-degradation simulation — quorum is synthesized
HEALTHY), moving each element toward its directive's target at a capped
speed:

```sh
python3 sdk/tools/choreo_sim.py --simulate \
    --script examples/cf21bl-formation/change-partners.choreo.toml \
    --elements 4 --plot
```

`--plot` renders a trajectory/timeline figure (lazy `matplotlib` import —
only this code path touches it; compiling or replaying a script stays
standard-library-only). This is deliberately NOT a fidelity simulator: no
repulsion, leash, or arena-clamp physics — that realism belongs to
`examples/webots-formation`. It is also not a replacement for
`tapestry-csm-sim`/`tapestry-scr-sim`, which validate partition tolerance
and quorum/election under injected network faults against the real
production C engine; `--simulate` assumes all of that away to get a fast
script check on the Python mirror. Run with `--help` for the full flag
reference (`--elements`, `--speed`, `--plot`, `--out`).

## Regeneration workflow

1. Edit `<name>.choreo.toml`.
2. `python3 sdk/tools/choreoc.py <name>.choreo.toml` (or with `-o` if not
   using the default output path).
3. Rebuild the firmware. Commit both the `.toml` and the regenerated
   header — CI/other builders never run `choreoc` themselves.

The Python side needs no rebuild step; it reads the `.toml` directly on
every run.

## See also

- [`README.md`](README.md) — the full L7 Choreographer API (single goals,
  lifecycle states, capability checks) that a script's steps are built from.
- `sdk/tools/choreoc.py` — the compiler; run with `--help` or read its
  module docstring for CLI details.
- `sdk/python/tapestry/script_toml.py` — the schema parser and validator;
  its module docstring is the authoritative parameter reference if this
  document and the code ever disagree.
- `examples/cf21bl-formation/change-partners.choreo.toml` — a
  flight-validated worked example (two-drone station swap) with a
  build+fly walkthrough in that example's own README.
- `sdk/tools/choreo_sim.py` — the offline replay/regression harness
  (`--replay`) and the synthetic script-authoring simulator
  (`--simulate`); run with `--help` or read its module docstring for CLI
  details.
- `examples/webots-formation/controllers/cf21bl/choreo_telemetry.h` — the
  per-tick CSV capture that feeds `choreo_sim.py --replay`.
