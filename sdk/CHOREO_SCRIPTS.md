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
| `form` | a point + shape (see [Frames and anchors](#frames-and-anchors)) | `target = [x, y, z]` (or `frame`/`anchor`), `radius` (required — radius 0 would send every element to the same vertex), `shape`, `spin` (see [Motion](#motion)) |
| `move` | an absolute point | `target = [x, y, z]` |
| `converge` | a point (see [Frames and anchors](#frames-and-anchors)) | `target = [x, y, z]` (or `frame`/`anchor`) |
| `disperse` | current positions, spread apart | `radius` (required — minimum spacing) |
| `orbit` | a preset — see [Motion](#motion) | `around`, `radius`, `rate` |

`hold` and `exchange` are **coordinate-free by design** — they reference the
collective's own configuration, not application-supplied coordinates, and
the parser rejects `target`/`radius`/`shape` on them. This is what lets the
same script fly regardless of where the elements actually start.

`exchange` rotates stations by `shift` around the ID-sorted ring of fresh
participants (frozen snapshots taken at step activation, never live-chasing
a moving peer); `shift = 1` with two elements is a swap. **`exchange` never
touches z** — an element's own altitude, however established, stays fixed
for the whole maneuver; only x/y stations are reassigned. `path = "arc"`
(the default) travels an arc about the formation centroid in the XY plane —
its angular sweep keeps every element rotating the same direction,
preserving mutual separation by construction, so it protects elements with
no altitude separation at all. `path = "direct"` beelines straight to the
destination. See
`examples/cf21bl-formation/change-partners.choreo.toml` for a worked case.

## Frames and anchors

`form` and `converge` normally take an absolute `target = [x, y, z]` — a
world-frame coordinate. `frame` lets you say what that point is defined
*relative to* instead, for platforms without absolute positioning, or
scripts that shouldn't care where the collective happens to be:

| `frame` | Target is... | Needs |
|---|---|---|
| `"absolute"` (default) | `target = [x, y, z]` literally | `target`, and (implicitly) `abs_position` — see [Common parameters](#common-parameters-every-goal) |
| `"collective"` | the live participant centroid | nothing — `target` is rejected |
| `"element"` | a resolved anchor element's position | `anchor = { select = ... }` — `target` is rejected |

`anchor.select` (frame `"element"` only):

| Select | Anchor is... |
|---|---|
| `"leader"` | the current L5-elected leader |
| `"self"` | this element (degenerate — station-keeps on itself) |
| `"lowest-energy"` | the fresh peer (or self) with the lowest `energy_level` |
| `"id:N"` | the explicit element `N` (testing/debug) |

`"newest"`/`"oldest"` (most/longest recently joined) are named in the
design doc but not implemented yet — they need join-order tracking the
world model doesn't keep; the parser rejects them by name rather than
silently ignoring them.

Anchor resolution is **debounced**: a newly-resolved anchor doesn't drive
a directive until it has been the same element for 2 seconds straight
(`TAPESTRY_BSE_ANCHOR_HOLD_MS`) — a single lucky/unlucky gossip frame
must not make the whole collective's anchor flicker (the same lesson
`QUORUM_UP_MS` encodes for quorum acquisition). Once locked, the anchor's
*position* tracks live with no further lag — only *switching which
element is the anchor* is debounced. An anchor that disappears outright
(no leader elected yet, an explicit `id:N` that was never fresh) falls
back to `HOLD` immediately, undebounced — the same fallback `exchange`
uses when it can't yet compute a snapshot.

## Motion

`form`'s `spin` parameter turns a static shape into a maintained,
rotating one — "form a circle" vs. "keep rotating in a circle" (Choreo
SDK Design doc §6). Each vertex's offset from the frame origin (see
[Frames and anchors](#frames-and-anchors)) rotates at the given rate;
achievement still works exactly the same way, just against the *moving*
vertex.

```toml
[[steps]]
[steps.form]
shape    = "circle"
target   = [0.5, 0.5]
radius   = "1m"
spin     = "0.15rad/s"
duration = "60s"
```

`spin` only applies to `form` — `converge`'s target *is* the frame
origin, so "rotating the offset" would be a no-op; the parser rejects it
there rather than silently accept a goal that visibly does nothing.

`spin` implicitly requires `locomotion` — see [Common
parameters](#common-parameters-every-goal) for what "implicitly" means
here and why the example above compiles clean without a `requires` line.

**`spin` never completes** — it's a maintained behavior, not an achieved-
and-done one, so `duration`/`timeout` is doing real work here, not just
satisfying the universal time-bound requirement. `until = "achieved"` is
still allowed alongside it as an early-lock ("stop spinning once I first
land on-station"), but never as the *only* exit.

**`orbit` preset** — `form` + `spin` around an anchor is common enough to
name directly:

```toml
[[steps]]
orbit = { around = "leader", radius = "0.5m", rate = "0.15rad/s",
         duration = "30s" }
```

desugars to exactly:

```toml
[[steps]]
[steps.form]
shape    = "circle"
frame    = "element"
anchor   = { select = "leader" }
radius   = "0.5m"
spin     = "0.15rad/s"
duration = "30s"
```

Pure TOML-layer sugar — no new C primitive, and `around` accepts the same
selectors as `anchor.select` above.

## Events and transitions

Any goal key can carry `name` (a step label) and `on` (a list of guarded
transitions) — the design doc's "welcome dance" (§8.3), reactive instead
of purely linear:

```toml
[[steps]]
name = "triangle"
[steps.form]
shape    = "circle"
target   = [0.0, 0.0]
radius   = "2m"
duration = "60s"
on = [ { event = "element_joined", goto = "welcome" } ]

[[steps]]
name = "welcome"
[steps.orbit]
around   = "leader"
radius   = "0.5m"
rate     = "0.15rad/s"
duration = "30s"
on = [ { event = "element_lost", goto = "triangle" } ]
```

`on` is checked **first**, in declaration order — the first matching
event wins and its `goto` becomes the next step, before
`until = "achieved"`/`duration` are even considered (they remain the
fallback for a step with no matching, or no declared, transition — every
step written before this feature existed is unaffected). `goto` names
another step's `name`, or the literal string `"end"` to complete the
script from anywhere.

Event vocabulary (a subset of the design doc's §8.2 — see that section
for why `quorum_degraded`/`quorum_lost`/`quorum_recovered` aren't here
yet):

| Event | Fires when |
|---|---|
| `achieved` | the step's own achievement predicate (scope-gated) — as an explicit transition target, not just an implicit next-step advance. |
| `element_joined` / `element_lost` | a debounced rise/fall in the fresh participant count (2 s stable, the same lesson [anchor debounce](#frames-and-anchors) encodes — one lucky/unlucky gossip frame must not fire this fleet-wide). |
| `count_gte` / `count_eq` | the live participant count crosses a `threshold` (required on these two events, rejected on every other). |
| `anchor_lost` | a `frame = "element"` goal (see above) couldn't resolve any anchor this tick. |

At most 4 transitions per step (the runtime's fixed-size limit) — the
parser rejects a 5th rather than silently truncating.

**Cycles need `max_runtime`**: a script whose step graph loops back on
itself (like the welcome dance above — `triangle` and `welcome` transition
into each other) can run indefinitely, so the parser requires a top-level
`max_runtime = "..."` bound when it detects one:

```toml
choreo      = "welcome-dance"
max_runtime = "10min"
```

`max_runtime` replaces the summed-step-durations bound
(`CHOREO_SCRIPT_TOTAL_TIMEOUT_MS`) for a cyclic script; an acyclic script
keeps the sum as before and doesn't need it.

## Effects

Any goal key can also carry `indicator` and/or `telemetry_tag` (design
doc §12 Stage 5) — declarative annotations for what a step should signal
and how it should be labeled in a telemetry capture, instead of hand-
computing either in application code:

```toml
[[steps]]
name = "spraying"
[steps.form]
target        = [10.0, 10.0, 3.0]
radius        = 5
duration      = "120s"
indicator     = "active"
telemetry_tag = "spraying_infected_zone"
```

| Key | Meaning |
|---|---|
| `indicator` | `"idle"` \| `"active"` \| `"degraded"` \| `"failed"` — while this step is active, the application's `choreo_current_indicator()` (`choreo_current_indicator` in C, `current_indicator()` in Python) returns this value instead of "no override". Omit for no override (the default, and the behavior of every step written before this feature existed). |
| `telemetry_tag` | An arbitrary non-empty string, surfaced verbatim by `choreo_current_telemetry_tag()` / `current_telemetry_tag()`. Omit for no tag (`NULL`/`None`, the default). |

**What `indicator` does and doesn't do:** Choreo itself never touches L1 —
it has no `substrate_set_signal()` call anywhere. The value above is only
made available for the application's main loop to read once per tick and
pass through, the same way it already reads `choreo_get_directive()` and
passes it to `substrate_move()`. `examples/cf21bl-formation/src/
formation.c`'s `demo_set_leds()` (and the identical, independently
duplicated copy in `examples/webots-formation/controllers/common/
tracker.c`) now take the step's declared indicator as an override,
falling back to their existing quorum/freshness heuristic when a step
leaves it unset — so a script that never sets `indicator` drives those
two apps exactly as before this feature existed.

**What `telemetry_tag` does and doesn't do:** this is local capture only.
`examples/webots-formation`'s `choreo_telemetry.h` CSV writer records it
alongside `script_step` on every tick, so a replay or `choreo-sim` run
can be identified by which authored step produced a given row without
depending on step index alone. It is **not** a wire-delivery mechanism to
an external consumer (e.g. a facility monitoring dashboard reading a live
telemetry stream) — no such consumer exists anywhere in this repo.

## Tracks

A script is normally one `[[steps]]` list every element runs — the
implicit "all" track. `[[tracks]]` (design doc §7) instead declares
several **concurrent, participant-scoped** step sequences; an element runs
exactly one of them, chosen by the **first** whose `filter` it matches:

```toml
choreo = "spill-response"

[[tracks]]                          # perimeter watch: needs a sensor
filter = { requires = ["sensing"] }
[[tracks.steps]]
hold = { duration = "300s" }

[[tracks]]                          # everyone else: catch-all (no filter)
[[tracks.steps]]
form = { target = [0, 0, 3], radius = 5, duration = "300s" }
```

A file gives either `[[steps]]` or `[[tracks]]`, never both. Each track's
`filter` table takes:

| Key | Meaning |
|---|---|
| `requires` | list of capability names, exactly like a step's own `requires` — matches when this element's capabilities satisfy them. |
| `energy_low` | `true`/`false` — matches when this element's own gossiped health state currently reports low battery. |

An empty or omitted `filter` (`filter = {}`, or no `filter` key at all)
matches **every** element — the catch-all a track table needs at least
one of, declared last, so every element has somewhere to run. Filter
membership is evaluated **locally**, against this element's own state
only — never a peer's — so it needs no coordination messages (design
doc's P4). At most `CHOREO_MAX_TRACKS` (4) tracks; a script that declares
more is rejected at parse time, and `choreo_submit_tracks()` itself
rejects a script where no track matches this element.

Each track's `[[tracks.steps]]` is a full, independent step list with the
same schema as `[[steps]]` above — including `name =` / `on = [...]`
transitions and its own `max_runtime` for a cyclic track (§8.4 applies
per-track: a cycle in one track doesn't bound the others). A `goto`
target only resolves within the **same** track — one track's steps can't
jump into another's.

Migrating to a different track (a filter-boundary crossing, e.g. battery
crossing the low-battery threshold) is debounced exactly like
`element_joined`/`element_lost` above, and activates the new track's
current step **fresh** — new snapshot, new timers, not a resumed state.
Each track's own step index is remembered while inactive, though, so
re-entering a track later resumes where it left off rather than
restarting from step 0.

**Why this matters to other elements**: an element gossips which track
it's currently active in (`current_track`, wire v4) so peers running
`frame = "collective"` or a `count_*`/`element_joined` event compute their
centroid/count from elements actually doing the SAME thing — a peer that
migrated off to charge its battery, say, is automatically excluded rather
than skewing the group everyone else is coordinating around. A script
with no `[[tracks]]` gossips `current_track = 0` unconditionally — byte-
identical to every script written before this feature existed.

Python:

```python
from tapestry.script_toml import load_tracks
tracks = load_tracks("spill-response.choreo.toml")
choreo.submit_tracks(wm_entries, tracks)
```

## Common parameters (every goal)

| Key | Meaning |
|---|---|
| `duration` / `timeout` | step time bound. **Required on every step** — this is the robustness net that keeps a script from stalling; give exactly one of the two names (they're the same field). |
| `until = "achieved"` | advance as soon as the achievement predicate fires (scope decides whose — see `scope` below), instead of waiting out the full duration. The timeout still applies as a fallback. Not allowed on `hold` — hold is trivially achieved, so hold steps are duration-governed (`until`/`eps`/`settle` on hold are rejected; reserved for future scoped-achievement semantics). |
| `eps` | achievement radius (default: BSE default if omitted). |
| `settle` | how long the error must stay within `eps` before achievement fires (default: BSE default). |
| `scope = "self"` \| `"all"` | whose achievement `until = "achieved"` waits for (default `"self"`). `"all"` advances only once this element **and** every fresh peer have achieved — aggregated from an `achieved` bit each element gossips every cycle ("achieved-bit" item). Eventually consistent, bounded by gossip latency — not a synchronization barrier (that's the doc's separate `barrier = true`, not implemented). A lone element with no fresh peers is vacuously "all achieved", so it can't deadlock alone. Only valid alongside `until = "achieved"`; not allowed on `hold`. |
| `requires` | list of capability names the executing element must have: `["locomotion", "bonding", "sensing", "signaling", "abs_position"]`. A step whose requirements the registered element can't satisfy is rejected at submit time. |

**Duration syntax**: `"30s"`, `"500ms"`, `"45min"`, `"2h"`, or a bare
number (seconds).
**Length syntax**: `"25cm"`, `"250mm"`, `"500um"`, `"0.25m"`, or a bare
number (meters).

**The derived floor** (Choreo SDK Design doc §11): some `requires`
capabilities are implied by a goal's *other* fields, whether or not you
write them yourself — the runtime unions them into what's actually
checked at deploy time regardless:

| If a step has... | it also requires... |
|---|---|
| `motion`/`spin` (i.e. `form` + `spin = ...`) | `locomotion` |
| `frame = "absolute"` (the default, `form`/`converge` only) | `abs_position` |

This is not optional and there is no way to opt out of it — it reflects
what the goal mechanically needs, independent of whether `requires` says
so. `choreoc` and `choreo_sim`'s `--simulate` **warn** (not reject) when
a step's `requires` doesn't already cover its derived floor, e.g.:

```
choreoc: warning: steps[0]: frame = "absolute" (the default) requires
abs_position at runtime (Choreo SDK Design doc §11) even though
'requires' doesn't list it — add requires = ["abs_position", ...] to
make it explicit, or opt into frame = "collective"/"element" if that's
what was intended
```

The script still compiles and the warning doesn't fail CI — the point is
authoring-time visibility, not another gate. Without it, a script author
who forgets `requires = ["abs_position"]` (or leaves `frame` at its
`"absolute"` default without meaning to) only finds out at flight time,
when `choreo_configure()`/`choreo_submit_script()` rejects the goal with
`-EPERM` on an element that doesn't have that hardware capability
granted at `scr_init()`.

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
- `hold`/`exchange` must not carry coordinates; `move` must carry
  `target`; `form`/`converge` must carry `target` (`frame = "absolute"`,
  the default) *or* `frame`/`anchor` instead (`target` is then rejected —
  see [Frames and anchors](#frames-and-anchors)); `disperse` must carry a
  `radius`.
- Unknown goal keys, unknown parameters, and unknown capability/shape names
  are rejected with a message naming the offending step index and the
  allowed set.
- `spin` only applies to `form` (and its `orbit` desugaring) — it is
  rejected as an unknown parameter everywhere else, including `converge`.
  Since every step already needs a real time bound (the first bullet
  above), a `spin` step can never be authored with `until = "achieved"`
  as its only exit — the C/Python API allows that combination directly
  and rejects it separately (a non-terminal motion never "completes").
- `goto` must name a step's own `name = "..."`, or be the literal `"end"`
  — `end` is reserved and cannot be used as a step name; a duplicate step
  name is rejected too. At most 4 transitions (`on = [...]`) per step.
  `count_gte`/`count_eq` require a `threshold`; every other event rejects
  one.
- A script whose step transitions form a cycle must declare a top-level
  `max_runtime` bound (see [Events and transitions](#events-and-transitions))
  — the parser detects this statically rather than let an unbounded show
  reach flight.
- `indicator` must be one of `"idle"`/`"active"`/`"degraded"`/`"failed"`
  (there is no `"none"` — omit the key instead); `telemetry_tag` must be
  a non-empty string. Both are allowed on every goal key (see
  [Effects](#effects)).

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

A `[[tracks]]` script (see [Tracks](#tracks)) generates a track table
instead of a flat step array — `CHOREO_N_TRACKS`/`k_choreo_tracks`
in place of `CHOREO_SCRIPT_LEN`/`k_choreo_script`, submitted via
`choreo_submit_tracks()` instead of `choreo_submit_script()`:

```c
#define CHOREO_N_TRACKS                2u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 300000u   /* worst case across tracks */

static const choreo_track_t k_choreo_tracks[CHOREO_N_TRACKS] = { ... };

choreo_init(element_id);
choreo_submit_tracks(&wm, k_choreo_tracks, CHOREO_N_TRACKS);

/* each cycle: gossip current_track alongside goal_achieved (own_state is
 * whatever this platform's gossip_send() reads from) */
own_state.current_track = choreo_current_track();
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

A recording must carry every input the replayed engine reads, and for a
`scope = "all"` step that includes each peer's gossiped `achieved` bit.
Recordings captured before `choreo_telemetry.c` wrote that field replay
with every peer looking never-achieved, so the collective step advances on
its timeout instead and reports divergences that are the recorder's fault,
not the engine's; `--replay` warns when it sees one. Re-capture with a
current build.

CI replays a committed recording (`sdk/tests/data/`) on every push. That
one is generated by the Python engine rather than captured from Webots, so
it proves self-consistency, not cross-language parity — it catches an
unintended change to the Python engine's tick-by-tick behavior. Proving
parity still takes a real capture, which is what this section describes.

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
3. **Regenerate every consumer.** One script can be compiled into more
   than one header — `change-partners.choreo.toml` feeds both
   `examples/cf21bl-formation/src/` and
   `examples/webots-formation/controllers/cf21bl/` — and each needs its
   own `-o` run. Miss one and that consumer silently keeps running the
   previous version of the show; this has happened.
4. Rebuild the firmware. Commit both the `.toml` and the regenerated
   headers — CI/other builders never run `choreoc` themselves.

To find out what is stale without regenerating anything:

```bash
python3 sdk/tools/choreoc.py --check
```

With no arguments this checks every generated header in the repository,
recovering each one's source script from the regenerate command line in
its own banner, and exits non-zero if any differs from what its script
generates today. CI runs exactly this, so a missed step 3 fails the build
instead of reaching a drone. Add `<script> -o <header>` to check a single
pair.

The Python side needs no rebuild step; it reads the `.toml` directly on
every run — which is also why a stale header shows up as the C and Python
engines disagreeing in a replay diff.

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
