# Warehouse AMRs — coordination with no coordinator, and what breaks without it

Eight simulated warehouse robots running Tapestry's real, unmodified L3–L7
stack against Webots physics. Three scenes, three Choreo scripts, one
controller binary. Scenes 2 and 3 each put an apples-to-apples, 8-robot
"cloud"-coordinated fleet on the same floor, under the same fault, so the
difference between decentralized and centralized coordination needs no
imagination.

This is the ground-vehicle counterpart to
[`examples/webots-formation`](../webots-formation/README.md), and the second
substrate written to that example's pattern. The point of building it on a
platform that drives rather than flies is that it tests the claim that
example makes: that everything in its `controllers/common/` is genuinely
substrate-agnostic. It is — `world_model.c` (L4), `scr.c` (L5), `bse.c` (L6),
`choreo.c` (L7) and `gossip.c` (L3 framing) are compiled here byte-for-byte
identically to the drone build and to real cf21bl hardware.

**There is no coordinator in any scene.** Each robot decides what to do from
its own L4 world model, built purely from gossip frames its own radio
received. The `warehouse_supervisor` controller renders on-screen labels and
has no authority over anything — it never writes to a robot and nothing in
L3–L7 reads it (see [`controllers/rover/status_tx.h`](controllers/rover/status_tx.h)).

## Run it

```sh
export WEBOTS_HOME=/Applications/Webots.app
export WEBOTS_HOME_PATH=/Applications/Webots.app/Contents   # macOS: note the /Contents

cd controllers/rover              && make
cd ../warehouse_supervisor        && make
cd ../cloud_bot                   && make   # needed for scenes 2 and 3
cd ../cloud_coordinator           && make   # needed for scenes 2 and 3
cd ../..

open -a Webots worlds/scene1_zone_allocation.wbt
```

(Adjust paths for your platform. Webots' Makefile system wants `WEBOTS_HOME`
to be the bundle root but `WEBOTS_HOME_PATH` to be where
`resources/Makefile.include` lives; on macOS those differ by `/Contents`, and
a `webots/robot.h file not found` failure is almost always that.)

Don't use `open -a Webots` for the `CONSISTENCY_BIAS=1.0` run below — `open`
launches the app through macOS LaunchServices, which does not forward shell
environment variables to the process it starts. Invoke the Webots binary
directly instead, as shown.

Every scene:

```sh
open -a Webots worlds/scene1_zone_allocation.wbt
open -a Webots worlds/scene2_partition_vs_cloud.wbt
open -a Webots worlds/scene3_failover_vs_cloud.wbt

# Scene 2's CP-mode comparison — must invoke the binary directly (see above)
CONSISTENCY_BIAS=1.0 /Applications/Webots.app/Contents/MacOS/webots \
    worlds/scene2_partition_vs_cloud.wbt
```

Headless, for a scripted run:

```sh
/Applications/Webots.app/Contents/MacOS/webots --batch --mode=fast \
    --no-rendering --minimize --stdout --stderr worlds/scene3_failover_vs_cloud.wbt
```

---

## Scene 1 — Decentralized task allocation without a cloud

`worlds/scene1_zone_allocation.wbt` ·
[`scene1-zone-allocation.choreo.toml`](scene1-zone-allocation.choreo.toml)

Eight AMRs leave the inbound dock, fan out across the floor, and settle into
nine pick zones. **No step in the script names an element, a zone, or a fleet
size.**

Each robot computes its own zone locally:

- `scr_tick()` sorts the trusted, non-stale peers in *this robot's own world
  model* by element ID, and hands back `task_slot` (its own ordinal in that
  sort) and `swarm_size` (the count) — [`scr.c`](../../tapestry-os/subsys/scr/scr.c).
- `bse_tick()` turns `FORM`+`grid` into a cell for exactly that rank —
  [`bse.c`](../../tapestry-os/subsys/bse/bse.c).

Every robot computes the same sorted order from its own converged copy, so
the assignment is consistent with no election, no assignment message, and no
arbiter. Change the fleet size and the layout re-solves; there is nothing to
reconfigure.

## Scene 2 — Partition recovery, decentralized vs. centralized

`worlds/scene2_partition_vs_cloud.wbt` ·
[`scene2-partition.choreo.toml`](scene2-partition.choreo.toml)

A steel mezzanine deck spans the middle of the warehouse, raised on legs so
AMRs drive underneath it. It is opaque to 2.4 GHz. Two fleets share the same
floor and the same fault, run side by side so the comparison needs no
imagination: **West (orange), 8 robots** run the decentralized Tapestry
script below, with zero cloud awareness. **East (blue), 8 robots** run under
a single centralized coordinator, commanded to cross the same deck — see
"the cloud comparison fleet" further down for what that side is and how it
fails. Apples-to-apples fleet size on both sides, so the only variable is
the coordination architecture.

**Nothing in this scene injects a fault.** The partition is a consequence of
the fleet doing its job: the script's `move` step sends the formation to pick
faces on the far side of the deck, and the fleet splits into a 5-element
island north of it and a 3-element island south of it because that is where
it was told to go. It heals when the script recalls the fleet to the dock.

Observed mid-partition — two islands, two irreconcilable views of how big
the fleet is, both reporting healthy quorum, and **each island renumbering
its own task slots**:

```
id=0 t=42.0 pos=(-2.60, 0.53) slot=0/3 q=H RF-BLOCKED   <- south island, fleet of 3
id=1 t=42.0 pos=( 0.37, 0.52) slot=1/3 q=H RF-BLOCKED
id=2 t=42.0 pos=( 3.34, 0.53) slot=2/3 q=H RF-BLOCKED
id=3 t=42.0 pos=(-2.65, 3.52) slot=0/5 q=H RF-BLOCKED   <- north island, fleet of 5
id=4 t=42.0 pos=( 0.33, 3.55) slot=1/5 q=H RF-BLOCKED
id=7 t=42.0 pos=( 0.32, 6.57) slot=4/5 q=H RF-BLOCKED
```

Element 3 is `slot=0/5` — it has become rank 0 of its own island, because
rank is an ordinal over the peers you can actually hear. Neither island is
wrong; they simply have different membership. After the recall every element
reports `slot=N/8` again with its original rank.

The occlusion decision is made entirely locally
([`rf_occlusion.c`](controllers/rover/rf_occlusion.c)) — each robot knows the
deck's footprint the way a real robot carries a site map, knows its own
position from its own GPS, and knows each peer's position only from that
peer's last gossip frame. It masks its own transmitter accordingly. Nothing
has a global view.

**Both islands keep working.** With `quorum_min=1`/`quorum_target=2`, a
3-robot island is still a working group, so both sides hold quorum and keep
executing the same script step, out of contact, on divergent world models.
That is the AP behaviour and it is the default.

To see the other side of the dial:

```sh
CONSISTENCY_BIAS=1.0 /Applications/Webots.app/Contents/MacOS/webots \
    worlds/scene2_partition_vs_cloud.wbt
```

`consistency_bias` scales L4's internal quorum threshold
(`bias * WM_QUORUM_FRACTION`, [`world_model.c`](../../tapestry-os/subsys/csm/world_model.c)).
At `1.0` an element whose fresh-peer fraction drops below 0.5 sets
`metric.degraded` and stops moving. L4 only reports the condition; freezing
is the application's choice, exactly as in
[`tapestry-csm-sim`](../../tapestry-csm-sim/zephyr/element/src/main.c) — this
scene is the same experiment as that harness's AP/CP plots, with robots
attached instead of a CSV.

Measured across the two runs:

| | AP (`0.0`, default) | CP (`1.0`) |
|---|---|---|
| element-seconds frozen | **0** | **28** |
| worst-hit elements | — | ids 6 and 7 (6–7 s each) |

**The freeze is a transient around the moment of the split, not a sustained
minority stall**, and the mechanism is worth following because it is easy to
predict wrongly. The denominator is *active* peers, not *known* peers. When
the fleet splits, the far island's entries stay active-but-stale for the
3.5 s between `WM_STALE_THRESHOLD_MS` (1.5 s) and `WM_EXPIRE_THRESHOLD_MS`
(5 s) — during that window an element on the wrong side of the ratio freezes
(`fresh 2/5 active` → 0.4 < 0.5). Once those entries *expire* they leave the
denominator entirely, the ratio returns to 1.0, and the element resumes with
a smaller world it is fully confident about. Elements 6 and 7 freeze longest
because they cross first and are isolated longest.

So CP here buys a bounded stall of a few seconds per element, not a
deadlock — and not a permanently immobilised minority either.

### The cloud comparison fleet

**What the comparison fleet is, stated plainly, because precision matters
here.** It is a minimal, honestly-labeled architectural analog for
centralized/cloud-dependent coordination — a single process
([`cloud_coordinator/main.c`](controllers/cloud_coordinator/main.c)) computes
every robot's task and pushes it hub-and-spoke; robots
([`cloud_bot/main.c`](controllers/cloud_bot/main.c)) have zero peer-to-peer
awareness of each other. It is **not** a reimplementation of any specific
real framework — no particular plan language, role-assignment algorithm, or
sync protocol is attempted here, and no claim is made about matching any
real system's resilience in every detail. What *is* modeled faithfully is
the one property
every system in that family shares by construction: a robot that depends on
one authority has nothing else to fall back on when that authority is
unreachable. See [`cloud_protocol.h`](controllers/cloud_common/cloud_protocol.h)
for the full framing.

The occlusion decision for the cloud fleet is made the identical way scene
2's own RF model works — at the RECEIVING robot, using its own live position
against the coordinator's known fixed one
([`cloud_occlusion.c`](controllers/cloud_bot/cloud_occlusion.c), a
standalone duplicate of `rf_occlusion.c`'s geometry, checked against it by
[`cloud_geom_check.c`](ci-check/cloud_geom_check.c) so the two can't silently
drift apart). The two fleets don't share a network, a process, or a byte of
state — only physics.

Measured, on a clean run:

```
cloud_bot id=16 FROZEN at t=14.0s (pos 5.05,2.40)
cloud_bot id=17 FROZEN at t=14.0s (pos 6.05,2.40)
cloud_bot id=18 FROZEN at t=14.0s (pos 7.05,2.40)
cloud_bot id=19 FROZEN at t=14.0s (pos 8.19,2.63)
cloud_bot id=20 FROZEN at t=14.0s (pos 4.05,2.40)
cloud_bot id=21 FROZEN at t=14.3s (pos 2.97,2.27)
cloud_bot id=22 FROZEN at t=14.3s (pos 1.97,2.27)
cloud_bot id=23 FROZEN at t=14.4s (pos 0.87,2.26)
```

All eight cloud robots freeze right at the deck's shadow boundary
(`RF_DECK_Y_MAX` = 2.6 m) and **never move again for the rest of the run** —
there is no mechanism on the robot, and none on the (still-alive) coordinator
either, for a blocked robot to improvise a path around a link it can't
predict clearing. In the same run, the Tapestry fleet crosses the same deck,
splits, and reconciles exactly as described above, unaffected by the cloud
fleet's presence (separate x-range, separate ports, zero errors, zero
separation violations on either fleet).

**The freeze timeout is deliberately `CLOUD_COORD_TIMEOUT_MS` = 1500 ms —
identical to Tapestry's own `WM_STALE_THRESHOLD_MS`.** The comparison is
about the architecture, not a rigged clock: both fleets get the same grace
period before declaring a link gone.

## Scene 3 — Element failure, one peer vs. the one authority

`worlds/scene3_failover_vs_cloud.wbt` ·
[`scene3-failover.choreo.toml`](scene3-failover.choreo.toml)

Two fleets, two different failures, at the same moment, so the comparison is
apples-to-apples: **West (orange), 8 robots** run the decentralized Tapestry
script below, with zero cloud awareness. **East (blue), 8 robots** orbit a
second ring under a single coordinator — see "the cloud comparison fleet"
further down for what that side is and how it fails.

Eight AMRs start in a grid at the dock and form a **ring** around
`(-5, 0)`, radius 4 m, then keep it continuously **spinning** (`bse.c`'s
`SPIN` motion, `spin = "0.15708rad/s"` in the `.choreo.toml` — the same
angular rate as the cloud fleet's own orbit, so side by side the two rings
visibly stay in lockstep right up to the moment the cloud one freezes; a
static Tapestry ring next to a moving cloud one would read as two different
demos, not one comparison). At t=50 s element 3 hard-fails: it stops driving,
stops gossiping, and **does not announce a departure**. A hard failure is
silence, not a goodbye — the departure mechanism
(`ELEMENT_HEALTH_DEPARTED`, and `choreo.h`'s departure policies) is a
different thing with different semantics, and using it here would prove the
wrong claim.

**Ring, not line.** An earlier version of this scene used `FORM`/`LINE`
along a pick face, and the reallocation logic was correct — but almost
impossible to actually *see*: 8 points respacing along a line by fractions
of a meter reads, at a glance, as "the line looks the same". Losing 1 of 8
evenly-spaced points on a ring changes every survivor's angular position by
a visibly different amount instead.

The seven survivors infer the loss on their own and re-rank onto a 7-point
ring at the same center, still spinning. Measured, on a clean run:

```
id=4 t=50.0  slot=4/8  goal=(-9.00, -0.00)    <- element 3 dies at t=50.0
id=4 t=52.0  slot=3/7  goal=(-8.60,  1.74)    <- reallocated, matches the
                                                  computed 7-point ring exactly
```

**The latency is `WM_STALE_THRESHOLD_MS` = 1500 ms, and it is worth being
precise about which threshold that is.** Both `scr.c`'s candidate scan and
`bse.c`'s `collect_participants()` filter on `!is_stale`, so a silent element
leaves `swarm_size`, `task_slot` and the FORM participant set 1.5 s after its
last frame. `WM_EXPIRE_THRESHOLD_MS` (5 s) is a *later, separate* transition
that clears `is_active`; it governs when the corpse stops voting in
`choreo_collective_achieved()`, not when its work is redistributed.

**The corpse is a physical obstacle the collective can no longer see** (the
tracker's repulsion acts on fresh peers only, and a dead element goes stale
1.5 s after its last frame), so it was worth checking, not assuming, that no
survivor's reallocation path passes close to where element 3 died. The
nearest approach — id4's move from `(-9,0)` to `(-8.60,1.74)` — comes no
closer than 1.34 m to the corpse at `(-7.83,2.83)`, clear of the 0.8 m
emergency-repulsion floor with margin. Every other survivor clears by 2.2 m
or more (see the `.choreo.toml`'s own comment for the full table).

**One known residual**, in the same spirit as scene 1's: converging from the
grid start onto the ring produces a burst of separation-violation ticks
(dense transit traffic, not a collision) confined entirely to the initial
convergence — zero violations occur during or after the spin-up, the
reallocation, or the cloud fleet's freeze, the parts of the run that actually
matter. Same underlying limitation as scene 1 (a differential drive's
avoidance reflex is not a planner); see that scene's entry below.

### The cloud comparison fleet

**West (orange):** the Tapestry ring above, centered on `(-5, 0)`. **East
(blue), 8 robots:** continuously **orbiting** a second ring, centered on
`(5, 0)`, radius 4 m, under a single coordinator, at the same angular rate as
the Tapestry ring — a 2 m gap separates the two rings (checked by
[`cloud_geom_check.c`](ci-check/cloud_geom_check.c)).

The comparison fleet loses something different from a peer — **its
coordinator**, because that is the honest analog of "one node fails" for an
architecture whose intelligence lives in exactly one place. The cloud robots
orbit continuously for the same reason the Tapestry ring above does: a
coordinator death has to interrupt real motion to read as a failure, not
"robots that were already idle stay idle". Each robot's orbital phase is
derived from its own start position
([`cloud_coordinator/main.c`](controllers/cloud_coordinator/main.c)), so it
begins orbiting immediately with no initial catch-up.

Measured, on a clean run — both failures land within 0.1 s of each other by
construction (both scripted for t=50 s):

```
id=3   *** HARD FAILURE at t=50.0s ***
id=4   t=50.0  slot=4/8            t=52.0  slot=3/7      <- reallocated in ~2s

cloud_coordinator  *** DEAD at t=50.1s ***
cloud_bot id=16    FROZEN at t=51.6s (pos 5.09, 4.04)   <- mid-orbit
cloud_bot id=17    FROZEN at t=51.6s (pos 0.98, 0.15)
cloud_bot id=18    FROZEN at t=51.6s (pos 4.91,-4.08)
cloud_bot id=19    FROZEN at t=51.6s (pos 9.11,-0.21)
cloud_bot id=20    FROZEN at t=51.6s (pos 2.37, 2.87)
cloud_bot id=21    FROZEN at t=51.6s (pos 1.91,-2.58)
cloud_bot id=22    FROZEN at t=51.6s (pos 6.63,-3.47)
cloud_bot id=23    FROZEN at t=51.6s (pos 7.92, 2.57)
```

t=51.6s is exactly 1.5 s after the coordinator's death — the freeze timeout
firing on schedule. None of the eight freeze positions match where that
robot started (`(9,0)`, `(5,4)`, `(1,0)`, `(5,-4)`, and the four points
between them) — each has visibly traveled a different distance around its
ring, confirming genuine continuous motion rather than eight robots that
happened to be parked. **One fleet loses one of eight members and adapts in
two seconds. The other loses its one authority and every member it has
stops, permanently, at whatever point of its orbit it happened to
occupy.** Zero errors on either fleet in the same run (the grid-to-ring
convergence violations noted above still occur here too — same Tapestry
fleet, same convergence, unrelated to either failure).

---

## What is new here, and what is reused

Reused unmodified from `examples/webots-formation/controllers/common/`: the
UDP transceiver, the Zephyr shim that lets `gossip.c` build outside Zephyr,
the Webots stubs, and `tracker.c`'s target-leash/repulsion/arena-clamp
follower. Two additive changes were made there, both no-ops for the drone
build:

- `udp_posix_set_rx_filter()` — an optional receive-side predicate consulted
  per datagram, default NULL (accepts everything). Not a transmit mask —
  see `rf_occlusion.h`'s long comment for why a transmit-side version of
  this does not work.
- `#ifndef` guards around `tracker.h`'s `DEMO_*` constants, so a substrate at
  a different physical scale can override them from its own build. This
  example overrides all four; see
  [`controllers/rover/sources.mk`](controllers/rover/sources.mk) for why each
  one is genuinely wrong at warehouse scale rather than merely conservative.

New: the differential-drive substrate, the spring follower, the RF model, the
avoidance reflex, the status feed, and the three scenes' scripts and worlds —
plus, for the cloud comparison in scenes 2/3,
[`cloud_bot/`](controllers/cloud_bot/), [`cloud_coordinator/`](controllers/cloud_coordinator/),
and the shared [`cloud_common/cloud_protocol.h`](controllers/cloud_common/cloud_protocol.h).
The comparison fleet links no Tapestry L3-L7 source at all (see
`cloud_bot/main.c`'s header comment for why that absence is the point) — it
reuses only `rover/substrate_webots.c` (pure motor/sensor plumbing) and
`rover/status_tx.c` (the presentation-only telemetry feed both fleets
share), unmodified, by relative path.

## Known limitations

**A failed element is an obstacle nobody can see.** Avoidance and repulsion
both act on *fresh* peers, so 1.5 s after a robot dies its hull stops
existing as far as the collective is concerned. Scene 3's ring reallocation
was checked, not assumed, for this reason: the nearest survivor path passes
1.34 m clear of where element 3 died (see scene 3's own section above for
the full numbers). Nothing in Tapestry prevents a script from routing a
survivor through a corpse's position — obstacle handling is below L6 — and a
script author on real hardware has to check this for their own geometry, the
same way this example's own `.choreo.toml` comment does.

**Avoidance is a reflex, not a planner.**
[`avoid.c`](controllers/rover/avoid.c) exists because `tracker.c`'s repulsion
was validated on holonomic airframes: a drone told to move sideways does, but
a differential drive told to back away from a head-on peer just stalls. The
first full scene-3 run logged 1140 separation-violation ticks with a
worst-case peer distance of 0.40 m — contact, between 0.5 m robots. The fix
adds a tangential term with a fixed rotation sense, so two robots resolve an
encounter to opposite sides without negotiating. It has no notion of static
obstacles (racking, mezzanine legs), no deadlock resolution, and no
guarantee.

**Scene 2 runs with zero separation violations. Scenes 1 and 3 do not**, and
the residual is worth stating rather than hiding — both are dense-transit
artifacts, not collisions, and both are confined to the opening seconds
rather than the events each scene is actually demonstrating. Scene 1: as the
fleet leaves its tight starting block for the assigned grid cells, the
closest pair gets to **0.53 m** centre-to-centre (bodies are 0.5 × 0.34 m)
around t=12 s, with only 6 of 1270 sampled ticks under the 0.8 m alarm
threshold — the settled grid itself sits at 3.43 m. Scene 3: converging from
the grid start onto the ring produces a burst of violation-ticks in the same
0.5–0.8 m range, entirely before the ring settles into its own continuous
spin — well clear of the failure event at t=50 s, and the settled,
spinning ring holds the full 4 m radius throughout. Widening the avoidance
engagement radius by 25% was tried (for scene 1) and measurably did not help
(worst case unchanged, violation count slightly up): the limit is not how
early the nudge engages but that a differential drive must stop and turn
before it can move at all. Fixing it properly needs a local planner, which
is out of scope for an example about coordination.

**The RF model is binary geometry.** No path loss, no fading, no partial
delivery, no antenna pattern. It exists to produce a clean, reproducible,
physically-motivated partition — not to be an RF simulator. Links do flap
while a robot transits the deck footprint; `QUORUM_UP_MS` (2 s) damps the
resulting re-election churn.

It is a **receive** filter, which is worth flagging because the intuitive
design is a transmit mask and the intuitive design does not work. A
transmitter can only consult what it last *heard* about a peer, and that
belief freezes at the instant the peer became unreachable — while the peer
was still on the near side. The element that stayed put therefore never
starts blocking, keeps transmitting, and keeps being heard. Two separate
transmit-side attempts produced a stable **one-way** link (north island
reporting `swarm_size` 8 while the south reported 3) before this moved to
the receiver, which is the only party holding both endpoints as live truth.
See [`rf_occlusion.h`](controllers/rover/rf_occlusion.h).

**`examples/webots-formation/ci-check` does not build on macOS.** Pre-existing
and unrelated to this example, but you will hit it: Apple's libc hides
`INADDR_LOOPBACK` when `_POSIX_C_SOURCE` is defined without `_DARWIN_C_SOURCE`,
so `transceiver_udp_posix.c` fails to compile there while passing on Linux CI.
This example's own [`ci-check/Makefile`](ci-check/Makefile) adds
`-D_DARWIN_C_SOURCE`; the drone example's has not been changed.

## CI

```sh
cd ci-check && make
```

No Webots install needed:

- **`link-check`** — compiles and links the whole rover controller (every
  Tapestry source L3–L7, the substrate, and the shared `common/` files)
  against stub Webots headers. Never run; it catches compile and signature
  breakage.
- **`sup-check`** — same for the supervisor.
- **`cloud-link-check`** — same for `cloud_bot/` and `cloud_coordinator/`
  (scenes 2/3's comparison fleet).
- **`geom-check`** — builds and **runs** two checks:
  - [`rf_geom_check.c`](ci-check/rf_geom_check.c) recomputes scene 2's
    station positions from `bse.c`'s own layout math and asserts that the
    partition still splits 3/5 with adequate clearance. Scene 2's whole
    claim is a coincidence between three files with no compile-time
    relationship — the deck footprint in `rf_occlusion.h`, the targets in
    the `.choreo.toml`, and the `Solid` in the `.wbt`. Nudge any one and the
    scene quietly stops demonstrating anything, with no symptom but a less
    interesting movie.
  - [`cloud_geom_check.c`](ci-check/cloud_geom_check.c) asserts the cloud
    fleet's crossing path clears the deck's support legs (the check that
    would have caught the clearance bug described above before ever
    launching Webots) and that `cloud_occlusion.c`'s geometry agrees with
    `rf_occlusion.c`'s at every sampled point — two independent files with
    no compile-time link between them, by design (see `cloud_occlusion.h`).

Neither compile target substitutes for building against the real Webots SDK
before trusting a change to the device calls themselves.

The three Choreo scripts are covered by the repo-wide check:

```sh
python3 sdk/tools/choreoc.py --check      # from the repo root
```

Their generated headers live at `controllers/rover/scene<N>/choreo_script.h`
— in per-scene subdirectories, each under that exact filename, because that
is what `--check` globs for. Named `choreo_script_scene<N>.h` they would have
been invisible to it, and a `.choreo.toml` edited without regenerating would
ship silently.
