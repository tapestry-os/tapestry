# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- **Remote L6 directive path over the wire** (`TAPESTRY_MSG_DIRECTIVE`, wire
  v5) — an edge/cloud BSE host (or an elected `SCR_CAP_BSE_HOST` element)
  can now stream per-element directives that steer an element directly,
  instead of every element only ever running its own local BSE. The local
  BSE and script keep ticking the whole time regardless, so falling back
  is bumpless by construction — never a resume-from-freeze. Bumps
  `TAPESTRY_WIRE_VERSION` to 5.

### Fixed
- **`HOLD` baked in tracker overshoot no longer a permanent station offset.**
  `HOLD` now inherits the prior intent's own achieved `MOVE_TO_POINT` goal point
  when one exists, rather than the live position at the moment it took
  over.
- **A cutebot booting in isolation (no peers yet negotiated) recovers.**
  The self-healing renegotiation retry already validated in
  `cf21bl-formation` (`transport_negotiate_id_retry()`) is now shared via
  `transport.c`, so cutebot gets it too.
- **`choreo_collective_achieved()` had no track filter.** It iterated
  every active peer with no `current_track` comparison, so on a
  multi-track script a peer working a *different* track could block a
  `scope = "all"` step on this one. `bse.c`'s `collect_participants()`
  already filters the collective centroid by `current_track`; the
  achievement predicate now applies the same filter. Latent until now —
  no shipped script uses tracks yet.
- **`form-grid`'s vertex spacing changed to align with Demo spacing**
  `radius` is now 50 (400 mm, 2.5x the floor) in line with `formation.c`'s
  current `DEMO_TARGET_SPACING`.

## [1.0.0] - 2026-08-27

### Added
- **`CONFIG_CF21BL_RADIO_ADDR_OVERRIDE` / `_LSB`** — give an element its own
  CRTP console address (`0xE7E7E7E7xx`) via `SYSLINK_RADIO_ADDRESS`. 
- **`cf21bl_lighthouse_fix_age_ms()`** — reports how stale a lost fix is, not
  merely that it is lost; surfaced in the fix-loss warning.
- **`CONFIG_CF21BL_STABILIZER_LOG_LEVEL`**, folded into
  `DEMO_CONSOLE_VERBOSE`. The console is not free: it shares USART6 with P2P
  gossip, and the STM32F4's 1-byte UART FIFO makes every byte an ISR
  competing with the 1 kHz control loop. 
- **CI compile check for `examples/webots-formation`** — rather than 
  an actual Webots install (~1 GB), a new sibling directory
  `ci-check/` compiles and links the same L3-L7 core sources (pulled in
  unmodified from `controllers/cf21bl/` and `controllers/common/`) against
  header stubs for the ~10 `wb_*` calls this controller actually makes
- **Python test suite for the Choreographer SDK, enforced in CI** — 227
  pytest tests (`sdk/tests/`) across `bse.py`, `choreo.py`,
  `script_toml.py`, `choreoc.py` and `choreo_sim.py`.
  Covers the L6 geometry and achievement predicate (`FORM` shapes, `MOVE`
  rigid translation, the `EXCHANGE` snapshot/arc/standoff rules), the L7
  lifecycle, capability gate, script validation and quorum suspension
  (including the per-goal exception that keeps `HOLD` ticking while
  suspended), every `.choreo.toml` accept and reject path, the emitted C
  of `choreoc`, and both `choreo_sim` modes. The `.choreo.toml` tests
  assert the error an author gets back, not just the value produced — a
  parser that accepts a typo silently is the failure that matters on the
  authoring surface. A new `python` CI job runs them plus `ruff` 
- **L3 gossip wire round-trip tests (`tapestry-os/tests/transport/`)** —
  18 ztests driving `gossip_send`/`gossip_drain` through a loopback
  transceiver, so real bytes are packed, transmitted and unpacked. Every
  other suite stubs the wire: the `scope = "all"` tests write
  `wm.entries[N].state.goal_achieved` directly, proving the consumer of a
  gossiped bit correct while assuming the bit ever arrives. Covers
  full-field round trip, the `achieved` bit in both polarities, multi-peer
  drain without cross-wiring, the frame-size and field-order wire
  contract, and rejection of version-mismatched, truncated and own-id
  frames
- **Authenticated and relay framing built and run in CI** — the transport
  suite is built three ways (default, `auth.conf`, `relay.conf`), adding
  four tests each for HMAC sign-and-verify (tampered payload and corrupt
  tag both rejected) and for two-hop relay (TTL decrement, exhausted TTL,
  duplicate-clock suppression). `CONFIG_TAPESTRY_WIRE_AUTH_ENABLED` and
  `CONFIG_TAPESTRY_MESH_RELAY` were previously compiled by nothing at all
  — not an untested feature but an unknown-broken one, as the fix below
  shows. Both still default to `n` and remain enabled by no application
- **Compile-only CI build combining `CONFIG_BT=y` with
  `CONFIG_TAPESTRY_WIRE_AUTH_ENABLED=y`** (`bbc_microbit_v2`, `hw-build`
  job) — the auth/relay coverage added above links `gossip.c` against a
  loopback transceiver, never `transceiver_ble.c`, so the `BUILD_ASSERT`
  in `transceiver_ble.c` guarding the 29-byte BLE advertising payload
  (exactly full once the 4-byte auth tag is appended) had never actually
  been evaluated by any build in this repo. This is the one CI build
  where both symbols compile together
- **QoS tiers now do something** — `TAPESTRY_QOS_BEST_EFFORT`/`SOFT_RT`/
  `HARD_RT` were previously accepted by `gossip_send`/`transport_send` and
  discarded; they are now packed onto the wire (see the `relay_qos` change
  below) and acted on in two places: the relay ring buffer evicts the
  lowest-tier queued frame to admit a higher-tier incoming one instead of
  dropping it outright, and `runtime.c` (plus `cf21bl-formation` and
  `webots-formation`'s controller, which run their own main loops instead
  of `runtime.c`) sends a `HARD_RT` frame immediately on the
  `SCR_ABORT_TRIGGERED` quorum-loss edge rather than waiting up to
  `GOSSIP_INTERVAL_MS` for the next scheduled cycle. `BEST_EFFORT` has no
  sender yet — telemetry doesn't travel over gossip — and stays defined
  but unused
- **`tapestry-os/tests/transport/` gains 2 tests** (18 → 20): the QoS tier
  round-trips through `relay_qos` without disturbing `hop_count`, and a
  `HARD_RT` frame evicts a queued `SOFT_RT` one when the relay queue (depth
  8) is full rather than being dropped for capacity
- **Full 6DoF pose tracked for every element** — `element_state_t` gains
  `position.z` (`position_t` extended to x/y/z) and `orientation`
  (`orientation_t`, unit quaternion — mirrors `substrate_quat_t`'s
  layout/convention without CSM depending on the L1 substrate header, the
  same way `position_t` never has). Both are gossiped
  (`tapestry_gossip_frame_t` gains `z`/`qw`/`qx`/`qy`/`qz`) and pass
  through `world_model.c` for free — every function there copies
  `element_state_t` wholesale. `orientation_identity()` (`{1,0,0,0}`) is
  the required default for anything without attitude sensing; a zero
  quaternion is not a valid rotation and would corrupt any consumer that
  assumes unit norm — every `element_state_t`-constructing call site
  across `runtime.c`, both simulators, and all three examples was audited
  and set explicitly, not left to zero-init
- **`examples/webots-formation`'s `substrate_webots.c` gains
  `substrate_webots_get_orientation()`** — converts the InertialUnit's
  already-read roll/pitch/yaw to a unit quaternion (ZYX intrinsic /
  aerospace convention) and feeds real ground-truth orientation into
  gossip, alongside the GPS `z` already available via
  `substrate_webots_get_position()`. The one place this example is more
  complete than `cf21bl-formation`, which has no attitude-estimate
  accessor to draw from — see that example's README
- **`tapestry-os/tests/transport/` gains z/orientation round-trip
  assertions** and the frame-size test now pins 42 bytes
  (`'<BfffffffIIBBBBB'`)
- **`orientation_t`'s reference-frame convention documented** (csm.h) —
  previously unspecified, which matters once orientation is gossiped and
  compared across elements rather than just logged locally.
  `orientation_t` must be expressed in the same world frame as
  `position_t` on that platform (Webots' world frame; the shared
  lighthouse world frame on cf21bl hardware) — never a local/body frame
  or wherever an element happened to be pointed at boot. If a platform's
  world frame is genuinely geographically anchored (a real compass/
  magnetometer reference), that frame must be ENU (+X=East, +Y=North,
  +Z=Up); no current platform has that (Webots is simulation-arbitrary,
  cf21bl's BMI088 has no magnetometer), so this is documented but not yet
  exercised by any code path. `wire.h` and `substrate.h` cross-reference
  the convention rather than restate it. `webots-formation`'s
  `substrate_webots_get_orientation()` satisfies it by construction
  (InertialUnit and GPS share Webots' world frame); `cf21bl-formation`'s
  main.c notes what a future real accessor would need to calibrate
  against (the existing "nose along lighthouse world +X" boot placement)
- **Choreo SDK Design doc stages 1–4 implemented: frames+anchors, motion,
  events/transitions, tracks** — the design doc's own staged rollout, each
  stage independently shippable. **Frames+anchors** (§5): FORM/CONVERGE
  targets can be `frame = "collective"` (live participant centroid) or
  `frame = "element"` (a resolved anchor's position —
  `leader`/`self`/`lowest-energy`/`id:N`, debounced 2 s before switching
  which element is the anchor, the same "stable before acting" lesson
  `QUORUM_UP_MS` encodes) instead of only `frame = "absolute"` (the
  zero-value, compat-preserving default — every script written before this
  existed is unaffected). **Motion** (§6): `form`'s new `spin` parameter
  turns a static shape into a maintained, rotating one; the `orbit` preset
  (`form` + `spin` around an anchor, pure TOML sugar) names the common case
  directly. **Events+transitions** (§8): any step can carry `on = [...]`
  guarded transitions (`achieved`, `element_joined`/`element_lost`,
  `count_gte`/`count_eq`, `anchor_lost`) that redirect to a named step
  instead of only falling through linearly — the "welcome dance" flagship
  demo (§8.3: a 4th element joining redirects the collective to an orbit
  step, leaving redirects back) is now expressible. A cyclic step graph
  requires an explicit top-level `max_runtime` (detected statically at
  parse time, not left to run away in flight). **Tracks** (§7):
  `[[tracks]]` lets different elements run different, concurrent step
  sequences, each choosing the FIRST filter (`requires`/`energy_low`) it
  matches — membership resolved locally from each element's own state, no
  coordination messages (P4). Track membership is gossiped
  (`current_track`, wire v4, `TAPESTRY_WIRE_VERSION` bumped to 4) so peers
  can filter `collect_participants()` to elements actually working the
  same track: capability-based filters can't be re-derived from gossiped
  state alone, so the settled result is gossiped instead — the same
  pattern `achieved` already established, not a new one invented for this.
  Full C/Python/TOML parity throughout (`choreoc.py` emits track tables
  too); `sdk/CHOREO_SCRIPTS.md` documents all four stages
- **Choreo/BSE positions and targets are genuinely 3D throughout, not 2D
  with altitude managed separately** — `tapestry_position_t` (BSE's own
  2D-only type, previously distinct from L4's already-3D `position_t`) is
  gone; `bse.h`/`bse.c`/`choreo.h` use `position_t` directly. No "z=0 means
  unset" special case anywhere: `target` is a required 3-tuple `[x, y, z]`
  at every authoring surface (Python `Goal.target`, TOML, `choreoc.py`'s
  emission — z is treated exactly like y, never conditional), and the
  achievement predicate and DISPERSE's spacing check are true 3D Euclidean
  distance. FORM's shapes (circle/line/grid) and `spin` stay planar (that
  is what those named patterns are) but position at any real z;
  CONVERGE/MOVE/HOLD extend automatically via the unified type. EXCHANGE
  deliberately does NOT touch z — if elements are separated in altitude
  some other way, that stays intact through a swap (only x/y stations are
  reassigned) — but v1.0 intentionally ships no mechanism to stage that
  separation from a script; kept as simple and general as possible rather
  than adding bespoke per-goal staggering syntax. `formation.c`'s
  repulsion (already 3D-aware in its *distance* check) never gains a
  vertical force component here either — deliberately deferred, not
  forgotten
- **`cf21bl-formation` and `webots-formation` real-flight altitude is now
  Choreo-commanded**, not a platform-hardcoded per-ID constant —
  `directive.target.z` drives `alt_cmd_m`, rate-limited toward it
  continuously (not just at takeoff) the same way the takeoff ramp always
  was. `cf21bl-formation/src/main.c`'s automatic `ALT_STEP_PER_ID_M`
  stagger is removed and NOT replaced — elements are no longer separated
  in altitude at all, so `change-partners.choreo.toml`'s `exchange` step
  keeps the default arc path (preserves horizontal separation by
  construction throughout the maneuver) instead of the faster `direct`
  beeline it briefly used, which needs a real deconfliction margin to be
  safe. Speeding the swap back up is deferred to a future pass, not solved
  by reintroducing platform- or script-level altitude staggering. The
  room's own validated lighthouse arrival-angle band (0.30–0.70 m, see
  `ALT_BASE_M`'s comment) remains documented for whenever that follow-up
  happens. Both platforms' geofence checks were re-examined (not left
  stale) and kept horizontal-only. Telemetry (`choreo_telemetry.c`,
  `helpers.py`, `choreo_sim.py`) gains `pos_z`/`directive_target_z`
  columns. 
- **Effects: per-step `indicator`/`telemetry_tag` annotations (Choreo SDK
  Design doc §12 Stage 5)** — `choreo_step_t` gains `indicator`
  (`substrate_signal_t`, reused directly rather than a parallel enum) and
  `telemetry_tag` (`const char *`), both defaulting to "no effect"
  (`SUBSTRATE_SIGNAL_NONE`/`NULL`) so every pre-existing step is
  unaffected. New accessors `choreo_current_indicator()`/
  `choreo_current_telemetry_tag()` (`current_indicator()`/
  `current_telemetry_tag()` in the Python mirror) report the active
  step's declared effect; Choreo itself never calls into L1 — the
  application reads these once per tick and acts on them, the same
  pattern `choreo_get_directive()` → `substrate_move()` already uses.
  TOML authoring: `indicator = "idle"|"active"|"degraded"|"failed"` and
  `telemetry_tag = "..."` are now valid on any goal key (see
  `sdk/CHOREO_SCRIPTS.md`'s new "Effects" section); `choreoc.py` emits
  both as C designated initializers, only when authored. `examples/
  cf21bl-formation/src/formation.c`'s `demo_set_leds()` and the
  previously independently-duplicated identical copy in `examples/
  webots-formation/controllers/common/tracker.c` now take the step's
  declared indicator as an override, falling back to their existing
  quorum/freshness heuristic when unset — byte-identical behavior for
  every script that doesn't opt in. `choreo_telemetry.c`'s CSV capture
  gains `indicator`/`telemetry_tag` columns. `report_telemetry` in the
  architecture paper's Factory Spill scenario implies wire delivery to an
  external monitoring consumer — deliberately NOT built: no such
  consumer exists anywhere in this repo, and `telemetry_tag` is local
  capture only (see `choreo.h`)
- **A `hold` step can now give up on permanent isolation, and a script can
  choose a safe fallback instead of freezing wherever quorum loss struck**
  — closes a real gap: `choreo_tick()`'s `SUSPENDED` case never advanced a
  step's own timer for ANY goal type, so an element that lost quorum and
  never regained it (all peers gone, a permanent partition, out-of-mesh-
  range — and, since OTA install is out of v1.0 scope, with no remote
  recovery path either) would station-keep forever with nothing left to
  ever revive it. A `hold` step's own `max_duration_ms` now keeps counting
  down while suspended (every other goal type's timer stays frozen, as
  before). New `on = [{ event = "quorum_lost", goto = ... }]` transition
  lets a script actively redirect to a safe `hold` instead of passively
  freezing at a transient position (e.g. mid-`exchange`-arc) — checked
  like any other `on[]` entry, before the automatic suspend, so the
  target step is simply the one that ends up suspended that same tick.
  `quorum_degraded`/`quorum_recovered` remain deliberately absent — no
  concrete use case identified for either. The `hold`-timeout-while-
  suspended change applies with no author opt-in required (it's a runtime
  behavior change, not a new field); `cf21bl-formation`'s shipped script
  declares no `quorum_lost` transition and isn't meaningfully affected —
  its `hold` steps' bounds (10s/8s) are far below anything a real
  partition would need to trigger this
- **Quorum-recovery hold moved into L5** (`scr_set_quorum_hold_ms()`) —
  `scr_tick()` can now require a `LOST` -> `>=DEGRADED` recovery to be
  SUSTAINED for a configurable duration before reporting it, instead of
  reporting the very first tick a peer looks fresh again. This generalizes
  a flight-tested (2026-07-19 flight 2) app-level filter that was
  duplicated identically in `cf21bl-formation`'s and `webots-formation`'s
  `main.c`: gating quorum recovery on instantaneous freshness let a
  single lucky gossip packet flicker Choreo awake for a tick, ratcheting
  the tracker's target toward a possibly-corrupt position estimate. Both
  apps now call `scr_set_quorum_hold_ms(&scr, QUORUM_UP_MS)` once at init
  and pass the real `scr` straight to `choreo_tick()`/`choreo_telemetry_
  write()` — no more local filter, no more debounced copy. Quorum LOSS
  remains immediate and unaffected (`SCR_ABORT_TRIGGERED` still fires the
  same tick quorum drops); only the recovery edge is held, and only when
  it follows a real prior LOST period — live fluctuation between
  `DEGRADED` and `HEALTHY` is reported as before. `SCR_ABORT_CLEARED` is
  now delayed by the same hold duration as a direct, intended consequence
  — this is a change to L5's exported abort-signal timing, not just an
  SDK addition, so it gets the same scrutiny as every other quorum-adjacent
  change in this release: verified via a scripted
  40-tick equivalence test asserting the new L5 mechanism reproduces the
  old app-level filter's output tick-for-tick under `quorum_min=
  quorum_target=1` (both apps' real config), plus new `scr_suite` ztests
  covering the hold/no-hold/reset/no-effect-on-in-zone-fluctuation cases,
  run in CI on `native_sim`. Exercised in flight on 2026-08-24 (flight 15):
  the held quorum drove choreo suspension in the air across a ~50 s
  peer-loss stretch
- **`choreo_publish_state()`** — one call replacing the two manual
  assignments (`own_state.goal_achieved = choreo_goal_achieved();
  own_state.current_track = choreo_current_track();`) every app that
  gossips had to remember, duplicated identically at all 5 call sites
  across `runtime.c`, `cf21bl-formation`, `webots-formation`, and
  `tapestry-scr-sim`. Choreo has no handle on `element_state_t` (an L3/L4
  wire struct), so this can't be made fully automatic, only consolidated
  to a single named call — this exact obligation was silently missed
  twice in this repo's history before either mistake was caught (see
  `tapestry-os/tests/transport`'s suite header comment). Pure mechanical
  dedup, no behavior change, migrated all 5 sites

### Changed
- **`tapestry_gossip_frame_t`'s `hop_count` byte repacked into `relay_qos`**
  (`hop_count` bits [1:0], qos tier bits [3:2]) to carry the QoS tier
  without growing the frame — the BLE advertising payload has zero spare
  bytes (see the `BUILD_ASSERT` entry above). `TAPESTRY_WIRE_VERSION`
  bumped 1 → 2: a v1 sender's hop_count values happen to decode under v2
  unchanged (upper bits were always zero), but the version check rejects
  the mismatch anyway rather than rely on that coincidence, since a v2
  sender's qos bits would misparse as an out-of-range v1 hop_count. All
  three Python wire mirrors regenerated (`gen_wire_protocol.py`); the two
  orchestrator `protocol.py` copies hand-gained `pack_relay_qos()`/
  `unpack_relay_qos()` since the generator only derives struct.Struct
  formats, not field-level codecs
- **Gossip authentication ported from the legacy Mbed TLS MD API to PSA
  Crypto** — `hmac4_sign()` now uses `psa_mac_compute()` with
  `PSA_ALG_HMAC(PSA_ALG_SHA_256)`. Wire bytes are unchanged: the tag is
  the same truncated HMAC-SHA256. One behavior change, confined to builds
  with `CONFIG_TAPESTRY_WIRE_AUTH_ENABLED=y`: `tx_frame()` now drops a
  frame whose tag cannot be computed instead of transmitting it with an
  uninitialised tag, since emitting unauthenticated frames would present
  a broken key as ordinary packet loss. With authentication off — the
  default, and every shipping configuration — `tx_frame()` is unchanged
- **README and `SECURITY.md` L3 claims brought in line with what's
  actually implemented** — README's architecture table claimed
  "routing" and "encryption" at L3; the real behavior is a 2-hop
  opportunistic relay flood (no routing table or topology awareness) and
  optional HMAC authentication (no encryption). `SECURITY.md` still
  listed gossip authentication as unimplemented, which stopped being true
  once `CONFIG_TAPESTRY_WIRE_AUTH_ENABLED` landed — it's now described as
  present but off by default and unused by any shipping configuration
- **`tapestry_gossip_frame_t` grows 22 → 42 bytes** (z + orientation, see
  above). `TAPESTRY_WIRE_VERSION` bumped 2 → 3. Every scaled-down fixed-
  point encoding was considered and rejected: even maximally compressed
  (fixed16 x/y/z + fixed16 quaternion) the frame plus the 4-byte auth tag
  does not fit legacy BLE advertising's 29-byte budget, so shrinking
  further wasn't the fix — `TAPESTRY_MAX_MSG_SIZE` (wire.h) was also
  hardcoded to assume the metric frame was always the larger of the two
  message bodies; that stopped being true here and is now computed as an
  actual max rather than a hardcoded assumption
- **`transceiver_ble.c` rewritten for LE Extended Advertising
  (Bluetooth 5.0+, `CONFIG_BT_EXT_ADV`)** — TX moves from
  `bt_le_adv_start`/`bt_le_adv_update_data` to
  `bt_le_ext_adv_create`/`_start`/`_set_data`; RX is unchanged, since
  Zephyr's legacy scan callback already receives extended advertising
  reports once `CONFIG_BT_EXT_ADV` is enabled (verified against
  `zephyr/subsys/bluetooth/controller/Kconfig.ll_sw_split`, which
  unconditionally selects `BT_CTLR_ADV_EXT_SUPPORT` for every Nordic
  nRF5x target). The `BUILD_ASSERT` budget check moves from a hardcoded
  29-byte legacy ceiling to a conservative, explicitly-not-precisely-
  verified single-PDU threshold (200 bytes) — this repo has no hardware
  on hand to confirm the real per-PDU limit, and chaining across multiple
  PDUs was deliberately not implemented (real added complexity this frame
  doesn't need). `bbc_microbit_v2.conf` (both copies —
  `tapestry-scr-hw` and `examples/cutebot-formation`) gains
  `CONFIG_BT_EXT_ADV=y`
- **BLE gossip dropped from `esp_wrover_kit_esp32_procpu.conf`** — the
  original ESP32 (ESP32-D0WD-V3)'s Bluetooth controller is 4.2-only;
  Espressif's own vendored HAL declares `SOC_BLE_50_SUPPORTED` for
  esp32c2/c3/c5/c6/h2/c61/s3 but not the plain esp32. Rather than keep
  that board on legacy advertising with a smaller, board-conditional
  frame — breaking the "one wire format, substrate-agnostic" invariant —
  its BLE leg is dropped; it keeps gossiping over WiFi/UDP, its other
  configured transport, so this costs redundancy, not gossip capability
- **`formation.c`'s peer-separation distance (`demo_compute_drive`,
  `demo_choreo_track`) becomes 3D** — an explicit choice, not a default:
  the spring field, `DEMO_MIN_SEP_M`/`EMERGENCY_K` emergency repulsion,
  and `wm_check_collisions`/`wm_nearest_elements` (via `position_distance`
  in csm.h) all now fold in z. The force VECTOR stays 2D (altitude is a
  separately-held control loop, `CONFIG_CF21BL_ALTITUDE_HOLD`, untouched
  by this change) — normalizing the force direction by the 3D distance
  instead of the horizontal-only one would have silently shrunk the
  horizontal push whenever z differs, so distance and direction are
  computed separately (`dist` vs `dist_xy`). `GEOFENCE_RADIUS_M`'s
  origin-distance check was deliberately left horizontal-only in both
  `cf21bl-formation` and `webots-formation` — folding a near-constant
  per-drone altitude into a radius check shrinks the effective margin for
  no safety benefit, and this specific case was never confirmed the way
  peer-separation math was. **This has not been flight-tested** — see
  `examples/cf21bl-formation/README.md`'s "Known limitations"

### Fixed
- **Lighthouse position validity was a latch, and a blinded drone flew away**
  (`cf21bl_lighthouse.c`). `g_pos_valid` was set `true` on the first fix and
  never cleared, so `cf21bl_lighthouse_is_valid()` reported "valid" forever
  regardless of the estimate's age. Occlude the base stations mid-flight and
  the position simply FROZE while still reporting good. Validity is now
  time-based (`LH2_POS_STALE_MS`, 400 ms = 2x `LH2_BLOCK_FRESH_MS`): a stale
  fix is a failure, not a value. `main.c`'s existing fix-loss handling was
  always correct — it had simply never been reachable in flight.
- **`cf21bl_stabilizer.c` could fly on an uninitialized position.** It asked
  `is_valid()` and then called `get_position()` without checking the return,
  leaving `lhpos` untouched on failure. Harmless while validity was a latch
  (the two could never disagree); a live hazard the moment it became
  time-based. Now reads once, zero-initialized, and branches on the result.
- **The nRF51 syslink handshake was fire-and-forget and lost on cold boot.**
  `SYSLINK_RADIO_CHANNEL` is not just the channel — the nRF51 sends no
  syslink packet until it first receives one, so it is the activation trigger
  for the entire P2P link. On a cold power-up the STM32 reached
  `syslink_init()` while the nRF51 was still booting and the message was
  dropped, with no retransmit at that layer. Invisible until now because
  `CONFIG_TAPESTRY_P2P_CHANNEL` and the nRF51's own `DEFAULT_RADIO_CHANNEL`
  are both 80, so a lost message looks exactly like a working one — except
  the link never activates. The nRF51 echoes both handshake messages once
  applied, so they are now resent every 250 ms until confirmed (~6 s, then
  `LOG_ERR`) rather than sent once and hoped for.
- **Recorded telemetry omitted the gossiped `achieved` bit** —
  `choreo_telemetry.c` wrote six of the seven `wm_entries` fields the
  Python engine reads, leaving out `achieved`. Replaying a capture of the
  shipped `change-partners` script therefore saw every peer as
  never-achieved, so its `scope = "all"` exchange step replayed on its
  timeout rather than on achievement and reported 81 divergences that
  were the recorder's fault, not the engine's. `choreo_sim --replay` now
  also warns when a recording predates the field. This was the last
  unchecked hop in a chain whose other end was fixed in 0.9.0: the bit is
  computed at L6, published by the application, carried on the wire,
  aggregated by L7 — and every hop had been inspected on its own

## [0.9.0] — 2026-08-16

### Added
- **Real L5 SCR wired into `examples/cf21bl-formation` and
  `examples/webots-formation`'s cf21bl controller** — both now call
  `scr_init()`/`scr_tick()`, so role, `task_slot`, leader election and the
  abort protocol are computed from the actual world model instead of being
  synthesized from raw L4 freshness. Note that `SUSPENDED` is additionally
  gated by an application-level debounce: both apps overwrite
  `scr_state_t::quorum_state` after `scr_tick()` with a sustained-freshness 
  signal (`QUORUM_UP_MS`, 2 s) before handing it to `choreo_tick()`. At the 
  thresholds both apps configure (`quorum_min` = `quorum_target` = 1) this 
  yields the same HEALTHY/LOST verdict L5 computes, plus a 2 s confirmation 
  before an up-transition is believed — quorum loss is still immediate. The
   debounce is applied to a copy handed to `choreo_tick()`, never written back
   into the live `scr_state_t`, so L5's own accessors stay mutually consistent.
   Hysteresis on quorum acquisition belongs inside L5 rather than being 
  repeated per application — deciding whether the collective has quorum is 
  L5's responsibility, and an obligation carried by every element main loop is 
  one every new element main loop can forget (TODO). Enforced real-world 
  geofence and mission-duration landing backstops.
- **Collective achievement (`scope = "all"`)** — each element now gossips
  an `achieved` bit every cycle (`tapestry_gossip_frame_t` gains an
  `achieved` field, a wire-compatible append — no `TAPESTRY_WIRE_VERSION`
  bump), aggregated via `choreo_collective_achieved()`. `choreo_step_t`
  and Choreo-script TOML gain `scope = "self"|"all"` (default `"self"`,
  existing scripts unaffected). `examples/cf21bl-formation`'s exchange
  step now uses `scope = "all"`, replacing the bow step's former role as
  an implicit per-drone-achievement workaround
- ztest coverage (native_sim, `examples/cf21bl-formation/tests`) for
  `FORM` shapes, `MOVE` translation, and `scope = "all"` collective
  achievement (including solo/vacuous-truth and stale-peer cases)
- **Webots simulation pattern (`examples/webots-formation/`)** — compiles
  the real, unmodified L3-L7 stack (`world_model.c`/`scr.c`/`bse.c`/
  `choreo.c`/`gossip.c`) into a plain host C Webots controller against
  simulated physics: a hardware-in-the-loop bridge. Structured as a
  reusable pattern rather than a single-robot demo: `controllers/common/`
  holds everything substrate-agnostic (POSIX UDP transceiver, Zephyr
  header shim, target tracker)
- **Offline capture/replay harness for the Choreo engine** — 
  `sdk/tools/choreo_sim.py
  --simulate` (renamed from `choreo_replay.py`, which is now its
  `--replay` mode; both share one tick loop and plot output) drives N
  in-process `Choreo` objects through a `.choreo.toml` with no C, Zephyr,
  or network involved. For someone editing a script who has no build 
  toolchain or Webots set up yet, or is designing for a substrate 
  that doesn't exist as code
- **`choreoc.py --check`, enforced in CI** — generated `choreo_script.h`
  headers are committed so firmware builds never need Python, which means
  a script edit that is not recompiled ships silently. `--check` writes
  nothing and exits non-zero if a committed header does not match what its
  script generates today, the same guarantee
  `gen_wire_protocol.py --check` already gave the wire mirrors. With no
  arguments it discovers every generated header in the repository and
  recovers each one's source from the regenerate command line in its own
  banner, so an added consumer is covered without editing CI — the drift
  it is meant to catch happened precisely because a second consumer was
  added and nothing knew to keep it in sync

### Changed
- **L6/L7 interface prepared for a queueing implementation** — no behavior
  change; three adjustments so a prioritised-goal-queue BSE can drop in
  behind the same headers without a breaking release. (1)
  `bse_submit_intent()` documented that the *displaced* intent's fate is
  implementation-defined; it previously promised the achievement predicate
  and every activation capture would be reset, which a preempting
  implementation must not do. The reference implementation still discards,
  as before. (2) The state an intent accumulates while active — achievement
  progress, HOLD station, EXCHANGE snapshot and arc progress, MOVE centroid
  offset — is now grouped as `bse_activation_t` in `bse.c` rather than
  fourteen loose statics, since that is exactly what a preempting engine has
  to stack; tick-scoped values are deliberately left outside it. (3)
  `choreo_goal_t` and `tapestry_bse_intent_t` gain a caller-assigned
  `id` (0 = anonymous, opaque to Tapestry, unread by this implementation),
  so per-goal cancel and preemption reporting can be added later without
  changing `choreo_submit_goal()`'s return convention — every one of its
  call sites tests it against 0 today. Also documented that a preempted
  goal reports as `SUSPENDED` rather than a new lifecycle state, so
  `choreo_state_t` need never grow a member and break existing switches
- **Status banners rewritten across `sdk/README.md`, `choreo.h`, `bse.h`,
  `bse.c`, `choreo.c` and their Python mirrors** — feature ready for v1.0 release. They now itemize the feature scope: what L6/L7
  implement today, and which capabilities (physics planner, ML runtime,
  goal queue with preemption and arbitration, hi-fi simulation bridge,
  monitor telemetry export) are deliberately out of scope rather than
  merely unfinished

### Fixed
- **`FORM` shape** — `shape = "line"` and `"grid"` were accepted and
  silently rendered a circle regardless; both now produce the requested
  layout (`bse.c`/`bse.py`)
- **`MOVE`** — previously identical to `CONVERGE` (collapsed every
  element onto the same point); now offset-preserving formation
  translation with each element keeping its position relative to 
  the formation centroid. A solo element still degenerates to
  `CONVERGE`, correctly, since there is no formation to preserve
- **`tapestry-scr-hw`'s movement loop** — `substrate_move()` is now
  directive-driven reading `choreo_get_directive()`
- **`examples/webots-formation` ran a stale Choreo script** — its generated
  `choreo_script.h` predated collective achievement and was missing
  `scope = "all"` on the exchange step, so the simulation did not run the
  same script as the hardware demo despite the README saying it did.
  Regenerated from `change-partners.choreo.toml`
- **Out-of-range element ID corrupted memory in L4/L5** — `wm_init()` and
  `wm_update_self()` indexed `world_model_t::entries[]` with an unvalidated
  `owner_id` (an out-of-bounds *write* for any ID ≥ `MAX_ELEMENTS`),
  `wm_check_collisions()` read the same way, and `scr_tick()` could write
  one entry past its `candidates[]` array because its peer loop skips only
  `own_id`. Peer IDs arriving over the wire were already validated; the
  owner's ID, which comes from the application, was not. It is now checked
  once in `wm_init()` — an out-of-range owner leaves the model inert rather
  than corrupting adjacent memory — and `scr_tick()`'s array is sized for
  the worst case. Reachable through a hand-configured element ID or an
  `ELEMENT_ID_INVALID` propagated from a failed identity negotiation
- **`transport.h` contradicted `wire.h` on QoS** — `transport_send()`'s
  documentation claimed `qos_tier` was "embedded in the gossip frame so
  peers can prioritize accordingly". It is not: `gossip_send()` ignores the
  parameter, and `wire.h` correctly documents the tiers as reserved
  placeholders. `transport.h` now says the same

## [0.8.0] — 2026-08-01

### Added
- **L6/L7: first flight-ready Choreo** — coordinate-free `HOLD` (station
  captured at activation) and `EXCHANGE` (snapshot stations, rotate by
  `slot_shift` around the ID-sorted ring, commanded target travels a CCW
  arc about the snapshot centroid so mutual separation is preserved by
  construction) goals, plus a minimal feedback controller (achievement
  predicate). L7 gains linear goal scripts (`choreo_submit_script`) with
  per-step timeout/advance-on-achieved and quiescence on completion
  (`IDLE` directive — platforms map it to their own inactive posture;
  "take off"/"land" never appear at L7). `SUSPENDED` now freezes the BSE
  and script timers, except `HOLD`, which keeps ticking since it
  references no peer
- **Choreo scripts authored in TOML** — `sdk/tools/choreoc.py` compiles a
  `.toml` script to a committed C header (stdlib-only, no dependencies);
  the Python SDK loads the identical file directly via
  `tapestry/script_toml.py`. New `sdk/CHOREO_SCRIPTS.md` authoring guide.
  Script files follow an `<name>.choreo.toml` naming convention
- **`examples/cf21bl-formation` Choreo mode** (new default) — one binary
  for every drone (IDs negotiated at boot over syslink P2P) running a
  hold → exchange → bow script through the full safety envelope
  (min-separation repulsion, target leash, arena clamp); script
  completion lands in place. The original spring-field showcase is
  preserved behind `DEMO_MODE_SHOWCASE`
- **Medium-agnostic auto-ID** — discovery beacons are now plain gossip
  frames over any transceiver, not BLE-only; new diagnostics
  (per-transceiver drop counters, live progress log, duplicate-ID frame
  counter) and self-healing (a drone hearing no peers grounds and
  retries instead of arming solo)
- **Wire schema version** — a one-byte `TAPESTRY_WIRE_VERSION` in every
  L3 message header, and in the gossip frame itself so BLE and syslink
  P2P (which carry it with no header wrapper at all) are covered too; a
  frame from a mismatched schema is rejected and logged at every layer
  that carries it, instead of silently misinterpreted
- **`tapestry-os/tools/gen_wire_protocol.py`** — generates the three
  previously hand-maintained Python wire-protocol mirrors
  (`tapestry-csm-sim`, `tapestry-scr-sim`, `tapestry-scr-hw`) directly
  from `wire.h`; CI now fails if a mirror drifts from source
- Choreo script ztest suite (native_sim) covering the hold → exchange →
  bow script end to end

### Changed
- Choreo-mode gossip cadence raised to 5 Hz (was 2 Hz) — the old cadence
  let peer entries go stale in roughly half of all windows; landed
  elements now keep gossiping in Choreo mode so a finisher's departure
  can't strip its still-flying partner's only peer
- Auto-ID window now opens after a 2.5 s radio settle delay, with
  discovery beacons at 150 ms during the window (from 2 Hz) to
  compensate for ~20–35% measured syslink P2P delivery under load
- TOML script validation tightened: `hold` rejects `until`/`eps`/`settle`
  (trivially achieved — would advance on the first tick), `form`
  requires a `radius` (radius 0 sent every element to the same vertex),
  `move` warns that it currently behaves like `converge`; duration and
  length units gained `"min"`, `"h"`, and `"um"`

### Fixed
- Landing touchdown could cut motors mid-air when the walked-down target
  reached zero before the airframe had actually descended
- A two-drone script's only "member departs" path (landing) could
  silently strip a still-flying partner's last peer, suspending its
  Choreo and freezing `EXCHANGE`'s step timeout — recoverable only by
  the outer mission-duration backstop

## [0.7.0] — 2026-07-14

### Added
- **Crazyflie 2.1 Brushless board support** (`crazyflie21bl`) — full Zephyr
  bring-up: ESC control (RC PWM 400 Hz and OneShot125 with BLHeli_S
  auto-detect), BMI088 IMU at 1 kHz with Mahony attitude filter, BMP390
  barometric altitude hold (cascaded position→velocity→thrust), watchdog,
  CRTP radio console, and battery monitoring/compensation with forced
  landing via nRF51 syslink
- **Lighthouse V2 positioning driver** (`cf21bl_lighthouse.c`) — SteamVR
  2.0 base-station triangulation from the Bitcraze Lighthouse deck: sweep
  pairing, OOTX sweep calibration, optical-FOV angle gating,
  miss-distance and speed-limit outlier rejection, and long-baseline
  velocity estimation
- **Lighthouse XY position hold** in the stabilizer — P+I+D with a
  rate-clamped body-frame level-trim integrator and yaw heading hold
- **Syslink P2P gossip transport** — drone-to-drone L4 gossip over the
  nRF51's 2.4 GHz radio, no co-processor reflash required
- **cf21bl-formation demo** — three drones self-organize (line → rotating
  triangle → member departs → line) with no leader, phase timers, or
  uplink: the formation is a pure function of fresh-peer count.  Includes
  per-drone staging (late join / early departure), centroid anchor,
  target leash, and a per-drone safety layer (fix-loss, geofence,
  mission-duration, land-in-place, gossip-silence after landing)
- **Formation-field unit tests** (ztest, native_sim) — regression tests
  for the flight-found spring-field bugs — plus CI coverage: both ztest
  suites run on native_sim and the demo + runtime element compile-check
  for `crazyflie21bl`
- **Shared lighthouse calibration** (`examples/lighthouse_cal.h`/`.yaml`)
  — single in-code copy of base-station geometry consumed by every
  position-aware example
- **`patches/`** — two Zephyr I2C v1 RTIO driver fixes required by the
  BMI088 on this board (pending upstream submission; reapply after
  `west update`)

### Changed
- **`examples/collective-formation` renamed to `examples/cutebot-formation`**
  (including the BLE advertised name) — the generic name now belongs to no
  single platform; the Crazyflie counterpart is `examples/cf21bl-formation`
- Retired superseded bring-up examples (`altitude-hold-test`, `lh2-hover`);
  their roles are covered by `altitude-hold-tether`, `lighthouse-test`, and
  `cf21bl-formation`
- Normalized all documentation and comments to American English spellings

### Fixed
- **Lighthouse position-hold sign inversion** — the XY correction was added
  to the angle setpoint unnegated (positive feedback bounded only by
  saturation); explains a long history of "under-damped orbit" that gain
  tuning never resolved
- **Stale elevation gate after base-station recalibration** — replaced
  room-specific empirical angle bounds with the LH2 optical FOV (a hardware
  constant), ending gate-induced single-station dropouts
- **Formation target detachment** — virtual targets could decouple
  unboundedly from their drones under emergency repulsion; now leashed to
  the stabilizer's error-saturation radius
- **Altitude-ramp integrator windup** — higher-cruise drones overshot and
  kept climbing after long takeoff ramps

## [0.6.1] — 2026-06-08

### Fixed
- **L4 sim element** — fixed uninitialized `comms.shutdown` flag in
  `comms_init`; could cause the element's main loop to exit immediately
  on startup if stack memory happened to be non-zero

## [0.6.0] — 2026-05-21

### Added
- **L2 Element Runtime** — added power state machine, main-loop ownership,
  devicetree energy harvester integration
- **L6 Behavior Synthesis Engine** (C) and **L7 SDK** (Python) — initial
  developer-facing API surface; wired into hardware and sim elements
- **L5 SCR** — roles, task slots, abort protocol, and BFT anomaly filtering
- **HMAC-SHA256 frame authentication** — optional per-deployment shared key,
  4-byte truncated tag appended to gossip frames
- **QoS tiers** and **energy/health gossip fields** in the wire format
- **Two-hop opportunistic relay** (BATMAN-inspired mesh) behind
  `CONFIG_TAPESTRY_MESH_RELAY`; prevents broadcast storms via hop_count cap
- **Auto-ID** — boot-time nonce exchange eliminates per-robot build flags;
  one binary flashes to all elements in a deployment
- **Collective-formation demo** — 4 Cutebot Mini robots self-organize into
  a regular formation using L4 gossip; validated on hardware

### Changed
- **L7 layer renamed** to "Choreographer" (`choreo`); lifecycle states and
  per-goal capability permissions added
- **L1 layer** refactored from actuation HAL to Physical Substrate Interface (PSI)
- **Transport subsystem** flattened into `tapestry-os/subsys/transport/` with
  a transceiver vtable
- Sim and runtime main-loop cadence improved

### Fixed
- Three architectural misattributions in comments and docs corrected
  to match the architecture paper

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

[Unreleased]: https://github.com/tapestry-os/tapestry/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/tapestry-os/tapestry/compare/v0.9.0...v1.0.0
[0.9.0]: https://github.com/tapestry-os/tapestry/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/tapestry-os/tapestry/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/tapestry-os/tapestry/compare/v0.6.1...v0.7.0
[0.6.1]: https://github.com/tapestry-os/tapestry/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/tapestry-os/tapestry/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/tapestry-os/tapestry/releases/tag/v0.5.0
