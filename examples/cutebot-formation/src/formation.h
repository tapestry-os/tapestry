/*
 * formation.h — Demo: Collective Formation (Choreo-driven ground rover)
 *
 * Differential-drive go-to-point control (demo_track_target(), driven by
 * the L6/L7 Choreo directive in main.c) over the L4 world model, with
 * dead-reckoning odometry to keep own_state.position current.
 *
 * ARENA SCALE — read this before changing any spatial constant.
 *   Every distance below is in logical units, and the unit is defined by
 *   DEMO_ARENA_MM alone (see "Arena scale" section).  Change the arena and
 *   you change the unit, which silently re-scales speed, wheel track,
 *   spacing and separation together.  They are therefore all DERIVED from
 *   DEMO_ARENA_MM via DEMO_MM() rather than written as literals — do not
 *   reintroduce a hardcoded unit count without re-deriving it here.
 *
 * Physical calibration:
 *   DEMO_MAX_SPEED — effective linearisation constant, NOT true 100% speed.
 *     The motor curve is non-linear: at 22% commanded speed Cutebots already
 *     do ~32% of max velocity.  Calibrate at the actual commanded speed (22%)
 *     so dead-reckoning is correct in practice.  Expressed physically:
 *       DEMO_MAX_SPEED = (speed_mm_per_s_at_22pct / 0.22) in units/s
 *     Measured fleet average 238 mm/s at 22% → 1081.8 mm/s at 100%.
 *     On the 1266.25 mm chessboard (1 unit = 12.6625 mm) that is 85.4.
 *     Per-robot values, rescaled from the 800 mm arena they were measured
 *     in (compile with -DDEMO_MAX_SPEED=N):
 *       Bot 0: 75.8   Bot 1: 86.6   Bot 2: 91.6   Bot 3: 87.8
 *       (were 120 / 137 / 145 / 139 at 8 mm per unit)
 *
 *   DEMO_WHEEL_TRACK — wheel-center-to-wheel-center in logical units.
 *     Cutebot Mini track ≈ 85 mm → 6.71 units here (was 10.6 at 8 mm/unit).
 *     This one feeds heading integration: a stale value corrupts dead
 *     reckoning itself and presents as drift, not as a bad constant.
 *
 * Formation tuning:
 *   DEMO_TARGET_SPACING — normalizes demo_force_to_twist()'s lateral-
 *     force-to-turn-rate gain (formation.c). Choreo takes its actual
 *     formation geometry from the .choreo.toml script, not this constant
 *     — it survives only as a turn-gain scale factor shared by every
 *     drive function (demo_track_target, demo_drive_straight).
 *
 * Override any constant at compile time:
 *   west build ... -- -DDEMO_TARGET_SPACING=40.0f
 */

#ifndef TAPESTRY_DEMO_FORMATION_H
#define TAPESTRY_DEMO_FORMATION_H

#include <stdint.h>
#include <tapestry/csm.h>
#include <tapestry/substrate.h>

/* ── Arena scale ────────────────────────────────────────────────────────── */
/*
 * Physical arena → logical [0, WORLD_SIZE] mapping.  ONE value to change
 * when the arena changes; every spatial constant below derives from it.
 *
 * Chessboard measured 2026-09-08: the 8 squares of a row span 1013 mm, so
 * one square is 126.625 mm, and the plain border outside the checkered
 * area is about one square wide on every side.  Counting that border —
 * usable surface, and the room the robots need to turn around in — the
 * board is 10 squares across:
 *
 *     DEMO_ARENA_MM     = 1013 + 2 x 126.625  = 1266.25 mm
 *     DEMO_MM_PER_UNIT  = 1266.25 / 100       = 12.6625 mm
 *     one square        = 126.625 / 12.6625   = 10.000 units exactly
 *
 * So the checkered 8x8 occupies units [10, 90], the border ring occupies
 * [0, 10] and [90, 100], and WORLD_SIZE coincides with the physical edge
 * of the board — which is what makes csm.h's position_clamp() usable as an
 * arena fence directly, should a caller want one (nothing calls it today).
 *
 * The 10-units-per-square result is a coincidence of this board's
 * proportions, not something to rely on: re-measure and re-derive if the
 * surface changes.
 */
#ifndef DEMO_ARENA_MM
#define DEMO_ARENA_MM      1266.25f   /* 8 checkered squares + border ring */
#endif

#define DEMO_MM_PER_UNIT   (DEMO_ARENA_MM / WORLD_SIZE)

/* Millimetres → logical units.  Also converts mm/s → units/s unchanged. */
#define DEMO_MM(mm)        ((mm) / DEMO_MM_PER_UNIT)

#define DEMO_SQUARE_UNITS  DEMO_MM(126.625f)  /* one chessboard square, 10.0 */

/*
 * DEMO_ARENA_FENCE_MARGIN — command-level containment.
 *
 * Not the same thing as the [0, WORLD_SIZE] clamp already inside
 * demo_odometry_update(): that clamp only pins the STORED position
 * estimate so gossiped numbers stay sane — it never touches speed_cmd/
 * rate_cmd, so once the true physical robot is beyond where its own
 * drifting estimate believes it is, the estimate stops advancing while
 * the commanded force keeps pointing outward (attraction toward a target
 * the frozen estimate can never reach), and the real robot just keeps
 * driving off the board. That gap is real on this codebase today, not
 * hypothetical — it is the likely mechanism behind a robot that looked
 * fine early and then drifted straight out over a long run.
 *
 * This margin is read by demo_arena_fence() (formation.c) and vetoes the
 * OUTWARD component of the commanded force once the estimate is within
 * it of an edge, so the robot stops driving further out before its
 * estimate ever pins at the wall. It is a fence on the estimate, not on
 * the physical robot — if the estimate has already drifted relative to
 * the real board (there is no grid-based correction yet), the robot can
 * still be physically outside the true chessboard while this fence
 * believes it is comfortably inside. It bounds how far an uncorrected
 * run can run away; it does not make the position estimate correct.
 */
#ifndef DEMO_ARENA_FENCE_MARGIN
#define DEMO_ARENA_FENCE_MARGIN DEMO_MM(80.0f)   /* 6.3 units */
#endif

/* ── Calibration defaults ───────────────────────────────────────────────── */

#ifndef DEMO_WHEEL_TRACK
#define DEMO_WHEEL_TRACK   DEMO_MM(85.0f)   /* Cutebot Mini track 85 mm → 6.71 */
#endif

#ifndef DEMO_MAX_SPEED
#define DEMO_MAX_SPEED     DEMO_MM(238.0f / 0.22f)  /* 1081.8 mm/s → 85.4 units/s
                                     * Fleet average 238 mm/s measured at 22%
                                     * commanded; see the header comment for
                                     * per-robot overrides. */
#endif

/* Maximum yaw rate (rad/s) at rate_norm=1.0.
 * Although DEMO_MAX_SPEED is a low-speed linearisation constant (not true 100%
 * speed), using it here works because the motor non-linearity at 22% raises
 * effective speed by the same factor (~1.43×), so DEMO_MAX_OMEGA correctly
 * predicts physical omega for both in-place and arc turns at stiction speed.
 * Not overridable: change DEMO_MAX_SPEED or DEMO_WHEEL_TRACK instead. */
#define DEMO_MAX_OMEGA    (2.0f * DEMO_MAX_SPEED / DEMO_WHEEL_TRACK)

#ifndef DEMO_TARGET_SPACING
#define DEMO_TARGET_SPACING DEMO_MM(500.0f)  /* 39.5 units — see doc above; only
                                     * used as demo_force_to_twist's turn-gain
                                     * denominator now, not a real spacing target. */
#endif

/* Boot-time placement radius about arena centre — see compute_start_pos()
 * in main.c.  One chessboard square out from the middle, ALONG EACH
 * ROBOT'S OWN HEADING AXIS (id 0/1/2/3 -> east/north/west/south, not
 * diagonal — compute_start_pos's x/y are exactly DEMO_START_RADIUS *
 * cos/sin(heading), a pure cardinal offset).  With WORLD center (50,50)
 * itself landing on a grid intersection (8 squares is even, so the
 * board's true center is a line-crossing, not inside a square) and one
 * square exactly 10 units, every seed position also lands exactly on a
 * grid intersection — the corner where four squares meet, one square out
 * from center — not centered in a square and not on a diagonal corner.
 * 4 robots at this radius sit 179 mm apart centre-to-centre, which clears
 * the ~100 mm Cutebot width; the previous 3.0-unit literal was 38 mm here
 * and would have had the robots physically overlapping at boot. */
#ifndef DEMO_START_RADIUS
#define DEMO_START_RADIUS   DEMO_SQUARE_UNITS      /* 10.0 units = 126.6 mm */
#endif

/* DEMO_MODE_STRAIGLINE only (see Kconfig) — fixed seed for a single
 * robot with no auto-ID, no peers to derive a position/heading from.
 * Default matches id=0's normal seed exactly: one square east of
 * center, heading 0 (see README's placement table) — "(1,0)" in
 * square-count terms. Override at compile time to test a different
 * fixed placement, e.g. -DDEMO_TEST_START_HEADING=1.5707963f for id=1's
 * position/heading instead. */
#ifndef DEMO_TEST_START_X
#define DEMO_TEST_START_X       60.0f
#endif
#ifndef DEMO_TEST_START_Y
#define DEMO_TEST_START_Y       50.0f
#endif
#ifndef DEMO_TEST_START_HEADING
#define DEMO_TEST_START_HEADING 0.0f
#endif

/* ── Choreo target-tracking tuning ───────────────────────────────────────
 * demo_track_target() drives toward a single L6/L7-commanded point — see
 * that function's doc. */

/* Attraction "force" magnitude commanded at full range (saturates
 * demo_force_to_twist's speed clamp — see FORCE_TO_SPEED in formation.c:
 * need force * FORCE_TO_SPEED >= 22 to reach demo_force_to_twist's
 * max_speed cap, so 40 clears it with margin). */
#define DEMO_TRACK_MAX_FORCE   40.0f

/* Inside this range of the target, attraction force ramps down linearly
 * to 0 instead of holding DEMO_TRACK_MAX_FORCE — a trapezoidal approach
 * profile so the commanded speed decelerates on final approach instead of
 * commanding full force right up to DEMO_TRACK_ARRIVE_EPS and relying on
 * the arrival snap alone. */
#define DEMO_TRACK_SLOW_RADIUS DEMO_MM(120.0f)   /* 9.5 units */

/* Inside this distance of the target, command zero motion outright.
 * Without an arrival snap, the residual attraction force there is too
 * small to clear demo_force_to_twist's MIN_STICTION floor on its own, so
 * the floor would keep forcing a nonzero speed command and the robot
 * would creep/oscillate around the target indefinitely instead of
 * settling — a distance gate rather than a force-magnitude hysteresis
 * band, since attraction force here is monotonic in distance (no sign to
 * hystrese around).
 *
 * 2.0 units (~6.7% of ring.choreo.toml's 30-unit radius) — briefly
 * relaxed to 6.0 (20%) on 2026-09-11 to chase a "settle, then correct a
 * little" hunting symptom, then reverted here the same day once the REAL
 * cause was found and fixed instead: bse.c recomputes each element's FORM
 * vertex from live rank/count every tick regardless of tolerance, so a
 * single dropped/delayed gossip frame nudges the target and no amount of
 * dead-band hides that — see ring.choreo.toml's "settled" HOLD step
 * (added the same day), which actually stops the live recompute instead.
 * With that fixed, the tight original value is safe again and back to
 * matching CONFIG_TAPESTRY_QUORUM comments-adjacent precision elsewhere.
 * Tune by adjusting the literal below — expressed as a flat unit count,
 * not a live percentage of whatever radius a given script happens to
 * use, so changing it here affects every demo_track_target() caller
 * uniformly.
 *
 * MUST stay smaller than the .choreo.toml script's own achieve_eps (see
 * ring.choreo.toml, kept at 3.0 for exactly this reason) — otherwise the
 * robot can stop (zero motion, this gate) farther from center than
 * achieve_eps requires, with no residual force left to close the
 * remaining gap, so the L6/L7 achievement predicate would never fire. */
#define DEMO_TRACK_ARRIVE_EPS  DEMO_MM(25.0f)    /* 2.0 units (~6.7% of 30) */

/* Emergency repulsion backstop, mirroring cf21bl-formation's
 * demo_choreo_track(): FORM's own vertex spacing is the primary
 * deconfliction; this only guards the transient approach if two robots'
 * paths cross closer than intended.  DEMO_TRACK_MIN_SEP must stay below
 * that spacing (or it fights the formation at equilibrium) and above a
 * physical collision (~100 mm robot width).
 *   ring.choreo.toml, radius 30: 4 robots sit 2·30·sin(45°) = 42.4 units
 *   apart, 3 robots 51.9 — both comfortably above the 25.3 below.
 *   form-grid.choreo.toml, radius 50: cell spacing 50 units.
 *
 * The original (12.6, 3.0) pair was under-strength for a genuine
 * head-on encounter — verified via a standalone two-robot host
 * simulation this session (DEMO_MODE_CONVERGE_TEST swap-test collision):
 * repulsion's own ceiling, (DEMO_TRACK_MIN_SEP - 0) * K = 37.8, could
 * never exceed DEMO_TRACK_MAX_FORCE = 40, so net force never actually
 * reversed — it only ever DAMPENED the closing speed, letting the
 * (collision-unaware) position ESTIMATE sail through the peer's
 * estimated position while the real chassis physically jammed. Simply
 * raising the ceiling past 40 creates a DIFFERENT failure instead: any
 * nonzero residual force still gets floored up to full MIN_STICTION
 * speed (demo_force_to_twist has no way to command "a little"), so a
 * near-exact balance point oscillates between full attraction and full
 * repulsion every tick forever, never settling — hence
 * DEMO_TRACK_REPEL_DEADBAND below, which converts "roughly balanced"
 * into an explicit hold instead of letting stiction amplify the noise.
 *
 * Tested outcome with (25.3, 4.0, deadband 5.0): a robot pair on a
 * DIRECT head-on approach (demo_track_target() beeline, not a real
 * Choreo EXCHANGE arc) settles at a safe ~206 mm separation and stays
 * there — no collision, no oscillation, but no completed pass-by
 * either. That is expected, not a bug: a scalar radial force opposing a
 * perfectly collinear approach has no lateral component to route around
 * the peer with (confirmed unchanged even with a small lateral start
 * offset). The real system avoids exactly this scenario by construction
 * — EXCHANGE goals travel an arc about the formation centroid
 * specifically so two swapping elements' paths are never collinear
 * (see bse.h) — this backstop's job is catching incidental close
 * encounters during some OTHER maneuver, not resolving a planned
 * head-on swap by itself. */
#define DEMO_TRACK_MIN_SEP     DEMO_MM(320.0f)   /* 25.3 units */
#define DEMO_TRACK_EMERGENCY_K 4.0f

/* See DEMO_TRACK_MIN_SEP's doc above for why this exists: once
 * repulsion is strong enough to genuinely oppose attraction, a
 * near-exact balance point would otherwise flip sign every tick and
 * get re-floored to full MIN_STICTION speed each time — chatter
 * forever, verified via host simulation. When repulsion was applied
 * this call AND the resulting net force magnitude is below this, hold
 * (zero output) instead of feeding a near-zero residual through
 * demo_force_to_twist. Same reasoning as DEMO_TRACK_ARRIVE_EPS's
 * distance gate just above, applied to a force magnitude instead of a
 * distance because there is no single "arrived" point here — the
 * balance point moves with wherever the peer currently is. */
#define DEMO_TRACK_REPEL_DEADBAND 5.0f

/* ── Dead-reckoning state ───────────────────────────────────────────────── */

typedef struct {
    float x;        /* Current position in logical world coords [0, WORLD_SIZE] */
    float y;
    float heading;  /* Radians, 0 = +x direction */
} demo_odometry_t;

/* Initialize odometry at (x, y) with heading 0 (+x direction). */
void demo_odometry_init(demo_odometry_t *odo, float x, float y);

/*
 * Update dead-reckoning estimate from the last motion command.
 *   speed_norm: forward velocity [-1.0, 1.0], passed to substrate_move().
 *   rate_norm:  yaw rate         [-1.0, 1.0], positive = CCW (turn left).
 *   dt_ms: elapsed milliseconds since last call (typically WM_CYCLE_MS).
 */
void demo_odometry_update(demo_odometry_t *odo,
                           float speed_norm, float rate_norm,
                           uint32_t dt_ms);

/* ── Grid-based drift correction ─────────────────────────────────────────── */
/*
 * demo_grid_correct — snap odo->x or odo->y to the nearest chessboard
 * gridline (a multiple of DEMO_SQUARE_UNITS) when a crossing is
 * detected, correcting accumulated dead-reckoning drift.
 *
 * `crossed` is whatever the caller's ground-sensor read reported as "at
 * least one transition since the last call" — this function is
 * intentionally sensor-agnostic (a plain bool, not a hardware sample
 * type) so formation.c stays hardware-independent and host-testable,
 * the same reason it takes floats everywhere else rather than a
 * substrate-specific struct. The board-specific sensor driver
 * (cutebot_line.h, tapestry-os/boards/bbc_microbit_v2/) lives one layer
 * below this and is not included here.
 *
 * This is deliberately coarse: it does not distinguish a genuine
 * chessboard gridline from any other dark/bright transition (there is
 * no fine-line pattern in play yet — see the Choreo design notes for
 * where that distinction becomes necessary), and it does not know WHICH
 * gridline was crossed, only that ONE was — it snaps whichever axis the
 * current heading indicates to the NEAREST multiple of
 * DEMO_SQUARE_UNITS, which corrects however much error had accumulated
 * since the last crossing, not to an absolute known position.
 *
 * Only corrects when heading is clearly axis-aligned, skipping a band
 * around the diagonal where there is no way to tell from a bare
 * transition count whether a row line or a column line was just
 * crossed (DEMO_GRID_AXIS_COS_MIN gates both |cos(heading)| and
 * |sin(heading)|). Corrects position only, never heading — a line
 * crossing carries no heading information.
 *
 * MUST be > 1/sqrt(2) (~0.7071): since cos^2+sin^2=1 always, |cos| and
 * |sin| can never BOTH be below that value at once, so a threshold at
 * or under it makes the "skip near 45 degrees" gate below mathematically
 * unreachable — verified empirically via a standalone host run this
 * session, which caught exactly that with an earlier 0.7 value (every
 * heading picked an axis via the tie-break, none were ever skipped).
 * 0.8 gives a real, deliberate ambiguous band of about 37-53 degrees off
 * either axis (~16 degrees wide, centered on the diagonal).
 */
#ifndef DEMO_GRID_AXIS_COS_MIN
#define DEMO_GRID_AXIS_COS_MIN 0.8f
#endif

void demo_grid_correct(demo_odometry_t *odo, bool crossed);

/* ── Grid-based heading correction ────────────────────────────────────────── */
/*
 * BENCH FINDING (2026-09-10) — this printed chessboard's black ink does
 * NOT read as dark to the ELECFREAKS IR line sensors, even though it
 * looks black to a human eye. IR reflectance sensors see near-infrared
 * reflectance, not visible color — verified via DEMO_MODE_LINE_SENSOR_
 * BENCH (raw P13/P14 logging): plain white paper read 1/1, plain black
 * paper read 0/0 (sensors and wiring both confirmed working correctly,
 * left/right mapping confirmed correct), but the printed board's black
 * AND white squares both read 1/1 — no distinction at all. Whatever
 * process printed this board (not a carbon-based laser toner, evidently)
 * produced ink that's IR-transparent to these sensors.
 *
 * Until the board is retaped/reprinted with real IR-dark material,
 * demo_grid_correct()/demo_grid_heading_correct() are silently no-ops —
 * zero real crossings ever reach them, not a bug in either function.
 * See CONFIG_DEMO_GRID_CORRECTION (Kconfig) to make "no correction"
 * explicit/deliberate rather than incidental for a run in this state.
 *
 * Fix: real IR-absorptive tape (matte black electrical tape, black
 * cardstock — anything that tested as genuinely 0 in the bench mode)
 * along the GRIDLINES ONLY, not filling entire checkerboard squares.
 * Functionally either works — both functions only react to a bright/
 * dark TRANSITION at a boundary, not to anything about what's inside a
 * square — but full squares would cost roughly half the board's area in
 * material for zero functional benefit over covering just the ~13 mm-
 * wide boundary strips. A quick pre-check before committing to either:
 * a single line drawn directly on the board with a black permanent
 * marker (most are carbon-based, usually IR-dark) should read 0 in the
 * bench mode if the theory is right.
 *
 * DEMO_LINE_SENSOR_SEPARATION — lateral distance between the two
 * ground-facing line sensors (NOT DEMO_WHEEL_TRACK — the sensors are
 * mounted at the front of the chassis, not at the wheel axle, and there
 * is no reason to assume the two distances match; measured 15 mm apart
 * on the physical Cutebot Mini, much narrower than the 85 mm wheel
 * track). This directly scales every heading correction
 * demo_grid_heading_correct() produces — a wrong value here over- or
 * under-corrects proportionally, with no way to tell which from firmware
 * behavior alone, so override if a different unit's sensors measure
 * differently:
 *   west build ... -- -DDEMO_LINE_SENSOR_SEPARATION_MM=<measured mm>
 */
#ifndef DEMO_LINE_SENSOR_SEPARATION_MM
#define DEMO_LINE_SENSOR_SEPARATION_MM 15.0f
#endif
#define DEMO_LINE_SENSOR_SEPARATION DEMO_MM(DEMO_LINE_SENSOR_SEPARATION_MM)

/* Reject a left/right entry-edge pair as "not really the same crossing"
 * beyond this time delta. In practice the stateless design below only
 * ever compares two timestamps captured within the SAME poll window
 * (WM_CYCLE_MS, 100 ms) — see demo_grid_heading_correct()'s doc for why
 * a delta from two separate crossings essentially cannot reach this
 * bound; kept as an explicit sanity check rather than relying on that
 * implicitly. */
#define DEMO_HEADING_MAX_PAIR_MS 200u

/* Reject a single correction event that implies more than this much
 * heading error outright, rather than applying it — a real crossing at
 * this fleet's speeds should never imply anywhere near 30 degrees; a
 * value this large is far more likely a noisy/spurious edge pairing
 * than a genuine reading. */
#define DEMO_HEADING_MAX_CORRECTION_RAD 0.5236f   /* 30 degrees */

/*
 * demo_grid_heading_correct — infer and correct heading error from the
 * TIME DELTA between the left and right line sensors crossing the same
 * gridline.
 *
 * Unlike demo_grid_correct() (position only, "a line crossing carries no
 * heading information" — true for a SINGLE sensor), two laterally
 * separated sensors crossing the same line at different instants DOES
 * carry heading information: if the robot crosses exactly perpendicular
 * to the line, both sensors reach it at the same instant (delta 0); if
 * the robot is rotated by some small angle from perpendicular, one
 * sensor — whichever is now leading into the line — reaches it measurably
 * sooner. That delta, combined with forward speed and the physical
 * sensor separation, gives a real heading-error estimate:
 *
 *   heading_error_rad = -(forward_speed_units_per_s * delta_s) / DEMO_LINE_SENSOR_SEPARATION
 *
 * (derivation: in the robot's own frame the left/right sensors sit at
 * lateral offset +-separation/2; rotating the robot by theta from the
 * nominal axis-aligned heading shifts each sensor's world-frame crossing
 * time by +-(separation/2)*theta/v in opposite directions, so the
 * measured delta t_right - t_left = -separation*theta/v, i.e. theta =
 * -v*delta/separation — SIGN NOT YET VERIFIED ON HARDWARE: confirm on
 * the bench by dragging a robot across a line at a small deliberate
 * angle and checking the corrected heading moves the right way before
 * trusting this in a real run; flip the sign here if it doesn't).
 *
 * Sensor-agnostic like demo_grid_correct() — takes plain bool/timestamp
 * pairs, not a cutebot_line_sample_t, so formation.c stays
 * hardware-independent and host-testable.
 *
 *   speed_norm              — the SAME normalized [-1,1] speed just
 *                              passed to substrate_move() this cycle
 *                              (needed to convert delta_s into an angle
 *                              via DEMO_MAX_SPEED — same conversion
 *                              demo_odometry_update() already uses).
 *   left/right_entered       — did THIS poll see a NEW line-entry edge on
 *                              that sensor (cutebot_line_sample_t's
 *                              left_entered/right_entered).
 *   left/right_entry_ms      — that edge's k_uptime_get_32() timestamp
 *                              (cutebot_line_sample_t's *_entry_ms).
 *
 * Deliberately stateless — no-ops unless BOTH sensors report a new entry
 * edge in the SAME poll call. A crossing's total dwell time is roughly
 * 21 ms (see cutebot_line.h) against a 100 ms poll period, so both
 * sensors' edges land in the same poll window in the overwhelming
 * majority of cases; the rare crossing that straddles a poll boundary
 * simply loses heading correction for that one event rather than adding
 * cross-call pairing state to catch it — same bias toward "skip a rare
 * case rather than risk mispairing two different crossings" as
 * demo_grid_correct()'s axis-ambiguity gate.
 *
 * Also skips (no-op) when: heading isn't clearly axis-aligned yet
 * (DEMO_GRID_AXIS_COS_MIN, same gate demo_grid_correct() uses — a
 * diagonal crossing's geometry isn't what this derivation assumes),
 * speed is near zero (no meaningful distance/angle conversion without
 * real forward motion), the pair's time delta exceeds
 * DEMO_HEADING_MAX_PAIR_MS, or the implied correction exceeds
 * DEMO_HEADING_MAX_CORRECTION_RAD.
 *
 * On a valid correction, REPLACES odo->heading with (nearest cardinal
 * axis to the current estimate) + the measured error — a full re-anchor,
 * not an incremental nudge, same "snap to nearest known-good reference"
 * philosophy demo_grid_correct() already uses for position, and more
 * robust than nudging a possibly-already-wrong running estimate.
 *
 * Returns the applied heading_err in radians (the same value folded into
 * odo->heading), or exactly 0.0f if no correction fired this call (either
 * of the no-op cases above). Callers that want a visible sign indicator
 * for bench verification (LEDs, since a nonzero return this rare an event
 * is easy to react to) can check the SIGN of this return value directly
 * — see main.c's DEMO_MODE_STRAIGHT_LINE branch for exactly that use.
 */
float demo_grid_heading_correct(demo_odometry_t *odo, float speed_norm,
                                 bool left_entered, uint32_t left_entry_ms,
                                 bool right_entered, uint32_t right_entry_ms);

/* ── Discovery detection (choreo-2 "king" / choreo-3 "spill") ────────────── */
/*
 * Unlike demo_grid_correct()/demo_grid_heading_correct() above, this does
 * NOT need to tell a target hit apart from a chessboard gridline crossing
 * — grid tape is out of scope for king/spill (see ring.choreo.toml-era
 * decision, 2026-09-16 session), and without it the fabric board itself
 * never reads IR-dark at all (confirmed on the bench). The only INTENDED
 * thing that can register on the line sensors is the printed target
 * marker — but NOT the only thing that actually can: a reflectance IR
 * sensor cannot tell "dark surface" apart from "no surface at all"
 * (open air) — both read as near-zero returned IR to a simple threshold
 * receiver, a real physical ambiguity, not a wiring/calibration issue
 * (confirmed on the bench 2026-09-16: lifting the robot clear of the
 * board reads exactly the same L=1 R=1 as the real marker). A robot that
 * drifts off the physical board edge during the search patrol — a real
 * risk here, since grid correction is deliberately not used for king/
 * spill — would otherwise falsely self-declare discovery. See
 * demo_discovery_in_safe_zone() below, which gates confirmation on the
 * position estimate being safely interior, for the mitigation; that
 * function IS position-aware for exactly this reason, unlike
 * demo_discovery_detect() itself which stays a pure sensor-level
 * debounce/latch (position-gating lives at the call site — see main.c).
 *
 * "Confirmed" still means something beyond just "not off-board": a lone
 * single-poll dark reading is rejected as a possible glitch (real
 * hardware does show occasional bounce — see cutebot_line.h) rather than
 * latched immediately. The 120mm marker (chosen 2026-09-16) at the
 * fleet's measured ~238 mm/s takes roughly half a second — ~5 polls at
 * WM_CYCLE_MS=100ms — to cross, so requiring DEMO_DISCOVERY_CONFIRM_POLLS
 * consecutive dark polls before confirming has wide margin against both
 * a glitch (one poll) and a genuine miss (the marker is never mistakenly
 * this narrow).
 *
 * Once confirmed, latches permanently — an element does not
 * un-discover (matches element_state_t::discovered's own doc, wire v6).
 * Explicit caller-owned state (like demo_odometry_t above), not a
 * hidden static local, so this is host-testable in isolation.
 */
#ifndef DEMO_DISCOVERY_CONFIRM_POLLS
#define DEMO_DISCOVERY_CONFIRM_POLLS 2u
#endif

/*
 * Backing up makes the robot rock and lift its front (an IR sensor that
 * lifts sees no surface, which reads as dark), so a backup is not allowed
 * to produce a detection: for DEMO_DISCOVERY_BACKUP_HOLDOFF_MS after the
 * last reverse command, detection is held off and any run of dark polls in
 * progress is cleared.  Only FORWARD motion can discover (a robot that is
 * stationary, lifted by hand, or reversing cannot).
 */
#ifndef DEMO_DISCOVERY_BACKUP_HOLDOFF_MS
#define DEMO_DISCOVERY_BACKUP_HOLDOFF_MS 500u
#endif
/* Normalized speed_cmd above which the robot counts as moving forward /
 * (negated) reversing. */
#ifndef DEMO_DISCOVERY_MIN_SPEED
#define DEMO_DISCOVERY_MIN_SPEED 0.02f
#endif

typedef struct {
    uint8_t  consecutive_dark_polls;
    bool     discovered;
    /* Consecutive dark polls needed to confirm.  demo_discovery_init()
     * sets DEMO_DISCOVERY_CONFIRM_POLLS (a filled marker is stood on for
     * several polls); set to 1 for a target whose boundary is only a thin
     * outline crossed in well under a poll, and feed the interrupt-driven
     * ENTRY flags (line.left_entered/right_entered) as dark_now so even a
     * ~20 ms crossing between two polls is seen. */
    uint8_t  confirm_polls;
    /* Remaining backup hold-off, ms — see DEMO_DISCOVERY_BACKUP_HOLDOFF_MS. */
    uint32_t holdoff_ms;
} demo_discovery_state_t;

/* Zero-initializes — not discovered, no consecutive polls yet, default
 * confirm count, no hold-off. */
void demo_discovery_init(demo_discovery_state_t *st);

/*
 * demo_discovery_motion_ok — call once per tick BEFORE demo_discovery_detect
 * with the speed command in force this tick and the tick period.  Returns
 * true only when detection is allowed: moving forward, and not within the
 * backup hold-off.  A reversing tick (re)starts the hold-off.
 */
bool demo_discovery_motion_ok(demo_discovery_state_t *st, float speed_cmd,
                              uint32_t elapsed_ms);

/*
 * demo_discovery_detect — feed one poll's raw sensor state in, get back
 * whether the target has now been (confirmed-)discovered.
 *
 *   dark_now — true if EITHER line sensor currently reads dark this
 *              poll (the caller's line.left || line.right — current
 *              level, not an edge/entry flag: unlike the grid
 *              correctors, this wants to know the target is being
 *              stood on for a sustained run of polls, not that a
 *              transition just happened).
 *
 * Returns st->discovered (so a caller can use the return value directly
 * without a separate field read). Once true, every subsequent call
 * returns true immediately regardless of dark_now — sticky, matching
 * the gossiped bit this feeds.
 */
bool demo_discovery_detect(demo_discovery_state_t *st, bool dark_now);

/*
 * demo_discovery_in_safe_zone — is `odo`'s estimate safely inside the
 * board, far enough from the true physical edge that a dark reading
 * here can be trusted as the marker rather than the sensor seeing off
 * the edge into open air (see this section's own doc above for why that
 * ambiguity is real and not a calibration bug)?
 *
 * Deliberately a much bigger margin than DEMO_ARENA_FENCE_MARGIN — that
 * constant only bounds how far a COMMANDED force can push the estimate
 * past the fence, which assumes the estimate is itself accurate; grid
 * correction (the thing that would keep it accurate) is deliberately
 * not used for king/spill (see the section doc above), so dead-
 * reckoning drift alone could put the TRUE position outside the fence
 * while the ESTIMATE still reads as comfortably inside it. Requiring the
 * estimate to additionally sit within the checkered play area
 * ([DEMO_SQUARE_UNITS, WORLD_SIZE - DEMO_SQUARE_UNITS] on both axes —
 * i.e. inside the physical border ring, not just inside the fence)
 * spends the whole width of that border ring as drift margin instead of
 * just DEMO_ARENA_FENCE_MARGIN's narrower one.
 *
 * Caller's job to gate demo_discovery_detect()'s own `dark_now` input on
 * this (see main.c) — kept as a separate function, not folded into
 * demo_discovery_detect() itself, so that function stays a pure
 * sensor-level primitive with no position dependency, host-testable
 * against synthetic sensor sequences alone.
 */
bool demo_discovery_in_safe_zone(const demo_odometry_t *odo);

/* ── Junction-based drift correction (Cutebot Pro's 4-way I2C sensor) ────── */
/*
 * CUTEBOT_PRO_LINE_DARK_THRESHOLD — app-level dark/light cutoff for
 * cutebot_pro_read_line_gray()'s raw per-channel value (0-255, "darkness
 * score" — higher means darker, despite the sensor's own doc calling it
 * reflectance), used INSTEAD OF the board's own firmware-thresholded
 * cutebot_pro_read_line_state() bit for building the line_state fed to
 * demo_junction_correct() below (see main.c's call site).
 *
 * Exists because of a real 2026-09-22 bench finding: this printed
 * chessboard's black ink does NOT cross the firmware's own fixed digital
 * threshold (measured ~102-153, i.e. it sits somewhere in that gap) —
 * the exact same "printed ink is IR-transparent to these sensors" problem
 * Cutebot Mini hit on 2026-09-10 (see DEMO_LINE_SENSOR_SEPARATION_MM's
 * doc below), just rediscovered on different hardware. Real measured
 * per-channel ranges, one static bench session, one robot, held (not
 * driven) over each surface:
 *
 *   white square:  ch1 0-48   ch2 0      ch3 0-6    ch4 0-9
 *   black ink:     ch1 55-95  ch2 0-45   ch3 0-63   ch4 17-66
 *   real marker:   ch1-4 all 102-255 (saturates near 255)
 *   open air:      ch1-4 all 153-255 (indistinguishable from — even
 *                  slightly exceeds — the real marker; see
 *                  demo_junction_correct()'s doc for why this alone does
 *                  NOT make junction detection safe against drift off the
 *                  board edge)
 *
 * 50 sits just above every observed white/no-line reading (worst case 48,
 * on ch1, which runs measurably "hotter" than the other three channels —
 * watch that one first if this needs retuning) with real margin below the
 * black-ink readings. It will still MISS some genuine ink crossings —
 * ch2/ch3 were observed reading as low as 0 while sitting on the black
 * square, so a channel that's only weakly over the ink, or a thin taped/
 * inked line crossed briefly rather than a whole filled square sat on,
 * may not register at all on every channel. That's the deliberate choice:
 * demo_junction_correct() snaps position on a match, so missing a real
 * corner (dead reckoning just continues uncorrected) is a far cheaper
 * mistake than falsely firing one (a real position gets overwritten with
 * a wrong one) — same bias toward "under-trigger, never over-trigger"
 * DEMO_DISCOVERY_CONFIRM_POLLS/BACKUP_HOLDOFF already apply elsewhere in
 * this file. Only bench-validated as a static hold on ONE robot over ONE
 * filled square — not yet validated driving across an actual thin
 * gridline at speed, which is a real, different, and untested case.
 */
#ifndef CUTEBOT_PRO_LINE_DARK_THRESHOLD
#define CUTEBOT_PRO_LINE_DARK_THRESHOLD 50u
#endif

/*
 * CUTEBOT_PRO_WHEEL_TRACK_MM — Cutebot Pro's real wheel-center-to-wheel-
 * center distance, used by main.c's encoder-fed odometry (omega = wheel
 * speed differential / wheel track — standard differential-drive
 * kinematics) instead of DEMO_WHEEL_TRACK (Mini's own measured 85mm
 * chassis, wrong hardware — Pro's is a visibly larger chassis).
 *
 * Derived from a real 2026-09-22 bench measurement, not guessed:
 * DEMO_MODE_CIRCLE_BENCH's arc-turn phase (left=20, right=50 cm/s) ran
 * for 8s and completed ~4 full rotations, observed by eye ("almost
 * exactly 4 times" — not lab-precise, but a real physical count, not an
 * assumption):
 *   omega = 4 * 2*pi / 8s = pi rad/s (0.5 rev/s)
 *   differential = 50 - 20 = 30 cm/s
 *   wheel_track = differential / omega = 30/pi = 95.49 mm
 * Re-derive (or measure directly with calipers) if a more precise value
 * is ever needed — this is a single eyeballed-rotation-count estimate.
 */
#ifndef CUTEBOT_PRO_WHEEL_TRACK_MM
#define CUTEBOT_PRO_WHEEL_TRACK_MM 95.5f
#endif
#define CUTEBOT_PRO_WHEEL_TRACK DEMO_MM(CUTEBOT_PRO_WHEEL_TRACK_MM)

/*
 * demo_junction_correct — snap BOTH odo->x and odo->y to the nearest grid
 * intersection when the raw 4-way line-sensor state (cutebot_pro_read_
 * line_state(), bit0..bit3 = channel 1..4) indicates a real intersection:
 * a crossroads (all 4 channels lit) or a T-junction (any 3 lit — three
 * bits set regardless of WHICH bit is the zero one; distinguishing which
 * side is open would need the sensor array's physical spacing/layout,
 * which is not documented anywhere checked so far — see cutebot_pro.h).
 * Returns which kind fired, or DEMO_JUNCTION_NONE for every other state
 * (line_state==0 no line, or 1-2 channels lit — ordinary line-following,
 * not handled here; see demo_grid_correct() for the single-line-crossing
 * case Cutebot Mini uses instead).
 *
 * A junction pins both axes at once (a real intersection is a known (x,y)
 * point on the grid), unlike demo_grid_correct()'s single-line crossing,
 * which only ever corrects one axis and needs the current heading to know
 * which. No such ambiguity here — both axes snap unconditionally,
 * regardless of which way the robot is facing when it crosses one.
 *
 * Deliberately takes the raw byte, not a cutebot_pro_*-typed sample, so
 * this stays hardware-agnostic and host-testable, same reason
 * demo_grid_correct() takes a plain bool instead of a sensor struct.
 */
typedef enum {
    DEMO_JUNCTION_NONE       = 0,
    DEMO_JUNCTION_T          = 1,
    DEMO_JUNCTION_CROSSROADS = 2,
} demo_junction_type_t;

demo_junction_type_t demo_junction_correct(demo_odometry_t *odo, uint8_t line_state);

/* ── Formation control ──────────────────────────────────────────────────── */

/*
 * Choreo tracking (L6/L7): drive toward a single commanded world point
 * (target_x, target_y) — e.g. a FORM step's ring vertex or a HOLD step's
 * captured station (tapestry/choreo.h's TAPESTRY_BSE_DIRECTIVE_MOVE_TO_
 * POINT target). Turn-then-drive differential-drive law: attraction force
 * points straight at the target (ramping down inside DEMO_TRACK_SLOW_RADIUS,
 * zeroed inside DEMO_TRACK_ARRIVE_EPS), summed with an emergency-repulsion
 * backstop against any fresh peer closer than DEMO_TRACK_MIN_SEP, then
 * projected onto the robot frame (shared via demo_force_to_twist in
 * formation.c) — turning force dominates until the robot is roughly
 * facing the target. There is no moving/stopped hysteresis state (odo is
 * read-only here): the arrival snap alone prevents dither at the target,
 * and there is no separate persisted setpoint to leash/glide — every call
 * recomputes fresh from odo's current dead-reckoning estimate and
 * whatever wm currently holds, since the differential-drive command is
 * instantaneous (no PID state to protect, unlike cf21bl-formation's
 * demo_choreo_track counterpart).
 */
void demo_track_target(const world_model_t *wm,
                        const demo_odometry_t *odo,
                        float target_x,
                        float target_y,
                        float *speed_out,
                        float *rate_out);

/*
 * demo_drive_straight — command constant forward force along the
 * robot's CURRENT heading (DEMO_TRACK_MAX_FORCE, unchanging — this
 * never turns), through the same arena fence demo_track_target() uses,
 * and nothing else: no target, no peer repulsion, no wm at all.
 *
 * For isolated single-motion-primitive testing — does the estimate
 * track straight, does demo_grid_correct() fire the expected number of
 * times for a known number of gridlines, does the fence actually stop
 * it near the true board edge — before trusting any of that inside the
 * full FORM/track_target/Choreo stack, where a wrong result could come
 * from any of several interacting pieces at once. See DEMO_MODE_
 * STRAIGHT_LINE (Kconfig) for the standalone main() that drives this.
 */
void demo_drive_straight(const demo_odometry_t *odo,
                          float *speed_out,
                          float *rate_out);

/* ── Signal feedback ────────────────────────────────────────────────────── */

/*
 * Set substrate signal to reflect L4 world model peer visibility, unless
 * overridden by the active Choreo step's declared indicator effect.
 *   step_indicator   — choreo_current_indicator() (§12 Stage 5).
 *                       SUBSTRATE_SIGNAL_NONE means no override (the
 *                       default, and the behavior of every call site
 *                       written before this feature existed); non-NONE
 *                       takes priority over the heuristic below. Same
 *                       pattern as cf21bl-formation's/webots-formation's
 *                       demo_set_leds().
 * Heuristic (no override):
 *   >=2 fresh peers → SUBSTRATE_SIGNAL_ACTIVE   (formation viable)
 *    1 fresh peer   → SUBSTRATE_SIGNAL_DEGRADED (partial)
 *    0 fresh peers  → SUBSTRATE_SIGNAL_FAILED   (isolated / starting up)
 */
void demo_set_leds(const world_model_t *wm, substrate_signal_t step_indicator);

/*
 * Display dead-reckoning position on the micro:bit 5×5 LED matrix.
 * Maps the 100×100 logical world onto the 5×5 grid (20 units per cell).
 * One lit pixel shows where this robot thinks it is.  Only redraws when
 * the pixel cell changes, so it is safe to call every main-loop cycle.
 *
 * Orientation (connector at bottom, as held during demo):
 *   col 0 = right  col 4 = left   (x-axis flipped)
 *   row 0 = top    row 4 = bottom  (y-axis flipped: large-y → top)
 */
void demo_display_position(const demo_odometry_t *odo);

#endif /* TAPESTRY_DEMO_FORMATION_H */
