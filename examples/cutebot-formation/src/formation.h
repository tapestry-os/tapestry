/*
 * formation.h — Demo: Collective Formation (L4 only)
 *
 * Spring-field repulsion/attraction over the L4 world model, with
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
 *   DEMO_TARGET_SPACING — desired peer-to-peer spacing in logical units.
 *     Showcase (L4 spring) mode only; Choreo mode takes its geometry from
 *     the .choreo.toml script instead.
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
#define DEMO_TARGET_SPACING DEMO_MM(500.0f)  /* 39.5 units — showcase mode only.
                                     * equilibrium side = T × 0.854 (4-robot square
                                     * geometry) → ≈ 33.7 units = 427 mm, which keeps
                                     * the whole square inside the checkered [10, 90]
                                     * region with the border still free for turns. */
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
 * demo_track_target() drives toward a single L6/L7-commanded point instead
 * of the spring field's peer-summed force — see that function's doc. */

/* Attraction "force" magnitude commanded at full range (saturates
 * demo_force_to_twist's speed clamp — see FORCE_TO_SPEED in formation.c:
 * need force * FORCE_TO_SPEED >= 22 to reach the same forward-speed cap
 * demo_compute_drive uses, so 40 clears it with margin). */
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
 * settling — the same problem demo_compute_drive's FORCE_STOP/FORCE_START
 * hysteresis solves for the spring field, expressed here as a distance
 * gate since attraction force is monotonic in distance (no sign to
 * hystrese around). Kept smaller than the .choreo.toml script's own
 * achieve_eps (4.0 units in ring.choreo.toml, 5.0 in form-grid) so the
 * controller settles before L6/L7 achievement is even evaluated, rather
 * than fighting it. */
#define DEMO_TRACK_ARRIVE_EPS  DEMO_MM(25.0f)    /* 2.0 units */

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
    bool  moving;   /* Hysteresis state for demo_compute_drive */
} demo_odometry_t;

/* Initialize odometry at (x, y) with heading 0 (+x direction). */
void demo_odometry_init(demo_odometry_t *odo, float x, float y);

/*
 * Update dead-reckoning estimate from the last motion command.
 *   speed_norm: forward velocity [-1.0, 1.0], passed to substrate_move().
 *   rate_norm:  yaw rate         [-1.0, 1.0], positive = CCW (turn left).
 *   Also resets odo->moving when peer count transitions from 0 → non-zero.
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

/* ── Formation control ──────────────────────────────────────────────────── */

/*
 * Compute motion command from the L4 world model.
 *
 * For each active, non-stale, non-self peer in wm, a spring force is applied:
 *   force = (distance - TARGET_SPACING) * SPRING_K
 *   direction = unit vector from own position toward peer
 *
 * The summed force vector is projected onto the robot frame and written to
 * *speed_out (forward velocity) and *rate_out (yaw rate), both normalized
 * [-1.0, 1.0].  Pass these directly to substrate_move() via substrate_twist_t.
 * When no peers are visible, the robot holds position (both outputs zero).
 */
void demo_compute_drive(const world_model_t *wm,
                         demo_odometry_t *odo,
                         float *speed_out,
                         float *rate_out);

/*
 * Choreo tracking (L6/L7): drive toward a single commanded world point
 * (target_x, target_y) — e.g. a FORM step's grid vertex or a HOLD step's
 * captured station (tapestry/choreo.h's TAPESTRY_BSE_DIRECTIVE_MOVE_TO_
 * POINT target) — instead of demo_compute_drive's peer-summed spring
 * force. Turn-then-drive differential-drive law: attraction force points
 * straight at the target (ramping down inside DEMO_TRACK_SLOW_RADIUS,
 * zeroed inside DEMO_TRACK_ARRIVE_EPS), summed with an emergency-repulsion
 * backstop against any fresh peer closer than DEMO_TRACK_MIN_SEP, then
 * projected onto the robot frame the same way demo_compute_drive already
 * does (shared via demo_force_to_twist in formation.c) — turning force
 * dominates until the robot is roughly facing the target, same as the
 * spring field's own behavior, because both share the same projection.
 * Unlike demo_compute_drive there is no moving/stopped hysteresis state
 * (odo is read-only here): the arrival snap alone prevents dither at the
 * target, and there is no separate persisted setpoint to leash/glide —
 * every call recomputes fresh from odo's current dead-reckoning estimate
 * and whatever wm currently holds, since the differential-drive command
 * is instantaneous (no PID state to protect, unlike cf21bl-formation's
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
