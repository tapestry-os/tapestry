/*
 * formation.h — CF21BL collective formation (L4 only, no SCR)
 *
 * Holonomic spring-field formation control driven by real lighthouse
 * position, not dead reckoning.
 *
 * Units: element_state_t.position (and everything in this file) is in
 * METERS, home-relative in the shared lighthouse world frame — NOT the
 * abstract [0,100] WORLD_SIZE convention csm.h's other constants
 * (MIN_SEPARATION, REPULSION_RADIUS, GOSSIP_RADIUS) assume.  Those CSM
 * constants are unused by this example for exactly that reason; formation.c
 * and main.c define their own meter-scale thresholds instead.
 *
 * REQUIRES all drones to share the SAME lighthouse base-station poses and
 * OOTX calibration (main.c) — gossiped positions are only comparable if
 * every drone's "home" is the same physical point in the same physical
 * frame.  Flash all drones from the same build of this example.
 *
 * Key differences from the old dead-reckoning version:
 *   - No heading/odometry integration: cf21bl_stabilizer's
 *     CF21BL_LIGHTHOUSE_POS_HOLD loop is holonomic (absolute X/Y setpoint,
 *     rotated into body frame internally by Mahony yaw), so formation
 *     control just outputs a target XY point, no forward/turn decomposition.
 *   - Peer distances and separation math use REAL measured position
 *     (broadcast by main.c from cf21bl_lighthouse_get_position()), not an
 *     integrated estimate — no drift accumulation.
 *   - demo_setpoint_t is a virtual, slowly-moving target the stabilizer's
 *     position PID chases; it is deliberately NOT the same as the drone's
 *     real position, so peers always gossip ground truth.
 */

#ifndef TAPESTRY_CF21BL_FORMATION_H
#define TAPESTRY_CF21BL_FORMATION_H

#include <stdint.h>
#include <stdbool.h>
#include <tapestry/csm.h>
#include <tapestry/substrate.h>

/* Desired peer spacing at equilibrium, meters. */
#ifndef DEMO_TARGET_SPACING_M
#define DEMO_TARGET_SPACING_M  1.0f
#endif

/* Max commanded approach speed, m/s — conservative for a first campaign. */
#ifndef DEMO_MAX_SPEED_MPS
#define DEMO_MAX_SPEED_MPS     0.3f
#endif

/* Spring constant — force per meter of spacing error. */
#define SPRING_K            1.0f

/* Hard-floor separation, meters — below this an extra repulsion term (on
 * top of the smooth spring) reacts faster than SPRING_K alone would.
 * 0.5 m is a props-clearance margin, not a contact distance. */
#define DEMO_MIN_SEP_M      0.5f
#define EMERGENCY_K         4.0f

/* Maps net spring force magnitude to a commanded speed fraction of
 * DEMO_MAX_SPEED_MPS. */
#define FORCE_TO_SPEED      0.15f

/* Hysteresis thresholds on net spring force magnitude (avoids dithering
 * right at equilibrium). */
#define FORCE_STOP          0.15f
#define FORCE_START         0.30f

/* Arena clamp for the commanded target — keeps formation.c's output well
 * inside the stabilizer's CF21BL_POS_MAX_M range (main.c enforces the
 * tighter, landing-triggering geofence on the drone's REAL position). */
#define DEMO_ARENA_LIMIT_M  ((float)CONFIG_CF21BL_POS_MAX_M - 0.3f)

/* ── Choreography terms (phase = pure function of fresh-peer count) ───────
 *
 * Rotation: with exactly TWO fresh peers (full triangle), each drone's
 * target orbits the formation centroid (self + fresh peers, real gossiped
 * positions — every drone computes ≈ the same point, no leader) at this
 * angular rate.  0.12 rad/s ≈ 6.9°/s → just over half a revolution across
 * a 30 s triangle phase; tangential speed at the 1 m-triangle circumradius
 * (0.58 m) is ~0.07 m/s, well inside DEMO_MAX_SPEED_MPS.  0.0f disables. */
#define DEMO_ROT_OMEGA_RADPS  0.12f

/* Alignment: with exactly ONE fresh peer (pair phase), the pair ROTATES
 * about its centroid until the pair axis lies along world X (mod π):
 * ω = −K·sin(2φ), where φ is the bearing to the peer.  sin(2φ) is
 * identical from both ends of the pair (φ vs φ+π), so both drones agree
 * on the rotation direction with no communication, and a pure rotation
 * preserves separation by construction.
 *
 * HISTORY (2026-07-11 rehearsal): the first version instead pulled each
 * target's y toward the pair centroid's y.  For a pair left oriented
 * north–south by the preceding rotation phase, "converge in y" means
 * "fly at each other" — the y-pull fought the springs to a standoff at
 * min_d≈0.34 m, |f|≈1.31 (= spring −0.66 + emergency 0.64 exactly), with
 * only the altitude stagger as real margin.  Alignment must be a torque,
 * never a translation.
 *
 * Peak rate 0.25 rad/s at φ=45°; tangential speed ≤ ~0.13 m/s at 0.5 m
 * radius.  φ=90° (exactly N–S) is the unstable equilibrium — sensor noise
 * seeds the rotation in either direction, both of which are correct.
 * Transient radio-contention staleness blips cannot falsely trigger the
 * pair phase: any active-but-stale peer freezes the whole drive
 * (hold-in-place check at the top of demo_compute_drive), so peer_count
 * only drops to 1 after a peer is fully expired
 * (WM_EXPIRE_THRESHOLD_MS = genuinely gone). */
#define DEMO_ALIGN_ROT_RADPS  0.25f

/* Centroid anchor: every drone adds v = K·(anchor − formation_centroid).
 * The springs/rotation/alignment terms control SHAPE and ORIENTATION only
 * — the field is otherwise translation-invariant, so the formation lives
 * wherever accumulated drift leaves it (2026-07-11 full run: the pair
 * re-formed a clean X-parallel line, but displaced ~(+0.2,+0.3) from the
 * ground marks; an earlier rehearsal drifted ~0.5 m west when one biased
 * member dragged the rest through the springs).  The anchor is identical
 * for every drone, so it is a pure translation — zero shape distortion,
 * still leaderless.  K=0.08 /s (τ≈12 s, ≤0.04 m/s at 0.5 m offset) —
 * deliberately the weakest term in the field: it parks the show over the
 * marks without visibly fighting the choreography.  Set K to 0 for
 * pure-emergence mode (no world-frame preference at all).  Anchor point
 * chosen between the marks-layout pair centroid (0.5, 0) and triangle
 * centroid (0.5, 0.29). */
#define DEMO_ANCHOR_X_M       0.5f
#define DEMO_ANCHOR_Y_M       0.15f
#define DEMO_ANCHOR_K         0.08f

/* Target leash: the virtual target may never be further than this from the
 * drone's REAL position.  Without it the target can detach unboundedly
 * (2026-07-11 flight: an emergency-repulsion episode pushed targets away
 * at the 0.3 m/s clamp for seconds, then the alignment torque — whose
 * tangential speed is ω·|target−centroid|, unbounded in radius — kept a
 * target orbiting 2.7 m from its drone at max speed; the drone chased a
 * point it could never reach).  0.75 m matches the stabilizer's own
 * error-saturation radius: commanding further is meaningless anyway. */
#define DEMO_TARGET_LEASH_M   0.75f

/* With ZERO fresh peers the target glides back over the drone's own
 * position at this rate (→ hover in place).  Previously it froze wherever
 * the field last left it — fine when the target had never moved, but
 * after a chaotic episode the last survivor chased a stranded far-away
 * target indefinitely (2026-07-11: id=0 "ran away" toward a target
 * abandoned at (−0.7,−0.8) after both peers landed). */
#define DEMO_SOLO_GLIDE_MPS   0.15f

/* ── Formation target state ──────────────────────────────────────────────── */

typedef struct {
    float x;   /* commanded X setpoint, meters, home-relative */
    float y;   /* commanded Y setpoint, meters, home-relative */
    bool  moving;
} demo_setpoint_t;

void demo_setpoint_init(demo_setpoint_t *sp, float x, float y);

/* Advance *target toward the spring-field equilibrium by dt_ms, using
 * REAL peer positions from wm and this drone's REAL position (own_pos_m,
 * meters) for the force calculation.  Also returns the minimum distance
 * observed to any fresh peer this call (for main.c's separation warning),
 * or -1.0f if no fresh peers were found.  own_id is only used to tag the
 * LOG_DBG line — with multiple drones sharing one radio channel (no
 * per-drone channel plan yet), interleaved console output is otherwise
 * impossible to attribute to a specific drone. */
float demo_compute_drive(const world_model_t *wm,
                          const position_t *own_pos_m,
                          demo_setpoint_t *target,
                          uint32_t dt_ms,
                          element_id_t own_id);

/* Choreo tracking (CONFIG_DEMO_MODE_CHOREO): advance *target toward the L6
 * directive point (cmd_x, cmd_y) at up to DEMO_MAX_SPEED_MPS, keeping the
 * spring field's defense-in-depth terms — emergency repulsion inside
 * DEMO_MIN_SEP_M of any FRESH peer, the target leash, and the arena clamp.
 * The BSE's exchange arc already deconflicts by construction; the repulsion
 * here is a backstop, not the primary separation mechanism.  Unlike
 * demo_compute_drive there is no hold-on-stale check: staleness handling
 * belongs to the caller's quorum mapping (stale peers → choreo SUSPENDED →
 * target frozen).  Returns the minimum fresh-peer distance seen (-1 if no
 * fresh peers). */
float demo_choreo_track(const world_model_t *wm,
                        const position_t *own_pos_m,
                        demo_setpoint_t *target,
                        float cmd_x, float cmd_y,
                        uint32_t dt_ms,
                        element_id_t own_id);

/* ── Signal feedback (LED) ────────────────────────────────────────────────── */

void demo_set_leds(const world_model_t *wm);

#endif /* TAPESTRY_CF21BL_FORMATION_H */
