/*
 * avoid.h — tangential collision avoidance for a non-holonomic platform
 *
 * WHY THIS EXISTS. tracker.c's emergency repulsion (DEMO_MIN_SEP_M /
 * EMERGENCY_K) was written and flight-validated for cf21bl airframes, which
 * are holonomic: a drone commanded to move sideways moves sideways, so a
 * repulsion vector pointing directly away from a peer is immediately and
 * fully realizable. A differential-drive AMR cannot do that. It can only
 * drive along its own heading, so a repulsion vector pointing straight back
 * down its approach path does not make it dodge — it makes it slow, stop,
 * or turn around. Two AMRs closing head-on both push their setpoints
 * backward along the same line and press into each other.
 *
 * This is not hypothetical. The first full run of scene 3 logged 1140
 * separation-violation ticks with a worst-case peer distance of 0.40 m,
 * between robots whose bodies are 0.5 m long — that is contact, not a near
 * miss. The repulsion was working exactly as designed and was simply the
 * wrong shape of correction for this platform.
 *
 * WHAT IT DOES. Adds a TANGENTIAL component to the avoidance, so a robot
 * slides around a neighbour instead of backing away from it. The tangential
 * direction is obtained by rotating the away-vector a fixed +90 degrees in
 * the world frame, which is what makes two robots resolve a head-on
 * encounter to OPPOSITE sides: each computes an away-vector that is the
 * negation of the other's, so rotating both the same way sends them in
 * opposite directions. It is the "everyone veers the same way" traffic rule
 * real AMR fleets use, and it needs no negotiation between the robots.
 *
 * WHERE IT SITS. Below Tapestry, not inside it. This is a platform-level
 * (L1/L2) reflex on the commanded setpoint, applied after L6's directive
 * has already been turned into a setpoint by tracker.c or spring_track.c.
 * It is rover-local for that reason — no Tapestry layer knows it exists,
 * and adding it to the shared controllers/common/ would push a
 * ground-vehicle assumption onto the drone example. L6 still owns WHERE the
 * collective is going; this only affects how a body gets there without
 * hitting a neighbour on the way.
 *
 * SCOPE. It is a reactive nudge, not a planner. It has no notion of static
 * obstacles (racking, the mezzanine legs, a failed robot's hull), no
 * deadlock resolution, and no guarantee. See the README's known limitations.
 */

#ifndef TAPESTRY_WAREHOUSE_AVOID_H
#define TAPESTRY_WAREHOUSE_AVOID_H

#include <tapestry/csm.h>
#include "tracker.h"   /* demo_setpoint_t + shared DEMO_* constants */

/* Engagement radius, 2x DEMO_MIN_SEP_M (0.8 m). Two constraints set it: far
 * enough out that a non-holonomic body has time to turn and slide before the
 * gap closes, and strictly below every settled formation spacing any scene
 * uses, so it can never perturb a goal the collective has already achieved.
 * The binding case is scene 3's POST-FAILURE line, which respaces seven
 * elements over 14 m at 2.33 m.
 *
 * Raising it to 2.5x was tried and measurably did not help — scene 1's
 * worst close pass stayed at 0.51 m and the violation count rose slightly.
 * The limit in dense dock traffic is not how early the nudge engages, it is
 * that a differential drive has to turn before it can move at all. Left at
 * the smaller, better-separated value. */
#define AVOID_RADIUS_M   1.6f

/* Fraction of the avoidance that is tangential rather than straight-away.
 * At 0.0 this degenerates to tracker.c's existing pure repulsion and the
 * head-on failure above returns; at 1.0 robots orbit each other without
 * ever separating. */
#define AVOID_TANGENT_W  0.75f

/* Metres of setpoint offset at full strength (peer at zero distance). Kept
 * below DEMO_TARGET_LEASH_M (1.5 m) so the nudge can never on its own push
 * the setpoint past the leash and get truncated in a direction-dependent
 * way. */
#define AVOID_GAIN_M     0.9f

/*
 * Nudge *target around any fresh peer inside AVOID_RADIUS_M, then re-apply
 * tracker.c's target leash and arena clamp so the result still satisfies the
 * invariants the setpoint had before this ran.
 *
 * Fresh peers only, deliberately: a stale peer's position is not trusted
 * enough to steer on, and — importantly for scene 3 — a FAILED element goes
 * stale 1.5 s after dying, so this will NOT dodge its hull. A dead robot is
 * an obstacle the collective can no longer see. That is a real consequence,
 * and scene 3's script is laid out to keep survivors' paths clear of the
 * corpse rather than to pretend otherwise.
 */
void avoid_apply(const world_model_t *wm,
                 const position_t *own_pos_m,
                 demo_setpoint_t *target);

#endif /* TAPESTRY_WAREHOUSE_AVOID_H */
