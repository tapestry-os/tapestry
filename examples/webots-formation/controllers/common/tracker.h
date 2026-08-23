/*
 * tracker.h — choreo target tracker (world-frame meters)
 *
 * Trimmed from examples/cf21bl-formation/src/formation.h: keeps only
 * demo_choreo_track (Choreo-mode target leash + emergency repulsion) and
 * its supporting types. demo_compute_drive (the showcase-mode leaderless
 * spring field) isn't needed here — this example only runs Choreo mode —
 * so it and its only-used constants (SPRING_K, DEMO_ROT_OMEGA_RADPS,
 * DEMO_ALIGN_ROT_RADPS, DEMO_ANCHOR_*, DEMO_SOLO_GLIDE_MPS) are dropped.
 *
 * demo_choreo_track itself is unmodified logic — same target-leash,
 * emergency-repulsion, and arena-clamp behavior already flight-validated
 * on cf21bl hardware. The one change is DEMO_ARENA_LIMIT_M: hardware
 * derives it from CONFIG_CF21BL_POS_MAX_M (a Zephyr Kconfig value); here
 * it's a plain constant sized for a Webots arena instead.
 *
 * Units: meters, world frame — matches Webots GPS output directly (no
 * "home" concept to convert through, unlike the lighthouse-relative
 * hardware version).
 */

#ifndef TAPESTRY_WEBOTS_TRACKER_H
#define TAPESTRY_WEBOTS_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include <tapestry/csm.h>
#include <tapestry/substrate.h>

/* Max commanded approach speed, m/s. */
#ifndef DEMO_MAX_SPEED_MPS
#define DEMO_MAX_SPEED_MPS     0.3f
#endif

/* Hard-floor separation, meters — emergency repulsion kicks in inside this
 * (on top of, not instead of, the L6 EXCHANGE arc's own deconfliction).
 * Same values as cf21bl hardware (formation.h) — this is a horizontal-only
 * backstop, not the primary mechanism, so there's no reason for this
 * example to diverge from the flight-validated constants. (An earlier
 * version of this file shrank these to chase a slow-convergence problem
 * that turned out to be unrelated — see the WEBOTS FORK NOTE in
 * ../../change-partners.choreo.toml for the real cause and fix.) */
#define DEMO_MIN_SEP_M      0.5f
#define EMERGENCY_K         4.0f

/* Maps net repulsion force magnitude to a commanded speed fraction. */
#define FORCE_TO_SPEED      0.15f

/* Arena clamp for the commanded target — a Webots-sized default, generous
 * relative to the ~1 m swap distances change-partners.choreo.toml uses. */
#define DEMO_ARENA_LIMIT_M  5.0f

/* Target leash: the virtual target may never be further than this from the
 * drone's REAL position — see formation.c's original comment (2026-07-11
 * flight incident) for why this exists. */
#define DEMO_TARGET_LEASH_M   0.75f

typedef struct {
    float x;   /* commanded X setpoint, meters, world frame */
    float y;   /* commanded Y setpoint, meters, world frame */
    bool  moving;
} demo_setpoint_t;

void demo_setpoint_init(demo_setpoint_t *sp, float x, float y);

/* Advance *target toward the L6 directive point (cmd_x, cmd_y) at up to
 * DEMO_MAX_SPEED_MPS, with emergency repulsion inside DEMO_MIN_SEP_M of any
 * fresh peer and the target leash/arena clamp as defense in depth. Returns
 * the minimum fresh-peer distance seen this call (-1 if none). */
float demo_choreo_track(const world_model_t *wm,
                        const position_t *own_pos_m,
                        demo_setpoint_t *target,
                        float cmd_x, float cmd_y,
                        uint32_t dt_ms,
                        element_id_t own_id);

/* Signal feedback — no-op on this backend (substrate_set_signal is a
 * logging-only stub), kept for main-loop parity with cf21bl-formation.
 * step_indicator: the active script step's declared indicator effect
 * (choreo_current_indicator(), §12 Stage 5) — SUBSTRATE_SIGNAL_NONE means
 * no override, the default and the behavior of every script written
 * before this feature existed.  Non-NONE takes priority over this
 * function's own quorum/freshness heuristic. */
void demo_set_leds(const world_model_t *wm, substrate_signal_t step_indicator);

#endif /* TAPESTRY_WEBOTS_TRACKER_H */
