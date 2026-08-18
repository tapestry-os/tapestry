/*
 * choreo_telemetry.h — per-tick L6/L7 CSV capture for offline replay
 *
 * Records, once per WM_CYCLE_MS tick, everything Choreo.tick() saw as
 * input (this element's own wm_entries snapshot, scr_state) and everything
 * it produced as output (script step, directive, achievement) — enough to
 * re-drive sdk/python/tapestry's Choreo engine against the exact same
 * inputs offline and diff its outputs against what really happened here.
 * This is capture infrastructure for the offline replay harness described
 * in ../../README.md and sdk/CHOREO_SCRIPTS.md's "Parity" section — not a
 * general logging facility, and not ML training (see tapestry/choreo.h's
 * status banner for that distinction).
 *
 * Opt-in only: set TAPESTRY_TELEMETRY_DIR to a writable directory before
 * launching Webots to enable capture. Unset (the default), this is a
 * complete no-op — every call below degrades to nothing so normal demo
 * runs pay no cost and write no files.
 *
 * One CSV per element: <dir>/choreo_<element_id>.csv. Kept per-element
 * (rather than one shared file) because each element's replay is
 * self-contained — it only ever needs its own recorded rows, never another
 * element's file, so there's no cross-file join to get right at replay
 * time. See sdk/tools/choreo_sim.py (--replay mode) for the reader.
 */

#ifndef TAPESTRY_WEBOTS_CHOREO_TELEMETRY_H
#define TAPESTRY_WEBOTS_CHOREO_TELEMETRY_H

#include <stdint.h>
#include <tapestry/csm.h>
#include <tapestry/scr.h>
#include <tapestry/choreo.h>

typedef struct choreo_telemetry choreo_telemetry_t;

/* Opens <TAPESTRY_TELEMETRY_DIR>/choreo_<element_id>.csv and writes the
 * header row. Returns NULL — telemetry disabled — if the env var is unset
 * or the file can't be opened; every other function silently tolerates a
 * NULL handle. */
choreo_telemetry_t *choreo_telemetry_open(element_id_t element_id);

/* Writes one row for the current tick: wm's per-element snapshot (as seen
 * by this element, from its own world model — id/position/active/stale
 * flags plus each peer's gossiped goal_achieved bit, which is what a
 * scope="all" step advances on and therefore a replay input, not a
 * nicety), scr's role/quorum/fresh_count, dir (the directive computed by
 * this tick's choreo_tick()), and the current choreo_script_step() /
 * choreo_script_complete() / choreo_current_goal_type() /
 * choreo_goal_achieved() globals. No-op if telemetry is NULL. */
void choreo_telemetry_write(choreo_telemetry_t *telemetry,
                            uint32_t tick,
                            double wall_time_s,
                            const world_model_t *wm,
                            const scr_state_t *scr,
                            const tapestry_bse_directive_t *dir);

/* Flushes and closes. Safe to call with NULL. */
void choreo_telemetry_close(choreo_telemetry_t *telemetry);

#endif /* TAPESTRY_WEBOTS_CHOREO_TELEMETRY_H */
