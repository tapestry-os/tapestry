/*
 * spring_track.h — MAINTAIN_SPRING directive follower (world-frame meters)
 *
 * WHY THIS EXISTS: L6 emits two directive types that move an element.
 * MOVE_TO_POINT carries a goal point, and ../../../webots-formation's
 * tracker.c (demo_choreo_track) already follows it. DISPERSE instead emits
 * TAPESTRY_BSE_DIRECTIVE_MAINTAIN_SPRING (bse.c), which carries NO goal
 * point at all — only a target `spacing` and a `spring_k`. An element whose
 * main loop only handles MOVE_TO_POINT sits still through an entire
 * disperse step, which is exactly what the cf21bl Webots controller does
 * (it only ever runs a script with no disperse in it).
 *
 * scene1-zone-allocation.choreo.toml opens with a disperse, so this
 * example needs the follower that example never wrote.
 *
 * Adapted from demo_compute_drive in examples/cf21bl-formation/src/
 * formation.c — same spring + emergency-repulsion + hold-on-stale
 * structure, with two deliberate changes:
 *
 *   1. spacing and spring_k are read from the L6 DIRECTIVE rather than from
 *      compile-time constants (formation.c's DEMO_TARGET_SPACING_M /
 *      SPRING_K). That is the more faithful reading of MAINTAIN_SPRING:
 *      those fields exist on the directive precisely so L6 owns them, and
 *      hardcoding them would mean the script's `radius` was ignored.
 *   2. Distances are 2D. formation.c folds z into the distance metric for
 *      airframes that stagger altitude; ground vehicles on a flat floor all
 *      share z, so folding it in would add a constant-zero term and imply a
 *      3D separation guarantee this platform does not provide.
 *
 * Output is a demo_setpoint_t advanced under the SAME leash and arena clamp
 * as tracker.c's MOVE_TO_POINT path, so main.c's downstream steering is
 * identical for both directive types.
 */

#ifndef TAPESTRY_WAREHOUSE_SPRING_TRACK_H
#define TAPESTRY_WAREHOUSE_SPRING_TRACK_H

#include <stdint.h>
#include <tapestry/csm.h>
#include "tracker.h"   /* demo_setpoint_t + the shared DEMO_* constants */

/*
 * Advance *target under the spring field implied by a MAINTAIN_SPRING
 * directive. Returns the minimum fresh-peer distance seen (-1 if none),
 * matching demo_choreo_track's return contract so main.c can apply one
 * separation check to both paths.
 *
 * spacing:  directive.spacing  — the separation the field settles at.
 * spring_k: directive.spring_k — proportional gain on the spacing error.
 *
 * HOLD-ON-STALE (inherited deliberately from demo_compute_drive): if any
 * ACTIVE peer has gone stale, the setpoint freezes rather than being
 * computed against a peer position no longer trusted. The consequence
 * worth knowing: during an RF partition, peers stay active-but-stale for
 * the 3.5 s between WM_STALE_THRESHOLD_MS and WM_EXPIRE_THRESHOLD_MS, so a
 * disperse step pauses for that window before resuming against the
 * surviving island. That is the conservative direction and it is why no
 * scene runs disperse across a partition.
 */
float demo_spring_track(const world_model_t *wm,
                        const position_t *own_pos_m,
                        demo_setpoint_t *target,
                        float spacing,
                        float spring_k,
                        uint32_t dt_ms);

#endif /* TAPESTRY_WAREHOUSE_SPRING_TRACK_H */
