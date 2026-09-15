/*
 * cloud_occlusion.h — the cloud fleet's own copy of the RF obstruction
 * geometry (scene 2).
 *
 * A STANDALONE DUPLICATE, NOT A SHARED INCLUDE — see main.c's header
 * comment for why: this controller has no dependency on any Tapestry
 * header, and pulling in ../rover/rf_occlusion.h (which reaches into
 * tapestry/csm.h for position_t) would blur the line this example is
 * trying to keep sharp between "the system being compared against" and
 * "the system it's being compared to".
 *
 * The four constants and the two-cell-then-span test below MUST match
 * ../rover/rf_occlusion.h's RF_DECK_* and rf_link_obstructed() exactly —
 * enforced by ../../ci-check/cloud_geom_check.c, the same
 * cross-file-invariant pattern already used between rf_occlusion.h and the
 * .wbt deck Solid. See that file for why this needs an active check
 * rather than a comment alone: an earlier version of the cloud fleet's own
 * x-placement (6/7/8/9 m) put its northbound path close enough to the
 * deck's east support leg (x=9.5 m) that one robot physically wedged
 * against it mid-crossing — a world-geometry clearance bug, not a logic
 * bug, and exactly the kind of thing worth an automated check rather than
 * re-discovering by watching a robot stall in a live run.
 */

#ifndef TAPESTRY_WAREHOUSE_CLOUD_OCCLUSION_H
#define TAPESTRY_WAREHOUSE_CLOUD_OCCLUSION_H

#include <stdbool.h>

#define CLOUD_RF_DECK_X_MIN  (-10.0f)
#define CLOUD_RF_DECK_X_MAX  ( 10.0f)
#define CLOUD_RF_DECK_Y_MIN  (  1.4f)
#define CLOUD_RF_DECK_Y_MAX  (  2.6f)

/* True if a link between (ax,ay) and (bx,by) is obstructed by the deck.
 * Same three-cell (south/under/north) model as rf_occlusion.c's
 * rf_link_obstructed() — see that file's long comment for why "under the
 * deck" has to be its own opaque cell rather than transparent. */
bool cloud_link_obstructed(float ax, float ay, float bx, float by);

#endif /* TAPESTRY_WAREHOUSE_CLOUD_OCCLUSION_H */
