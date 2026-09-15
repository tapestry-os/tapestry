/*
 * scene.h — which of the three demonstrations this controller is running
 *
 * One binary serves all three worlds; the scene index arrives in
 * controllerArgs (see main.c's usage). Each scene is a different Choreo
 * script plus a different physical world — nothing about the L3-L7 stack
 * changes between them.
 *
 * The three generated script headers cannot be included in one translation
 * unit: choreoc.py emits fixed symbol names (k_choreo_script, CHOREO_NAME,
 * CHOREO_SCRIPT_LEN, ...) under a fixed include guard, by design. So each
 * one is compiled into its own TU (script_scene1.c, script_scene2.c,
 * script_scene3.c) that exposes it through the accessor below, and the
 * generated headers stay untouched.
 *
 * They live in scene1/, scene2/, scene3/ and are each named exactly
 * choreo_script.h so that `choreoc.py --check` — which globs the repo for
 * that filename and recovers each header's source script from its own
 * banner — verifies all three along with every other committed header.
 */

#ifndef TAPESTRY_WAREHOUSE_SCENE_H
#define TAPESTRY_WAREHOUSE_SCENE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <tapestry/choreo.h>

typedef struct {
    const choreo_step_t *steps;
    size_t               len;
    const char          *name;
    uint32_t             total_timeout_ms;
} scene_script_t;

/* Defined one per script_sceneN.c. */
void scene1_script(scene_script_t *out);
void scene2_script(scene_script_t *out);
void scene3_script(scene_script_t *out);

/* Fills *out for scene in [1, 3]. Returns false for any other value. */
bool scene_get_script(int scene, scene_script_t *out);

/* True only for scene 2 — the only world containing the steel mezzanine
 * deck that rf_occlusion.c models. Scenes 1 and 3 run with the RF model
 * inert, so their fleets stay fully connected throughout. */
bool scene_has_rf_deck(int scene);

#endif /* TAPESTRY_WAREHOUSE_SCENE_H */
