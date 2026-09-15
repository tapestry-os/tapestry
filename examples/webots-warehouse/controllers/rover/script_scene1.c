/*
 * script_scene1.c — carries scene 1's generated Choreo script.
 *
 * One TU per script: the generated headers all define the same symbol names
 * under the same include guard (see scene.h), so this is what lets one
 * binary hold all three without editing generated files.
 *
 * Each header lives in its own scene<N>/ subdirectory and is named exactly
 * choreo_script.h — not choreo_script_scene<N>.h — because that is the
 * filename sdk/tools/choreoc.py's `--check` mode globs for when it verifies
 * that every committed header still matches its source script. Named
 * anything else, these three would be invisible to that check and a script
 * edited without regenerating would ship silently.
 */

#include "scene.h"
#include "scene1/choreo_script.h"

void scene1_script(scene_script_t *out)
{
    out->steps            = k_choreo_script;
    out->len              = CHOREO_SCRIPT_LEN;
    out->name             = CHOREO_NAME;
    out->total_timeout_ms = CHOREO_SCRIPT_TOTAL_TIMEOUT_MS;
}
