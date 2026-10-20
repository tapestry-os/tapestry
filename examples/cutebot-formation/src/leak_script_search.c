/*
 * leak_script_search.c — leak.choreo.toml for CONFIG_DEMO_LEAK_DUAL (see
 * leak_scripts.h).  choreo_leak.h is generated:
 *
 *   python3 tapestry/sdk/tools/choreoc.py \
 *       tapestry/examples/cutebot-formation/leak.choreo.toml \
 *       -o tapestry/examples/cutebot-formation/src/choreo_leak.h
 */

#include "choreo_leak.h"

/* main.c is compiled once for both scripts: they must agree on every
 * switch it reads from a script header. */
#if !defined(CHOREO_USES_DISCOVERY) || !defined(CHOREO_USES_LANDMARKS)
#error "both leak scripts must read finds as shared facts (see leak_scripts.h)"
#endif
#include "leak_scripts.h"

#if !defined(CHOREO_SCRIPT_LEN) || defined(CHOREO_N_TRACKS)
#error "choreo_leak.h must be a [[steps]] script"
#endif

const demo_script_t demo_script_leak = {
    .name             = CHOREO_NAME,
    .steps            = k_choreo_script,
    .n_steps          = CHOREO_SCRIPT_LEN,
    .total_timeout_ms = CHOREO_SCRIPT_TOTAL_TIMEOUT_MS,
};
