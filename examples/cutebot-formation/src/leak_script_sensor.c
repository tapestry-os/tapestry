/*
 * leak_script_sensor.c — leak-sensor.choreo.toml for CONFIG_DEMO_LEAK_DUAL
 * (see leak_scripts.h).  The header is the one examples/leak-sensor ships,
 * so the robots and the sensor can never run different copies.
 */

#include "../../leak-sensor/src/choreo_script.h"

/* main.c is compiled once for both scripts: they must agree on every
 * switch it reads from a script header. */
#if !defined(CHOREO_USES_DISCOVERY) || !defined(CHOREO_USES_LANDMARKS)
#error "both leak scripts must read finds as shared facts (see leak_scripts.h)"
#endif
#include "leak_scripts.h"

#ifndef CHOREO_N_TRACKS
#error "the leak-sensor script must be a [[tracks]] script"
#endif

const demo_script_t demo_script_leak_sensor = {
    .name             = CHOREO_NAME,
    .tracks           = k_choreo_tracks,
    .n_tracks         = CHOREO_N_TRACKS,
    .total_timeout_ms = CHOREO_SCRIPT_TOTAL_TIMEOUT_MS,
};
