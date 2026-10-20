/*
 * leak_scripts.h — both leak demos in one firmware (CONFIG_DEMO_LEAK_DUAL)
 *
 * The robots carry leak.choreo.toml (search the board for the printed
 * marker) and leak-sensor.choreo.toml (wait for the stationary sensor's
 * button) and pick one at boot: leak-sensor if a stationary element was
 * heard during the start barrier, leak otherwise.  Showing the other demo
 * takes a power cycle, not a reflash.
 *
 * Each script's generated header defines the same names, so each lives in
 * its own translation unit (leak_script_search.c, leak_script_sensor.c)
 * and is reached through a demo_script_t.  Every compile-time switch
 * main.c reads from a script header has the same value in both, which is
 * what lets one firmware run either: the two .c files #error if that ever
 * stops being true.
 */

#ifndef LEAK_SCRIPTS_H
#define LEAK_SCRIPTS_H

#include <stdint.h>
#include <tapestry/choreo.h>

typedef struct {
    const char           *name;
    const choreo_step_t  *steps;     /* [[steps]] script, else NULL */
    uint8_t               n_steps;
    const choreo_track_t *tracks;    /* [[tracks]] script, else NULL */
    uint8_t               n_tracks;
    uint32_t              total_timeout_ms;
} demo_script_t;

extern const demo_script_t demo_script_leak;          /* leak.choreo.toml */
extern const demo_script_t demo_script_leak_sensor;   /* leak-sensor.choreo.toml */

/* Both scripts: finds become shared facts, one finder (main.c). */
#define CHOREO_USES_DISCOVERY 1u
#define CHOREO_USES_LANDMARKS 1u

#endif /* LEAK_SCRIPTS_H */
