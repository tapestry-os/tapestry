/*
 * choreo_script.h — GENERATED from form-grid.choreo.toml — DO NOT EDIT.
 *
 * Choreo: "form-grid"
 * Regenerate after editing the script file:
 *   python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cutebot-formation/form-grid.choreo.toml -o tapestry/examples/cutebot-formation/src/choreo_script.h
 *
 * Every step is time-bounded by construction (choreoc requires it): the
 * script cannot stall in flight, and CHOREO_SCRIPT_TOTAL_TIMEOUT_MS is a
 * hard upper bound on script runtime for mission-backstop math.
 */

#ifndef TAPESTRY_CHOREO_SCRIPT_H
#define TAPESTRY_CHOREO_SCRIPT_H

#include <tapestry/choreo.h>

#define CHOREO_NAME                    "form-grid"
#define CHOREO_SCRIPT_LEN              3u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 65000u

static const choreo_step_t k_choreo_script[CHOREO_SCRIPT_LEN] = {
    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 5000u,
      .advance_on_achieved = false },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 25.0f,
                .shape = TAPESTRY_BSE_SHAPE_GRID,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION,
                .achieve_eps = 5.0f,
                .achieve_hold_ms = 2000u },
      .max_duration_ms = 45000u,
      .advance_on_achieved = true, .scope = CHOREO_SCOPE_ALL },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 15000u,
      .advance_on_achieved = false },
};

#endif /* TAPESTRY_CHOREO_SCRIPT_H */
