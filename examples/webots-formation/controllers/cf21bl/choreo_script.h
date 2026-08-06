/*
 * choreo_script.h — GENERATED from change-partners.choreo.toml — DO NOT EDIT.
 *
 * Choreo: "change-partners"
 * Regenerate after editing the script file:
 *   python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cf21bl-formation/change-partners.choreo.toml -o tapestry/examples/webots-formation/controllers/cf21bl/choreo_script.h
 *
 * Every step is time-bounded by construction (choreoc requires it): the
 * script cannot stall in flight, and CHOREO_SCRIPT_TOTAL_TIMEOUT_MS is a
 * hard upper bound on script runtime for mission-backstop math.
 */

#ifndef TAPESTRY_CHOREO_SCRIPT_H
#define TAPESTRY_CHOREO_SCRIPT_H

#include <tapestry/choreo.h>

#define CHOREO_NAME                    "change-partners"
#define CHOREO_SCRIPT_LEN              3u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 48000u

static const choreo_step_t k_choreo_script[CHOREO_SCRIPT_LEN] = {
    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 10000u,
      .advance_on_achieved = false },

    { .goal = { .type = CHOREO_GOAL_EXCHANGE,
                .required_caps = CHOREO_CAP_LOCOMOTION,
                .direct_path = true,
                .achieve_eps = 0.25f,
                .achieve_hold_ms = 3000u },
      .max_duration_ms = 30000u,
      .advance_on_achieved = true },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 8000u,
      .advance_on_achieved = false },
};

#endif /* TAPESTRY_CHOREO_SCRIPT_H */
