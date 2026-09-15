/*
 * choreo_script.h — GENERATED from scene3-failover.choreo.toml — DO NOT EDIT.
 *
 * Choreo: "wh-failover"
 * Regenerate after editing the script file:
 *   python3 tapestry/sdk/tools/choreoc.py tapestry/examples/webots-warehouse/scene3-failover.choreo.toml -o tapestry/examples/webots-warehouse/controllers/rover/scene3/choreo_script.h
 *
 * Every step is time-bounded by construction (choreoc requires it): the
 * script cannot stall in flight, and CHOREO_SCRIPT_TOTAL_TIMEOUT_MS is a
 * hard upper bound on script runtime for mission-backstop math.
 */

#ifndef TAPESTRY_CHOREO_SCRIPT_H
#define TAPESTRY_CHOREO_SCRIPT_H

#include <tapestry/choreo.h>

#define CHOREO_NAME                    "wh-failover"
#define CHOREO_SCRIPT_LEN              4u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 105000u

/* Element departure policy — call choreo_set_departure_policy() (and,
 * for CHOREO_DEPARTURE_RECALL, choreo_set_departure_recall_point_fn())
 * before choreo_submit_script(). */
#define CHOREO_DEPARTURE_POLICY           CHOREO_DEPARTURE_CONTINUE
#define CHOREO_DEPARTURE_REASONS          CHOREO_DEPARTURE_REASONS_ALL
#define CHOREO_DEPARTURE_MIN_PARTICIPANTS 0u

static const choreo_step_t k_choreo_script[CHOREO_SCRIPT_LEN] = {
    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 5000u,
      .advance_on_achieved = false },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { -5.0f, 0.0f, 0.0f },
                .radius = 4.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION,
                .achieve_eps = 0.4f,
                .achieve_hold_ms = 2000u },
      .max_duration_ms = 50000u,
      .advance_on_achieved = true, .scope = CHOREO_SCOPE_ALL },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { -5.0f, 0.0f, 0.0f },
                .radius = 4.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .motion = TAPESTRY_BSE_MOTION_SPIN,
                .spin_rate_radps = 0.15708f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION,
                .achieve_eps = 0.4f,
                .achieve_hold_ms = 2000u },
      .max_duration_ms = 40000u,
      .advance_on_achieved = false },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 10000u,
      .advance_on_achieved = false },
};

#endif /* TAPESTRY_CHOREO_SCRIPT_H */
