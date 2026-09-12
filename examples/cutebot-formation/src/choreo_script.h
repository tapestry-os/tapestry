/*
 * choreo_script.h — GENERATED from ring.choreo.toml — DO NOT EDIT.
 *
 * Choreo: "ring"
 * Regenerate after editing the script file:
 *   python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cutebot-formation/ring.choreo.toml -o tapestry/examples/cutebot-formation/src/choreo_script.h
 *
 * Every step is time-bounded by construction (choreoc requires it): the
 * script cannot stall in flight, and CHOREO_SCRIPT_TOTAL_TIMEOUT_MS is a
 * hard upper bound on script runtime for mission-backstop math.
 */

#ifndef TAPESTRY_CHOREO_SCRIPT_H
#define TAPESTRY_CHOREO_SCRIPT_H

#include <tapestry/choreo.h>

#define CHOREO_NAME                    "ring"
#define CHOREO_SCRIPT_LEN              3u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 600000u

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
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 30.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION,
                .achieve_eps = 3.0f,
                .achieve_hold_ms = 2000u },
      .max_duration_ms = 90000u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_ACHIEVED, .goto_step_idx = 2u } }, .n_transitions = 1u, .indicator = SUBSTRATE_SIGNAL_ACTIVE, .telemetry_tag = "ring" },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 300000u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_ELEMENT_LOST, .goto_step_idx = 1u }, { .event = CHOREO_EVENT_ELEMENT_JOINED, .goto_step_idx = 1u } }, .n_transitions = 2u },
};

#endif /* TAPESTRY_CHOREO_SCRIPT_H */
