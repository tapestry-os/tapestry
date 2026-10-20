/*
 * choreo_script.h — GENERATED from leak.choreo.toml — DO NOT EDIT.
 *
 * Choreo: "leak"
 * Regenerate after editing the script file:
 *   python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cutebot-formation/leak.choreo.toml -o tapestry/examples/cutebot-formation/src/choreo_leak.h
 *
 * Every step is time-bounded by construction (choreoc requires it): the
 * script cannot stall in flight, and CHOREO_SCRIPT_TOTAL_TIMEOUT_MS is a
 * hard upper bound on script runtime for mission-backstop math.
 */

#ifndef TAPESTRY_CHOREO_SCRIPT_H
#define TAPESTRY_CHOREO_SCRIPT_H

#include <tapestry/choreo.h>

#define CHOREO_NAME                    "leak"
#define CHOREO_SCRIPT_LEN              23u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 900000u
#define CHOREO_USES_DISCOVERY           1u   /* arms the application's find detection */
#define CHOREO_USES_LANDMARKS           1u   /* reads finds as shared facts */

/* Element departure policy — call choreo_set_departure_policy() (and,
 * for CHOREO_DEPARTURE_RECALL, choreo_set_departure_recall_point_fn())
 * before choreo_submit_script(). */
#define CHOREO_DEPARTURE_POLICY           CHOREO_DEPARTURE_CONTINUE
#define CHOREO_DEPARTURE_REASONS          CHOREO_DEPARTURE_REASONS_ALL
#define CHOREO_DEPARTURE_MIN_PARTICIPANTS 0u

static const choreo_step_t k_choreo_script[CHOREO_SCRIPT_LEN] = {
    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 22.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 0.785398f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 3000u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 22.83f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 1.13446f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 23.67f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 1.48353f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 24.5f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 1.8326f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 25.33f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 2.18166f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 26.17f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 2.53073f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 27.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 2.87979f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 27.83f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 3.22886f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 28.67f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 3.57792f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 29.5f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 3.92699f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 30.33f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 4.27606f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 31.17f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 4.62512f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 32.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 4.97419f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 32.83f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 5.32325f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 33.67f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 5.67232f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 34.5f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 6.02139f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 35.33f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 6.37045f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 36.17f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 6.71952f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 37.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .rotation_rad = 7.06858f,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION },
      .max_duration_ms = 1500u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 600000u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 20u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 1u, .indicator = SUBSTRATE_SIGNAL_DEGRADED },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 4000u,
      .advance_on_achieved = false, .indicator = SUBSTRATE_SIGNAL_ACTIVE },

    { .goal = { .type = CHOREO_GOAL_FORM,
                .radius = 20.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .frame = TAPESTRY_BSE_FRAME_ELEMENT,
                .anchor = TAPESTRY_BSE_ANCHOR_LANDMARK,
                .landmark_type = FACT_TYPE_TARGET,
                .landmark_rank = 0u,
                .required_caps = CHOREO_CAP_LOCOMOTION,
                .achieve_eps = 3.0f,
                .achieve_hold_ms = 2000u },
      .max_duration_ms = 90000u,
      .advance_on_achieved = true, .on = { { .event = CHOREO_EVENT_ACHIEVED, .goto_step_idx = 22u } }, .n_transitions = 1u, .indicator = SUBSTRATE_SIGNAL_ACTIVE, .telemetry_tag = "leak" },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 300000u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_ELEMENT_LOST, .goto_step_idx = 21u }, { .event = CHOREO_EVENT_ELEMENT_JOINED, .goto_step_idx = 21u } }, .n_transitions = 2u, .indicator = SUBSTRATE_SIGNAL_ACTIVE },
};

#endif /* TAPESTRY_CHOREO_SCRIPT_H */
