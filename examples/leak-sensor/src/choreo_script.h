/*
 * choreo_script.h — GENERATED from leak-sensor.choreo.toml — DO NOT EDIT.
 *
 * Choreo: "leak-sensor" (2 tracks, §7)
 * Regenerate after editing the script file:
 *   python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cutebot-formation/leak-sensor.choreo.toml -o tapestry/examples/leak-sensor/src/choreo_script.h
 *
 * Every step is time-bounded by construction (choreoc requires it).  An
 * element runs exactly one track at a time, chosen by the first filter it
 * matches (choreo_submit_tracks()); CHOREO_SCRIPT_TOTAL_TIMEOUT_MS is the
 * worst case across every track — a hard upper bound on any single
 * element's runtime for mission-backstop math.
 */

#ifndef TAPESTRY_CHOREO_SCRIPT_H
#define TAPESTRY_CHOREO_SCRIPT_H

#include <tapestry/choreo.h>

#define CHOREO_NAME                    "leak-sensor"
#define CHOREO_N_TRACKS                2u
#define CHOREO_SCRIPT_TOTAL_TIMEOUT_MS 5400000u
#define CHOREO_USES_DISCOVERY           1u   /* arms the application's find detection */
#define CHOREO_USES_LANDMARKS           1u   /* reads finds as shared facts */

/* Element departure policy — call choreo_set_departure_policy() (and,
 * for CHOREO_DEPARTURE_RECALL, choreo_set_departure_recall_point_fn())
 * before choreo_submit_tracks(). */
#define CHOREO_DEPARTURE_POLICY           CHOREO_DEPARTURE_CONTINUE
#define CHOREO_DEPARTURE_REASONS          CHOREO_DEPARTURE_REASONS_ALL
#define CHOREO_DEPARTURE_MIN_PARTICIPANTS 0u

static const choreo_step_t k_choreo_track0_steps[4] = {
    { .goal = { .type = CHOREO_GOAL_FORM,
                .target = { 50.0f, 50.0f, 0.0f },
                .radius = 20.0f,
                .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                .required_caps = CHOREO_CAP_LOCOMOTION | CHOREO_CAP_ABS_POSITION,
                .achieve_eps = 3.0f,
                .achieve_hold_ms = 2000u },
      .max_duration_ms = 90000u,
      .advance_on_achieved = true, .on = { { .event = CHOREO_EVENT_ACHIEVED, .goto_step_idx = 1u }, { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 2u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 2u, .indicator = SUBSTRATE_SIGNAL_IDLE },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 4500000u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_LANDMARK_ADDED, .goto_step_idx = 2u, .fact_type = FACT_TYPE_TARGET }, { .event = CHOREO_EVENT_ELEMENT_JOINED, .goto_step_idx = 0u }, { .event = CHOREO_EVENT_ELEMENT_LOST, .goto_step_idx = 0u } }, .n_transitions = 3u, .indicator = SUBSTRATE_SIGNAL_IDLE },

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
      .advance_on_achieved = true, .on = { { .event = CHOREO_EVENT_ACHIEVED, .goto_step_idx = 3u }, { .event = CHOREO_EVENT_LANDMARK_RETRACTED, .goto_step_idx = 0u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 2u, .indicator = SUBSTRATE_SIGNAL_ACTIVE, .telemetry_tag = "leak" },

    { .goal = { .type = CHOREO_GOAL_HOLD,
                .required_caps = CHOREO_CAP_LOCOMOTION },
      .max_duration_ms = 4500000u,
      .advance_on_achieved = false, .on = { { .event = CHOREO_EVENT_ELEMENT_LOST, .goto_step_idx = 2u }, { .event = CHOREO_EVENT_ELEMENT_JOINED, .goto_step_idx = 2u }, { .event = CHOREO_EVENT_LANDMARK_RETRACTED, .goto_step_idx = 0u, .fact_type = FACT_TYPE_TARGET } }, .n_transitions = 3u, .indicator = SUBSTRATE_SIGNAL_ACTIVE },
};

static const choreo_step_t k_choreo_track1_steps[1] = {
    { .goal = { .type = CHOREO_GOAL_HOLD },
      .max_duration_ms = 5400000u,
      .advance_on_achieved = false },
};

static const choreo_track_t k_choreo_tracks[CHOREO_N_TRACKS] = {
    { .filter = { .required_caps = CHOREO_CAP_LOCOMOTION },
      .steps = k_choreo_track0_steps, .n_steps = 4u },
    { .filter = { 0 },
      .steps = k_choreo_track1_steps, .n_steps = 1u },
};

#endif /* TAPESTRY_CHOREO_SCRIPT_H */
