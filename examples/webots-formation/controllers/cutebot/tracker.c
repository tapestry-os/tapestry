/*
 * tracker.c — see formation.h. demo_track_target/demo_arena_fence/
 * demo_force_to_twist are unmodified from
 * examples/cutebot-formation/src/formation.c.
 */

#include "formation.h"

#include <math.h>
#include <stdbool.h>

/* File-local — same values and same scope (never exported via the
 * hardware formation.h either) as cutebot-formation/src/formation.c. */
#define FORCE_TO_SPEED  0.6f
#define TURN_GAIN       12.0f
#define MIN_STICTION    22

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void demo_odometry_init(demo_odometry_t *odo, float x, float y)
{
    odo->x       = x;
    odo->y       = y;
    odo->heading = 0.0f;
}

/* See cutebot-formation/src/formation.c's demo_arena_fence doc: veto the
 * outward component of a commanded force once odo is within
 * DEMO_ARENA_FENCE_MARGIN of a [0, WORLD_SIZE] edge. Component-wise, not
 * radial — a safety backstop, not a precision behavior. */
static void demo_arena_fence(const demo_odometry_t *odo, float *fx, float *fy)
{
    if (odo->x < DEMO_ARENA_FENCE_MARGIN && *fx < 0.0f) {
        *fx = 0.0f;
    }
    if (odo->x > WORLD_SIZE - DEMO_ARENA_FENCE_MARGIN && *fx > 0.0f) {
        *fx = 0.0f;
    }
    if (odo->y < DEMO_ARENA_FENCE_MARGIN && *fy < 0.0f) {
        *fy = 0.0f;
    }
    if (odo->y > WORLD_SIZE - DEMO_ARENA_FENCE_MARGIN && *fy > 0.0f) {
        *fy = 0.0f;
    }
}

/*
 * demo_force_to_twist — project a world-frame force onto the robot's
 * heading (forward/lateral), map to speed/turn, and snap any nonzero
 * result up to MIN_STICTION so commands actually overcome real motor
 * stiction — see cutebot-formation/src/formation.c's full doc for the
 * asymmetric-turn-stall rationale. Unmodified.
 */
static void demo_force_to_twist(const demo_odometry_t *odo, float fx, float fy,
                                 float max_speed, float max_turn,
                                 float *speed_out, float *rate_out)
{
    float cos_h = cosf(odo->heading);
    float sin_h = sinf(odo->heading);
    float f_fwd = fx *  cos_h + fy * sin_h;
    float f_lat = fx * -sin_h + fy * cos_h;

    float speed = clampf(f_fwd * FORCE_TO_SPEED, -max_speed, max_speed);
    float turn  = clampf(f_lat * TURN_GAIN / DEMO_TARGET_SPACING,
                         -max_turn, max_turn);

    float abs_turn = fabsf(turn);
    float needed   = (float)MIN_STICTION + abs_turn;
    if (speed > 0.0f && speed < needed) {
        speed = needed;
    } else if (speed < 0.0f && -speed < needed) {
        speed = -needed;
    } else if (speed == 0.0f && abs_turn > 0.0f && abs_turn < (float)MIN_STICTION) {
        turn = (turn > 0.0f) ? (float)MIN_STICTION : -(float)MIN_STICTION;
    }

    *speed_out = speed / 100.0f;
    *rate_out  = turn  / 100.0f;
}

void demo_track_target(const world_model_t *wm,
                        const demo_odometry_t *odo,
                        float target_x, float target_y,
                        float *speed_out, float *rate_out)
{
    float dx   = target_x - odo->x;
    float dy   = target_y - odo->y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < DEMO_TRACK_ARRIVE_EPS) {
        *speed_out = 0.0f;
        *rate_out  = 0.0f;
        return;
    }

    float mag = (dist > DEMO_TRACK_SLOW_RADIUS)
                ? DEMO_TRACK_MAX_FORCE
                : DEMO_TRACK_MAX_FORCE * (dist / DEMO_TRACK_SLOW_RADIUS);
    float fx = mag * (dx / dist);
    float fy = mag * (dy / dist);

    bool repelled = false;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self || e->is_stale) {
            continue;
        }

        float pdx   = e->state.position.x - odo->x;
        float pdy   = e->state.position.y - odo->y;
        float pdist = sqrtf(pdx * pdx + pdy * pdy);

        if (pdist < 0.01f || pdist >= DEMO_TRACK_MIN_SEP) {
            continue;
        }

        float force = (DEMO_TRACK_MIN_SEP - pdist) * DEMO_TRACK_EMERGENCY_K;
        fx -= force * (pdx / pdist);
        fy -= force * (pdy / pdist);
        repelled = true;
    }

    if (repelled) {
        float net_mag = sqrtf(fx * fx + fy * fy);
        if (net_mag < DEMO_TRACK_REPEL_DEADBAND) {
            *speed_out = 0.0f;
            *rate_out  = 0.0f;
            return;
        }
    }

    demo_arena_fence(odo, &fx, &fy);
    demo_force_to_twist(odo, fx, fy, 22.0f, 15.0f, speed_out, rate_out);
}
