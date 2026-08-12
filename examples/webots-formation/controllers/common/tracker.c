/*
 * tracker.c — see tracker.h. demo_choreo_track/demo_setpoint_init are
 * unmodified from examples/cf21bl-formation/src/formation.c.
 */

#include "tracker.h"

#include <math.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void demo_setpoint_init(demo_setpoint_t *sp, float x, float y)
{
    sp->x      = x;
    sp->y      = y;
    sp->moving = false;
}

float demo_choreo_track(const world_model_t *wm,
                        const position_t *own_pos_m,
                        demo_setpoint_t *target,
                        float cmd_x, float cmd_y,
                        uint32_t dt_ms,
                        element_id_t own_id)
{
    (void)own_id;
    float dt = (float)dt_ms * 0.001f;

    /* Emergency repulsion + minimum-distance bookkeeping over fresh peers.
     * No attraction term — the L6 directive owns where we are going. */
    float fx         = 0.0f;
    float fy         = 0.0f;
    float min_dist_m = -1.0f;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self || e->is_stale) {
            continue;
        }

        float dx   = e->state.position.x - own_pos_m->x;
        float dy   = e->state.position.y - own_pos_m->y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (min_dist_m < 0.0f || dist < min_dist_m) {
            min_dist_m = dist;
        }
        if (dist < 0.01f) {
            continue;   /* coincident reading — direction undefined */
        }
        if (dist < DEMO_MIN_SEP_M) {
            float force = (DEMO_MIN_SEP_M - dist) * EMERGENCY_K;
            fx -= force * (dx / dist);
            fy -= force * (dy / dist);
        }
    }

    /* Approach velocity toward the directive point. ad/dt (not just the
     * speed clamp) lets the target land exactly on cmd once close. */
    float vx = fx * FORCE_TO_SPEED;
    float vy = fy * FORCE_TO_SPEED;

    float ax = cmd_x - target->x;
    float ay = cmd_y - target->y;
    float ad = sqrtf(ax * ax + ay * ay);
    if (ad > 1e-6f) {
        float speed = ad / dt;
        if (speed > DEMO_MAX_SPEED_MPS) {
            speed = DEMO_MAX_SPEED_MPS;
        }
        vx += (ax / ad) * speed;
        vy += (ay / ad) * speed;
    }

    float v_mag = sqrtf(vx * vx + vy * vy);
    if (v_mag > DEMO_MAX_SPEED_MPS) {
        vx *= DEMO_MAX_SPEED_MPS / v_mag;
        vy *= DEMO_MAX_SPEED_MPS / v_mag;
    }

    target->x = clampf(target->x + vx * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);
    target->y = clampf(target->y + vy * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);

    /* Never command further than the body can meaningfully chase. */
    {
        float lx = target->x - own_pos_m->x;
        float ly = target->y - own_pos_m->y;
        float ld = sqrtf(lx * lx + ly * ly);
        if (ld > DEMO_TARGET_LEASH_M) {
            target->x = own_pos_m->x + lx / ld * DEMO_TARGET_LEASH_M;
            target->y = own_pos_m->y + ly / ld * DEMO_TARGET_LEASH_M;
        }
    }

    return min_dist_m;
}

void demo_set_leds(const world_model_t *wm)
{
    int fresh  = 0;
    int active = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self) {
            continue;
        }
        active++;
        if (!e->is_stale) {
            fresh++;
        }
    }

    substrate_signal_t sig;
    if      (active == 0)      sig = SUBSTRATE_SIGNAL_FAILED;
    else if (fresh  <  active) sig = SUBSTRATE_SIGNAL_DEGRADED;
    else                       sig = SUBSTRATE_SIGNAL_ACTIVE;
    substrate_set_signal(sig);
}
