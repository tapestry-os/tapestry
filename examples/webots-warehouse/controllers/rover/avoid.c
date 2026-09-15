/*
 * avoid.c — see avoid.h
 */

#include "avoid.h"

#include <math.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void avoid_apply(const world_model_t *wm,
                 const position_t *own_pos_m,
                 demo_setpoint_t *target)
{
    float ax = 0.0f;
    float ay = 0.0f;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self || e->is_stale) {
            continue;
        }

        float dx   = e->state.position.x - own_pos_m->x;
        float dy   = e->state.position.y - own_pos_m->y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist >= AVOID_RADIUS_M) {
            continue;
        }
        if (dist < 0.01f) {
            continue;   /* coincident — direction undefined */
        }

        /* Unit vector pointing away from the peer. */
        float wx = -dx / dist;
        float wy = -dy / dist;

        /* Rotated +90 degrees in the world frame. The fixed rotation is what
         * makes the rule symmetric-breaking: the peer's own away-vector is
         * the negation of ours, so rotating both the same way sends the two
         * robots to opposite sides. */
        float tx = -wy;
        float ty =  wx;

        /* Linear falloff to zero at the engagement radius. */
        float strength = (AVOID_RADIUS_M - dist) / AVOID_RADIUS_M;

        ax += strength * ((1.0f - AVOID_TANGENT_W) * wx + AVOID_TANGENT_W * tx);
        ay += strength * ((1.0f - AVOID_TANGENT_W) * wy + AVOID_TANGENT_W * ty);
    }

    if (ax == 0.0f && ay == 0.0f) {
        return;
    }

    target->x += ax * AVOID_GAIN_M;
    target->y += ay * AVOID_GAIN_M;

    /* Re-establish the invariants tracker.c guarantees on the setpoint —
     * this ran after it, so it is this function's job to restore them. */
    target->x = clampf(target->x, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);
    target->y = clampf(target->y, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);

    {
        float lx = target->x - own_pos_m->x;
        float ly = target->y - own_pos_m->y;
        float ld = sqrtf(lx * lx + ly * ly);
        if (ld > DEMO_TARGET_LEASH_M) {
            target->x = own_pos_m->x + lx / ld * DEMO_TARGET_LEASH_M;
            target->y = own_pos_m->y + ly / ld * DEMO_TARGET_LEASH_M;
        }
    }
}
