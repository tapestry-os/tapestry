/*
 * movement.c — Tapestry element position update
 */

#include <math.h>
#include <zephyr/random/random.h>
#include "movement.h"

/* ── Internal helpers ────────────────────────────────────────────────────── */

/*
 * rand_float_11 — Return a float in [-1.0, 1.0] using Zephyr's PRNG.
 * Uses the lower 16 bits of sys_rand32_get() for speed.
 */
static float rand_float_11(void)
{
    uint32_t r = sys_rand32_get() & 0xFFFF;
    return ((float)r / 32767.5f) - 1.0f;
}

/* ── API ─────────────────────────────────────────────────────────────────── */

void movement_tick(element_state_t *own_state, const world_model_t *wm)
{
    /* 1. Random walk: independent x/y deltas in [-WALK_STEP_MAX, WALK_STEP_MAX] */
    float dx = rand_float_11() * WALK_STEP_MAX;
    float dy = rand_float_11() * WALK_STEP_MAX;

    /* 2. Repulsion from peers within REPULSION_RADIUS */
    element_id_t near_ids[MAX_ELEMENTS];
    uint8_t near_count = wm_nearest_elements(wm,
                                              &own_state->position,
                                              REPULSION_RADIUS,
                                              near_ids,
                                              MAX_ELEMENTS);

    for (int i = 0; i < near_count; i++) {
        const wm_entry_t *e = wm_get_entry(wm, near_ids[i]);
        if (e == NULL) {
            continue;
        }

        float dist = position_distance(&own_state->position,
                                       &e->state.position);

        if (dist < 0.001f) {
            /* Exactly coincident (or indistinguishably close):
             * add a random kick so elements don't stay stuck */
            dx += rand_float_11() * REPULSION_RADIUS;
            dy += rand_float_11() * REPULSION_RADIUS;
            continue;
        }

        /* Unit vector pointing away from the peer */
        float ux = (own_state->position.x - e->state.position.x) / dist;
        float uy = (own_state->position.y - e->state.position.y) / dist;

        /* Force ∝ (1 − dist/REPULSION_RADIUS), scaled to REPULSION_RADIUS */
        float force = (REPULSION_RADIUS - dist) / REPULSION_RADIUS;
        dx += force * ux * REPULSION_RADIUS;
        dy += force * uy * REPULSION_RADIUS;
    }

    /* 3. Clamp net displacement magnitude to WALK_STEP_MAX */
    float magnitude = sqrtf(dx * dx + dy * dy);
    if (magnitude > WALK_STEP_MAX) {
        float scale = WALK_STEP_MAX / magnitude;
        dx *= scale;
        dy *= scale;
    }

    own_state->position.x += dx;
    own_state->position.y += dy;
    position_clamp(&own_state->position);
    own_state->update_seq++;
}
