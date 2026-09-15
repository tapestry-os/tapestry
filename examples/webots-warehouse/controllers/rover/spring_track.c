/*
 * spring_track.c — see spring_track.h
 */

#include "spring_track.h"

#include <math.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float demo_spring_track(const world_model_t *wm,
                        const position_t *own_pos_m,
                        demo_setpoint_t *target,
                        float spacing,
                        float spring_k,
                        uint32_t dt_ms)
{
    float dt = (float)dt_ms * 0.001f;

    /* Measured first, ahead of the hold-on-stale return below — the same
     * ordering fix formation.c carries: freezing is the right response to a
     * stale peer, but riding that freeze out with no idea how close the
     * peer was is the blind spot this measurement closes. */
    float min_dist_m = -1.0f;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self || e->is_stale) {
            continue;
        }
        float dx = e->state.position.x - own_pos_m->x;
        float dy = e->state.position.y - own_pos_m->y;
        float d  = sqrtf(dx * dx + dy * dy);
        if (min_dist_m < 0.0f || d < min_dist_m) {
            min_dist_m = d;
        }
    }

    /* Hold in place if any active peer is stale — see spring_track.h. */
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_active && !e->is_self && e->is_stale) {
            target->moving = false;
            return min_dist_m;
        }
    }

    float fx = 0.0f;
    float fy = 0.0f;
    int   peer_count = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self || e->is_stale) {
            continue;
        }

        float dx   = e->state.position.x - own_pos_m->x;
        float dy   = e->state.position.y - own_pos_m->y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist < 0.01f) {
            continue;   /* coincident — push direction undefined */
        }

        /* Bidirectional spring toward the DIRECTIVE's spacing: positive
         * force points toward the peer (too far apart, pull in), negative
         * away (too close, push out). This is what settles the field into a
         * lattice at `spacing` rather than simply scattering. */
        float force = (dist - spacing) * spring_k;

        /* Emergency repulsion inside the hard separation floor, on top of
         * the smooth spring — reacts faster than the spring term alone. */
        if (dist < DEMO_MIN_SEP_M) {
            force -= (DEMO_MIN_SEP_M - dist) * EMERGENCY_K;
        }

        fx += force * (dx / dist);
        fy += force * (dy / dist);
        peer_count++;
    }

    if (peer_count == 0) {
        /* No fresh peers: nothing to disperse from. Glide the setpoint back
         * onto our own position rather than leaving it wherever the field
         * last pushed it — a frozen far-away target makes the last element
         * standing chase it indefinitely (formation.c's DEMO_SOLO_GLIDE
         * lesson). */
        float gx = own_pos_m->x - target->x;
        float gy = own_pos_m->y - target->y;
        float gd = sqrtf(gx * gx + gy * gy);
        if (gd > 0.01f) {
            float step = DEMO_MAX_SPEED_MPS * dt;
            if (step > gd) { step = gd; }
            target->x += gx / gd * step;
            target->y += gy / gd * step;
        }
        target->moving = false;
        return min_dist_m;
    }

    float vx = fx * FORCE_TO_SPEED;
    float vy = fy * FORCE_TO_SPEED;

    float v_mag = sqrtf(vx * vx + vy * vy);
    if (v_mag > DEMO_MAX_SPEED_MPS) {
        vx *= DEMO_MAX_SPEED_MPS / v_mag;
        vy *= DEMO_MAX_SPEED_MPS / v_mag;
    }
    target->moving = (v_mag > 1e-3f);

    target->x = clampf(target->x + vx * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);
    target->y = clampf(target->y + vy * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);

    /* Same leash as tracker.c's MOVE_TO_POINT path — never command further
     * than the body can meaningfully chase. */
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
