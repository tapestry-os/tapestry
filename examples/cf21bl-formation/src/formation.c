/*
 * formation.c — CF21BL holonomic spring-field formation control
 *
 * See formation.h for the metres-based unit convention and why this
 * differs from examples/collective-formation's dead-reckoning approach.
 */

#include "formation.h"

#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(formation, LOG_LEVEL_DBG);

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

float demo_compute_drive(const world_model_t *wm,
                          const position_t *own_pos_m,
                          demo_setpoint_t *target,
                          uint32_t dt_ms,
                          element_id_t own_id)
{
    float dt = (float)dt_ms * 0.001f;

    /* Hold in place if any active peer is stale — the spring field would
     * otherwise be computed against a peer position we no longer trust. */
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_active && !e->is_self && e->is_stale) {
            target->moving = false;
            return -1.0f;
        }
    }

    float fx          = 0.0f;
    float fy          = 0.0f;
    int   peer_count  = 0;
    float min_dist_m  = -1.0f;

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

        /* Smooth spring toward DEMO_TARGET_SPACING_M. */
        float force = (dist - DEMO_TARGET_SPACING_M) * SPRING_K;

        /* Extra repulsion once inside the hard-floor separation — reacts
         * faster than the smooth spring term alone. */
        if (dist < DEMO_MIN_SEP_M) {
            force -= (DEMO_MIN_SEP_M - dist) * EMERGENCY_K;
        }

        fx += force * (dx / dist);
        fy += force * (dy / dist);
        peer_count++;
    }

    if (peer_count == 0) {
        target->moving = false;
        return min_dist_m;
    }

    float force_mag = sqrtf(fx * fx + fy * fy);

    if (!target->moving && force_mag >= FORCE_START) {
        target->moving = true;
    } else if (target->moving && force_mag < FORCE_STOP) {
        target->moving = false;
    }

    if (!target->moving || force_mag < 1e-6f) {
        return min_dist_m;
    }

    float speed = clampf(force_mag * FORCE_TO_SPEED, 0.0f, DEMO_MAX_SPEED_MPS);
    float vx    = (fx / force_mag) * speed;
    float vy    = (fy / force_mag) * speed;

    target->x = clampf(target->x + vx * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);
    target->y = clampf(target->y + vy * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);

    LOG_DBG("id=%u fx=%.2f fy=%.2f |f|=%.2f v=(%.2f,%.2f) tgt=(%.2f,%.2f) peers=%d min_d=%.2f",
            (unsigned)own_id,
            (double)fx, (double)fy, (double)force_mag,
            (double)vx, (double)vy,
            (double)target->x, (double)target->y,
            peer_count, (double)min_dist_m);

    return min_dist_m;
}

/* ── Signal feedback (LED) ────────────────────────────────────────────────── */

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
