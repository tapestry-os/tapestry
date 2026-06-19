/*
 * formation.c — CF21BL spring-field formation control + dead-reckoning
 *
 * Adapted from examples/collective-formation/src/formation.c.
 * Removed: micro:bit display calls (mb_display_get / mb_display_image).
 * Changed: spring and speed constants tuned for a quadrotor arena.
 */

#include "formation.h"

#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(formation, LOG_LEVEL_DBG);

#define M_PI_F  3.14159265f

/* Spring constant — force per logical unit of spacing error. */
#define SPRING_K        2.0f

/* Maps resultant force magnitude to forward speed command. */
#define FORCE_TO_SPEED  0.3f

/* Steering gain: scales lateral force to yaw rate. */
#define TURN_GAIN       8.0f

/* Hysteresis thresholds on net spring force magnitude. */
#define FORCE_STOP   10.0f
#define FORCE_START  20.0f

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── Odometry ─────────────────────────────────────────────────────────────── */

void demo_odometry_init(demo_odometry_t *odo, float x, float y)
{
    odo->x       = x;
    odo->y       = y;
    odo->heading = 0.0f;
    odo->moving  = false;
}

void demo_odometry_update(demo_odometry_t *odo,
                           float speed_norm, float rate_norm,
                           uint32_t dt_ms)
{
    float dt    = (float)dt_ms * 0.001f;
    float v_ctr = speed_norm * DEMO_MAX_SPEED;
    float omega = rate_norm  * DEMO_MAX_OMEGA;

    odo->heading += omega * dt;
    while (odo->heading >  M_PI_F) { odo->heading -= 2.0f * M_PI_F; }
    while (odo->heading < -M_PI_F) { odo->heading += 2.0f * M_PI_F; }

    odo->x += v_ctr * cosf(odo->heading) * dt;
    odo->y += v_ctr * sinf(odo->heading) * dt;

    if (odo->x < 0.0f)      { odo->x = 0.0f; }
    if (odo->x > WORLD_SIZE) { odo->x = WORLD_SIZE; }
    if (odo->y < 0.0f)      { odo->y = 0.0f; }
    if (odo->y > WORLD_SIZE) { odo->y = WORLD_SIZE; }
}

/* ── Formation control ────────────────────────────────────────────────────── */

void demo_compute_drive(const world_model_t *wm,
                         demo_odometry_t *odo,
                         float *speed_out, float *rate_out)
{
    /* Hold if any active peer is stale (world model incomplete). */
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_active && !e->is_self && e->is_stale) {
            odo->moving = false;
            *speed_out  = 0.0f;
            *rate_out   = 0.0f;
            return;
        }
    }

    float fx         = 0.0f;
    float fy         = 0.0f;
    int   peer_count = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self || e->is_stale) {
            continue;
        }

        float dx   = e->state.position.x - odo->x;
        float dy   = e->state.position.y - odo->y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist < 0.01f) {
            continue;
        }

        float force = (dist - DEMO_TARGET_SPACING) * SPRING_K;
        fx += force * (dx / dist);
        fy += force * (dy / dist);
        peer_count++;
    }

    if (peer_count == 0) {
        odo->moving = false;
        *speed_out  = 0.0f;
        *rate_out   = 0.0f;
        return;
    }

    float force_mag = sqrtf(fx * fx + fy * fy);

    if (!odo->moving && force_mag >= FORCE_START) {
        odo->moving = true;
    } else if (odo->moving && force_mag < FORCE_STOP) {
        odo->moving = false;
    }

    if (!odo->moving) {
        *speed_out = 0.0f;
        *rate_out  = 0.0f;
        return;
    }

    float cos_h = cosf(odo->heading);
    float sin_h = sinf(odo->heading);
    float f_fwd = fx *  cos_h + fy * sin_h;
    float f_lat = fx * -sin_h + fy * cos_h;

    float speed = clampf(f_fwd * FORCE_TO_SPEED, -0.5f, 0.5f);
    float turn  = clampf(f_lat * TURN_GAIN / DEMO_TARGET_SPACING, -0.4f, 0.4f);

    *speed_out = speed;
    *rate_out  = turn;

    LOG_DBG("fx=%.2f fy=%.2f fwd=%.2f lat=%.2f spd=%.2f rate=%.2f peers=%d",
            (double)fx, (double)fy,
            (double)f_fwd, (double)f_lat,
            (double)speed, (double)turn, peer_count);
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
