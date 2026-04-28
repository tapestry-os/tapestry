/*
 * formation.c — Demo: spring-field formation control + dead-reckoning
 */

#include "formation.h"
#include <tapestry/actuation.h>

#include <math.h>
#include <zephyr/display/mb_display.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(formation, LOG_LEVEL_DBG);

#define M_PI_F      3.14159265f

/* Spring constant — force per logical unit of spacing error.
 * Must be large enough that typical displacements (5–10 units) produce
 * a net force that exceeds FORCE_DEADBAND and commands above stiction. */
#define SPRING_K        8.0f

/* Maps resultant force magnitude to motor speed percent. */
#define FORCE_TO_SPEED  0.6f

/* Steering gain: scales lateral force to differential turn. */
#define TURN_GAIN       12.0f

/* Motor speed when wandering with no peers visible.
 * Must be above stiction threshold (~20% measured). */
#define BASE_WANDER     22

/* Minimum motor % that overcomes stiction (measured). Any non-zero
 * speed command is snapped up to this so the motors actually turn. */
#define MIN_STICTION    22

/* Hysteresis thresholds on net spring force magnitude.
 * A stopped robot only starts moving when force exceeds FORCE_START.
 * A moving robot stops when force drops below FORCE_STOP.
 * The gap between them prevents oscillation near equilibrium: a small
 * correction that slightly overshoots does not immediately trigger a
 * counter-correction, and gossip-propagated micro-adjustments from
 * neighbours do not restart a robot that has just settled. */
#define FORCE_STOP   25.0f   /* ~2 units / 6 mm from equilibrium  */
#define FORCE_START  90.0f   /* > one-cycle overshoot force (~81 units) */

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── Odometry ─────────────────────────────────────────────────────────────── */

void demo_odometry_init(demo_odometry_t *odo, float x, float y)
{
    odo->x       = x;
    odo->y       = y;
    odo->heading = 0.0f;
}

void demo_odometry_update(demo_odometry_t *odo,
                           int left_pct, int right_pct,
                           uint32_t dt_ms)
{
    float dt    = (float)dt_ms * 0.001f;
    float scale = DEMO_SPEED_SCALE / 100.0f;

    float vl = (float)left_pct  * scale;   /* logical units/s */
    float vr = (float)right_pct * scale;

    float v_center = (vl + vr) * 0.5f;
    float omega    = (vr - vl) / DEMO_WHEEL_TRACK;   /* rad/s */

    odo->heading += omega * dt;

    /* Normalise to (-π, π] */
    while (odo->heading >  M_PI_F) { odo->heading -= 2.0f * M_PI_F; }
    while (odo->heading < -M_PI_F) { odo->heading += 2.0f * M_PI_F; }

    odo->x += v_center * cosf(odo->heading) * dt;
    odo->y += v_center * sinf(odo->heading) * dt;

    /* Clamp to world bounds */
    if (odo->x < 0.0f)      { odo->x = 0.0f; }
    if (odo->x > WORLD_SIZE) { odo->x = WORLD_SIZE; }
    if (odo->y < 0.0f)      { odo->y = 0.0f; }
    if (odo->y > WORLD_SIZE) { odo->y = WORLD_SIZE; }
}

/* ── Formation control ────────────────────────────────────────────────────── */

void demo_compute_drive(const world_model_t *wm,
                         const demo_odometry_t *odo,
                         int *left_out, int *right_out)
{
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

        /*
         * Spring force along the line between self and peer.
         * Positive force  = toward peer (attraction, dist > target).
         * Negative force = away from peer (repulsion, dist < target).
         */
        float force = (dist - DEMO_TARGET_SPACING) * SPRING_K;

        fx += force * (dx / dist);
        fy += force * (dy / dist);
        peer_count++;
    }

    if (peer_count == 0) {
        /* No peers visible: hold position and wait for BLE gossip.
         * BLE scanning is passive — physical movement does not help
         * discovery, and wandering corrupts the dead-reckoning origin. */
        *left_out  = 0;
        *right_out = 0;
        return;
    }

    /* Hysteresis: require a larger force to start moving than to stop.
     * Prevents oscillation and gossip-cascade near equilibrium. */
    static bool moving = false;
    float force_mag = sqrtf(fx * fx + fy * fy);

    if (!moving && force_mag >= FORCE_START) {
        moving = true;
    } else if (moving && force_mag < FORCE_STOP) {
        moving = false;
    }

    if (!moving) {
        *left_out  = 0;
        *right_out = 0;
        return;
    }

    /*
     * Project world-frame force (fx, fy) onto the robot frame.
     *
     * Robot forward axis in world frame: (cos h, sin h)
     * Robot left    axis in world frame: (-sin h, cos h)
     *
     * f_fwd > 0 → move forward
     * f_lat > 0 → force is to the robot's left → turn left
     */
    float cos_h = cosf(odo->heading);
    float sin_h = sinf(odo->heading);
    float f_fwd = fx *  cos_h + fy * sin_h;
    float f_lat = fx * -sin_h + fy * cos_h;

    float speed = clampf(f_fwd * FORCE_TO_SPEED, -22.0f, 22.0f);
    float turn  = clampf(f_lat * TURN_GAIN / DEMO_TARGET_SPACING,
                         -15.0f, 15.0f);

    /* Snap: any non-zero speed command must reach the stiction threshold
     * or the motors will not turn.  The force_mag check above already
     * guards the zero case so anything reaching here should move. */
    if (speed > 0.0f && speed < (float)MIN_STICTION) {
        speed = (float)MIN_STICTION;
    } else if (speed < 0.0f && speed > -(float)MIN_STICTION) {
        speed = -(float)MIN_STICTION;
    }

    *left_out  = clampi((int)(speed - turn), -100, 100);
    *right_out = clampi((int)(speed + turn), -100, 100);

    LOG_DBG("fx=%.2f fy=%.2f fwd=%.2f lat=%.2f L=%d R=%d peers=%d",
            (double)fx, (double)fy,
            (double)f_fwd, (double)f_lat,
            *left_out, *right_out, peer_count);
}

/* ── Position display (micro:bit 5×5 matrix) ─────────────────────────────── */

void demo_display_position(const demo_odometry_t *odo)
{
    /* 100-unit world → 5 cells of 20 units each. */
    int col     = 4 - (int)(odo->x / 20.0f);
    int led_row = 4 - (int)(odo->y / 20.0f);

    if (col     < 0) { col     = 0; } else if (col     > 4) { col     = 4; }
    if (led_row < 0) { led_row = 0; } else if (led_row > 4) { led_row = 4; }

    static int last_col = -1;
    static int last_row = -1;
    if (col == last_col && led_row == last_row) {
        return;
    }
    last_col = col;
    last_row = led_row;

    struct mb_image img = {0};
    img.row[led_row] = (uint8_t)(0x10u >> col);  /* bit4=col0 … bit0=col4 */

    struct mb_display *disp = mb_display_get();
    mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, SYS_FOREVER_MS, &img, 1);
}

/* ── LED feedback ─────────────────────────────────────────────────────────── */

void demo_set_leds(const world_model_t *wm)
{
    int fresh = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_active && !e->is_self && !e->is_stale) {
            fresh++;
        }
    }

    static int last_fresh = -1;
    if (fresh != last_fresh) {
        static const char *const colors[] = { "red", "yellow", "green", "white" };
        LOG_INF("peers=%d  LED=%s", fresh,
                colors[fresh < 3 ? fresh : 3]);
        last_fresh = fresh;
    }

    if      (fresh >= 3) actuation_set_leds(255, 255, 255); /* white  — 3+ peers */
    else if (fresh == 2) actuation_set_leds(0,   200,   0); /* green  — 2 peers  */
    else if (fresh == 1) actuation_set_leds(200, 200,   0); /* yellow — 1 peer   */
    else                 actuation_set_leds(200,   0,   0); /* red    — isolated */
}
