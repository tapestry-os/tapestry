/*
 * formation.c — Demo: spring-field formation control + dead-reckoning
 */

#include "formation.h"

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

/* Minimum motor % that overcomes stiction (measured). Any non-zero
 * speed command is snapped up to this so the motors actually turn. */
#define MIN_STICTION    22

/* Hysteresis thresholds on net spring force magnitude.
 * A stopped robot only starts moving when force exceeds FORCE_START.
 * A moving robot stops when force drops below FORCE_STOP.
 * The gap between them prevents oscillation near equilibrium: a small
 * correction that slightly overshoots does not immediately trigger a
 * counter-correction, and gossip-propagated micro-adjustments from
 * neighbors do not restart a robot that has just settled. */
#define FORCE_STOP   25.0f   /* ~3 units / 24 mm from equilibrium (800 mm arena)  */
#define FORCE_START  50.0f   /* > FORCE_STOP; initial cluster forces are 300+ so FORCE_START
                              * is easily exceeded at boot */

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
    float dt       = (float)dt_ms * 0.001f;
    float v_center = speed_norm * DEMO_MAX_SPEED;   /* logical units/s */
    float omega    = rate_norm  * DEMO_MAX_OMEGA;   /* rad/s           */

    odo->heading += omega * dt;

    /* Normalize to (-π, π] */
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

/* ── Force → twist projection (shared by demo_compute_drive and
 * demo_track_target) ─────────────────────────────────────────────────────
 *
 * Projects a world-frame force/velocity vector (fx, fy) onto the robot
 * frame and converts it to a normalized [-1,1] speed/rate twist, applying
 * the same stiction floor both callers need — this is the part of the
 * old demo_compute_drive that has nothing to do with WHERE the force came
 * from (peer springs vs. a single tracked target), so it is written once.
 *
 *   Robot forward axis in world frame: (cos h, sin h)
 *   Robot left    axis in world frame: (-sin h, cos h)
 *   f_fwd > 0 → move forward
 *   f_lat > 0 → force is to the robot's left → turn left
 *
 * Both wheels must clear stiction independently. Substrate computes:
 * left = speed - turn, right = speed + turn. The inner wheel
 * (speed - |turn|) stalls if speed <= |turn| + MIN_STICTION, causing an
 * uncontrolled pivot and rapid heading error in dead-reckoning. Fix: boost
 * forward speed so the inner wheel always reaches MIN_STICTION. For
 * in-place turns (speed==0), snap the turn command itself to MIN_STICTION
 * so both wheels overcome stiction.
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

/* ── Formation control ────────────────────────────────────────────────────── */

void demo_compute_drive(const world_model_t *wm,
                         demo_odometry_t *odo,
                         float *speed_out, float *rate_out)
{
    /* Require all active peers to be fresh before moving.
     * A stale entry means the world model is incomplete — forces computed
     * from partial data are asymmetric and will drive the formation wrong.
     * This makes the green/yellow LED state the hard gate on actuation. */
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
         * discovery, and wandering corrupts the dead-reckoning origin.
         * Reset moving so the robot re-evaluates force when peers return. */
        odo->moving = false;
        *speed_out  = 0.0f;
        *rate_out   = 0.0f;
        return;
    }

    /* Hysteresis: require a larger force to start moving than to stop.
     * Prevents oscillation and gossip-cascade near equilibrium. */
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

    demo_force_to_twist(odo, fx, fy, 22.0f, 15.0f, speed_out, rate_out);

    LOG_DBG("fx=%.2f fy=%.2f spd=%.2f rate=%.2f peers=%d",
            (double)fx, (double)fy,
            (double)*speed_out, (double)*rate_out, peer_count);
}

/* ── Choreo tracking (see formation.h) ───────────────────────────────────── */

void demo_track_target(const world_model_t *wm,
                        const demo_odometry_t *odo,
                        float target_x, float target_y,
                        float *speed_out, float *rate_out)
{
    float dx   = target_x - odo->x;
    float dy   = target_y - odo->y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < DEMO_TRACK_ARRIVE_EPS) {
        /* Close enough — stop rather than let the residual attraction
         * force get overridden by demo_force_to_twist's stiction floor
         * into a nonzero creep (see formation.h's DEMO_TRACK_ARRIVE_EPS
         * doc). */
        *speed_out = 0.0f;
        *rate_out  = 0.0f;
        return;
    }

    /* Attraction: full DEMO_TRACK_MAX_FORCE beyond DEMO_TRACK_SLOW_RADIUS,
     * ramping linearly to 0 inside it (trapezoidal approach profile). */
    float mag = (dist > DEMO_TRACK_SLOW_RADIUS)
                ? DEMO_TRACK_MAX_FORCE
                : DEMO_TRACK_MAX_FORCE * (dist / DEMO_TRACK_SLOW_RADIUS);
    float fx = mag * (dx / dist);
    float fy = mag * (dy / dist);

    /* Emergency repulsion backstop — see formation.h's doc. Repulsion
     * only (no attraction term): the target above already IS the
     * attraction. */
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!e->is_active || e->is_self || e->is_stale) {
            continue;
        }

        float pdx  = e->state.position.x - odo->x;
        float pdy  = e->state.position.y - odo->y;
        float pdist = sqrtf(pdx * pdx + pdy * pdy);

        if (pdist < 0.01f || pdist >= DEMO_TRACK_MIN_SEP) {
            continue;
        }

        float force = (DEMO_TRACK_MIN_SEP - pdist) * DEMO_TRACK_EMERGENCY_K;
        fx -= force * (pdx / pdist);
        fy -= force * (pdy / pdist);
    }

    demo_force_to_twist(odo, fx, fy, 22.0f, 15.0f, speed_out, rate_out);

    LOG_DBG("choreo cmd=(%.2f,%.2f) dist=%.2f spd=%.2f rate=%.2f",
            (double)target_x, (double)target_y,
            (double)dist, (double)*speed_out, (double)*rate_out);
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

void demo_set_leds(const world_model_t *wm, substrate_signal_t step_indicator)
{
    if (step_indicator != SUBSTRATE_SIGNAL_NONE) {
        substrate_set_signal(step_indicator);
        return;
    }

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

    static int last_fresh  = -1;
    static int last_active = -1;
    if (fresh != last_fresh || active != last_active) {
        LOG_INF("peers fresh=%d active=%d", fresh, active);
        last_fresh  = fresh;
        last_active = active;
    }

    substrate_signal_t sig;
    if      (active == 0)      sig = SUBSTRATE_SIGNAL_FAILED;    /* isolated        */
    else if (fresh  <  active) sig = SUBSTRATE_SIGNAL_DEGRADED;  /* some stale      */
    else                       sig = SUBSTRATE_SIGNAL_ACTIVE;    /* all fresh       */
    substrate_set_signal(sig);
}
