/*
 * formation.c — Demo: Choreo-driven differential-drive control + dead-reckoning
 */

#include "formation.h"

#include <math.h>
#include <zephyr/display/mb_display.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(formation, LOG_LEVEL_DBG);

#define M_PI_F      3.14159265f

/* Maps resultant force magnitude to motor speed percent. */
#define FORCE_TO_SPEED  0.6f

/* Steering gain: scales lateral force to differential turn. */
#define TURN_GAIN       12.0f

/* Minimum motor % that overcomes stiction (measured). Any non-zero
 * speed command is snapped up to this so the motors actually turn. */
#define MIN_STICTION    22

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * demo_arena_fence — veto the OUTWARD component of a commanded force
 * once the (dead-reckoning) position estimate is within
 * DEMO_ARENA_FENCE_MARGIN of an edge. Component-wise, not radial: near a
 * corner this can leave a force that still has an inward-diagonal
 * component, which reads as "sliding along the wall" rather than a clean
 * bounce — acceptable for a safety backstop, not a precision behavior.
 *
 * See DEMO_ARENA_FENCE_MARGIN's doc (formation.h) for what this can and
 * cannot guarantee: it bounds commanded travel relative to THIS robot's
 * own estimate, which is not the same as bounding it relative to the
 * physical board if the estimate has already drifted. */
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

/* ── Odometry ─────────────────────────────────────────────────────────────── */

void demo_odometry_init(demo_odometry_t *odo, float x, float y)
{
    odo->x       = x;
    odo->y       = y;
    odo->heading = 0.0f;
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

    /* Clamp the STORED ESTIMATE to world bounds — this keeps the number
     * gossiped to peers sane, nothing more. It does NOT stop the robot:
     * speed_cmd/rate_cmd are computed independently (demo_track_target)
     * from whatever force this tick's estimate and target produce, and
     * this clamp cannot see or influence that. If
     * the commanded force still points outward when the estimate pins
     * here, the estimate stops advancing while the real robot keeps
     * driving — see demo_arena_fence(), which is the actual command-
     * level guard against that. */
    if (odo->x < 0.0f)      { odo->x = 0.0f; }
    if (odo->x > WORLD_SIZE) { odo->x = WORLD_SIZE; }
    if (odo->y < 0.0f)      { odo->y = 0.0f; }
    if (odo->y > WORLD_SIZE) { odo->y = WORLD_SIZE; }
}

/* ── Grid-based drift correction ──────────────────────────────────────────── */

void demo_grid_correct(demo_odometry_t *odo, bool crossed)
{
    if (!crossed) {
        return;
    }

    float ac = fabsf(cosf(odo->heading));
    float as = fabsf(sinf(odo->heading));

    if (ac < DEMO_GRID_AXIS_COS_MIN && as < DEMO_GRID_AXIS_COS_MIN) {
        /* Too close to a 45-degree heading to know whether this was a
         * row or column crossing — skip rather than guess wrong. */
        return;
    }

    if (ac >= as) {
        /* Moving mostly along x: the line just crossed runs along y
         * (constant x) — correct x. */
        odo->x = roundf(odo->x / DEMO_SQUARE_UNITS) * DEMO_SQUARE_UNITS;
    } else {
        odo->y = roundf(odo->y / DEMO_SQUARE_UNITS) * DEMO_SQUARE_UNITS;
    }
}

/* ── Grid-based heading correction (see formation.h's doc) ───────────────── */

float demo_grid_heading_correct(demo_odometry_t *odo, float speed_norm,
                                 bool left_entered, uint32_t left_entry_ms,
                                 bool right_entered, uint32_t right_entry_ms)
{
    if (!left_entered || !right_entered) {
        /* Need both sensors' edges from the SAME poll to form a pair —
         * see the header doc for why this is deliberately stateless.
         * Only log when at least ONE side fired (a real, if incomplete,
         * event) — logging every tick neither sensor sees anything would
         * be pure per-tick noise at LOG_INF, unlike the rare cases below. */
        if (left_entered || right_entered) {
            LOG_INF("heading correct: SKIP unpaired entry (L=%d R=%d)",
                    (int)left_entered, (int)right_entered);
        }
        return 0.0f;
    }

    float ac = fabsf(cosf(odo->heading));
    float as = fabsf(sinf(odo->heading));
    if (ac < DEMO_GRID_AXIS_COS_MIN && as < DEMO_GRID_AXIS_COS_MIN) {
        LOG_INF("heading correct: SKIP off-axis heading=%.1f deg",
                (double)(odo->heading * (180.0f / M_PI_F)));
        return 0.0f;   /* same diagonal-ambiguity gate as demo_grid_correct() */
    }

    int32_t dt_ms = (int32_t)(right_entry_ms - left_entry_ms);
    if (dt_ms > (int32_t)DEMO_HEADING_MAX_PAIR_MS ||
        dt_ms < -(int32_t)DEMO_HEADING_MAX_PAIR_MS) {
        LOG_INF("heading correct: SKIP pair too far apart dt=%d ms (max %u)",
                (int)dt_ms, (unsigned)DEMO_HEADING_MAX_PAIR_MS);
        return 0.0f;   /* too far apart to plausibly be the same crossing */
    }

    float v = fabsf(speed_norm) * DEMO_MAX_SPEED;   /* logical units/s */
    if (v < 0.01f) {
        LOG_INF("heading correct: SKIP not moving (speed_norm=%.3f)",
                (double)speed_norm);
        return 0.0f;   /* not moving — no distance to convert the delta into */
    }

    float dt_s        = (float)dt_ms * 0.001f;
    float heading_err = -(v * dt_s) / DEMO_LINE_SENSOR_SEPARATION;

    if (fabsf(heading_err) > DEMO_HEADING_MAX_CORRECTION_RAD) {
        LOG_INF("heading correct: SKIP implausible err=%.1f deg (dt=%d ms v=%.2f)",
                (double)(heading_err * (180.0f / M_PI_F)), (int)dt_ms, (double)v);
        return 0.0f;   /* implausible for a real crossing — likely a mispair */
    }

    /* Full re-anchor to (nearest cardinal axis) + measured error, not an
     * incremental nudge to whatever odo->heading currently is — see the
     * header doc for why. */
    float nearest_axis = roundf(odo->heading / (M_PI_F * 0.5f)) * (M_PI_F * 0.5f);
    odo->heading = nearest_axis + heading_err;

    while (odo->heading >  M_PI_F) { odo->heading -= 2.0f * M_PI_F; }
    while (odo->heading < -M_PI_F) { odo->heading += 2.0f * M_PI_F; }

    /* LOG_INF, not LOG_DBG: this fires only on an actual paired crossing
     * (sparse, not per-tick noise like formation.c's other LOG_DBG
     * traces), and being visible at the default log level is the whole
     * point on a bench test verifying this correction's sign — see
     * formation.h's doc. Degrees, not radians, so a bench test doesn't
     * need mental radian math to read the sign/magnitude. */
    LOG_INF("heading correct: R-L dt=%d ms v=%.2f units/s err=%.1f deg "
            "-> heading=%.1f deg (was axis %.1f deg)",
            (int)dt_ms, (double)v,
            (double)(heading_err * (180.0f / M_PI_F)),
            (double)(odo->heading * (180.0f / M_PI_F)),
            (double)(nearest_axis * (180.0f / M_PI_F)));

    return heading_err;
}

/* ── Force → twist projection (shared by demo_drive_straight and
 * demo_track_target) ─────────────────────────────────────────────────────
 *
 * Projects a world-frame force/velocity vector (fx, fy) onto the robot
 * frame and converts it to a normalized [-1,1] speed/rate twist, applying
 * the same stiction floor both callers need — written once since it has
 * nothing to do with WHERE the force came from.
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

/* ── Straight-line drive (isolated motion-primitive testing) ─────────────── */

void demo_drive_straight(const demo_odometry_t *odo,
                          float *speed_out, float *rate_out)
{
    float fx = DEMO_TRACK_MAX_FORCE * cosf(odo->heading);
    float fy = DEMO_TRACK_MAX_FORCE * sinf(odo->heading);

    /* Same fence demo_track_target() uses — the only thing in this
     * function's whole path that can stop it besides running off the
     * board unbounded. No peer repulsion here at all: this function
     * never looks at wm, by design (see its header doc). */
    demo_arena_fence(odo, &fx, &fy);
    demo_force_to_twist(odo, fx, fy, 22.0f, 15.0f, speed_out, rate_out);
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
    bool repelled = false;
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
        repelled = true;
    }

    /* See DEMO_TRACK_REPEL_DEADBAND's doc (formation.h) — holds rather
     * than letting demo_force_to_twist's stiction floor turn a near-
     * cancelled residual into a full-speed lurch that flips sign again
     * next tick. */
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
