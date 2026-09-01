/*
 * formation.c — CF21BL holonomic spring-field formation control
 *
 * See formation.h for the meters-based unit convention and why this
 * differs from examples/cutebot-formation's dead-reckoning approach.
 */

#include "formation.h"

#include <math.h>
#include <stddef.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(formation, LOG_LEVEL_DBG);

/* Per-tick trace throttle.  The tracking traces below fire once per
 * control tick (10 Hz per drone, 20 Hz for a pair) — at full rate they
 * were the single largest source of console splicing on a shared CRTP
 * address, and they destroyed the 1 Hz status timeline in flights 15/16.
 * Dropping the whole module to INF fixed the splicing but took min_d with
 * it, which is a safety number; throttling instead keeps the trace and
 * its min_d while cutting the volume tenfold.  DEMO_CONSOLE_VERBOSE
 * restores the full per-tick rate for single-drone debugging. */
#ifdef CONFIG_DEMO_CONSOLE_VERBOSE
#define DEMO_TRACE_DIV 1
#else
#define DEMO_TRACE_DIV 10      /* ~1 Hz at WM_CYCLE_MS = 100 */
#endif

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

/* A peer worth measuring against or steering by: active, not ourselves,
 * and actually localized.  ELEMENT_HEALTH_NO_POSITION means the element is
 * gossiping its zero-init placeholder because it has no fix yet — see the
 * flag's comment in csm.h for the flight-41 phantom-at-the-origin this
 * excludes.  Applied to the distance scan and to both force loops: a
 * position nobody has measured is not something to warn about OR steer by. */
static bool peer_has_position(const wm_entry_t *e)
{
    return e->is_active && !e->is_self &&
           (e->state.health_flags & ELEMENT_HEALTH_NO_POSITION) == 0;
}

/* See formation.h.  This is deliberately the ONLY place the two drive
 * functions get their min_dist from: their force loops still skip stale
 * peers, so leaving the bookkeeping inside those loops is exactly what
 * made the check inert.
 *
 * 3D distance, matching the force loops' metric — see the block comment in
 * demo_compute_drive for why z is folded in here too. */
float demo_min_separation(const world_model_t *wm,
                          const position_t *own_pos_m,
                          demo_sep_t *sep)
{
    float      min_dist_m = -1.0f;
    demo_sep_t s          = { .stale = false, .age_ms = 0 };

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!peer_has_position(e)) {
            continue;
        }

        float dx   = e->state.position.x - own_pos_m->x;
        float dy   = e->state.position.y - own_pos_m->y;
        float dz   = e->state.position.z - own_pos_m->z;
        float dist = sqrtf(dx * dx + dy * dy + dz * dz);

        if (min_dist_m < 0.0f || dist < min_dist_m) {
            min_dist_m = dist;
            s.stale    = e->is_stale;
            s.age_ms   = e->age_ms;
        }
    }

    if (sep != NULL) {
        *sep = s;
    }
    return min_dist_m;
}

float demo_compute_drive(const world_model_t *wm,
                          const position_t *own_pos_m,
                          demo_setpoint_t *target,
                          uint32_t dt_ms,
                          element_id_t own_id,
                          demo_sep_t *sep_out)
{
    float dt = (float)dt_ms * 0.001f;

    /* Measured FIRST, ahead of the hold-on-stale return below: freezing the
     * drive is the right response to a stale peer, but riding that freeze
     * out with no idea how close the peer was is the blind spot this
     * measurement exists to close (the old code returned -1.0f here). */
    float min_dist_m = demo_min_separation(wm, own_pos_m, sep_out);

    /* Hold in place if any active peer is stale — the spring field would
     * otherwise be computed against a peer position we no longer trust. */
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_active && !e->is_self && e->is_stale) {
            target->moving = false;
            return min_dist_m;
        }
    }

    float fx          = 0.0f;
    float fy          = 0.0f;
    int   peer_count  = 0;
    float peer_sum_x  = 0.0f;   /* for the formation centroid */
    float peer_sum_y  = 0.0f;
    float pair_dx     = 0.0f;   /* bearing to the (single) fresh peer — only
                                 * meaningful when peer_count ends up 1 */
    float pair_dy     = 0.0f;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!peer_has_position(e) || e->is_stale) {
            continue;
        }

        peer_sum_x += e->state.position.x;
        peer_sum_y += e->state.position.y;

        /* 3D peer distance — see position_distance()'s comment in csm.h
         * for why this now folds in z, including for this safety-adjacent
         * threshold (DEMO_MIN_SEP_M/EMERGENCY_K below), and why that is an
         * explicit, requested choice rather than an incidental one.
         *
         * The force VECTOR (fx/fy) stays 2D: altitude is a separate,
         * independently-held control loop (CONFIG_CF21BL_ALTITUDE_HOLD)
         * that this change does not touch. That means the force MAGNITUDE
         * (spring + repulsion, below) is computed from the 3D dist, but
         * the direction it gets applied in must be normalized by the 2D
         * (horizontal-only) magnitude, dist_xy — normalizing by the 3D
         * dist instead would silently shrink the horizontal push whenever
         * dz != 0 (dx/dist is not a unit vector once z is in dist), which
         * is not what "fold z into the distance metric" was asked for. */
        float dx      = e->state.position.x - own_pos_m->x;
        float dy      = e->state.position.y - own_pos_m->y;
        float dz      = e->state.position.z - own_pos_m->z;
        float dist    = sqrtf(dx * dx + dy * dy + dz * dz);
        float dist_xy = sqrtf(dx * dx + dy * dy);

        pair_dx = dx;
        pair_dy = dy;

        if (dist_xy < 0.01f) {
            continue;   /* coincident horizontally — push direction undefined */
        }

        /* Smooth spring toward DEMO_TARGET_SPACING_M. */
        float force = (dist - DEMO_TARGET_SPACING_M) * SPRING_K;

        /* Extra repulsion once inside the hard-floor separation — reacts
         * faster than the smooth spring term alone. */
        if (dist < DEMO_MIN_SEP_M) {
            force -= (DEMO_MIN_SEP_M - dist) * EMERGENCY_K;
        }

        fx += force * (dx / dist_xy);
        fy += force * (dy / dist_xy);
        peer_count++;
    }

    if (peer_count == 0) {
        /* No fresh peers: glide the target back over our own position and
         * hover (see DEMO_SOLO_GLIDE_MPS — a frozen far-away target made
         * the last drone standing chase it indefinitely). */
        float gx = own_pos_m->x - target->x;
        float gy = own_pos_m->y - target->y;
        float gd = sqrtf(gx * gx + gy * gy);
        if (gd > 0.01f) {
            float step = DEMO_SOLO_GLIDE_MPS * dt;
            if (step > gd) { step = gd; }
            target->x += gx / gd * step;
            target->y += gy / gd * step;
        }
        target->moving = false;
        return min_dist_m;
    }

    float force_mag = sqrtf(fx * fx + fy * fy);

    if (!target->moving && force_mag >= FORCE_START) {
        target->moving = true;
    } else if (target->moving && force_mag < FORCE_STOP) {
        target->moving = false;
    }

    /* Velocity assembled from up to three terms.  The spring term keeps
     * its start/stop hysteresis; the choreography terms below (rotation,
     * alignment — see formation.h) are deliberately NOT gated by it, since
     * they must act precisely when the springs are at equilibrium. */
    float vx = 0.0f;
    float vy = 0.0f;

    if (target->moving && force_mag >= 1e-6f) {
        float speed = clampf(force_mag * FORCE_TO_SPEED, 0.0f, DEMO_MAX_SPEED_MPS);
        vx += (fx / force_mag) * speed;
        vy += (fy / force_mag) * speed;
    }

    /* Formation centroid over self + fresh peers (real positions). */
    float cx = (peer_sum_x + own_pos_m->x) / (float)(peer_count + 1);
    float cy = (peer_sum_y + own_pos_m->y) / (float)(peer_count + 1);

    if (peer_count >= 2) {
        /* Triangle phase: orbit the centroid (v = ω ⟂ r, CCW). */
        vx += -DEMO_ROT_OMEGA_RADPS * (target->y - cy);
        vy +=  DEMO_ROT_OMEGA_RADPS * (target->x - cx);
    } else if (peer_count == 1) {
        /* Pair phase: rotate the pair about its centroid until its axis
         * lies along world X.  A torque, never a translation — see the
         * DEMO_ALIGN_ROT_RADPS block comment for why the earlier y-pull
         * version drove a north–south pair into a spring standoff. */
        float phi   = atan2f(pair_dy, pair_dx);
        float omega = -DEMO_ALIGN_ROT_RADPS * sinf(2.0f * phi);
        vx += -omega * (target->y - cy);
        vy +=  omega * (target->x - cx);
    }

    /* Centroid anchor (see DEMO_ANCHOR_* block comment): identical for
     * every drone → pure translation of the whole formation toward the
     * anchor point, no shape distortion. */
    vx += DEMO_ANCHOR_K * (DEMO_ANCHOR_X_M - cx);
    vy += DEMO_ANCHOR_K * (DEMO_ANCHOR_Y_M - cy);

    float v_mag = sqrtf(vx * vx + vy * vy);
    if (v_mag < 1e-6f) {
        return min_dist_m;
    }
    if (v_mag > DEMO_MAX_SPEED_MPS) {
        vx *= DEMO_MAX_SPEED_MPS / v_mag;
        vy *= DEMO_MAX_SPEED_MPS / v_mag;
    }

    target->x = clampf(target->x + vx * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);
    target->y = clampf(target->y + vy * dt, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);

    /* Leash the target to the drone's real position (see
     * DEMO_TARGET_LEASH_M): every term above moves the TARGET, but the
     * forces are computed from REAL positions — once the body can't
     * follow, an unleashed target's relationship to the field is
     * fiction. */
    {
        float lx = target->x - own_pos_m->x;
        float ly = target->y - own_pos_m->y;
        float ld = sqrtf(lx * lx + ly * ly);
        if (ld > DEMO_TARGET_LEASH_M) {
            target->x = own_pos_m->x + lx / ld * DEMO_TARGET_LEASH_M;
            target->y = own_pos_m->y + ly / ld * DEMO_TARGET_LEASH_M;
        }
    }

    static int drive_trace_div;
    if (++drive_trace_div >= DEMO_TRACE_DIV) {
        drive_trace_div = 0;
        LOG_DBG("id=%u fx=%.2f fy=%.2f |f|=%.2f v=(%.2f,%.2f) tgt=(%.2f,%.2f) peers=%d min_d=%.2f",
                (unsigned)own_id,
                (double)fx, (double)fy, (double)force_mag,
                (double)vx, (double)vy,
                (double)target->x, (double)target->y,
                peer_count, (double)min_dist_m);
    }

    return min_dist_m;
}

bool demo_deconflict_point(const world_model_t *wm, float *x, float *y)
{
    bool moved = false;

    /* Horizontal-only, deliberately: this places a LANDING/return point on
     * the floor, and z is a separately-held loop that a landed peer is not
     * on anyway.  (The in-flight separation metric folds in z; see
     * demo_min_separation.  Different question, different geometry.)
     *
     * A bounded relaxation, not iterated to convergence: clearing one peer
     * can push the point inside another's floor, but in a crowded arena
     * there may be no clear point at all, and returning a spot 0.4 m from a
     * peer beats spinning here.  Four passes settles the 2-3 element case. */
    for (int pass = 0; pass < 4; pass++) {
        bool clear = true;

        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm->entries[i];
            if (!peer_has_position(e)) {
                continue;
            }

            float dx = *x - e->state.position.x;
            float dy = *y - e->state.position.y;
            float d  = sqrtf(dx * dx + dy * dy);
            if (d >= DEMO_MIN_SEP_M) {
                continue;
            }

            /* Point sits on top of the peer: every direction is equally
             * good, so pick a deterministic one rather than dividing by
             * ~zero. */
            if (d < 0.01f) {
                dx = 1.0f;
                dy = 0.0f;
                d  = 1.0f;
            }

            *x    = e->state.position.x + dx / d * DEMO_MIN_SEP_M;
            *y    = e->state.position.y + dy / d * DEMO_MIN_SEP_M;
            clear = false;
            moved = true;
        }

        if (clear) {
            break;
        }
    }

    if (moved) {
        *x = clampf(*x, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);
        *y = clampf(*y, -DEMO_ARENA_LIMIT_M, DEMO_ARENA_LIMIT_M);
    }
    return moved;
}

/* ── Choreo tracking (see formation.h) ───────────────────────────────────── */

float demo_choreo_track(const world_model_t *wm,
                        const position_t *own_pos_m,
                        demo_setpoint_t *target,
                        float cmd_x, float cmd_y,
                        uint32_t dt_ms,
                        element_id_t own_id,
                        demo_sep_t *sep_out)
{
    float dt = (float)dt_ms * 0.001f;

    /* Separation is measured over every ACTIVE peer (stale included);
     * emergency repulsion below is applied for FRESH peers only.  That
     * split is the whole point of this function's share of the fix: unlike
     * demo_compute_drive there is no hold-on-stale freeze here, so a peer
     * sitting in the stale-but-not-expired band used to mean flying with
     * the DEMO_MIN_SEP_M check switched off entirely. */
    float min_dist_m = demo_min_separation(wm, own_pos_m, sep_out);

    /* Emergency repulsion over fresh peers.  Same constants and force
     * convention as the spring field, but no attraction term — the L6
     * directive owns where we are going. */
    float fx = 0.0f;
    float fy = 0.0f;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (!peer_has_position(e) || e->is_stale) {
            continue;
        }

        /* Same 3D-distance / 2D-force-direction split as demo_compute_drive
         * above — see that function's comment for the full rationale. */
        float dx      = e->state.position.x - own_pos_m->x;
        float dy      = e->state.position.y - own_pos_m->y;
        float dz      = e->state.position.z - own_pos_m->z;
        float dist    = sqrtf(dx * dx + dy * dy + dz * dz);
        float dist_xy = sqrtf(dx * dx + dy * dy);

        if (dist_xy < 0.01f) {
            continue;   /* coincident horizontally — push direction undefined */
        }
        if (dist < DEMO_MIN_SEP_M) {
            float force = (DEMO_MIN_SEP_M - dist) * EMERGENCY_K;
            fx -= force * (dx / dist_xy);
            fy -= force * (dy / dist_xy);
        }
    }

    /* Approach velocity toward the directive point.  ad/dt (not just the
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

    /* Same leash rationale as the spring field: never command further than
     * the body can meaningfully chase. */
    {
        float lx = target->x - own_pos_m->x;
        float ly = target->y - own_pos_m->y;
        float ld = sqrtf(lx * lx + ly * ly);
        if (ld > DEMO_TARGET_LEASH_M) {
            target->x = own_pos_m->x + lx / ld * DEMO_TARGET_LEASH_M;
            target->y = own_pos_m->y + ly / ld * DEMO_TARGET_LEASH_M;
        }
    }

    static int choreo_trace_div;
    if (++choreo_trace_div >= DEMO_TRACE_DIV) {
        choreo_trace_div = 0;
        LOG_DBG("id=%u choreo cmd=(%.2f,%.2f) tgt=(%.2f,%.2f) min_d=%.2f",
                (unsigned)own_id,
                (double)cmd_x, (double)cmd_y,
                (double)target->x, (double)target->y,
                (double)min_dist_m);
    }

    return min_dist_m;
}

/* ── Signal feedback (LED) ────────────────────────────────────────────────── */

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

    substrate_signal_t sig;
    if      (active == 0)      sig = SUBSTRATE_SIGNAL_FAILED;
    else if (fresh  <  active) sig = SUBSTRATE_SIGNAL_DEGRADED;
    else                       sig = SUBSTRATE_SIGNAL_ACTIVE;
    substrate_set_signal(sig);
}
