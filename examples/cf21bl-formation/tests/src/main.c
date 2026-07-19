/*
 * main.c — formation.c spring-field unit tests (no hardware, no radio)
 *
 * Every non-trivial test here is a regression test for a bug that was
 * found IN FLIGHT during the 2026-07 three-drone demo campaign:
 *
 *   - target leash            → an emergency-repulsion episode detached a
 *                               virtual target 2.7 m from its drone, which
 *                               then chased a point it could never reach
 *   - alignment-as-torque     → the first (y-pull) alignment drove a
 *                               north–south pair into a spring standoff at
 *                               min_d 0.34 m (emergency spring the only
 *                               thing preventing contact)
 *   - solo target glide       → after both peers landed, the survivor
 *                               chased a target stranded far away by the
 *                               preceding chaos until physically restrained
 *   - hold-on-stale           → transient radio-contention staleness must
 *                               freeze the drive, not distort the field
 *
 * The multi-agent tests use a perfect-tracking closed loop (each body
 * teleports onto its own target every cycle) — crude, but it exercises the
 * exact term interactions (springs + rotation + alignment + anchor) that
 * produced the flight failures.
 *
 * Build:  west build -p always -b native_sim tapestry/examples/cf21bl-formation/tests
 *         (on a 64-bit-only host, e.g. the aarch64 Pi: -b native_sim/native/64)
 * Run:    ./build/zephyr/zephyr.exe
 */

#include <zephyr/ztest.h>
#include <math.h>
#include <string.h>

#include "formation.h"

#define DT_MS      100u                       /* WM_CYCLE_MS-equivalent */
#define DT_S       ((float)DT_MS * 0.001f)
#define EPS        0.001f

/* ── World-model scaffolding ──────────────────────────────────────────────── */

static world_model_t wm;

static void wm_reset(void)
{
    memset(&wm, 0, sizeof(wm));
}

static void wm_set_peer(int slot, float x, float y, bool stale)
{
    wm.entries[slot].is_active        = true;
    wm.entries[slot].is_self          = false;
    wm.entries[slot].is_stale         = stale;
    wm.entries[slot].state.id         = (element_id_t)slot;
    wm.entries[slot].state.position.x = x;
    wm.entries[slot].state.position.y = y;
}

static float dist2d(float ax, float ay, float bx, float by)
{
    return sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

/* ── Single-drone field behavior ──────────────────────────────────────────── */

ZTEST(formation_field, test_no_peers_returns_no_distance)
{
    wm_reset();
    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    float d = demo_compute_drive(&wm, &own, &tgt, DT_MS, 0);
    zassert_true(d < 0.0f, "no fresh peers must report min_d = -1");
}

ZTEST(formation_field, test_hold_on_stale_freezes_target)
{
    wm_reset();
    wm_set_peer(1, 1.0f, 0.0f, false);
    wm_set_peer(2, 0.3f, 0.3f, true);   /* active but stale → freeze all */

    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.2f, 0.2f);

    float d = demo_compute_drive(&wm, &own, &tgt, DT_MS, 0);
    zassert_true(d < 0.0f, "stale peer must report min_d = -1");
    zassert_within(tgt.x, 0.2f, EPS, "target x moved during stale hold");
    zassert_within(tgt.y, 0.2f, EPS, "target y moved during stale hold");
}

ZTEST(formation_field, test_spring_repels_when_too_close)
{
    wm_reset();
    wm_set_peer(1, 0.5f, 0.0f, false);  /* at the hard floor: strong repel */

    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    (void)demo_compute_drive(&wm, &own, &tgt, DT_MS, 0);
    zassert_true(tgt.x < 0.0f, "target must flee away from a too-close peer");
}

ZTEST(formation_field, test_spring_attracts_when_too_far)
{
    wm_reset();
    wm_set_peer(1, 1.8f, 0.0f, false);  /* 0.8 m past spacing: attract */

    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    (void)demo_compute_drive(&wm, &own, &tgt, DT_MS, 0);
    zassert_true(tgt.x > 0.0f, "target must move toward a too-distant peer");
}

ZTEST(formation_field, test_min_dist_reported)
{
    wm_reset();
    wm_set_peer(1, 0.2f, 0.0f, false);
    wm_set_peer(2, 1.5f, 0.0f, false);

    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    float d = demo_compute_drive(&wm, &own, &tgt, DT_MS, 0);
    zassert_within(d, 0.2f, EPS, "min_d must be the closest fresh peer");
}

/* ── Regression: target leash ─────────────────────────────────────────────── */
/* Body pinned (can't fly), peer parked inside the hard floor: the target
 * flees at max speed every cycle.  Pre-leash this integrated without bound
 * (flight: 2.67 m detachment).  The target must never leave the leash. */
ZTEST(formation_field, test_target_leash_bounds_detachment)
{
    wm_reset();
    wm_set_peer(1, 0.2f, 0.0f, false);

    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    for (int i = 0; i < 600; i++) {        /* 60 s of continuous fleeing */
        (void)demo_compute_drive(&wm, &own, &tgt, DT_MS, 0);
        float detach = dist2d(tgt.x, tgt.y, own.x, own.y);
        zassert_true(detach <= DEMO_TARGET_LEASH_M + EPS,
                     "target detached %.2f m from body (leash %.2f) at i=%d",
                     (double)detach, (double)DEMO_TARGET_LEASH_M, i);
    }
}

/* ── Regression: solo glide ───────────────────────────────────────────────── */
ZTEST(formation_field, test_solo_target_glides_home)
{
    wm_reset();                             /* zero peers */
    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.5f, -0.4f);  /* stranded by prior chaos */

    float prev = dist2d(tgt.x, tgt.y, own.x, own.y);
    for (int i = 0; i < 100; i++) {         /* 10 s */
        (void)demo_compute_drive(&wm, &own, &tgt, DT_MS, 0);
        float d = dist2d(tgt.x, tgt.y, own.x, own.y);
        zassert_true(d <= prev + EPS, "solo glide must be monotonic");
        prev = d;
    }
    zassert_true(prev < 0.05f,
                 "target still %.2f m from body after 10 s of solo glide",
                 (double)prev);
}

/* ── Multi-agent closed-loop tests (perfect tracking: body := target) ─────── */

typedef struct {
    position_t      pos;
    demo_setpoint_t tgt;
} sim_drone_t;

/* One cycle: each drone sees the others' CURRENT bodies, updates its own
 * target, then tracks it perfectly. */
static void sim_step(sim_drone_t *d, int n)
{
    for (int i = 0; i < n; i++) {
        wm_reset();
        for (int j = 0; j < n; j++) {
            if (j != i) {
                wm_set_peer(j, d[j].pos.x, d[j].pos.y, false);
            }
        }
        (void)demo_compute_drive(&wm, &d[i].pos, &d[i].tgt, DT_MS,
                                 (element_id_t)i);
    }
    for (int i = 0; i < n; i++) {
        d[i].pos.x = d[i].tgt.x;
        d[i].pos.y = d[i].tgt.y;
    }
}

static void sim_init(sim_drone_t *d, float x, float y)
{
    d->pos.x = x;
    d->pos.y = y;
    demo_setpoint_init(&d->tgt, x, y);
}

/* Regression: the y-pull alignment squeezed a north–south pair to
 * min_d 0.34 m in flight.  The torque version must rotate the pair toward
 * the world-X axis WITHOUT the separation ever collapsing. */
ZTEST(formation_field, test_alignment_rotates_pair_without_squeeze)
{
    sim_drone_t d[2];
    /* 15 deg off exactly north–south (φ=90° is the unstable equilibrium —
     * in flight, noise seeds it; in a noiseless sim we must not start ON it) */
    sim_init(&d[0], 0.5f - 0.5f * sinf(0.26f), 0.15f - 0.5f * cosf(0.26f));
    sim_init(&d[1], 0.5f + 0.5f * sinf(0.26f), 0.15f + 0.5f * cosf(0.26f));

    float min_sep = 10.0f;
    for (int i = 0; i < 600; i++) {         /* 60 s */
        sim_step(d, 2);
        float sep = dist2d(d[0].pos.x, d[0].pos.y, d[1].pos.x, d[1].pos.y);
        if (sep < min_sep) { min_sep = sep; }
    }

    zassert_true(min_sep > 0.6f,
                 "pair squeezed to %.2f m during alignment (flight bug was 0.34)",
                 (double)min_sep);

    float bearing = atan2f(d[1].pos.y - d[0].pos.y, d[1].pos.x - d[0].pos.x);
    float off_x   = fabsf(bearing);         /* want ≈ 0 or ≈ π */
    if (off_x > 1.5708f) { off_x = 3.14159f - off_x; }
    zassert_true(off_x < 0.35f,             /* within 20 deg of world X */
                 "pair axis still %.1f deg off world X after 60 s",
                 (double)(off_x * 57.2958f));
}

/* Rotation phase: three drones must orbit without the shape distorting —
 * pairwise spacing preserved while the formation actually turns. */
ZTEST(formation_field, test_rotation_turns_triangle_without_distortion)
{
    sim_drone_t d[3];
    /* equilateral, side 1.0, centroid at the anchor point */
    sim_init(&d[0], 0.5f - 0.5f, 0.15f - 0.2887f);
    sim_init(&d[1], 0.5f + 0.5f, 0.15f - 0.2887f);
    sim_init(&d[2], 0.5f,        0.15f + 0.5774f);

    float cx0 = 0.5f, cy0 = 0.15f;
    float bearing0 = atan2f(d[2].pos.y - cy0, d[2].pos.x - cx0);

    for (int i = 0; i < 100; i++) {         /* 10 s */
        sim_step(d, 3);
        for (int a = 0; a < 3; a++) {
            for (int b = a + 1; b < 3; b++) {
                float sep = dist2d(d[a].pos.x, d[a].pos.y,
                                   d[b].pos.x, d[b].pos.y);
                zassert_true(sep > 0.85f && sep < 1.2f,
                             "triangle distorted: side %.2f m at i=%d",
                             (double)sep, i);
            }
        }
    }

    float cx = (d[0].pos.x + d[1].pos.x + d[2].pos.x) / 3.0f;
    float cy = (d[0].pos.y + d[1].pos.y + d[2].pos.y) / 3.0f;
    float turned = atan2f(d[2].pos.y - cy, d[2].pos.x - cx) - bearing0;
    while (turned >  3.14159f) { turned -= 6.28318f; }
    while (turned < -3.14159f) { turned += 6.28318f; }
    zassert_true(fabsf(turned) > 0.5f,      /* expect ~ω·t = 1.2 rad */
                 "triangle barely rotated (%.2f rad in 10 s, expected ~1.2)",
                 (double)turned);
}

/* Anchor: a displaced formation must translate back toward the anchor
 * point without its internal spacing changing. */
ZTEST(formation_field, test_anchor_recenters_formation)
{
    sim_drone_t d[2];
    sim_init(&d[0], 1.5f, 1.5f);            /* pair centered ~2 m from anchor */
    sim_init(&d[1], 2.5f, 1.5f);

    for (int i = 0; i < 200; i++) {         /* 20 s */
        sim_step(d, 2);
    }

    float cx = (d[0].pos.x + d[1].pos.x) * 0.5f;
    float cy = (d[0].pos.y + d[1].pos.y) * 0.5f;
    float to_anchor = dist2d(cx, cy, DEMO_ANCHOR_X_M, DEMO_ANCHOR_Y_M);
    zassert_true(to_anchor < 0.6f,
                 "centroid still %.2f m from anchor after 20 s", (double)to_anchor);

    float sep = dist2d(d[0].pos.x, d[0].pos.y, d[1].pos.x, d[1].pos.y);
    zassert_true(sep > 0.8f && sep < 1.3f,
                 "anchor translation distorted spacing to %.2f m", (double)sep);
}

/* ── demo_choreo_track (choreo-mode target tracking) ──────────────────────── */

ZTEST(formation_field, test_choreo_track_converges_onto_cmd)
{
    wm_reset();
    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    for (int i = 0; i < 100; i++) {         /* 10 s at 0.3 m/s: plenty */
        (void)demo_choreo_track(&wm, &own, &tgt, 0.5f, 0.2f, DT_MS, 0);
        own = (position_t){ tgt.x, tgt.y }; /* perfect tracking (leash) */
    }
    zassert_within(tgt.x, 0.5f, 0.01f, "target x did not converge (%.3f)",
                   (double)tgt.x);
    zassert_within(tgt.y, 0.2f, 0.01f, "target y did not converge (%.3f)",
                   (double)tgt.y);
}

ZTEST(formation_field, test_choreo_track_repulsion_and_min_dist)
{
    wm_reset();
    wm_set_peer(1, 0.25f, 0.0f, false);     /* inside DEMO_MIN_SEP_M */

    position_t own = { 0.0f, 0.0f };
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    float d = demo_choreo_track(&wm, &own, &tgt, 0.0f, 0.0f, DT_MS, 0);
    zassert_within(d, 0.25f, EPS, "min_d must report the close peer");
    zassert_true(tgt.x < 0.0f,
                 "target must be pushed away from a too-close peer");
}

ZTEST(formation_field, test_choreo_track_leash_bounds_target)
{
    wm_reset();
    position_t own = { 0.0f, 0.0f };        /* body pinned */
    demo_setpoint_t tgt;
    demo_setpoint_init(&tgt, 0.0f, 0.0f);

    for (int i = 0; i < 200; i++) {         /* chase an unreachable cmd */
        (void)demo_choreo_track(&wm, &own, &tgt, 2.5f, 0.0f, DT_MS, 0);
        float detach = dist2d(tgt.x, tgt.y, own.x, own.y);
        zassert_true(detach <= DEMO_TARGET_LEASH_M + EPS,
                     "target detached %.2f m > leash", (double)detach);
    }
}

ZTEST_SUITE(formation_field, NULL, NULL, NULL, NULL, NULL);

/* ── Choreo script (L6 BSE + L7 Choreographer, singleton per process) ─────── */
/*
 * These mirror the flight script in ../src/main.c: hold → exchange → hold
 * (bow) → quiescence.  bse.c/choreo.c are per-element singletons, so the
 * "partner" is emulated by mirroring this element's body through the
 * formation centroid — exactly the symmetric arc a real second drone flies
 * (both run the same CCW rule, so the pair stays antipodal).
 */

#include <tapestry/choreo.h>

static void wm_set_self(int slot, element_id_t id, float x, float y)
{
    wm.entries[slot].is_active        = true;
    wm.entries[slot].is_self          = true;
    wm.entries[slot].is_stale         = false;
    wm.entries[slot].state.id         = id;
    wm.entries[slot].state.position.x = x;
    wm.entries[slot].state.position.y = y;
}

ZTEST(choreo_script, test_swap_script_end_to_end)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD },
          .max_duration_ms = 2000 },
        { .goal = { .type = CHOREO_GOAL_EXCHANGE,
                    .achieve_eps = 0.25f, .achieve_hold_ms = 1000 },
          .max_duration_ms = 45000, .advance_on_achieved = true },
        { .goal = { .type = CHOREO_GOAL_HOLD },
          .max_duration_ms = 2000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 3), 0, "submit failed");

    position_t  body = { 0.0f, 0.0f };      /* partner starts at (1, 0)   */
    const float cx = 0.5f, cy = 0.0f;       /* pair centroid              */
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    float min_sep = 1e9f;
    int   ticks   = 0;
    while (ticks < 4000 && !choreo_script_complete()) {
        ticks++;
        wm_reset();
        wm_set_self(0, 0, body.x, body.y);
        wm_set_peer(1, 2.0f * cx - body.x, 2.0f * cy - body.y, false);

        choreo_tick(&wm, &scr);
        const tapestry_bse_directive_t *d = choreo_get_directive();
        if (d->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
            body.x = d->target.x;           /* perfect tracking */
            body.y = d->target.y;
        }
        if (choreo_script_step() == 1) {
            float sep = dist2d(body.x, body.y,
                               2.0f * cx - body.x, 2.0f * cy - body.y);
            if (sep < min_sep) { min_sep = sep; }
        }
    }

    zassert_true(choreo_script_complete(),
                 "script did not complete in %d ticks", ticks);
    /* 20 hold + ~210 arc + 10 achievement hold + 20 bow ≈ 260 ticks */
    zassert_true(ticks >= 240 && ticks <= 320,
                 "unexpected script length %d ticks", ticks);
    zassert_within(body.x, 1.0f, 0.02f,
                   "did not end on partner station: x=%.3f", (double)body.x);
    zassert_within(body.y, 0.0f, 0.02f,
                   "did not end on partner station: y=%.3f", (double)body.y);
    zassert_true(min_sep > 0.9f,
                 "separation dipped to %.2f m during exchange (arc must "
                 "preserve it)", (double)min_sep);
    zassert_equal(choreo_get_directive()->type, TAPESTRY_BSE_DIRECTIVE_IDLE,
                  "script completion must leave the quiescence directive");
}

ZTEST(choreo_script, test_exchange_waits_for_peer_snapshot)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_EXCHANGE },
          .max_duration_ms = 60000, .advance_on_achieved = true },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    /* No fresh peer: the snapshot cannot be captured — directive HOLD. */
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    for (int i = 0; i < 10; i++) {
        choreo_tick(&wm, &scr);
        zassert_equal(choreo_get_directive()->type,
                      TAPESTRY_BSE_DIRECTIVE_HOLD,
                      "exchange without a peer must HOLD");
    }

    /* Peer appears → capture succeeds on the next tick. */
    wm_set_peer(1, 1.0f, 0.0f, false);
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_get_directive()->type,
                  TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                  "exchange must engage once a peer is fresh");
}

ZTEST(choreo_script, test_suspension_freezes_step_timer)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 1000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_LOST;

    /* 5 s of quorum LOST — five times the step duration.  A frozen timer
     * must not advance the script. */
    for (int i = 0; i < 50; i++) {
        choreo_tick(&wm, &scr);
    }
    zassert_false(choreo_script_complete(),
                  "suspended script must not time its steps out");
    zassert_equal(choreo_goal_status(), CHOREO_STATE_SUSPENDED,
                  "quorum loss must suspend");

    scr.quorum_state = SCR_QUORUM_HEALTHY;
    for (int i = 0; i < 12; i++) {
        choreo_tick(&wm, &scr);
    }
    zassert_true(choreo_script_complete(),
                 "script must resume and complete after quorum recovery");
}

ZTEST(choreo_script, test_unadvanceable_step_rejected)
{
    static const choreo_step_t stall[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD } },   /* no exit condition */
    };
    choreo_init(0);
    zassert_equal(choreo_submit_script(stall, 1), -1,
                  "a step with no exit condition must be rejected");
    zassert_equal(choreo_goal_status(), CHOREO_STATE_IDLE,
                  "rejected script must leave the lifecycle in IDLE");
}

ZTEST(choreo_script, test_hold_is_coordinate_free)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.37f, -0.21f);      /* arbitrary current station */

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_equal(d->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                  "hold must station-keep, not idle");
    zassert_within(d->target.x, 0.37f, EPS, "hold station x");
    zassert_within(d->target.y, -0.21f, EPS, "hold station y");
    zassert_true(choreo_goal_achieved(),
                 "hold is trivially achieved (duration governs)");
}

ZTEST_SUITE(choreo_script, NULL, NULL, NULL, NULL, NULL);
