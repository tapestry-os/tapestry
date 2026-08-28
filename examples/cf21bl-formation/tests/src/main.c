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
    /* sim_drone_t is an uninitialised stack local, and position_t grew a
     * third component when the world model went 6DoF (465920b).  Peers
     * come from the memset-zeroed wm, so they sit at z=0, but an unset own z
     * left
     * demo_compute_drive folding stack garbage into dz — these tests passed
     * on aarch64 and failed on x86 CI purely on what the frame happened to
     * hold.  This is a horizontal-plane fixture: say so. */
    d->pos.z = 0.0f;
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

#include <errno.h>
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

/* direct_path exchange: the commanded target must be the destination
 * station from the first tick (no arc), and achievement must be measurable
 * immediately — body at dest + settle time → achieved. */
ZTEST(choreo_script, test_exchange_direct_path_beelines)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_EXCHANGE,
                    .direct_path = true,
                    .achieve_eps = 0.25f, .achieve_hold_ms = 1000 },
          .max_duration_ms = 30000, .advance_on_achieved = true },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 1.0f, 0.0f, false);

    choreo_tick(&wm, &scr);
    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_equal(d->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT, "no move");
    /* The peer still sits ON the destination — the occupied-dest standoff
     * must command the approach-line point short of it, not the station. */
    zassert_within(d->target.x,
                   1.0f - TAPESTRY_BSE_EXCHANGE_STANDOFF_M, EPS,
                   "occupied dest must command the standoff point "
                   "(got %.3f)", (double)d->target.x);
    zassert_false(choreo_goal_achieved(),
                  "must not achieve onto an occupied station");

    /* Peer vacates (its own exchange moved it): beeline completes and
     * achievement runs after the 1 s settle. */
    wm_reset();
    wm_set_self(0, 0, 0.5f, 0.0f);
    wm_set_peer(1, 0.0f, 0.0f, false);
    choreo_tick(&wm, &scr);
    d = choreo_get_directive();
    zassert_within(d->target.x, 1.0f, EPS,
                   "vacated dest must be commanded directly (got %.3f)",
                   (double)d->target.x);

    for (int i = 0; i < 12; i++) {
        wm_reset();
        wm_set_self(0, 0, 1.0f, 0.0f);
        wm_set_peer(1, 0.0f, 0.0f, false);
        choreo_tick(&wm, &scr);
    }
    zassert_true(choreo_script_complete(),
                 "direct swap must complete in ~settle time");
}

ZTEST(choreo_script, test_suspension_freezes_step_timer)
{
    /* CONVERGE (peer-referential-shaped target, not self-referential) —
     * HOLD is the one goal type this no longer holds for; see
     * test_suspended_hold_times_out_while_still_isolated below. */
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 1.0f, 1.0f } },
          .max_duration_ms = 1000 },
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

/* ── Isolation give-up: HOLD's timer keeps running while SUSPENDED ───────── */

ZTEST(choreo_script, test_suspended_hold_times_out_while_still_isolated)
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

    /* 1.1 s of quorum LOST — just past the step's own bound.  Unlike
     * non-HOLD goals (see test_suspension_freezes_step_timer above), a
     * HOLD step's own max_duration_ms is not frozen by SUSPENDED — this
     * is the isolation give-up mechanism (choreo_state_t's doc). */
    for (int i = 0; i < 11; i++) {
        choreo_tick(&wm, &scr);
    }
    zassert_true(choreo_script_complete(),
                 "an isolated HOLD must time itself out without waiting "
                 "for quorum recovery");
}

ZTEST(choreo_script, test_suspended_hold_advances_to_next_step_while_isolated)
{
    /* Two HOLD steps back to back: the first's timeout must fire while
     * still SUSPENDED, activate the second fresh (its own timer starting
     * from 0), and the second must ALSO keep counting down while still
     * isolated — not just the first step specially. */
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 500 },
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 500 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 2), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_LOST;

    for (int i = 0; i < 6; i++) {   /* 600 ms — past step 0's 500 ms bound */
        choreo_tick(&wm, &scr);
    }
    zassert_equal(choreo_script_step(), 1,
                  "step 0's isolated timeout must advance to step 1");
    zassert_equal(choreo_goal_status(), CHOREO_STATE_SUSPENDED,
                  "still isolated — advancing must not fake a recovery");
    zassert_false(choreo_script_complete(), "step 1 hasn't timed out yet");

    for (int i = 0; i < 6; i++) {   /* another 600 ms, still isolated */
        choreo_tick(&wm, &scr);
    }
    zassert_true(choreo_script_complete(),
                 "step 1's own isolated timeout must also fire");
}

/* ── Isolation escape hatch: on = quorum_lost ────────────────────────────── */

ZTEST(choreo_script, test_quorum_lost_transition_redirects_before_suspending)
{
    /* Step 0 is peer-referential (EXCHANGE-shaped via CONVERGE target, so
     * the test doesn't need a real peer snapshot); on quorum loss it
     * must jump to step 1 (a HOLD) rather than freezing at step 0's
     * directive, and step 1 must then run under the normal SUSPENDED
     * HOLD carve-out (still ticking BSE) on that SAME tick. */
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 5.0f, 5.0f } },
          .max_duration_ms = 60000,
          .on = { { .event = CHOREO_EVENT_QUORUM_LOST, .goto_step_idx = 1 } },
          .n_transitions = 1 },
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 2), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 3.0f, 4.0f);   /* current station, away from target */

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_script_step(), 0, "starts on step 0");

    scr.quorum_state = SCR_QUORUM_LOST;
    choreo_tick(&wm, &scr);

    zassert_equal(choreo_script_step(), 1,
                  "quorum_lost must redirect to step 1 immediately");
    zassert_equal(choreo_goal_status(), CHOREO_STATE_SUSPENDED,
                  "the redirected HOLD step correctly suspends right "
                  "after activating — it knows how to run safely there");
    zassert_equal(choreo_current_goal_type(), CHOREO_GOAL_HOLD,
                  "must have actually switched goals, not just index");

    /* The redirect itself only submits the new HOLD intent — station
     * capture happens on the SUSPENDED branch's own bse_tick(), which
     * runs starting the NEXT cycle (per-goal quorum carve-out, same as
     * always for HOLD). */
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_script_step(), 1, "still on the redirected step");
    zassert_equal(choreo_goal_status(), CHOREO_STATE_SUSPENDED, "still isolated");

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.x, 3.0f, EPS,
                   "redirected HOLD must station-keep at the CURRENT "
                   "position, not freeze at the old CONVERGE target");
    zassert_within(d->target.y, 4.0f, EPS, "same, y");
}

ZTEST(choreo_script, test_no_quorum_lost_transition_freezes_as_before)
{
    /* A step with no quorum_lost transition must behave exactly as
     * before this feature existed: freeze at its own directive. */
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 5.0f, 5.0f } },
          .max_duration_ms = 60000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 3.0f, 4.0f);

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_LOST;
    choreo_tick(&wm, &scr);

    zassert_equal(choreo_script_step(), 0, "no transition declared — stays put");
    zassert_equal(choreo_goal_status(), CHOREO_STATE_SUSPENDED, "still suspends");
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

/* Regression (2026-07-19 flight 2 class): a HOLD activating while the
 * Choreo is SUSPENDED must capture its station as soon as a position is
 * available — NOT deferred to quorum recovery, by which time the element
 * may have drifted and would bake the drifted position in as its station. */
ZTEST(choreo_script, test_hold_captures_station_while_suspended)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_LOST;

    /* First tick: no self position yet — capture cannot happen; the
     * lifecycle suspends on quorum loss. */
    wm_reset();
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_goal_status(), CHOREO_STATE_SUSPENDED,
                  "quorum loss must suspend");
    zassert_equal(choreo_get_directive()->type, TAPESTRY_BSE_DIRECTIVE_HOLD,
                  "no position yet — directive must be HOLD");

    /* Position becomes available while STILL suspended: self-referential
     * goals don't need quorum — the station must be captured now. */
    wm_set_self(0, 0, 0.41f, 0.13f);
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_goal_status(), CHOREO_STATE_SUSPENDED,
                  "still suspended (timers frozen)");
    zassert_equal(choreo_get_directive()->type,
                  TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                  "station capture must not wait for quorum");
    zassert_within(choreo_get_directive()->target.x, 0.41f, EPS, "station x");
    zassert_within(choreo_get_directive()->target.y, 0.13f, EPS, "station y");
    zassert_equal(choreo_current_goal_type(), CHOREO_GOAL_HOLD,
                  "current goal type accessor");
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

/* ── FORM shapes + MOVE offset-preserving translation ─────────────────────── */

ZTEST(choreo_script, test_form_shape_line)
{
    /* 3 elements, radius=3 target=(10,10): evenly spaced on X,
     * spanning [target.x - radius, target.x + radius]. */
    choreo_goal_t goal = {
        .type   = CHOREO_GOAL_FORM,
        .target = { 10.0f, 10.0f },
        .radius = 3.0f,
        .shape  = TAPESTRY_BSE_SHAPE_LINE,
    };
    const float expect_x[3] = { 7.0f, 10.0f, 13.0f };

    for (int rank = 0; rank < 3; rank++) {
        choreo_init((element_id_t)rank);
        zassert_equal(choreo_submit_goal(&goal), 0, "submit failed");

        wm_reset();
        wm_set_self(rank, (element_id_t)rank, 0.0f, 0.0f);
        for (int i = 0; i < 3; i++) {
            if (i != rank) {
                wm_set_peer(i, 0.0f, 0.0f, false);   /* id = slot = i */
            }
        }

        /* FORM reads task_slot/swarm_size from scr, not a second
         * independently-computed rank — scr_tick() must run first. */
        scr_state_t scr;
        scr_init(&scr, (element_id_t)rank, 0, 0, SCR_CAP_NONE);
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);

        const tapestry_bse_directive_t *d = choreo_get_directive();
        zassert_equal(d->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                      "form line must move");
        zassert_within(d->target.x, expect_x[rank], EPS,
                       "rank %d line x", rank);
        zassert_within(d->target.y, 10.0f, EPS, "rank %d line y", rank);
    }
}

ZTEST(choreo_script, test_form_shape_grid)
{
    /* 4 elements, radius=2 (cell spacing), target=(0,0): 2x2 grid,
     * corners at (+-1, +-1). */
    choreo_goal_t goal = {
        .type   = CHOREO_GOAL_FORM,
        .target = { 0.0f, 0.0f },
        .radius = 2.0f,
        .shape  = TAPESTRY_BSE_SHAPE_GRID,
    };
    const float expect_x[4] = { -1.0f, 1.0f, -1.0f, 1.0f };
    const float expect_y[4] = { -1.0f, -1.0f, 1.0f, 1.0f };

    for (int rank = 0; rank < 4; rank++) {
        choreo_init((element_id_t)rank);
        zassert_equal(choreo_submit_goal(&goal), 0, "submit failed");

        wm_reset();
        for (int i = 0; i < 4; i++) {
            if (i == rank) {
                wm_set_self(i, (element_id_t)i, 0.0f, 0.0f);
            } else {
                wm_set_peer(i, 0.0f, 0.0f, false);   /* id = slot = i */
            }
        }

        /* FORM reads task_slot/swarm_size from scr, not a second
         * independently-computed rank — scr_tick() must run first. */
        scr_state_t scr;
        scr_init(&scr, (element_id_t)rank, 0, 0, SCR_CAP_NONE);
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);

        const tapestry_bse_directive_t *d = choreo_get_directive();
        zassert_equal(d->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                      "form grid must move");
        zassert_within(d->target.x, expect_x[rank], EPS,
                       "rank %d grid x", rank);
        zassert_within(d->target.y, expect_y[rank], EPS,
                       "rank %d grid y", rank);
    }
}

ZTEST(choreo_script, test_move_preserves_offset)
{
    /* Self at (0,0), peer at (2,0): centroid (1,0), self offset (-1,0).
     * MOVE to (10,10) must land self at (9,10) — NOT at (10,10), which is
     * what CONVERGE would do. */
    choreo_goal_t goal = {
        .type   = CHOREO_GOAL_MOVE,
        .target = { 10.0f, 10.0f },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_goal(&goal), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 2.0f, 0.0f, false);

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_equal(d->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                  "move must move");
    zassert_within(d->target.x, 9.0f, EPS, "move offset x");
    zassert_within(d->target.y, 10.0f, EPS, "move offset y");

    /* Offset is captured once at activation — simulate physical drift and
     * confirm the commanded point does not re-derive from the new
     * position (mirrors the HOLD/EXCHANGE capture-once contract). */
    wm_set_self(0, 0, 5.0f, 5.0f);
    wm_set_peer(1, 2.0f, 0.0f, false);
    choreo_tick(&wm, &scr);
    d = choreo_get_directive();
    zassert_within(d->target.x, 9.0f, EPS, "move offset must stay captured");
    zassert_within(d->target.y, 10.0f, EPS, "move offset must stay captured");
}

ZTEST(choreo_script, test_converge_collapses_to_target)
{
    /* Regression guard: CONVERGE (unlike MOVE) still sends every element
     * to the identical point regardless of starting offset. */
    choreo_goal_t goal = {
        .type   = CHOREO_GOAL_CONVERGE,
        .target = { 5.0f, 5.0f },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_goal(&goal), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 2.0f, 0.0f, false);

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.x, 5.0f, EPS, "converge x");
    zassert_within(d->target.y, 5.0f, EPS, "converge y");
}

/* ── Collective achievement (scope = all) ──────────────────────────────────── */

ZTEST(choreo_script, test_scope_all_waits_for_peer_achievement)
{
    /* HOLD is trivially self-achieved, so with scope=all the ONLY thing
     * gating advance is the peer's gossiped achieved bit (simulated here
     * by writing wm.entries[1].state.goal_achieved directly — on the wire
     * this arrives via the gossip frame's 'achieved' field). */
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD },
          .max_duration_ms = 60000, .advance_on_achieved = true,
          .scope = CHOREO_SCOPE_ALL },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 5.0f, 5.0f, false);
    wm.entries[1].state.goal_achieved = false;

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    for (int i = 0; i < 20; i++) {
        choreo_tick(&wm, &scr);
        zassert_false(choreo_script_complete(),
                      "must not advance while a fresh peer is unachieved");
    }

    wm.entries[1].state.goal_achieved = true;
    choreo_tick(&wm, &scr);
    zassert_true(choreo_script_complete(),
                 "must advance once every fresh peer is achieved");
}

ZTEST(choreo_script, test_scope_all_vacuous_when_solo)
{
    /* No ACTIVE peer to disagree — scope=all must not deadlock a lone
     * survivor.  Vacuity is reserved for genuinely solo elements now that
     * a merely stale peer still votes (see
     * test_scope_all_waits_for_stale_peer). */
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD },
          .max_duration_ms = 60000, .advance_on_achieved = true,
          .scope = CHOREO_SCOPE_ALL },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    zassert_true(choreo_script_complete(),
                 "scope=all must be vacuously true with no active peers");
}

ZTEST(choreo_script, test_scope_all_waits_for_stale_peer)
{
    /* A merely STALE peer still votes, from its last-received achieved bit.
     * Skipping stale peers let scope=all degrade silently into per-element
     * achievement under packet loss: script_advance() runs before
     * choreo_tick() suspends on the quorum loss that the same staleness
     * threshold triggers, so every quorum-loss transition let a
     * personally-arrived element advance alone (2026-08-24 flight 15 —
     * partners finished 6.3 s apart on a 17%-delivery link). */
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD },
          .max_duration_ms = 60000, .advance_on_achieved = true,
          .scope = CHOREO_SCOPE_ALL },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 5.0f, 5.0f, /* stale = */ true);
    wm.entries[1].state.goal_achieved = false;

    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    zassert_false(choreo_script_complete(),
                  "a stale peer's last-known unachieved bit must still block "
                  "scope=all");

    /* Its last-known bit flipping to achieved releases the step even while
     * the entry stays stale — the vote is the peer's, not its freshness. */
    wm.entries[1].state.goal_achieved = true;
    choreo_tick(&wm, &scr);

    zassert_true(choreo_script_complete(),
                 "a stale peer reporting achieved must release scope=all");
}

/* ── Goal queue: preemption + resume ───────────────────────────────────────
 * choreo_preempt_goal() saves the running goal (and, for a script, its
 * exact step/timer position) instead of discarding it; choreo_terminate()
 * (and therefore choreo_cancel_goal()) resumes it automatically once the
 * preempting goal ends, instead of going to IDLE. */

ZTEST(choreo_script, test_preempt_resumes_hold_station)
{
    choreo_init(0);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    wm_reset();
    wm_set_self(0, 0, 5.0f, 5.0f);

    choreo_goal_t hold_goal = { .type = CHOREO_GOAL_HOLD };
    zassert_equal(choreo_submit_goal(&hold_goal), 0, "submit HOLD failed");
    choreo_tick(&wm, &scr);   /* capture the (5,5) station */
    zassert_false(choreo_is_preempted(), "nothing parked yet");

    choreo_goal_t rth = { .type = CHOREO_GOAL_CONVERGE, .target = {0.0f, 0.0f} };
    zassert_equal(choreo_preempt_goal(&rth), 0, "preempt failed");
    zassert_true(choreo_is_preempted(), "preempt must park the HOLD goal");
    zassert_equal(choreo_current_goal_type(), CHOREO_GOAL_CONVERGE,
                  "the preempting goal must be active");

    /* A second preempt while one is already parked is rejected — depth 1. */
    choreo_goal_t another = { .type = CHOREO_GOAL_CONVERGE, .target = {1.0f, 1.0f} };
    zassert_equal(choreo_preempt_goal(&another), -EBUSY,
                  "a second preempt must be rejected while one is parked");

    choreo_tick(&wm, &scr);
    zassert_within(choreo_get_directive()->target.x, 0.0f, EPS,
                   "preempting CONVERGE target x");
    zassert_within(choreo_get_directive()->target.y, 0.0f, EPS,
                   "preempting CONVERGE target y");

    /* Cancelling the preempting goal resumes HOLD at its ORIGINALLY
     * captured (5,5) station — not a fresh capture at wherever self is
     * now — proving the saved activation state, not just the goal type,
     * survives the round trip. */
    choreo_cancel_goal();
    zassert_false(choreo_is_preempted(), "nothing parked after resume");
    zassert_equal(choreo_current_goal_type(), CHOREO_GOAL_HOLD,
                  "HOLD must be active again after resume");
    choreo_tick(&wm, &scr);
    zassert_within(choreo_get_directive()->target.x, 5.0f, EPS,
                   "resumed HOLD station x must be the ORIGINAL capture");
    zassert_within(choreo_get_directive()->target.y, 5.0f, EPS,
                   "resumed HOLD station y must be the ORIGINAL capture");
}

ZTEST(choreo_script, test_preempt_mid_script_resumes_at_same_step)
{
    choreo_init(0);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);

    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 100000 },
    };
    zassert_equal(choreo_submit_script(script, 1), 0, "submit script failed");
    zassert_equal(choreo_script_step(), 0, "must start at step 0");

    for (int i = 0; i < 5; i++) {
        choreo_tick(&wm, &scr);   /* 500 ms of the 100000 ms step elapsed */
    }

    choreo_goal_t rth = { .type = CHOREO_GOAL_CONVERGE, .target = {9.0f, 9.0f} };
    zassert_equal(choreo_preempt_goal(&rth), 0, "preempt mid-script failed");
    zassert_equal(choreo_current_goal_type(), CHOREO_GOAL_CONVERGE,
                  "preempting goal must be active");

    /* Run the preempting goal for far longer than the parked step's
     * remaining budget would tolerate if its timer kept accumulating. */
    for (int i = 0; i < 200; i++) {
        choreo_tick(&wm, &scr);
    }

    choreo_cancel_goal();   /* resume the parked script */
    zassert_equal(choreo_current_goal_type(), CHOREO_GOAL_HOLD,
                  "resumed script's HOLD step must be active");
    zassert_equal(choreo_script_step(), 0,
                  "must resume at the SAME step, not advance");

    /* If step_ms had kept accumulating during the 200 preempting ticks
     * (20000 ms), 500+20000 = 20500 ms would already be well past this
     * check point; confirm the step is still short of its 100000 ms
     * bound after only ~99000 ms more (99500 ms since the step started,
     * counting only pre- and post-preemption HOLD ticks). */
    for (int i = 0; i < 990; i++) {
        choreo_tick(&wm, &scr);
    }
    zassert_equal(choreo_script_step(), 0,
                  "step_ms must not have counted the preempting goal's runtime");
    zassert_false(choreo_script_complete(), "not yet due");

    for (int i = 0; i < 10; i++) {
        choreo_tick(&wm, &scr);   /* push past 100000 ms total HOLD time */
    }
    zassert_true(choreo_script_complete(),
                 "script must complete once its OWN accumulated time is due");
}

ZTEST(choreo_script, test_preempt_from_idle_rejected)
{
    choreo_init(0);
    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE, .target = {1.0f, 1.0f} };
    zassert_equal(choreo_preempt_goal(&g), -1,
                  "preempt with nothing running must be rejected");
}

ZTEST(choreo_script, test_ordinary_submit_while_preempted_drops_parked_goal)
{
    choreo_init(0);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);

    choreo_goal_t hold_goal = { .type = CHOREO_GOAL_HOLD };
    choreo_submit_goal(&hold_goal);
    choreo_tick(&wm, &scr);

    choreo_goal_t rth = { .type = CHOREO_GOAL_CONVERGE, .target = {0.0f, 0.0f} };
    choreo_preempt_goal(&rth);
    zassert_true(choreo_is_preempted(), "preempted");

    /* An ORDINARY submit — not choreo_preempt_goal() — always fully
     * replaces everything, including anything parked. */
    choreo_goal_t fresh = { .type = CHOREO_GOAL_CONVERGE, .target = {3.0f, 3.0f} };
    zassert_equal(choreo_submit_goal(&fresh), 0, "ordinary submit failed");
    zassert_false(choreo_is_preempted(),
                  "ordinary submit must drop the parked goal, not stack on it");

    /* A subsequent preempt must succeed — the stack must genuinely be
     * empty, not just reporting empty while still logically full. */
    choreo_goal_t another = { .type = CHOREO_GOAL_CONVERGE, .target = {1.0f, 1.0f} };
    zassert_equal(choreo_preempt_goal(&another), 0,
                  "preempt after a dropping submit must succeed (stack truly empty)");

    /* Cancelling now resumes `fresh`, not the long-gone HOLD. */
    choreo_cancel_goal();
    zassert_equal(choreo_current_goal_type(), CHOREO_GOAL_CONVERGE,
                  "must resume the goal from the dropping submit, not the discarded HOLD");
}

/* ── Frames + anchors (Choreo SDK Design doc §5, FORM/CONVERGE only) ────── */

ZTEST(choreo_script, test_frame_absolute_is_unchanged_default)
{
    /* frame left at its zero value (ABSOLUTE) must behave byte-identical
     * to every goal submitted before this feature existed. */
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE, .target = { 7.0f, 3.0f } };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_equal(d->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT, "directive");
    zassert_within(d->target.x, 7.0f, EPS, "absolute target.x unchanged");
    zassert_within(d->target.y, 3.0f, EPS, "absolute target.y unchanged");
}

ZTEST(choreo_script, test_frame_collective_converge_gathers_at_centroid)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 4.0f, 0.0f, false);   /* centroid = (2,0) */
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE,
                        .frame = TAPESTRY_BSE_FRAME_COLLECTIVE };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.x, 2.0f, EPS, "collective centroid.x");
    zassert_within(d->target.y, 0.0f, EPS, "collective centroid.y");
}

ZTEST(choreo_script, test_frame_collective_form_centers_shape_on_centroid)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 10.0f, 0.0f, false);   /* centroid = (5,0) */
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_FORM, .shape = TAPESTRY_BSE_SHAPE_LINE,
                        .radius = 3.0f, .frame = TAPESTRY_BSE_FRAME_COLLECTIVE };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);

    const tapestry_bse_directive_t *d = choreo_get_directive();
    /* self is rank 0 of 2 on a LINE spanning [-3,+3] around centroid (5,0) */
    zassert_within(d->target.x, 2.0f, EPS, "form-collective rank0 x");
}

ZTEST(choreo_script, test_frame_element_anchor_debounces_before_locking)
{
    /* A brand-new anchor resolution must not drive a directive until it
     * has been stable for TAPESTRY_BSE_ANCHOR_HOLD_MS (2000ms) — the same
     * flight-discovered lesson QUORUM_UP_MS encodes elsewhere. */
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 3.0f, 4.0f);
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE,
                        .frame = TAPESTRY_BSE_FRAME_ELEMENT,
                        .anchor = TAPESTRY_BSE_ANCHOR_SELF };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");

    for (int i = 0; i < 19; i++) {
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);
    }
    zassert_equal(choreo_get_directive()->type, TAPESTRY_BSE_DIRECTIVE_HOLD,
                  "still debouncing before the hold time elapses");

    scr_tick(&scr, &wm); choreo_tick(&wm, &scr);
    scr_tick(&scr, &wm); choreo_tick(&wm, &scr);   /* tick 21: 2000ms reached */

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_equal(d->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                  "locked in after the hold time");
    zassert_within(d->target.x, 3.0f, EPS, "self-anchor target.x");
    zassert_within(d->target.y, 4.0f, EPS, "self-anchor target.y");
}

ZTEST(choreo_script, test_frame_element_leader_anchor_tracks_live)
{
    choreo_init(1);
    scr_state_t scr;
    scr_init(&scr, 1, 0, 0, SCR_CAP_NONE);   /* id 0 outranks id 1 -> leader */

    wm_reset();
    wm_set_self(1, 1, 9.0f, 9.0f);
    wm_set_peer(0, 5.0f, 5.0f, false);
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE,
                        .frame = TAPESTRY_BSE_FRAME_ELEMENT,
                        .anchor = TAPESTRY_BSE_ANCHOR_LEADER };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    for (int i = 0; i < 21; i++) {
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);
    }

    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.x, 5.0f, EPS, "leader-anchor target.x");
    zassert_within(d->target.y, 5.0f, EPS, "leader-anchor target.y");

    /* The leader vanishing (goes stale) must not silently keep steering
     * toward its last known position, nor snap to the new leader before
     * that switch has been confirmed stable — it falls back to HOLD
     * immediately (undebounced loss, same as EXCHANGE's own can't-
     * compute-this-tick fallback; debouncing is only for choosing between
     * competing VALID candidates). */
    wm.entries[0].is_stale = true;
    scr_tick(&scr, &wm);
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_get_directive()->type, TAPESTRY_BSE_DIRECTIVE_HOLD,
                  "leader vanishing falls back to HOLD immediately");

    wm.entries[0].is_stale = false;
    for (int i = 0; i < 21; i++) {
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);
    }
    d = choreo_get_directive();
    zassert_within(d->target.x, 5.0f, EPS, "leader-anchor relocks after reappearing");
}

ZTEST(choreo_script, test_frame_element_id_anchor_live_no_lag)
{
    /* §5.3: element-frame anchors bind LIVE, not snapshotted — once
     * locked, the SAME anchor's position must track it every tick with
     * no additional debounce (debounce only gates switching WHICH id is
     * the anchor). */
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 1.0f, 1.0f, false);
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE,
                        .frame = TAPESTRY_BSE_FRAME_ELEMENT,
                        .anchor = TAPESTRY_BSE_ANCHOR_ID, .anchor_id = 1 };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    for (int i = 0; i < 21; i++) {
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);
    }
    zassert_within(choreo_get_directive()->target.x, 1.0f, EPS, "locked onto peer 1");

    wm_set_peer(1, 8.0f, 8.0f, false);
    scr_tick(&scr, &wm);
    choreo_tick(&wm, &scr);
    zassert_within(choreo_get_directive()->target.x, 8.0f, EPS,
                   "live tracking, no additional debounce lag");
}

ZTEST(choreo_script, test_frame_element_lowest_energy_anchor)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm.entries[0].state.energy_level = 90;
    wm_set_peer(1, 5.0f, 5.0f, false);
    wm.entries[1].state.energy_level = 80;
    wm_set_peer(2, 9.0f, 9.0f, false);
    wm.entries[2].state.energy_level = 20;   /* lowest */
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE,
                        .frame = TAPESTRY_BSE_FRAME_ELEMENT,
                        .anchor = TAPESTRY_BSE_ANCHOR_LOWEST_ENERGY };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    for (int i = 0; i < 21; i++) {
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);
    }
    zassert_within(choreo_get_directive()->target.x, 9.0f, EPS,
                   "lowest-energy anchor picks peer 2");
}

/* ── Motion: spin (Choreo SDK Design doc §6, FORM only) ─────────────────── */

ZTEST(choreo_script, test_motion_spin_rotates_the_form_vertex)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 5.0f, 0.0f);
    scr_tick(&scr, &wm);

    choreo_goal_t g = {
        .type = CHOREO_GOAL_FORM, .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
        .radius = 5.0f, .target = { 0.0f, 0.0f },
        .motion = TAPESTRY_BSE_MOTION_SPIN, .spin_rate_radps = 0.5f,
    };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");

    scr_tick(&scr, &wm); choreo_tick(&wm, &scr);   /* t=0.1s */
    const tapestry_bse_directive_t *d = choreo_get_directive();
    float theta = 0.5f * 0.1f;
    zassert_within(d->target.x, 5.0f * cosf(theta), EPS, "spin tick1 x");
    zassert_within(d->target.y, 5.0f * sinf(theta), EPS, "spin tick1 y");

    for (int i = 0; i < 19; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    d = choreo_get_directive();   /* t=2.0s total */
    theta = 0.5f * 2.0f;
    zassert_within(d->target.x, 5.0f * cosf(theta), EPS, "spin t=2s x");
    zassert_within(d->target.y, 5.0f * sinf(theta), EPS, "spin t=2s y");
}

ZTEST(choreo_script, test_motion_spin_ignored_by_converge)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    choreo_goal_t g = {
        .type = CHOREO_GOAL_CONVERGE, .target = { 4.0f, 4.0f },
        .motion = TAPESTRY_BSE_MOTION_SPIN, .spin_rate_radps = 1.0f,
    };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    for (int i = 0; i < 50; i++) {
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);
    }
    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.x, 4.0f, EPS, "converge ignores motion x");
    zassert_within(d->target.y, 4.0f, EPS, "converge ignores motion y");
}

ZTEST(choreo_script, test_motion_spin_with_no_duration_bound_is_rejected)
{
    /* A non-terminal motion never "completes" — until=achieved alone is
     * not a sufficient exit, only a valid early-advance on top of a real
     * duration bound. */
    choreo_init(0);
    static const choreo_step_t bad[] = {
        { .goal = { .type = CHOREO_GOAL_FORM, .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                   .radius = 3.0f, .motion = TAPESTRY_BSE_MOTION_SPIN,
                   .spin_rate_radps = 0.5f },
          .advance_on_achieved = true, .max_duration_ms = 0 },
    };
    zassert_equal(choreo_submit_script(bad, 1), -1,
                  "spin with no duration bound must be rejected");

    static const choreo_step_t good[] = {
        { .goal = { .type = CHOREO_GOAL_FORM, .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                   .radius = 3.0f, .motion = TAPESTRY_BSE_MOTION_SPIN,
                   .spin_rate_radps = 0.5f },
          .max_duration_ms = 60000 },
    };
    zassert_equal(choreo_submit_script(good, 1), 0,
                  "spin with a real duration bound must be accepted");
}

/* ── Events + transitions (Choreo SDK Design doc §8, single track) ──────── */

ZTEST(choreo_script, test_explicit_achieved_transition_skips_a_step)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000,
          .on = { { .event = CHOREO_EVENT_ACHIEVED, .goto_step_idx = 2 } },
          .n_transitions = 1 },
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 99.0f, 99.0f } },
          .max_duration_ms = 60000 },
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 7.0f, 7.0f } },
          .max_duration_ms = 60000 },
    };
    zassert_equal(choreo_submit_script(script, 3), 0, "submit failed");
    for (int i = 0; i < 3; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_equal(choreo_script_step(), 2, "must skip step 1 via the explicit transition");
    zassert_within(choreo_get_directive()->target.x, 7.0f, EPS, "landed on step 2");
}

/* Regression scenario: the design doc's "welcome dance" (§8.3) — the
 * flagship demo for this feature. A fourth element joining redirects to
 * an "orbit" step; it leaving redirects back. */
ZTEST(choreo_script, test_element_joined_and_lost_cycle_the_welcome_dance)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);   /* solo: swarm_size = 1 */

    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 300000,
          .on = { { .event = CHOREO_EVENT_ELEMENT_JOINED, .goto_step_idx = 1 } },
          .n_transitions = 1 },
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 1.0f, 1.0f } },
          .max_duration_ms = 300000,
          .on = { { .event = CHOREO_EVENT_ELEMENT_LOST, .goto_step_idx = 0 } },
          .n_transitions = 1 },
    };
    zassert_equal(choreo_submit_script(script, 2), 0, "submit failed");
    for (int i = 0; i < 5; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_equal(choreo_script_step(), 0, "still step 0 while solo");

    wm_set_peer(1, 5.0f, 5.0f, false);
    for (int i = 0; i < 19; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_equal(choreo_script_step(), 0, "still debouncing the join");
    for (int i = 0; i < 2; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_equal(choreo_script_step(), 1, "element_joined fired after the debounce");

    wm.entries[1].is_stale = true;
    for (int i = 0; i < 21; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_equal(choreo_script_step(), 0, "element_lost cycled back to step 0");
}

ZTEST(choreo_script, test_count_transitions_check_in_declaration_order)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 1.0f, 1.0f, false);
    wm_set_peer(2, 2.0f, 2.0f, false);
    scr_tick(&scr, &wm);   /* swarm_size = 3 */

    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000,
          .on = {
              { .event = CHOREO_EVENT_COUNT_EQ, .threshold = 3, .goto_step_idx = 2 },
              { .event = CHOREO_EVENT_COUNT_GTE, .threshold = 2, .goto_step_idx = 1 },
          },
          .n_transitions = 2 },
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 1.0f, 1.0f } },
          .max_duration_ms = 60000 },
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 3.0f, 3.0f } },
          .max_duration_ms = 60000 },
    };
    zassert_equal(choreo_submit_script(script, 3), 0, "submit failed");
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_script_step(), 2,
                  "count_eq(3), declared first, wins over count_gte(2)");
}

ZTEST(choreo_script, test_anchor_lost_transition)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE,
                   .frame = TAPESTRY_BSE_FRAME_ELEMENT,
                   .anchor = TAPESTRY_BSE_ANCHOR_ID, .anchor_id = 9 },
          .max_duration_ms = 60000,
          .on = { { .event = CHOREO_EVENT_ANCHOR_LOST, .goto_step_idx = 1 } },
          .n_transitions = 1 },
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 4.0f, 4.0f } },
          .max_duration_ms = 60000 },
    };
    zassert_equal(choreo_submit_script(script, 2), 0, "submit failed");
    choreo_tick(&wm, &scr);
    zassert_equal(choreo_script_step(), 1,
                  "an anchor that was never fresh transitions immediately");
}

ZTEST(choreo_script, test_goto_end_completes_the_script_early)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000,
          .on = { { .event = CHOREO_EVENT_ACHIEVED, .goto_step_idx = 1 } },
          .n_transitions = 1 },   /* goto_step_idx == n_steps == "end" */
    };
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");
    for (int i = 0; i < 3; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_true(choreo_script_complete(), "goto=end must complete the script");
}

ZTEST(choreo_script, test_out_of_range_goto_is_rejected_at_submit)
{
    choreo_init(0);
    static const choreo_step_t bad[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000,
          .on = { { .event = CHOREO_EVENT_ACHIEVED, .goto_step_idx = 5 } },
          .n_transitions = 1 },
    };
    zassert_equal(choreo_submit_script(bad, 1), -1,
                  "an out-of-range goto_step_idx must be rejected up front");
}

ZTEST(choreo_script, test_track_capability_filter_falls_through_to_catchall)
{
    choreo_init(0);
    scr_state_t scr;
    /* no sensor, but ABS_POSITION so the catchall track's implicit-
     * ABSOLUTE CONVERGE below satisfies its derived floor (choreo.c's
     * derived_caps()) — this test is about capability-filtered track
     * SELECTION, not about the frame/positioning axis. */
    scr_init(&scr, 0, 0, 0, SCR_CAP_ABS_POSITION);
    choreo_register_scr(&scr);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    static const choreo_step_t sensing_steps[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };
    static const choreo_step_t catchall_steps[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 9.0f, 9.0f } },
          .max_duration_ms = 60000 },
    };
    choreo_track_t tracks[2] = {
        { .filter = { .required_caps = CHOREO_CAP_SENSING },
          .steps = sensing_steps, .n_steps = 1 },
        { .filter = { 0 }, .steps = catchall_steps, .n_steps = 1 },
    };
    zassert_equal(choreo_submit_tracks(&wm, tracks, 2), 0, "submit failed");
    zassert_equal(choreo_current_track(), 1,
                  "no SENSOR cap must fall through to the catch-all track");
}

ZTEST(choreo_script, test_track_capability_filter_matches_first_track)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_SENSOR | SCR_CAP_ABS_POSITION);
    choreo_register_scr(&scr);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    static const choreo_step_t sensing_steps[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };
    static const choreo_step_t catchall_steps[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 9.0f, 9.0f } },
          .max_duration_ms = 60000 },
    };
    choreo_track_t tracks[2] = {
        { .filter = { .required_caps = CHOREO_CAP_SENSING },
          .steps = sensing_steps, .n_steps = 1 },
        { .filter = { 0 }, .steps = catchall_steps, .n_steps = 1 },
    };
    zassert_equal(choreo_submit_tracks(&wm, tracks, 2), 0, "submit failed");
    zassert_equal(choreo_current_track(), 0,
                  "SENSOR cap must match the first declared track");
}

ZTEST(choreo_script, test_track_no_match_and_no_catchall_is_rejected)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    choreo_register_scr(&scr);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    static const choreo_step_t steps[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };
    choreo_track_t tracks[1] = {
        { .filter = { .required_caps = CHOREO_CAP_SENSING }, .steps = steps, .n_steps = 1 },
    };
    zassert_equal(choreo_submit_tracks(&wm, tracks, 1), -1,
                  "no catch-all and no match must be rejected at submission");
}

ZTEST(choreo_script, test_track_energy_low_migration_is_debounced)
{
    choreo_init(0);
    scr_state_t scr;
    /* ABS_POSITION for the low-battery track's implicit-ABSOLUTE CONVERGE
     * below (derived_caps()) — this test is about track migration
     * debouncing, not the frame/positioning axis. */
    scr_init(&scr, 0, 0, 0, SCR_CAP_ABS_POSITION);
    choreo_register_scr(&scr);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm.entries[0].state.health_flags = ELEMENT_HEALTH_OK;
    scr_tick(&scr, &wm);

    static const choreo_step_t low_battery_steps[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE, .target = { 0.0f, 0.0f } },
          .max_duration_ms = 60000 },
    };
    static const choreo_step_t normal_steps[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };
    choreo_track_t tracks[2] = {
        { .filter = { .requires_energy_low = true },
          .steps = low_battery_steps, .n_steps = 1 },
        { .filter = { 0 }, .steps = normal_steps, .n_steps = 1 },
    };
    zassert_equal(choreo_submit_tracks(&wm, tracks, 2), 0, "submit failed");
    zassert_equal(choreo_current_track(), 1, "starts on the catch-all track, not low");

    wm.entries[0].state.health_flags = ELEMENT_HEALTH_LOW_BATTERY;
    for (int i = 0; i < 19; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_equal(choreo_current_track(), 1, "still debouncing the low-battery switch");
    for (int i = 0; i < 2; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_equal(choreo_current_track(), 0,
                  "migrated to the low-battery track after the debounce hold");
    zassert_within(choreo_get_directive()->target.x, 0.0f, EPS,
                   "directive now driven by the low-battery track's goal");
}

ZTEST(choreo_script, test_track_scoped_collective_excludes_other_track_peer)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    choreo_register_scr(&scr);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 10.0f, 0.0f, false);
    wm.entries[1].state.current_track = 0;    /* same track as self */
    wm_set_peer(2, 100.0f, 100.0f, false);
    wm.entries[2].state.current_track = 1;    /* DIFFERENT track */
    scr_tick(&scr, &wm);

    static const choreo_step_t steps[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE,
                   .frame = TAPESTRY_BSE_FRAME_COLLECTIVE },
          .max_duration_ms = 60000 },
    };
    choreo_track_t tracks[1] = { { .filter = { 0 }, .steps = steps, .n_steps = 1 } };
    zassert_equal(choreo_submit_tracks(&wm, tracks, 1), 0, "submit failed");
    choreo_tick(&wm, &scr);

    /* Centroid of self(0,0) + peer1(10,0) only == (5,0); peer2 on a
     * different track must not skew it toward (100,100). */
    zassert_within(choreo_get_directive()->target.x, 5.0f, EPS,
                   "collective centroid must exclude the different-track peer");
    zassert_within(choreo_get_directive()->target.y, 0.0f, EPS,
                   "collective centroid must exclude the different-track peer");
}

ZTEST(choreo_script, test_track_default_still_counts_default_track_peer)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    choreo_register_scr(&scr);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 10.0f, 0.0f, false);   /* current_track defaults to 0, like every peer today */
    scr_tick(&scr, &wm);

    static const choreo_step_t steps[] = {
        { .goal = { .type = CHOREO_GOAL_CONVERGE,
                   .frame = TAPESTRY_BSE_FRAME_COLLECTIVE },
          .max_duration_ms = 60000 },
    };
    zassert_equal(choreo_submit_script(steps, 1), 0, "submit failed");
    zassert_equal(choreo_current_track(), 0, "a no-tracks script reports track 0");
    choreo_tick(&wm, &scr);
    zassert_within(choreo_get_directive()->target.x, 5.0f, EPS,
                   "a no-tracks script still counts a default-track peer");
}

ZTEST(choreo_script, test_track_ordinary_goal_drops_multitrack_mode)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    choreo_register_scr(&scr);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    static const choreo_step_t steps[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };
    choreo_track_t tracks[2] = {
        { .filter = { .requires_energy_low = true }, .steps = steps, .n_steps = 1 },
        { .filter = { 0 }, .steps = steps, .n_steps = 1 },
    };
    zassert_equal(choreo_submit_tracks(&wm, tracks, 2), 0, "submit tracks failed");
    zassert_equal(choreo_current_track(), 1, "on the catch-all track");

    choreo_goal_t plain = { .type = CHOREO_GOAL_HOLD };
    zassert_equal(choreo_submit_goal(&plain), 0, "submit plain goal over tracks failed");
    zassert_equal(choreo_current_track(), 0,
                  "an ordinary goal submission must drop multi-track mode");
}

/* ── 3D positions/targets (space is 3D throughout, not an optional axis) ── */

ZTEST(choreo_script, test_converge_to_a_real_3d_target_requires_all_3_axes)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE,
                        .target = { 2.0f, 3.0f, 1.5f } };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);
    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.z, 1.5f, EPS, "directive carries the real z");

    /* xy correct but z is 1.5 off -- must NOT be achieved. */
    wm.entries[0].state.position.x = 2.0f;
    wm.entries[0].state.position.y = 3.0f;
    for (int i = 0; i < 40; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_false(choreo_goal_achieved(), "xy-only match must not achieve a 3D goal");

    wm.entries[0].state.position.z = 1.5f;
    for (int i = 0; i < 40; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_true(choreo_goal_achieved(), "real 3D match achieves");
}

ZTEST(choreo_script, test_hold_captures_a_real_3d_station)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 1.0f, 2.0f);
    wm.entries[0].state.position.z = 0.7f;
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_HOLD };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);
    zassert_within(choreo_get_directive()->target.z, 0.7f, EPS,
                   "HOLD captured the real z, not just x/y");
}

ZTEST(choreo_script, test_disperse_achievement_is_real_3d_distance)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 0.1f, 0.0f, false);
    wm.entries[1].state.position.z = 5.0f;   /* mostly vertical separation */
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_DISPERSE, .radius = 2.0f };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    for (int i = 0; i < 40; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_true(choreo_goal_achieved(),
                 "3D distance (mostly vertical) counts as spread");
}

ZTEST(choreo_script, test_disperse_close_in_3d_is_not_achieved)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 0.5f, 0.0f, false);
    wm.entries[1].state.position.z = 0.5f;   /* 3D dist ~0.707m < 2.0 */
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_DISPERSE, .radius = 2.0f };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    for (int i = 0; i < 40; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_false(choreo_goal_achieved(), "close in 3D must not be spread");
}

ZTEST(choreo_script, test_exchange_never_touches_z_even_when_stations_differ)
{
    /* EXCHANGE reassigns x/y stations only -- z stays fixed at this
     * element's OWN altitude for the whole maneuver, not the swap
     * partner's, so vertical separation established elsewhere (e.g. a
     * HOLD z-stagger) survives a horizontal crossing. */
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, -1.0f, 0.0f);
    wm.entries[0].state.position.z = 0.5f;
    wm_set_peer(1, 1.0f, 0.0f, false);
    wm.entries[1].state.position.z = 1.5f;
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_EXCHANGE };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);
    zassert_within(choreo_get_directive()->target.z, 0.5f, EPS,
                   "arc starts at own z");

    for (int i = 0; i < 40; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_within(choreo_get_directive()->target.z, 0.5f, EPS,
                   "mid-arc z is still own z -- never drifts toward the peer's");

    for (int i = 0; i < 400; i++) { scr_tick(&scr, &wm); choreo_tick(&wm, &scr); }
    zassert_within(choreo_get_directive()->target.z, 0.5f, EPS,
                   "arc completes still at own z, not the destination station's");
}

ZTEST(choreo_script, test_exchange_direct_path_staggered_altitude_skips_standoff)
{
    /* A peer at a genuinely different altitude is never "occupying" this
     * element's real destination (dest x/y at OWN z) -- direct_path
     * proceeds immediately, no step-skew standoff needed. This exercises
     * EXCHANGE's own z math against an arbitrary already-separated wm
     * snapshot; whether/how a real script establishes that separation is
     * a separate question (see change-partners.choreo.toml's own comment
     * for why it keeps the default arc rather than direct_path today). */
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, -1.0f, 0.0f);
    wm.entries[0].state.position.z = 0.3f;
    wm_set_peer(1, 1.0f, 0.0f, false);
    wm.entries[1].state.position.z = 1.5f;   /* well outside OCCUPIED_M */
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_EXCHANGE, .direct_path = true };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);
    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.x, 1.0f, EPS, "beelines immediately, no standoff pullback");
    zassert_within(d->target.z, 0.3f, EPS, "stays at own z");
}

ZTEST(choreo_script, test_form_shapes_stay_planar_at_the_frame_origins_z)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 5.0f, 5.0f, false);
    wm.entries[1].state.position.z = 5.0f;
    wm_set_peer(2, 10.0f, 10.0f, false);
    wm.entries[2].state.position.z = 10.0f;
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_FORM, .shape = TAPESTRY_BSE_SHAPE_CIRCLE,
                        .target = { 0.0f, 0.0f, 3.0f }, .radius = 2.0f };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);
    zassert_within(choreo_get_directive()->target.z, 3.0f, EPS,
                   "FORM vertex shares the frame origin's real z");
}

ZTEST(choreo_script, test_move_preserves_the_real_z_offset)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm.entries[0].state.position.z = 1.0f;   /* self 1m above the centroid */
    wm_set_peer(1, 0.0f, 0.0f, false);
    wm.entries[1].state.position.z = -1.0f;
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_MOVE, .target = { 10.0f, 10.0f, 5.0f } };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);
    /* centroid z = 0, own offset = +1 -> commanded z = target.z(5) + 1 = 6 */
    zassert_within(choreo_get_directive()->target.z, 6.0f, EPS,
                   "MOVE preserves this element's real z offset");
}

ZTEST(choreo_script, test_frame_collective_centroid_averages_z_too)
{
    choreo_init(0);
    scr_state_t scr;
    scr_init(&scr, 0, 0, 0, SCR_CAP_NONE);
    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    wm_set_peer(1, 10.0f, 0.0f, false);
    wm.entries[1].state.position.z = 4.0f;
    scr_tick(&scr, &wm);

    choreo_goal_t g = { .type = CHOREO_GOAL_CONVERGE,
                        .frame = TAPESTRY_BSE_FRAME_COLLECTIVE };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");
    choreo_tick(&wm, &scr);
    const tapestry_bse_directive_t *d = choreo_get_directive();
    zassert_within(d->target.x, 5.0f, EPS, "centroid x");
    zassert_within(d->target.z, 2.0f, EPS, "collective centroid averages z too");
}

/* ── Effects (§12 Stage 5) ────────────────────────────────────────────────── */

ZTEST(choreo_script, test_current_indicator_and_tag_are_none_before_any_script)
{
    choreo_init(0);
    zassert_equal(choreo_current_indicator(), SUBSTRATE_SIGNAL_NONE,
                  "no script active — must be NONE");
    zassert_is_null(choreo_current_telemetry_tag(),
                    "no script active — must be NULL");
}

ZTEST(choreo_script, test_current_indicator_and_tag_reflect_the_active_step)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000,
          .indicator = SUBSTRATE_SIGNAL_ACTIVE, .telemetry_tag = "watching" },
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 2), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    zassert_equal(choreo_current_indicator(), SUBSTRATE_SIGNAL_ACTIVE,
                  "step 0 declared ACTIVE");
    zassert_true(strcmp(choreo_current_telemetry_tag(), "watching") == 0,
                "step 0 declared \"watching\"");
}

ZTEST(choreo_script, test_effects_default_to_none_when_not_declared)
{
    static const choreo_step_t script[] = {
        { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 60000 },
    };

    choreo_init(0);
    zassert_equal(choreo_submit_script(script, 1), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    zassert_equal(choreo_current_indicator(), SUBSTRATE_SIGNAL_NONE,
                  "step declared no indicator");
    zassert_is_null(choreo_current_telemetry_tag(),
                    "step declared no telemetry_tag");
}

ZTEST(choreo_script, test_current_indicator_is_none_for_a_bare_goal_not_a_script)
{
    /* A single choreo_submit_goal() has no choreo_step_t wrapper to
     * annotate — only a script's steps carry effects. */
    choreo_init(0);
    choreo_goal_t g = { .type = CHOREO_GOAL_HOLD };
    zassert_equal(choreo_submit_goal(&g), 0, "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    choreo_tick(&wm, &scr);

    zassert_equal(choreo_current_indicator(), SUBSTRATE_SIGNAL_NONE,
                  "a bare goal has no step to annotate");
    zassert_is_null(choreo_current_telemetry_tag(),
                    "a bare goal has no step to annotate");
}

/* ── Remote L6 directives (wire.h v5): adoption, fallback, re-adoption ────── */
/*
 * The degraded-mode ladder's element side (choreo.h's remote-directive
 * section).  The remote BSE host is emulated by feeding
 * choreo_remote_directive() in the same order runtime.c does (frame
 * arrives BEFORE choreo_tick each cycle).  The local script is a single
 * long HOLD at (0,0), so the local BSE's directive is MOVE_TO_POINT to
 * the captured station — distinguishable from the remote target by
 * coordinates alone, which is what lets these tests watch the steering
 * source switch without peeking at internals.
 */

#define REMOTE_ADOPT_TICKS  (CHOREO_REMOTE_ADOPT_HOLD_MS / WM_CYCLE_MS)
#define REMOTE_STALE_TICKS  (CHOREO_REMOTE_STALE_MS / WM_CYCLE_MS)

static const choreo_step_t remote_hold_script[] = {
    { .goal = { .type = CHOREO_GOAL_HOLD }, .max_duration_ms = 600000 },
};

/* One runtime cycle with a fresh remote frame: feed, then tick. */
static void remote_feed_tick(const scr_state_t *scr, float tx, float ty)
{
    tapestry_bse_directive_t rd = {
        .type   = TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
        .target = { .x = tx, .y = ty },
    };
    choreo_remote_directive(&rd, 0x77u, 9u);
    choreo_tick(&wm, scr);
}

ZTEST(choreo_script, test_remote_directive_adopts_only_after_stability_hold)
{
    choreo_init(0);
    zassert_equal(choreo_submit_script(remote_hold_script, 1), 0,
                  "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    /* One tick short of the hold: still steering locally (HOLD station). */
    for (unsigned i = 0; i < REMOTE_ADOPT_TICKS - 1u; i++) {
        remote_feed_tick(&scr, 7.0f, 8.0f);
        zassert_false(choreo_remote_active(),
                      "tick %u: adoption must wait out the stability hold", i);
        zassert_within(choreo_get_directive()->target.x, 0.0f, 0.001f,
                       "tick %u: still the local HOLD station", i);
    }

    /* The hold elapses — remote steers from this cycle on. */
    remote_feed_tick(&scr, 7.0f, 8.0f);
    zassert_true(choreo_remote_active(), "adopted after the hold");
    zassert_within(choreo_get_directive()->target.x, 7.0f, 0.001f,
                   "remote target must steer once adopted");
    zassert_within(choreo_get_directive()->target.y, 8.0f, 0.001f,
                   "remote target must steer once adopted");
}

ZTEST(choreo_script, test_remote_staleness_falls_back_to_local_bumplessly)
{
    choreo_init(0);
    zassert_equal(choreo_submit_script(remote_hold_script, 1), 0,
                  "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    for (unsigned i = 0; i < REMOTE_ADOPT_TICKS; i++) {
        remote_feed_tick(&scr, 7.0f, 8.0f);
    }
    zassert_true(choreo_remote_active(), "adopted");

    /* The stream dies.  Remote keeps steering until CHOREO_REMOTE_STALE_MS
     * from the last frame, then falls back the same tick — and the local
     * BSE has been ticking the whole time, so the fallback directive is
     * the CURRENT station-keep, not a thawed stale one. */
    for (unsigned i = 0; i < REMOTE_STALE_TICKS - 2u; i++) {
        choreo_tick(&wm, &scr);
        zassert_true(choreo_remote_active(),
                     "tick %u after loss: last frame still fresh", i);
    }
    choreo_tick(&wm, &scr);
    zassert_false(choreo_remote_active(),
                  "stale stream must stop steering immediately");
    zassert_equal(choreo_get_directive()->type,
                  TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                  "local BSE directive current on the fallback tick");
    zassert_within(choreo_get_directive()->target.x, 0.0f, 0.001f,
                   "fallback steers to the live local HOLD station");
}

ZTEST(choreo_script, test_remote_readoption_is_debounced_like_first_adoption)
{
    choreo_init(0);
    zassert_equal(choreo_submit_script(remote_hold_script, 1), 0,
                  "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    for (unsigned i = 0; i < REMOTE_ADOPT_TICKS; i++) {
        remote_feed_tick(&scr, 7.0f, 8.0f);
    }
    zassert_true(choreo_remote_active(), "adopted");

    /* Outage long enough to go stale. */
    for (unsigned i = 0; i < REMOTE_STALE_TICKS + 2u; i++) {
        choreo_tick(&wm, &scr);
    }
    zassert_false(choreo_remote_active(), "stale after the outage");

    /* The stream returns: a flapping link must NOT steer again instantly —
     * the full stability hold applies to re-adoption too. */
    for (unsigned i = 0; i < REMOTE_ADOPT_TICKS - 1u; i++) {
        remote_feed_tick(&scr, 7.0f, 8.0f);
        zassert_false(choreo_remote_active(),
                      "tick %u after return: still debouncing", i);
    }
    remote_feed_tick(&scr, 7.0f, 8.0f);
    zassert_true(choreo_remote_active(), "re-adopted after the full hold");
}

ZTEST(choreo_script, test_remote_never_steers_while_suspended)
{
    choreo_init(0);
    zassert_equal(choreo_submit_script(remote_hold_script, 1), 0,
                  "submit failed");

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    for (unsigned i = 0; i < REMOTE_ADOPT_TICKS; i++) {
        remote_feed_tick(&scr, 7.0f, 8.0f);
    }
    zassert_true(choreo_remote_active(), "adopted");

    /* Quorum loss: L5 is the safety authority — a perfectly fresh remote
     * stream must not steer a SUSPENDED element (a remote host's view of a
     * partitioned collective is exactly what cannot be trusted). */
    scr.quorum_state = SCR_QUORUM_LOST;
    for (unsigned i = 0; i < 5u; i++) {
        remote_feed_tick(&scr, 7.0f, 8.0f);
        zassert_false(choreo_remote_active(),
                      "tick %u: SUSPENDED must steer locally", i);
        zassert_within(choreo_get_directive()->target.x, 0.0f, 0.001f,
                       "tick %u: local HOLD station while suspended", i);
    }

    /* Recovery: the stream stayed fresh and adopted throughout, so remote
     * steering resumes with quorum — no second debounce for a link that
     * never actually flapped. */
    scr.quorum_state = SCR_QUORUM_HEALTHY;
    remote_feed_tick(&scr, 7.0f, 8.0f);
    zassert_true(choreo_remote_active(), "steers again on quorum recovery");
    zassert_within(choreo_get_directive()->target.x, 7.0f, 0.001f,
                   "remote target after recovery");
}

ZTEST(choreo_script, test_remote_directive_cannot_activate_an_idle_element)
{
    /* No goal, no script — a directive stream must not wake a parked
     * element into motion.  The quiescence directive stands. */
    choreo_init(0);

    wm_reset();
    wm_set_self(0, 0, 0.0f, 0.0f);
    scr_state_t scr = { 0 };
    scr.quorum_state = SCR_QUORUM_HEALTHY;

    for (unsigned i = 0; i < REMOTE_ADOPT_TICKS * 2u; i++) {
        remote_feed_tick(&scr, 7.0f, 8.0f);
        zassert_false(choreo_remote_active(),
                      "tick %u: IDLE never adopts remote steering", i);
        zassert_equal(choreo_get_directive()->type,
                      TAPESTRY_BSE_DIRECTIVE_IDLE,
                      "tick %u: quiescence directive must stand", i);
    }
}

ZTEST_SUITE(choreo_script, NULL, NULL, NULL, NULL, NULL);
