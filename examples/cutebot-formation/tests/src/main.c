/*
 * main.c — formation.c unit tests (no hardware, no BLE radio)
 *
 * Covers the two pieces this session added on top of the existing,
 * flight-validated spring field (demo_compute_drive, exercised only
 * lightly below as a regression check that the demo_force_to_twist
 * extraction didn't change its behavior):
 *
 *   - demo_track_target()  the new differential-drive go-to-point law
 *                          (arrival snap, trapezoidal approach, emergency
 *                          repulsion backstop, the exact-180-degree
 *                          reverse case)
 *   - the form-grid Choreo script (hold -> form(grid) -> hold), run
 *     end-to-end through REAL demo_odometry_t + demo_track_target +
 *     demo_odometry_update integration (not a teleporting perfect
 *     tracker) — this is the actual control loop main.c runs, so a script
 *     that "completes" here is completing against physically plausible
 *     motion, not just BSE's own achievement math in isolation.
 *
 * Build:  west build -p always -b native_sim tapestry/examples/cutebot-formation/tests
 *         (on a 64-bit-only host, e.g. the aarch64 Pi: -b native_sim/native/64)
 * Run:    ./build/zephyr/zephyr.exe
 *
 * NOT run this session (no ZEPHYR_BASE / west toolchain available) — the
 * new math in formation.c was instead verified with a standalone host
 * build (plain clang, no Zephyr) exercising the same scenarios as the
 * ztest cases below; see the session notes. This file is the intended
 * permanent, CI-buildable form of that verification and should be run
 * for real the first time native_sim is available.
 */

#include <zephyr/ztest.h>
#include <math.h>
#include <string.h>

#include "formation.h"

#define DT_MS      100u
#define EPS        0.001f

/* Zephyr builds with -std=c17, under which glibc's <math.h> does not expose
 * M_PI (it is a _DEFAULT_SOURCE extension, not ISO C). Use the same float
 * constant formation.c itself uses, so the headings here are bit-identical
 * to the ones its wrap-around math produces. */
#define M_PI_F     3.14159265f

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

static void wm_set_self(int slot, element_id_t id, float x, float y)
{
    wm.entries[slot].is_active        = true;
    wm.entries[slot].is_self          = true;
    wm.entries[slot].is_stale         = false;
    wm.entries[slot].state.id         = id;
    wm.entries[slot].state.position.x = x;
    wm.entries[slot].state.position.y = y;
}

static float dist2d(float ax, float ay, float bx, float by)
{
    return sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

/* ── demo_compute_drive regression (unchanged spring field, now routed
 * through the extracted demo_force_to_twist helper) ─────────────────────── */

ZTEST(formation_field, test_spring_repels_when_too_close)
{
    wm_reset();
    wm_set_peer(1, 5.0f, 0.0f, false);   /* well inside DEMO_TARGET_SPACING */

    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 0.0f);
    odo.heading = 0.0f;   /* facing the peer: escape is directly behind */

    float speed, rate;
    /* Two ticks: force must exceed FORCE_START before movement engages. */
    demo_compute_drive(&wm, &odo, &speed, &rate);
    demo_compute_drive(&wm, &odo, &speed, &rate);
    zassert_true(odo.moving, "strong repulsion must clear FORCE_START");
    zassert_true(speed < 0.0f,
                 "facing the (too-close) peer, repulsion must reverse away");
}

ZTEST(formation_field, test_spring_attracts_when_too_far)
{
    wm_reset();
    wm_set_peer(1, DEMO_TARGET_SPACING + 40.0f, 0.0f, false);

    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 0.0f);
    odo.heading = 0.0f;   /* facing the peer */

    float speed, rate;
    demo_compute_drive(&wm, &odo, &speed, &rate);
    demo_compute_drive(&wm, &odo, &speed, &rate);
    zassert_true(odo.moving, "large spacing error must clear FORCE_START");
    zassert_true(speed > 0.0f, "too far must attract (drive forward)");
}

/* ── demo_track_target ────────────────────────────────────────────────────── */

ZTEST(formation_field, test_track_converges_off_axis)
{
    wm_reset();
    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 0.0f);
    odo.heading = M_PI_F - 0.4f;   /* mostly facing away, off-axis */

    float speed, rate;
    demo_track_target(&wm, &odo, 30.0f, 0.0f, &speed, &rate);
    zassert_true(fabsf(rate) > 0.01f, "must turn when facing away");

    bool converged = false;
    for (int i = 0; i < 400 && !converged; i++) {
        demo_track_target(&wm, &odo, 30.0f, 0.0f, &speed, &rate);
        demo_odometry_update(&odo, speed, rate, DT_MS);
        converged = dist2d(odo.x, odo.y, 30.0f, 0.0f) < DEMO_TRACK_ARRIVE_EPS + 0.5f;
    }
    zassert_true(converged, "did not converge onto target within 40s");
}

/* Regression: target exactly 180 deg behind is a degenerate case for the
 * shared force->twist projection (f_lat is exactly zero — no turn is
 * possible from force alone) — the controller must back straight up
 * instead of stalling, and still converge. */
ZTEST(formation_field, test_track_reverses_when_exactly_behind)
{
    wm_reset();
    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 0.0f);
    odo.heading = M_PI_F;

    float speed, rate;
    demo_track_target(&wm, &odo, 30.0f, 0.0f, &speed, &rate);
    zassert_within(rate, 0.0f, EPS, "exactly behind: no turn possible");
    zassert_true(speed < 0.0f, "exactly behind: must reverse toward it");

    bool converged = false;
    for (int i = 0; i < 400 && !converged; i++) {
        demo_track_target(&wm, &odo, 30.0f, 0.0f, &speed, &rate);
        demo_odometry_update(&odo, speed, rate, DT_MS);
        converged = dist2d(odo.x, odo.y, 30.0f, 0.0f) < DEMO_TRACK_ARRIVE_EPS + 0.5f;
    }
    zassert_true(converged, "reverse approach did not converge within 40s");
}

ZTEST(formation_field, test_track_arrival_zeroes_output)
{
    wm_reset();
    demo_odometry_t odo;
    demo_odometry_init(&odo, 30.0f, 0.0f);
    odo.heading = 0.5f;

    float speed, rate;
    demo_track_target(&wm, &odo, 30.5f, 0.0f, &speed, &rate);
    zassert_within(speed, 0.0f, EPS, "must hold still inside arrival eps");
    zassert_within(rate, 0.0f, EPS, "must hold still inside arrival eps");
}

/* Collinear peer/self/target: f_lat is exactly zero (same degenerate case
 * as the exactly-behind test above), so repulsion's observable effect is
 * on SPEED — it must override forward attraction with reverse rather than
 * drive through the too-close peer toward the distant target. */
ZTEST(formation_field, test_track_repulsion_overpowers_attraction_collinear)
{
    wm_reset();
    wm_set_peer(1, 2.0f, 0.0f, false);   /* well inside DEMO_TRACK_MIN_SEP */

    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 0.0f);
    odo.heading = 0.0f;

    float speed, rate;
    demo_track_target(&wm, &odo, 50.0f, 0.0f, &speed, &rate);
    zassert_true(speed < 0.0f,
                 "a peer dead ahead inside MIN_SEP must override forward "
                 "attraction with reverse");

    wm_reset();   /* baseline: no peer -> forward */
    demo_track_target(&wm, &odo, 50.0f, 0.0f, &speed, &rate);
    zassert_true(speed > 0.0f, "without the peer, attraction alone drives forward");
}

ZTEST(formation_field, test_track_repulsion_steers_off_axis)
{
    wm_reset();
    wm_set_peer(1, 5.0f, 1.0f, false);   /* just off the +x line, close */

    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 0.0f);
    odo.heading = 0.0f;

    float speed, rate;
    demo_track_target(&wm, &odo, 50.0f, 0.0f, &speed, &rate);
    zassert_true(fabsf(rate) > 0.01f,
                 "an off-axis close peer must perturb the heading command");
}

ZTEST(formation_field, test_track_ignores_stale_peer_repulsion)
{
    /* Unlike demo_compute_drive, demo_track_target has no hold-on-stale
     * gate (staleness handling belongs to the caller's quorum mapping —
     * see formation.h) — a stale peer must simply not contribute
     * repulsion, not silently freeze the drive. */
    wm_reset();
    wm_set_peer(1, 2.0f, 0.0f, /* stale = */ true);

    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 0.0f);
    odo.heading = 0.0f;

    float speed, rate;
    demo_track_target(&wm, &odo, 50.0f, 0.0f, &speed, &rate);
    zassert_true(speed > 0.0f,
                 "a stale peer must not contribute repulsion");
}

ZTEST_SUITE(formation_field, NULL, NULL, NULL, NULL, NULL);

/* ── Choreo script (L6 BSE + L7 Choreographer, singleton per process) ─────── */

#include <tapestry/choreo.h>
#include "choreo_script.h"

/* One tick from ONE element's perspective (choreo.c/bse.c are per-element
 * singletons — this mirrors cf21bl-formation/tests's own pattern): real
 * scr_tick() derives task_slot/swarm_size from wm, real choreo_tick()
 * decomposes the FORM goal into this element's own target vertex, and the
 * result is driven by the REAL demo_track_target + demo_odometry_update
 * loop main.c runs — not a teleporting perfect tracker. */
static float sim_tick(scr_state_t *scr, demo_odometry_t *odo, element_id_t id)
{
    scr_tick(scr, &wm);
    choreo_tick(&wm, scr);
    /* wm is hand-built (wm_reset/wm_set_self/wm_set_peer, not wm_init())
     * so wm.owner_id was never set — index by the slot this test always
     * uses for the self entry (id, per wm_set_self's caller convention
     * below), not wm.owner_id, which would silently stay 0 for every
     * element but #0. */
    choreo_publish_state(&wm.entries[id].state);

    const tapestry_bse_directive_t *dir = choreo_get_directive();
    float speed = 0.0f, rate = 0.0f;
    if (dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
        demo_track_target(&wm, odo, dir->target.x, dir->target.y, &speed, &rate);
    }
    demo_odometry_update(odo, speed, rate, DT_MS);
    return dist2d(odo->x, odo->y, dir->target.x, dir->target.y);
}

ZTEST(choreo_script, test_form_grid_script_end_to_end)
{
    /* 4 elements at their form-grid.choreo.toml boot-time-analog stations
     * (arbitrary distinct starting points — FORM doesn't care where they
     * started, only wm's live peer set at activation time). Each
     * element's own perspective is simulated in turn: wm is rebuilt every
     * tick from all 4 bodies' CURRENT positions before that element's
     * own scr_tick/choreo_tick/demo_track_target/demo_odometry_update. */
    demo_odometry_t odo[4];
    demo_odometry_init(&odo[0],  5.0f,  5.0f);
    demo_odometry_init(&odo[1], 90.0f, 10.0f);
    demo_odometry_init(&odo[2], 10.0f, 85.0f);
    demo_odometry_init(&odo[3], 80.0f, 80.0f);
    odo[0].heading = 0.3f; odo[1].heading = 2.1f;
    odo[2].heading = -1.0f; odo[3].heading = 1.7f;

    scr_state_t scr[4];
    for (int i = 0; i < 4; i++) {
        choreo_init((element_id_t)i);   /* re-inits the singleton each pass below */
        scr_init(&scr[i], (element_id_t)i, 1, 1, SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
    }

    /* choreo.c/bse.c are singletons: run each element's FULL script to
     * completion in its own pass (its wm view is rebuilt from the OTHER
     * three elements' bodies as of the tick before this pass started —
     * a reasonable approximation for this test's purpose, verifying each
     * element's own script logic and controller against the shared grid
     * geometry, not true 4-way concurrent simulation). */
    bool completed[4] = {0};
    for (int who = 0; who < 4; who++) {
        choreo_init((element_id_t)who);
        zassert_equal(choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN), 0,
                      "submit failed for element %d", who);

        int ticks = 0;
        while (ticks < 2000 && !choreo_script_complete()) {
            wm_reset();
            wm_set_self(who, (element_id_t)who, odo[who].x, odo[who].y);
            for (int j = 0; j < 4; j++) {
                if (j != who) {
                    wm_set_peer(j, odo[j].x, odo[j].y, false);
                }
            }
            sim_tick(&scr[who], &odo[who], (element_id_t)who);
            ticks++;
        }
        completed[who] = choreo_script_complete();
        zassert_true(completed[who],
                     "element %d script did not complete in %d ticks", who, ticks);
    }

    /* Each element must have settled near ONE of the 4 expected grid
     * vertices (bse.c's TAPESTRY_BSE_SHAPE_GRID: target=(50,50),
     * radius=25 -> corners at (50 +- 12.5, 50 +- 12.5)), and collectively
     * every vertex must be covered exactly once — permutation-invariant,
     * since task_slot ordering (ascending element_id when all are fresh)
     * determines which element gets which corner, not this test. */
    const float verts[4][2] = {
        { 37.5f, 37.5f }, { 62.5f, 37.5f }, { 37.5f, 62.5f }, { 62.5f, 62.5f },
    };
    bool vertex_used[4] = {0};
    for (int who = 0; who < 4; who++) {
        int matched = -1;
        for (int v = 0; v < 4; v++) {
            /* 3.0 units, not a tight 1.0: the controller's own arrival
             * snap (DEMO_TRACK_ARRIVE_EPS=2.0) and the script's
             * achieve_eps (5.0, form-grid.choreo.toml) both allow
             * settling short of the exact vertex — verified empirically
             * (max observed error ~1.84 units) via a standalone host run
             * of this exact scenario this session; see the session notes. */
            if (!vertex_used[v] &&
                dist2d(odo[who].x, odo[who].y, verts[v][0], verts[v][1]) < 3.0f) {
                matched = v;
                break;
            }
        }
        zassert_true(matched >= 0,
                     "element %d ended at (%.2f,%.2f), not on any unclaimed "
                     "grid vertex", who, (double)odo[who].x, (double)odo[who].y);
        vertex_used[matched] = true;
    }
}

ZTEST_SUITE(choreo_script, NULL, NULL, NULL, NULL, NULL);
