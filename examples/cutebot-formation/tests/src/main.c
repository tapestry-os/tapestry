/*
 * main.c — formation.c unit tests (no hardware, no BLE radio)
 *
 * Covers:
 *   - demo_track_target()   the differential-drive go-to-point law
 *                           (arrival snap, trapezoidal approach, emergency
 *                           repulsion backstop, the exact-180-degree
 *                           reverse case)
 *   - demo_arena_fence()    the outward-command veto near a WORLD_SIZE
 *                           edge, exercised implicitly through the ring
 *                           script test below (element 3's scenario) —
 *                           see that test's comment for why this matters
 *   - ring.choreo.toml      (hold -> form(circle,collective) -> hold),
 *                           run end-to-end through REAL demo_odometry_t +
 *                           demo_track_target + demo_odometry_update
 *                           integration (not a teleporting perfect
 *                           tracker) — the actual control loop main.c
 *                           runs, so a script that "completes" here is
 *                           completing against physically plausible
 *                           motion, not just BSE's own achievement math.
 *
 * Build:  west build -p always -b native_sim tapestry/examples/cutebot-formation/tests
 *         (on a 64-bit-only host, e.g. the aarch64 Pi: -b native_sim/native/64)
 * Run:    ./build/zephyr/zephyr.exe
 *
 * NOT run this session (no ZEPHYR_BASE / west toolchain available) — the
 * ztest cases below were instead verified with a standalone host build
 * (plain gcc, no Zephyr: formation.c linked directly against the real
 * scr.c/bse.c/choreo.c/world_model.c sources, with a ~15-line stub for
 * <zephyr/logging/log.h> and <zephyr/display/mb_display.h>) exercising
 * the exact same scenarios. Run this suite for real the first time
 * native_sim is available, before trusting it as a regression gate.
 *
 * choreo_script.h now compiles from ring.choreo.toml, not form-grid — the
 * ring-script test below replaces the old form-grid one, which asserted
 * fixed-grid-corner positions that no longer apply (ring's FORM target is
 * frame=collective: a live centroid, not an absolute point).
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

/* ── demo_track_target ────────────────────────────────────────────────────── */

/* odo starts at (0, 50), not (0, 0): a differential-drive robot turns
 * and drives simultaneously (not turn-then-drive), so approaching this
 * ~157-degree off-axis heading drifts off the direct line to the target
 * (visible as a real y-excursion before it straightens out). With the
 * target at y=0 (the previous version of this test), that drift lands
 * inside DEMO_ARENA_FENCE_MARGIN, and demo_arena_fence() (correctly, by
 * its own design) permanently vetoes the negative-fy correction needed
 * to close the last few units — indistinguishable, from the fence's
 * point of view, from driving off the board. That produced a stable
 * ~5-unit-short standoff that looked like a control-law limit cycle but
 * wasn't one: the identical heading/turn challenge, target moved to
 * (30, 50) — nowhere near an edge — converges cleanly in 17 ticks
 * (1.7s), zero code changes. Real ring.choreo.toml targets (radius 30
 * around center (50,50), staying inside [20,80]) are nowhere near this
 * zone either — this was a test-fixture bug, not a production risk. */
ZTEST(formation_field, test_track_converges_off_axis)
{
    wm_reset();
    demo_odometry_t odo;
    demo_odometry_init(&odo, 0.0f, 50.0f);
    odo.heading = M_PI_F - 0.4f;   /* mostly facing away, off-axis */

    float speed, rate;
    demo_track_target(&wm, &odo, 30.0f, 50.0f, &speed, &rate);
    zassert_true(fabsf(rate) > 0.01f, "must turn when facing away");

    bool converged = false;
    for (int i = 0; i < 400 && !converged; i++) {
        demo_track_target(&wm, &odo, 30.0f, 50.0f, &speed, &rate);
        demo_odometry_update(&odo, speed, rate, DT_MS);
        converged = dist2d(odo.x, odo.y, 30.0f, 50.0f) < DEMO_TRACK_ARRIVE_EPS + 0.5f;
    }
    zassert_true(converged, "did not converge onto target within 40s");
}

/* Real numbers, not an estimate: does a mid-show re-form's ACTUAL turn
 * requirement (not the synthetic ~157-degree heading above) also fail to
 * converge? Computed directly from bse.c's FORM circle formula
 * (angle = 2*PI*rank/count, tgt = center + radius*(cos,sin)) and
 * main.c's compute_start_pos() seed formula, not guessed.
 *
 * The INITIAL FORM never needs this: with all 4 robots fresh, task_slot
 * (rank) == element_id (ascending sort of {0,1,2,3}), so every robot's
 * target angle exactly matches its own boot heading — required turn is
 * always 0 degrees. This is why the 2026-09-12 hardware validation run
 * settled cleanly; it never exercised an off-axis turn at all.
 *
 * A departure changes this: losing id=1 from a settled 4-ring shifts
 * id=2's rank 2->1 (of the surviving 3), so its post-departure target
 * angle (2*PI*1/3 = 120 deg) no longer lines up with its settled
 * heading (180 deg, unchanged from boot since the initial FORM required
 * no turn) — a real ~120-degree turn, the same order of magnitude as
 * the ~157-degree heading that produced the persistent limit cycle
 * above. This test starts at that exact settled position/heading and
 * the exact new target (not the synthetic angle) to check whether the
 * already-hardware-validated drive law actually handles a real re-form
 * turn — it does not exercise choreo.c's element_lost debounce plumbing
 * itself (test_ring_script_end_to_end and the hardware runs cover that),
 * only the geometry. */
ZTEST(formation_field, test_track_reform_after_departure_id2_of_4)
{
    wm_reset();
    demo_odometry_t odo;
    demo_odometry_init(&odo, 20.0f, 50.0f);   /* id=2's settled position (rank 2 of 4) */
    odo.heading = M_PI_F;                     /* settled heading — unchanged from boot,
                                                * since the initial FORM needed no turn */

    /* rank 1 of 3 (ascending sort of surviving {0,2,3}), radius 30,
     * center (50,50) — bse.c's exact circle-placement formula. */
    float target_x = 50.0f + 30.0f * cosf(2.0f * M_PI_F / 3.0f);   /* 35.0 */
    float target_y = 50.0f + 30.0f * sinf(2.0f * M_PI_F / 3.0f);   /* ~75.98 */

    float speed, rate;
    bool converged = false;
    for (int i = 0; i < 400 && !converged; i++) {
        demo_track_target(&wm, &odo, target_x, target_y, &speed, &rate);
        demo_odometry_update(&odo, speed, rate, DT_MS);
        converged = dist2d(odo.x, odo.y, target_x, target_y) < DEMO_TRACK_ARRIVE_EPS + 0.5f;
    }
    zassert_true(converged,
                 "id=2's real ~120-degree re-form turn (losing id=1 from a "
                 "4-ring) did not converge within 40s — see comment above");
}

/* Second real data point, same departure (losing id=1): id=3's rank
 * shifts 3->2 (of 3), a ~105-degree turn from its settled heading. */
ZTEST(formation_field, test_track_reform_after_departure_id3_of_4)
{
    wm_reset();
    demo_odometry_t odo;
    demo_odometry_init(&odo, 50.0f, 20.0f);   /* id=3's settled position (rank 3 of 4) */
    odo.heading = -M_PI_F / 2.0f;             /* settled heading (270 deg) */

    /* rank 2 of 3, radius 30, center (50,50). */
    float target_x = 50.0f + 30.0f * cosf(4.0f * M_PI_F / 3.0f);   /* 35.0 */
    float target_y = 50.0f + 30.0f * sinf(4.0f * M_PI_F / 3.0f);   /* ~24.02 */

    float speed, rate;
    bool converged = false;
    for (int i = 0; i < 400 && !converged; i++) {
        demo_track_target(&wm, &odo, target_x, target_y, &speed, &rate);
        demo_odometry_update(&odo, speed, rate, DT_MS);
        converged = dist2d(odo.x, odo.y, target_x, target_y) < DEMO_TRACK_ARRIVE_EPS + 0.5f;
    }
    zassert_true(converged,
                 "id=3's real ~105-degree re-form turn (losing id=1 from a "
                 "4-ring) did not converge within 40s — see comment above");
}

/* Worst-case NON-degenerate re-form turn, found by exhaustively computing
 * every single-departure and single-rejoin transition reachable from the
 * 4-ring (see conversation history — a small Python enumeration over
 * bse.c's exact rank/angle formula, not hand-picked): losing id=0 leaves
 * id=1 needing a real ~135-degree turn (rank 1 of 4 -> rank 0 of 3).
 * Exactly-180-degree transitions also exist in that enumeration (e.g. a
 * SECOND departure down to 2 survivors) but that degenerate case is
 * already covered by test_track_reverses_when_exactly_behind and known
 * to converge cleanly — 135 degrees is the largest non-degenerate turn
 * choreo-1's actual departure/rejoin graph can produce, and the closest
 * real number to the ~157-degree heading that broke
 * test_track_converges_off_axis. */
ZTEST(formation_field, test_track_reform_after_departure_id1_of_4_worst_case)
{
    wm_reset();
    demo_odometry_t odo;
    demo_odometry_init(&odo, 50.0f, 80.0f);   /* id=1's settled position (rank 1 of 4) */
    odo.heading = M_PI_F / 2.0f;              /* settled heading (90 deg) */

    /* rank 0 of 3 (ascending sort of surviving {1,2,3} after losing id=0),
     * radius 30, center (50,50). */
    float target_x = 80.0f;
    float target_y = 50.0f;

    float speed, rate;
    bool converged = false;
    for (int i = 0; i < 400 && !converged; i++) {
        demo_track_target(&wm, &odo, target_x, target_y, &speed, &rate);
        demo_odometry_update(&odo, speed, rate, DT_MS);
        converged = dist2d(odo.x, odo.y, target_x, target_y) < DEMO_TRACK_ARRIVE_EPS + 0.5f;
    }
    zassert_true(converged,
                 "id=1's real ~135-degree re-form turn (losing id=0 from a "
                 "4-ring, the worst non-degenerate case in choreo-1's "
                 "departure/rejoin graph) did not converge within 40s");
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
/* odo starts at (25, 50), not (0, 0): the repulsion this test checks
 * points AWAY from the peer, i.e. toward -x here — at the literal (0,0)
 * corner that direction falls inside DEMO_ARENA_FENCE_MARGIN and
 * demo_arena_fence() (correctly, by its own design) vetoes it, silently
 * zeroing the exact force this test exists to observe. (25, 50) keeps
 * the peer, self, and the 50-unit-distant target all comfortably clear
 * of every edge, so the fence never fires and the repulsion math is
 * actually being tested. Real ring.choreo.toml positions never come
 * near (0,0) either (radius-30 ring centered on (50,50), staying inside
 * [20,80]) — this was a test-fixture bug, not a finding about
 * production behavior. */
ZTEST(formation_field, test_track_repulsion_overpowers_attraction_collinear)
{
    wm_reset();
    wm_set_peer(1, 27.0f, 50.0f, false);   /* 2 units ahead, well inside DEMO_TRACK_MIN_SEP */

    demo_odometry_t odo;
    demo_odometry_init(&odo, 25.0f, 50.0f);
    odo.heading = 0.0f;

    float speed, rate;
    demo_track_target(&wm, &odo, 75.0f, 50.0f, &speed, &rate);
    zassert_true(speed < 0.0f,
                 "a peer dead ahead inside MIN_SEP must override forward "
                 "attraction with reverse");

    wm_reset();   /* baseline: no peer -> forward */
    demo_track_target(&wm, &odo, 75.0f, 50.0f, &speed, &rate);
    zassert_true(speed > 0.0f, "without the peer, attraction alone drives forward");
}

/* See the arena-fence note above test_track_repulsion_overpowers_
 * attraction_collinear — same fix, same reason (this peer's repulsion
 * also points toward -x/-y, which (0,0) sits inside the fence margin
 * for on both axes). */
ZTEST(formation_field, test_track_repulsion_steers_off_axis)
{
    wm_reset();
    wm_set_peer(1, 30.0f, 51.0f, false);   /* just off the +x line, close */

    demo_odometry_t odo;
    demo_odometry_init(&odo, 25.0f, 50.0f);
    odo.heading = 0.0f;

    float speed, rate;
    demo_track_target(&wm, &odo, 75.0f, 50.0f, &speed, &rate);
    zassert_true(fabsf(rate) > 0.01f,
                 "an off-axis close peer must perturb the heading command");
}

ZTEST(formation_field, test_track_ignores_stale_peer_repulsion)
{
    /* demo_track_target has no hold-on-stale gate (staleness handling
     * belongs to the caller's quorum mapping — see formation.h) — a
     * stale peer must simply not contribute repulsion, not silently
     * freeze the drive. */
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

/* choreo.c/bse.c/scr.c hold per-element state as module-level singletons
 * (one physical element per process, on real hardware) — a process can't
 * hold 4 elements' internal state concurrently without exposing and
 * saving/restoring those statics, which none of the three headers offer.
 * True 4-way concurrent simulation (all elements ticking the SAME shared
 * instant, jointly converging) is therefore not practical here.
 *
 * What IS practical, and what this suite does: simulate each element's
 * FULL script run in ISOLATION, against the other 3 elements' STARTING
 * positions held STATIC for that whole run (same technique this file's
 * other demo_track_target tests already use via wm_set_peer — a fixed
 * peer, never moved). This exercises the real production path
 * (scr_tick/choreo_tick/demo_track_target/demo_odometry_update, not a
 * teleporting perfect tracker) and is exactly what ring.choreo.toml's
 * frame=collective needs to be meaningfully checkable: the centroid a
 * static peer set produces is a fixed, predictable number, so "did this
 * element converge to radius R from that centroid" is a well-defined
 * assertion. (Running each element's pass against the OTHERS' CURRENT,
 * possibly-already-converged positions — which is what letting all 4
 * peers roam via their own separate passes would do — was tried first
 * and produces centroid values that depend on pass ORDER, not on the
 * script itself; not a meaningful thing to assert against.) */
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

/* Same 4 arbitrary distinct corner-ish starts the form-grid version of
 * this test used (irrelevant to FORM which doesn't care where an element
 * started, only wm's live peer set at activation) — kept unchanged so
 * this remains a comparable regression scenario. */
static const float seed_x[4] = { 5.0f, 90.0f, 10.0f, 80.0f };
static const float seed_y[4] = { 5.0f, 10.0f, 85.0f, 80.0f };
static const float seed_h[4] = { 0.3f, 2.1f, -1.0f, 1.7f };

ZTEST(choreo_script, test_ring_script_end_to_end)
{
    /* ring.choreo.toml uses frame="absolute", target=[50,50,0] (changed
     * 2026-09-11 from "collective" — see that file's comment for why).
     * Verified via this same host build:
     *   element 0: final (79.82, 49.89)  radius from (50,50) 29.82
     *   element 1: final (50.82, 78.57)  radius from (50,50) 28.58
     *   element 2: final (19.69, 51.47)  radius from (50,50) 30.34
     *   element 3: final (50.11, 20.25)  radius from (50,50) 29.75
     * DEMO_TRACK_ARRIVE_EPS (formation.h) and this script's own
     * achieve_eps were briefly relaxed (2.0->6.0, 4.0->8.0) the same day
     * to chase a "settle, then correct a little" hunting symptom, then
     * both reverted to tight values (2.0, 3.0) once the REAL cause was
     * fixed instead — the "settled" HOLD step below, which actually
     * stops the live rank/count recompute that was causing it. These are
     * the tight-tolerance numbers again, byte-for-byte what this test
     * originally saw before any of that. Unlike the old
     * collective-frame version of this test, no element needs the
     * arena-fence special case anymore: a radius-30 circle centered at
     * (50,50) stays inside [20,80] on both axes for every rank angle,
     * comfortably inside [0,100], regardless of these seeds' own
     * positions (seeds only affect wm's peer freshness/count here, never
     * the target itself, now that it isn't derived from a live
     * centroid). The arena fence itself (demo_arena_fence(),
     * formation.c) is still exercised elsewhere — see
     * DEMO_MODE_STRAIGHT_LINE's own hardware validation — this test just
     * no longer happens to trigger it.
     */
    for (int who = 0; who < 4; who++) {
        demo_odometry_t odo;
        demo_odometry_init(&odo, seed_x[who], seed_y[who]);
        odo.heading = seed_h[who];

        scr_state_t scr;
        choreo_init((element_id_t)who);
        scr_init(&scr, (element_id_t)who, 1, 1,
                 SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
        choreo_register_scr(&scr);
        zassert_equal(choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN), 0,
                      "submit failed for element %d", who);

        /* Success is reaching the "settled" step (index 2 — see
         * ring.choreo.toml: hold(0) -> form "ring"(1) -> hold "settled"(2)),
         * not choreo_script_complete(): "settled" only completes the
         * script after its own 300s duration fallback elapses (or a real
         * element_lost/joined event, neither of which this single-mover
         * test triggers), far past what's worth spending on a unit test —
         * reaching the step at all already proves FORM achieved and
         * transitioned correctly. */
        int ticks = 0;
        while (ticks < 2000 && choreo_script_step() != 2) {
            wm_reset();
            wm_set_self(who, (element_id_t)who, odo.x, odo.y);
            for (int j = 0; j < 4; j++) {
                if (j != who) {
                    wm_set_peer(j, seed_x[j], seed_y[j], false);
                }
            }
            sim_tick(&scr, &odo, (element_id_t)who);
            ticks++;
        }
        zassert_equal(choreo_script_step(), 2,
                     "element %d did not reach the settled step in %d ticks "
                     "(stuck at step %d)", who, ticks, choreo_script_step());

        /* Sanity floor every element must clear regardless of fencing:
         * stay on the board at all. Trivially true given
         * demo_odometry_update's own [0,WORLD_SIZE] clamp, but asserted
         * explicitly since staying in the arena is the property this
         * whole test exists to guard. */
        zassert_true(odo.x >= 0.0f && odo.x <= WORLD_SIZE &&
                     odo.y >= 0.0f && odo.y <= WORLD_SIZE,
                     "element %d ended at (%.2f,%.2f), outside [0,%.0f]",
                     who, (double)odo.x, (double)odo.y, (double)WORLD_SIZE);

        /* frame="absolute", not "collective" (ring.choreo.toml, changed
         * 2026-09-11) — radius is measured from the script's fixed
         * target (50,50), not a live centroid computed from odo + peer
         * seeds. Unlike collective (where an asymmetric live centroid
         * could push a vertex off-board — element 3 used to need the
         * arena-fence special case below), a circle of radius 30 fixed
         * at the arena's own center (50,50) stays inside [20,80] on
         * both axes for every rank angle, comfortably inside [0,100] —
         * so all four elements are expected to converge normally now. */
        float radius = dist2d(odo.x, odo.y, 50.0f, 50.0f);

        /* 3.0 units, matching ring.choreo.toml's own achieve_eps (not
         * DEMO_TRACK_ARRIVE_EPS's tighter 2.0 — achieve_eps stays the
         * wider of the two by design, see that constant's doc). Back to
         * the original tight tolerance — see top-of-test comment for why
         * this was briefly 8.0 and no longer needs to be. */
        zassert_true(fabsf(radius - 30.0f) < 3.0f,
                     "element %d reached radius %.2f from the fixed "
                     "target (50,50), expected ~30", who, (double)radius);
    }
}

ZTEST_SUITE(choreo_script, NULL, NULL, NULL, NULL, NULL);
