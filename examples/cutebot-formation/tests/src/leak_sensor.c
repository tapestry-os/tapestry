/*
 * leak_sensor.c — leak-sensor.choreo.toml with a stationary sensor element
 *
 * Covers:
 *   - Track selection: the same compiled script puts a robot (locomotion)
 *     on "responders" and the sensor (none) on the catch-all.
 *   - The script end to end, per robot: "home" first, then parked while no
 *     leak is held; one slot of a FOUR-vertex ring about the sensor once the
 *     TARGET fact is held (the sensor, on another track, is no vertex);
 *     "home" when the fact is retracted, then waiting again.
 *   - Membership while waiting: a robot leaving sends the rest home; the
 *     sensor dropping out does not (it is on another track).
 *   - All four robots driving at once through the real control law: to the
 *     ring around the sensor, out from the start cross to home, and home
 *     from the 2026-10-19 rejoin position (three on a home triangle, the
 *     fourth back on its start cross).
 *
 * Its own translation unit: it includes the header examples/leak-sensor
 * ships (generated from the same .toml), while main.c keeps the ring
 * script that src/choreo_script.h currently holds.
 */

#include <zephyr/ztest.h>
#include <math.h>
#include <string.h>

#include "formation.h"
#include <tapestry/choreo.h>
#include <tapestry/bse.h>
#include "../../../leak-sensor/src/choreo_script.h"

#define DT_MS     100u
#define M_PI_F    3.14159265f

#define SENSOR_ID 31
#define SENSOR_X  70.0f
#define SENSOR_Y  70.0f
#define RING_R    20.0f
#define HOME_R    20.0f
#define EPS       3.0f     /* the script's achieve eps */
/* Centre to centre at contact: a ~110 mm robot against a board about its
 * size. */
#define CONTACT   DEMO_MM(110.0f)

#define STEP_HOME    0
#define STEP_WAIT    1
#define STEP_RING    2
#define STEP_SETTLED 3

static world_model_t wm;

static float dist2d(float ax, float ay, float bx, float by)
{
    return sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

static void start_pos(int id, float *x, float *y)
{
    /* main.c's compute_start_pos() for n_total = 4. */
    float a = 2.0f * M_PI_F * (float)id / 4.0f;
    *x = 50.0f + DEMO_START_RADIUS * cosf(a);
    *y = 50.0f + DEMO_START_RADIUS * sinf(a);
}

static void put(int id, float x, float y, bool self, bool stationary,
                uint8_t track)
{
    wm_entry_t *e = &wm.entries[id];
    e->is_active          = true;
    e->is_self            = self;
    e->is_stale           = false;
    e->state.id           = (element_id_t)id;
    e->state.position.x   = x;
    e->state.position.y   = y;
    e->state.health_flags = stationary ? ELEMENT_HEALTH_STATIONARY : 0u;
    e->state.current_track = track;
}

static void wm_clear(element_id_t owner)
{
    memset(&wm, 0, sizeof(wm));
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        wm.entries[i].state.id = ELEMENT_ID_INVALID;
    }
    wm.owner_id = owner;
    wm_facts_init(&wm);
}

static fact_t leak_fact(void)
{
    fact_t f = {0};
    f.key.type   = FACT_TYPE_TARGET;
    f.key.origin = SENSOR_ID;
    f.key.epoch  = 0x1234u;
    f.key.seq    = 0u;
    f.flags      = FACT_IMMUTABLE | FACT_PRIO(3);
    f.stamp      = 1u;
    f.pos        = (position_t){ SENSOR_X, SENSOR_Y, 0.0f };
    return f;
}

static void choreo_start(element_id_t id, scr_state_t *scr, scr_capability_t caps)
{
    choreo_init(id);
    scr_init(scr, id, 1, 1, caps);
    choreo_register_scr(scr);
    bse_set_form_bounds(DEMO_SQUARE_UNITS, DEMO_SQUARE_UNITS,
                        WORLD_SIZE - DEMO_SQUARE_UNITS,
                        WORLD_SIZE - DEMO_SQUARE_UNITS);
    zassert_equal(choreo_submit_tracks(&wm, k_choreo_tracks, CHOREO_N_TRACKS),
                  0, "submit_tracks failed for element %u", (unsigned)id);
}

/* One tick of element `who`'s Choreo; the robot is teleported onto its
 * directive (this test checks what the script asks for, not the driving —
 * test_all_four_reach_the_ring_around_the_sensor does that). */
static const tapestry_bse_directive_t *script_tick(scr_state_t *scr, int who)
{
    scr_tick(scr, &wm);
    choreo_tick(&wm, scr);
    choreo_publish_state(&wm.entries[who].state);
    return choreo_get_directive();
}

/* ── Tracks ───────────────────────────────────────────────────────────────── */

ZTEST(leak_sensor, test_robot_and_sensor_pick_their_tracks)
{
    scr_state_t scr;

    wm_clear(SENSOR_ID);
    put(SENSOR_ID, SENSOR_X, SENSOR_Y, true, true, 0);
    choreo_start(SENSOR_ID, &scr, SCR_CAP_SENSOR | SCR_CAP_ABS_POSITION);
    zassert_equal(choreo_current_track(), 1, "sensor should take the catch-all");

    wm_clear(0);
    put(0, 60.0f, 50.0f, true, false, 0);
    choreo_start(0, &scr, SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
    zassert_equal(choreo_current_track(), 0, "robot should take responders");
}

/* ── The script, per robot ────────────────────────────────────────────────── */

static position_t ring_slot[4];

/* `who` at (self_x, self_y), the other robots already on their home slots
 * (where they wait), the sensor on the board.  The peers hold still: this
 * checks what one robot's script asks for, and every robot must see the
 * same picture of the others or each would pick slots for a different
 * world. */
static void setup_world(int who, float self_x, float self_y)
{
    wm_clear((element_id_t)who);
    for (int j = 0; j < 4; j++) {
        float a = 2.0f * M_PI_F * (float)j / 4.0f;
        if (j == who) {
            put(j, self_x, self_y, true, false, 0);
        } else {
            put(j, 50.0f + HOME_R * cosf(a), 50.0f + HOME_R * sinf(a),
                false, false, 0);
        }
    }
    put(SENSOR_ID, SENSOR_X, SENSOR_Y, false, true, 1);
}

ZTEST(leak_sensor, test_script_waits_rings_and_resets)
{
    for (int who = 0; who < 4; who++) {
        float x, y;
        start_pos(who, &x, &y);
        setup_world(who, x, y);

        scr_state_t scr;
        choreo_start((element_id_t)who, &scr,
                     SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
        zassert_equal(choreo_current_track(), 0, "robot %d track", who);

        /* Boot: out to home (teleported onto its directive), then wait. */
        const tapestry_bse_directive_t *dir = NULL;
        zassert_equal(choreo_script_step(), STEP_HOME, "robot %d starts home", who);
        dir = script_tick(&scr, who);
        position_t home0 = dir->target;
        zassert_within(dist2d(home0.x, home0.y, 50.0f, 50.0f), HOME_R, 0.01f,
                       "robot %d first home slot", who);
        zassert_within(dist2d(home0.x, home0.y, x, y), HOME_R - DEMO_START_RADIUS,
                       0.01f, "robot %d should go straight out along its axis",
                       who);
        int ticks = 0;
        while (ticks < 100 && choreo_script_step() != STEP_WAIT) {
            wm.entries[who].state.position = home0;
            dir = script_tick(&scr, who);
            ticks++;
        }

        /* No leak: parked for a while, directive on its own station. */
        for (int i = 0; i < 50; i++) {
            dir = script_tick(&scr, who);
        }
        zassert_equal(choreo_script_step(), STEP_WAIT,
                      "robot %d left wait with no leak (step %d)", who,
                      choreo_script_step());
        if (dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
            zassert_true(dist2d(dir->target.x, dir->target.y,
                                home0.x, home0.y) < 0.5f,
                         "robot %d told to move while waiting", who);
        }

        /* The sensor's button: the fact arrives. */
        fact_t f = leak_fact();
        zassert_true(wm_fact_receive(&wm, &f), "fact not taken");

        ticks = 0;
        while (ticks < 50 && choreo_script_step() != STEP_RING) {
            dir = script_tick(&scr, who);
            ticks++;
        }
        zassert_equal(choreo_script_step(), STEP_RING,
                      "robot %d never reached ring (step %d)", who,
                      choreo_script_step());
        dir = script_tick(&scr, who);
        zassert_equal(dir->type, TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                      "robot %d ring directive", who);
        ring_slot[who] = dir->target;
        float r = dist2d(dir->target.x, dir->target.y, SENSOR_X, SENSOR_Y);
        zassert_within(r, RING_R, 0.01f,
                       "robot %d slot (%.1f,%.1f) is %.2f from the sensor",
                       who, (double)dir->target.x, (double)dir->target.y,
                       (double)r);

        /* Arrive (teleported) and settle into "settled". */
        ticks = 0;
        while (ticks < 100 && choreo_script_step() != STEP_SETTLED) {
            wm.entries[who].state.position = ring_slot[who];
            script_tick(&scr, who);
            ticks++;
        }
        zassert_equal(choreo_script_step(), STEP_SETTLED,
                      "robot %d never settled (step %d)", who,
                      choreo_script_step());

        /* sw1: the leak is cleared — home, a ring about the centre. */
        fact_t held;
        zassert_true(wm_fact_find(&wm, FACT_TYPE_TARGET, 0, &held), "held");
        zassert_equal(wm_fact_retract(&wm, &held.key), 0, "retract");
        ticks = 0;
        while (ticks < 50 && choreo_script_step() != STEP_HOME) {
            dir = script_tick(&scr, who);
            ticks++;
        }
        zassert_equal(choreo_script_step(), STEP_HOME,
                      "robot %d did not go home on retraction (step %d)", who,
                      choreo_script_step());
        dir = script_tick(&scr, who);
        position_t home = dir->target;
        zassert_within(dist2d(home.x, home.y, 50.0f, 50.0f), HOME_R, 0.01f,
                       "robot %d home slot (%.1f,%.1f)", who,
                       (double)home.x, (double)home.y);
        zassert_true(dist2d(home.x, home.y, SENSOR_X, SENSOR_Y) >=
                     DEMO_TRACK_STATIONARY_SEP + EPS,
                     "robot %d home slot inside the sensor's keep-out", who);

        ticks = 0;
        while (ticks < 100 && choreo_script_step() != STEP_WAIT) {
            wm.entries[who].state.position = home;
            script_tick(&scr, who);
            ticks++;
        }
        zassert_equal(choreo_script_step(), STEP_WAIT,
                      "robot %d did not wait again after home (step %d)", who,
                      choreo_script_step());
    }

    /* Four distinct slots, 90 degrees apart: a four-vertex ring, so the
     * sensor (track 1) was not counted. */
    for (int a = 0; a < 4; a++) {
        for (int b = a + 1; b < 4; b++) {
            float d = dist2d(ring_slot[a].x, ring_slot[a].y,
                             ring_slot[b].x, ring_slot[b].y);
            zassert_true(d > 2.0f * RING_R * sinf(M_PI_F / 4.0f) - 0.1f,
                         "robots %d and %d slots %.1f apart — not a "
                         "4-vertex ring", a, b, (double)d);
        }
    }
}

/* ── Membership while waiting ─────────────────────────────────────────────── */

/* Robot 0 waits on home with robots 1-3 and the sensor.  The sensor going
 * silent is not a membership change (another track); robot 3 going silent
 * is, and sends robot 0 home to re-form a 3-vertex ring. */
ZTEST(leak_sensor, test_wait_re_forms_home_on_membership_only)
{
    setup_world(0, 70.0f, 50.0f);   /* robot 0's home slot */
    scr_state_t scr;
    choreo_start(0, &scr, SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);

    int ticks = 0;
    while (ticks < 100 && choreo_script_step() != STEP_WAIT) {
        script_tick(&scr, 0);
        ticks++;
    }
    zassert_equal(choreo_script_step(), STEP_WAIT, "robot 0 never waited");

    /* Sensor silent for 10 s: still waiting. */
    wm.entries[SENSOR_ID].is_active = false;
    for (int i = 0; i < 100; i++) {
        script_tick(&scr, 0);
    }
    zassert_equal(choreo_script_step(), STEP_WAIT,
                  "the sensor dropping out must not move the robots (step %d)",
                  choreo_script_step());
    wm.entries[SENSOR_ID].is_active = true;

    /* Robot 3 switched off: home, as three. */
    wm.entries[3].is_active = false;
    ticks = 0;
    while (ticks < 100 && choreo_script_step() != STEP_HOME) {
        script_tick(&scr, 0);
        ticks++;
    }
    zassert_equal(choreo_script_step(), STEP_HOME,
                  "a robot leaving while waiting should send the rest home");
    const tapestry_bse_directive_t *dir = script_tick(&scr, 0);
    zassert_within(dist2d(dir->target.x, dir->target.y, 50.0f, 50.0f),
                   HOME_R, 0.01f, "home slot as three");
}

/* ── All four at once ─────────────────────────────────────────────────────── */

/* The ring slots, driven concurrently: every robot sees the others' live
 * positions and the stationary sensor.  The slots are computed here, not
 * taken from the script test, so the two tests stay independent. */
ZTEST(leak_sensor, test_all_four_reach_the_ring_around_the_sensor)
{
    /* rotation 0, count 4: vertices at 0/90/180/270 degrees about the
     * sensor, assigned by the minimum-total-distance rule from the start
     * cross — the same slots test_script_waits_rings_and_resets checks. */
    position_t slot[4];
    position_t vtx[4] = {
        { SENSOR_X + RING_R, SENSOR_Y, 0 }, { SENSOR_X, SENSOR_Y + RING_R, 0 },
        { SENSOR_X - RING_R, SENSOR_Y, 0 }, { SENSOR_X, SENSOR_Y - RING_R, 0 },
    };
    /* Brute force over the 24 assignments — independent of bse.c's. */
    int perm[4] = {0, 1, 2, 3}, best[4] = {0, 1, 2, 3};
    float best_cost = 1e9f;
    for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++)
    for (int c = 0; c < 4; c++) for (int d = 0; d < 4; d++) {
        if (a == b || a == c || a == d || b == c || b == d || c == d) {
            continue;
        }
        perm[0] = a; perm[1] = b; perm[2] = c; perm[3] = d;
        float cost = 0.0f;
        for (int k = 0; k < 4; k++) {
            float x, y;
            start_pos(k, &x, &y);
            cost += dist2d(x, y, vtx[perm[k]].x, vtx[perm[k]].y);
        }
        if (cost < best_cost) {
            best_cost = cost;
            memcpy(best, perm, sizeof(best));
        }
    }

    demo_odometry_t odo[4];
    for (int k = 0; k < 4; k++) {
        float x, y;
        start_pos(k, &x, &y);
        demo_odometry_init(&odo[k], x, y);
        odo[k].heading = 2.0f * M_PI_F * (float)k / 4.0f;
        slot[k] = vtx[best[k]];
    }

    float min_sensor = 1e9f, min_robot = 1e9f;
    bool  arrived = false;
    int   t;
    for (t = 0; t < 900 && !arrived; t++) {   /* 90 s: the ring step's timeout */
        float sp[4], rt[4];
        for (int k = 0; k < 4; k++) {
            wm_clear((element_id_t)k);
            for (int j = 0; j < 4; j++) {
                put(j, odo[j].x, odo[j].y, j == k, false, 0);
            }
            put(SENSOR_ID, SENSOR_X, SENSOR_Y, false, true, 1);
            demo_track_target(&wm, &odo[k], slot[k].x, slot[k].y, &sp[k], &rt[k]);
        }
        arrived = true;
        for (int k = 0; k < 4; k++) {
            demo_odometry_update(&odo[k], sp[k], rt[k], DT_MS);
            min_sensor = fminf(min_sensor,
                               dist2d(odo[k].x, odo[k].y, SENSOR_X, SENSOR_Y));
            for (int j = k + 1; j < 4; j++) {
                min_robot = fminf(min_robot, dist2d(odo[k].x, odo[k].y,
                                                    odo[j].x, odo[j].y));
            }
            if (dist2d(odo[k].x, odo[k].y, slot[k].x, slot[k].y) >= EPS) {
                arrived = false;
            }
        }
    }

    for (int k = 0; k < 4; k++) {
        TC_PRINT("robot %d -> slot (%.1f,%.1f), at (%.1f,%.1f)\n", k,
                 (double)slot[k].x, (double)slot[k].y,
                 (double)odo[k].x, (double)odo[k].y);
    }
    TC_PRINT("%s after %.1f s; closest to sensor %.1f, robot-robot %.1f\n",
             arrived ? "arrived" : "NOT arrived", (double)t * 0.1,
             (double)min_sensor, (double)min_robot);

    zassert_true(arrived, "not all robots reached their slots in 90 s");
    zassert_true(min_sensor > CONTACT, "a robot came within %.1f of the "
                 "sensor (contact %.1f)", (double)min_sensor, (double)CONTACT);
    zassert_true(min_robot > CONTACT, "two robots came within %.1f "
                 "(contact %.1f)", (double)min_robot, (double)CONTACT);
}

/* ── Rejoin: a robot switched back on while three wait at home ───────────── */

/* Slot k of a `count`-vertex circle (bse.c form_slot_offset(), rotation 0). */
static position_t circle_slot(float cx, float cy, float r, int k, int count)
{
    float a = 2.0f * M_PI_F * (float)k / (float)count;
    return (position_t){ cx + r * cosf(a), cy + r * sinf(a), 0.0f };
}

/* Minimum-total-distance assignment of 4 robots to 4 slots (bse.c's rule),
 * brute force: out[k] = slot index for robot k. */
static void assign4(const demo_odometry_t odo[4], const position_t slot[4],
                    int out[4])
{
    float best = 1e9f;
    for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++)
    for (int c = 0; c < 4; c++) for (int d = 0; d < 4; d++) {
        if (a == b || a == c || a == d || b == c || b == d || c == d) {
            continue;
        }
        int p[4] = { a, b, c, d };
        float cost = 0.0f;
        for (int k = 0; k < 4; k++) {
            cost += dist2d(odo[k].x, odo[k].y, slot[p[k]].x, slot[p[k]].y);
        }
        if (cost < best - 1e-4f) {
            best = cost;
            memcpy(out, p, sizeof(p));
        }
    }
}

typedef struct {
    bool  arrived;
    float t_s;
    int   reassignments;   /* slot changes after the first tick */
    float min_robot;
} form_run_t;

/* FORM about (cx, cy) with live re-assignment every tick, all four robots
 * driving at once through the real control law, sensor on the board. */
static form_run_t sim_form(demo_odometry_t odo[4], float cx, float cy, float r)
{
    position_t slot[4];
    for (int k = 0; k < 4; k++) {
        slot[k] = circle_slot(cx, cy, r, k, 4);
    }
    form_run_t res = { .min_robot = 1e9f };
    int prev[4] = { -1, -1, -1, -1 };
    int t;
    for (t = 0; t < 900 && !res.arrived; t++) {
        int asg[4];
        assign4(odo, slot, asg);
        for (int k = 0; k < 4; k++) {
            if (prev[k] >= 0 && asg[k] != prev[k]) {
                res.reassignments++;
            }
            prev[k] = asg[k];
        }
        float sp[4], rt[4];
        for (int k = 0; k < 4; k++) {
            wm_clear((element_id_t)k);
            for (int j = 0; j < 4; j++) {
                put(j, odo[j].x, odo[j].y, j == k, false, 0);
            }
            put(SENSOR_ID, SENSOR_X, SENSOR_Y, false, true, 1);
            position_t g = slot[asg[k]];
            demo_track_target(&wm, &odo[k], g.x, g.y, &sp[k], &rt[k]);
        }
        res.arrived = true;
        for (int k = 0; k < 4; k++) {
            demo_odometry_update(&odo[k], sp[k], rt[k], DT_MS);
            for (int j = k + 1; j < 4; j++) {
                res.min_robot = fminf(res.min_robot, dist2d(odo[k].x, odo[k].y,
                                                            odo[j].x, odo[j].y));
            }
            position_t g = slot[asg[k]];
            if (dist2d(odo[k].x, odo[k].y, g.x, g.y) >= EPS) {
                res.arrived = false;
            }
        }
    }
    res.t_s = (float)t * 0.1f;
    return res;
}

/* Three survivors on the 3-vertex home ring, robot `back` re-placed on its
 * start-cross position — the hardware rejoin of 2026-10-19. */
static void crowded_start(int back, demo_odometry_t odo[4])
{
    int k3 = 0;
    for (int k = 0; k < 4; k++) {
        if (k == back) {
            float x, y;
            start_pos(k, &x, &y);
            demo_odometry_init(&odo[k], x, y);
            odo[k].heading = 2.0f * M_PI_F * (float)k / 4.0f;
        } else {
            position_t p = circle_slot(50.0f, 50.0f, HOME_R, k3++, 3);
            demo_odometry_init(&odo[k], p.x, p.y);
            odo[k].heading = atan2f(p.y - 50.0f, p.x - 50.0f);
        }
    }
}

/* Deterministic +-1 unit placement error, so no robot sits exactly on
 * another's path (real placement never is: from exact slots a crossing
 * robot meets a parked one dead-centre, where a radial repulsion has no
 * sideways component). */
static uint32_t s_rng = 12345u;
static float jitter(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return ((float)(s_rng >> 8) / 16777216.0f) * 2.0f - 1.0f;
}

static void shake(demo_odometry_t odo[4])
{
    for (int k = 0; k < 4; k++) {
        odo[k].x += jitter();
        odo[k].y += jitter();
        odo[k].heading += 0.1f * jitter();
    }
}

/* Boot: the start cross out to home.  Each robot goes straight out along
 * its own axis; no slot changes hands on the way. */
ZTEST(leak_sensor, test_start_cross_to_home_is_clean)
{
    for (int trial = 0; trial < 3; trial++) {
        demo_odometry_t odo[4];
        for (int k = 0; k < 4; k++) {
            float x, y;
            start_pos(k, &x, &y);
            demo_odometry_init(&odo[k], x, y);
            odo[k].heading = 2.0f * M_PI_F * (float)k / 4.0f;
        }
        shake(odo);
        form_run_t home = sim_form(odo, 50.0f, 50.0f, HOME_R);
        zassert_true(home.arrived, "trial %d: home not reached", trial);
        zassert_equal(home.reassignments, 0, "trial %d: %d slot changes",
                      trial, home.reassignments);
        zassert_true(home.min_robot > CONTACT, "trial %d: robots %.1f apart",
                     trial, (double)home.min_robot);
    }
}

/* The 2026-10-19 rejoin: three waiting on the home triangle, the fourth
 * switched back on at its start-cross spot.  Ringing straight from there
 * re-assigned slots mid-transit on hardware (and in this simulation: 0-4
 * changes, 2 of 12 runs never completing).  The script now sends everyone
 * home first; that must be clean, and so must the ring after it. */
ZTEST(leak_sensor, test_rejoin_goes_home_cleanly_then_rings)
{
    for (int trial = 0; trial < 3; trial++)
    for (int back = 0; back < 4; back++) {
        demo_odometry_t odo[4];

        crowded_start(back, odo);
        shake(odo);
        form_run_t home = sim_form(odo, 50.0f, 50.0f, HOME_R);
        shake(odo);
        form_run_t ring = sim_form(odo, SENSOR_X, SENSOR_Y, RING_R);

        TC_PRINT("rejoin id %d: home %.1f s, %d changes, min %.1f; "
                 "ring %.1f s, %d changes, min %.1f\n", back,
                 (double)home.t_s, home.reassignments, (double)home.min_robot,
                 (double)ring.t_s, ring.reassignments, (double)ring.min_robot);
        zassert_true(home.arrived && home.reassignments == 0,
                     "rejoin id %d trial %d: home %s, %d slot changes", back,
                     trial, home.arrived ? "reached" : "NOT reached",
                     home.reassignments);
        zassert_true(ring.arrived, "rejoin id %d trial %d: ring not reached",
                     back, trial);
        zassert_true(home.min_robot > CONTACT && ring.min_robot > CONTACT,
                     "rejoin id %d trial %d: robots within contact", back, trial);
    }
}

ZTEST_SUITE(leak_sensor, NULL, NULL, NULL, NULL, NULL);
