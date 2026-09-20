/*
 * main.c — Tapestry Demo: Collective Formation (Cutebot ground rover)
 *
 * Build modes (Kconfig choice DEMO_MODE, see Kconfig):
 *
 *   DEMO_MODE_CHOREO (default) — the L5/L6/L7 path, the only "normal"
 *     build mode (the original L4-only spring-field showcase was removed
 *     2026-09-13 — see git history). ONE BINARY for all robots: element
 *     IDs are negotiated at boot (transport_negotiate_id). A real L5 SCR
 *     (scr_init()/scr_tick() below) drives quorum from the actual world
 *     model. A declarative L7 Choreo script (../ring.choreo.toml,
 *     "choreo-1") drives the robots through the L6 BSE:
 *       1. hold — station-keep at the boot-time position (coordinate-free)
 *       2. form (shape=circle, frame=absolute) — arrange into an
 *          equidistant ring via demo_track_target()'s differential-drive
 *          go-to-point controller (formation.c) — the FIRST real-hardware
 *          consumer of Choreo's FORM goal anywhere in this repo
 *       3. hold ("settled") — freezes the achieved ring; only re-forms on
 *          a debounced element_lost/element_joined, not on ordinary
 *          gossip jitter (see ring.choreo.toml's own comments — this is
 *          what fixed the "settle, then keep re-adjusting" hunting seen
 *          on real hardware before 2026-09-12)
 *     Script completion → directive IDLE → quiescence: this platform maps
 *     it to simply holding still (no takeoff/landing concept for a ground
 *     rover, unlike cf21bl-formation).
 *
 *   DEMO_MODE_STRAIGHT_LINE — single-robot motion-primitive test, no
 *     transport/auto-ID/SCR/Choreo at all: demo_drive_straight() (constant
 *     forward force through the arena fence) + demo_grid_correct(), traced
 *     every second. See Kconfig's help text.
 *
 *   DEMO_MODE_CONVERGE_TEST — two-robot motion-primitive test, one step up
 *     from STRAIGHT_LINE: real transport/gossip for live peer-position
 *     awareness, but still no SCR/Choreo — demo_track_target() toward a
 *     fixed swap target, exercising its existing close-range-only
 *     repulsion backstop in isolation. See Kconfig's help text.
 *
 *   DEMO_MODE_WHEEL_CHARACTERIZE — single-robot per-wheel actuator test,
 *     bypassing formation.c entirely: a scripted sequence of raw
 *     left/right wheel percentages (per-wheel stiction ramp, a
 *     straight-line speed sweep, in-place CW/CCW rotation), judged by
 *     physically observing the robot, not by anything it self-reports.
 *     See Kconfig's help text.
 *
 *   DEMO_MODE_LINE_SENSOR_BENCH — no motors, no odometry: just logs
 *     P13/P14's raw GPIO levels every 500 ms so the line sensors' actual
 *     hardware response can be checked directly against different
 *     materials/angles, decoupled from any driving dynamics.
 *     See Kconfig's help text.
 *
 * DEMO_MODE_CHOREO ran end-to-end on four physical robots on 2026-08-24
 * (hold -> form(grid) -> hold, BLE gossip, real L5 quorum) and again on
 * 2026-09-12 with the ring script (hold -> form(circle) -> settled hold),
 * which settled cleanly with no jitter after the FORM/HOLD-debounce and
 * sync-barrier fixes. The tests/ suite covers the same path host-side.
 * See the README's "Known limitations" for the FORM/abs_position caveat
 * (dead-reckoning drift).
 *
 * ID assignment is handled by transport_negotiate_id() — see transport.h and
 * CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS for the auto-ID protocol details.
 *
 * See formation.h for physical calibration constants.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <tapestry/csm.h>
#include <tapestry/transport.h>
#include <tapestry/substrate.h>
#ifdef CONFIG_DEMO_MODE_CHOREO
#include <tapestry/scr.h>      /* real L5 quorum */
#include <tapestry/choreo.h>
/* The show itself. GENERATED from ../form-grid.choreo.toml (the file to
 * edit) by sdk/tools/choreoc.py — see the regeneration command in its
 * banner. */
#include "choreo_script.h"
#endif
#ifdef CONFIG_I2C
/* Ground-facing line-tracking sensors — real hardware only (CONFIG_I2C
 * is this app's proxy for "building for the physical Cutebot," same
 * condition CMakeLists.txt uses to compile cutebot_line.c at all).
 * Drives demo_grid_correct() (formation.h) below. */
#include "cutebot_line.h"
/* Raw 4-pin sensor bring-up diagnostic — DEMO_MODE_LINE_SENSOR_BENCH
 * only, see cutebot_line_bench.h. */
#include "cutebot_line_bench.h"
#endif

#include "formation.h"

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

#define M_PI_F       3.14159265f

/*
 * Convergence hold, shared by the swarm branch and
 * DEMO_MODE_CONVERGE_TEST: wait until fresh peers actually show up in
 * the world model (a real gossip frame received and not yet stale)
 * before starting to move — successful auto-ID negotiation alone does
 * NOT mean that has happened yet.
 *
 * UNBOUNDED — CHANGED 2026-09-11 from a 4s-then-give-up cap. The cap was
 * exactly the mechanism behind "bots starting at different times taint
 * the whole formation": a robot that happened to see fresh peers within
 * 4s proceeded into the real choreo loop (HOLD, then FORM — i.e. started
 * actually moving) while a robot powered on even slightly later was
 * still negotiating/gossiping, so the early robot's FORM target ended up
 * computed against a partial/stale view of the group. Removing the cap
 * makes this a true barrier: NO robot enters the main loop (motors are
 * never commanded before it) until every expected peer is confirmed
 * fresh, so a late power-on can no longer put an early one at an
 * advantage — every robot starts choreo from the same fully-formed view
 * of the group, regardless of how staggered the actual power-on was.
 * There is deliberately no solo-allowed escape hatch here (matches the
 * swarm branch's existing rationale below) — a genuinely missing robot
 * should be an obvious, visible "still waiting" state (see
 * DEMO_SYNC_LOG_INTERVAL_MS), not something that silently times out and
 * proceeds shorthanded.
 *
 * DEBOUNCED — ADDED 2026-09-11: `fresh >= n_total - 1` must hold
 * CONTINUOUSLY for DEMO_SYNC_SETTLE_MS before the barrier actually
 * releases, not just be true for one lucky poll — same "one lucky/
 * unlucky gossip frame must not fire this fleet-wide" lesson
 * CHOREO_MEMBERSHIP_HOLD_MS already encodes for element_lost/joined,
 * applied to the mirror-image case (a peer newly arriving). Any drop
 * back below n_total-1 resets the debounce timer.
 *
 * VALUE IS TIGHTLY BOUNDED BY WM_STALE_THRESHOLD_MS (csm.h, 1500 ms) and
 * GOSSIP_INTERVAL_MS (500 ms) — first attempt used 3000 ms (matching
 * CHOREO_MEMBERSHIP_HOLD_MS) and real hardware never satisfied it at
 * all: that's asking all 3 peer-links to each independently avoid a
 * >1500ms gap TWICE in a row, simultaneously, with zero misses across
 * any of them — a much harder bar than it looks on paper given real BLE
 * reception. Symptom was every robot's LED flickering yellow/green
 * (1/2 fresh) forever, never all four together — the barrier was
 * correctly refusing to release, just against a bar nothing could clear.
 * 800 ms is a little over half of one WM_STALE_THRESHOLD_MS renewal
 * window — enough to rule out a single-instant fluke without demanding
 * near-perfect multi-second reception across three links at once.
 */
#define DEMO_SYNC_LOG_INTERVAL_MS 2000u
#define DEMO_SYNC_SETTLE_MS       800u

#ifdef CONFIG_DEMO_MODE_CHOREO
/* Quorum-recovery hold, fed to scr_set_quorum_hold_ms() below — requires
 * a LOST -> >=DEGRADED recovery to be SUSTAINED before scr_tick() reports
 * it, filtering a single lucky gossip frame from flickering quorum up for
 * one cycle (see scr_set_quorum_hold_ms()'s own doc for the full
 * semantics). Same value cf21bl-formation and webots-formation use
 * (flight-tested there, 2026-07-19 flight 2) — kept as the best available
 * starting point for BLE gossip timing here too, NOT independently tuned
 * against real Cutebot/BLE loss rates (see this file's top-of-file
 * note on the 2026-08-24 run). */
#define QUORUM_UP_MS 2000
#endif

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/*
 * Seed each robot near the arena center on a DEMO_START_RADIUS circle,
 * and return the outward-facing heading angle.
 *
 * The radius keeps all robots visually clustered at start while placing
 * them well below DEMO_TARGET_SPACING apart, so spring repulsion
 * immediately drives them outward.  It is one chessboard square (10 units
 * = 126.6 mm), which puts 4 robots 179 mm apart centre-to-centre — clear
 * of the ~100 mm Cutebot width, and placeable by eye against the board's
 * own squares.  The former hardcoded 3.0 was 24 mm in the 800 mm arena it
 * was written for and only 38 mm here: a seed the robots could not
 * physically occupy without overlapping.
 *
 * The heading is set to the outward angle to match the physical placement
 * convention (see README) — a deterministic, non-degenerate starting
 * orientation, not a functional requirement: demo_track_target's
 * go-to-point law (FORM, and DEMO_MODE_CONVERGE_TEST's fixed-target swap)
 * works from any starting heading.
 *
 * Physical placement: orient each robot so its physical forward direction
 * matches its assigned heading (see README for per-ID compass directions).
 */
static void compute_start_pos(element_id_t id, int n_total,
                               float *x, float *y, float *heading)
{
    float a = 2.0f * M_PI_F * (float)id / (float)(n_total > 1 ? n_total : 1);
    *x       = 50.0f + DEMO_START_RADIUS * cosf(a);
    *y       = 50.0f + DEMO_START_RADIUS * sinf(a);
    *heading = a;
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    if (substrate_init() != 0) {
        LOG_WRN("substrate init failed — movement and signal disabled");
    }

#ifdef CONFIG_I2C
    if (cutebot_line_init() != 0) {
        LOG_WRN("line sensor init failed — grid drift correction disabled");
    }
#endif

#if defined(CONFIG_DEMO_MODE_STRAIGHT_LINE)
    /*
     * Isolated motion-primitive test: no transport/auto-ID, no SCR, no
     * Choreo, no peers — see this Kconfig option's help text for why
     * peer repulsion structurally cannot fire here. Seed odometry at a
     * fixed known placement and just keep calling demo_drive_straight()
     * + demo_grid_correct() forever, tracing every second.
     */
    LOG_INF("STRAIGHT-LINE TEST MODE — start=(%.1f,%.1f) heading=%.2f rad, "
            "no swarm, no auto-ID",
            (double)DEMO_TEST_START_X, (double)DEMO_TEST_START_Y,
            (double)DEMO_TEST_START_HEADING);

    demo_odometry_t odo;
    demo_odometry_init(&odo, DEMO_TEST_START_X, DEMO_TEST_START_Y);
    odo.heading = DEMO_TEST_START_HEADING;

    uint32_t trace_accum = 0;
#ifdef CONFIG_I2C
    uint32_t trace_left_edges  = 0;
    uint32_t trace_right_edges = 0;
#endif

    /*
     * Visual sign indicator for demo_grid_heading_correct()'s bench
     * verification (see formation.h's doc / README's "Known
     * limitations") — this mode has no cable long enough to watch the
     * serial console while physically dragging a robot around the
     * board, so a correction's SIGN needs to be readable from across the
     * room instead: orange (SUBSTRATE_SIGNAL_DEGRADED) briefly on a
     * POSITIVE correction, red (SUBSTRATE_SIGNAL_FAILED) briefly on a
     * NEGATIVE one, overriding the normal green/blue fence-status signal
     * for HEADING_FLASH_MS so it's not missed. DEGRADED/FAILED are
     * otherwise unused in this mode (no quorum, no real failure state to
     * confuse this with), unlike CONVERGE_TEST/the swarm mode where
     * FAILED means an actual grounding error — this flash is
     * STRAIGHT_LINE-only for exactly that reason.
     */
#define HEADING_FLASH_MS 1500u
    uint32_t heading_flash_remaining_ms = 0;
    bool     heading_flash_positive     = false;

    while (true) {
        float speed_cmd = 0.0f;
        float rate_cmd  = 0.0f;
        demo_drive_straight(&odo, &speed_cmd, &rate_cmd);

        demo_odometry_update(&odo, speed_cmd, rate_cmd, WM_CYCLE_MS);

#ifdef CONFIG_I2C
        cutebot_line_sample_t line;
        cutebot_line_poll(&line);
        float heading_err = 0.0f;
#ifdef CONFIG_DEMO_GRID_CORRECTION
        demo_grid_correct(&odo, (line.left_edges > 0) || (line.right_edges > 0));
        heading_err = demo_grid_heading_correct(
            &odo, speed_cmd,
            line.left_entered, line.left_entry_ms,
            line.right_entered, line.right_entry_ms);
#endif
        if (heading_err != 0.0f) {
            heading_flash_remaining_ms = HEADING_FLASH_MS;
            heading_flash_positive     = (heading_err > 0.0f);
        }
        trace_left_edges  += line.left_edges;
        trace_right_edges += line.right_edges;
#endif

        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);

        if (heading_flash_remaining_ms > 0) {
            /* Overrides the fence-status signal below for HEADING_FLASH_MS
             * after a correction fires — see the comment above. */
            substrate_set_signal(heading_flash_positive
                                      ? SUBSTRATE_SIGNAL_DEGRADED   /* orange: + */
                                      : SUBSTRATE_SIGNAL_FAILED);   /* red: -    */
            heading_flash_remaining_ms = (heading_flash_remaining_ms > WM_CYCLE_MS)
                                              ? heading_flash_remaining_ms - WM_CYCLE_MS
                                              : 0;
        } else {
            /* Green while the fence still lets it drive; blue once the
             * fence has vetoed the forward force and it's effectively
             * parked (the visible "did it stop at the border" signal). */
            substrate_set_signal(fabsf(speed_cmd) > 0.001f
                                      ? SUBSTRATE_SIGNAL_ACTIVE
                                      : SUBSTRATE_SIGNAL_IDLE);
        }

        trace_accum += WM_CYCLE_MS;
        if (trace_accum >= 1000) {
            trace_accum = 0;
#ifdef CONFIG_I2C
            LOG_INF("est=(%.2f,%.2f) hdg=%.2f spd=%.2f rate=%.2f "
                    "line L=%d R=%d edges(1s) L=%u R=%u",
                    (double)odo.x, (double)odo.y, (double)odo.heading,
                    (double)speed_cmd, (double)rate_cmd,
                    (int)line.left, (int)line.right,
                    (unsigned)trace_left_edges, (unsigned)trace_right_edges);
            trace_left_edges  = 0;
            trace_right_edges = 0;
#else
            LOG_INF("est=(%.2f,%.2f) hdg=%.2f spd=%.2f rate=%.2f "
                    "(no line sensor built — CONFIG_I2C off)",
                    (double)odo.x, (double)odo.y, (double)odo.heading,
                    (double)speed_cmd, (double)rate_cmd);
#endif
        }

        k_msleep(WM_CYCLE_MS);
    }
#elif defined(CONFIG_DEMO_MODE_CONVERGE_TEST)
    /*
     * Two-robot converge/swap test — see Kconfig's help text. Real
     * transport/gossip (demo_track_target()'s repulsion backstop needs
     * a live peer position), no SCR, no Choreo: target is fixed at
     * boot, not FORM-derived.
     */
    if (transport_init() != 0) {
        LOG_WRN("transport init failed — no peer awareness");
    }

    int n_total;
    element_id_t element_id = transport_negotiate_id(&n_total);

    if (n_total < 2) {
        /* Staggered power-on is the NORMAL case (hex files flashed to USB
         * mass storage sequentially), not a rare fault — see the swarm
         * branch's identical comment below. Retry rather than ground:
         * omitting this the first time this test was built is exactly
         * what turned "one bot powers on before the other" into "the
         * late bot's peer permanently never appears," which is a
         * distinct failure from a repulsion problem — it means the
         * early bot's world model NEVER receives a gossip frame from
         * the late one at all, not that repulsion looked at a peer and
         * failed to react to it. */
        LOG_ERR("id=%u auto-ID heard NO peers — grounded self-healing "
                "recovery (renegotiates until a peer is found)",
                (unsigned)element_id);
        substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);

        element_id = transport_negotiate_id_retry(element_id, &n_total);

        LOG_WRN("id=%u recovered — n_total=%d, proceeding",
                (unsigned)element_id, n_total);
        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
    }

    if (n_total != 2) {
        LOG_ERR("id=%u CONVERGE TEST requires exactly 2 robots, saw "
                "n_total=%d — grounded (not guessing who to swap with)",
                (unsigned)element_id, n_total);
        substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);
        while (true) {
            k_msleep(1000);
        }
    }

    substrate_identify(element_id);

    float sx, sy, shead;
    compute_start_pos(element_id, n_total, &sx, &sy, &shead);

    /* The other of exactly two — same deterministic formula the peer
     * itself used for its own seed, computed locally, not gossiped.
     * Live peer POSITION for the repulsion backstop still comes from
     * real gossip below; only the fixed target point is derived this
     * way. */
    float tx, ty, theading_unused;
    compute_start_pos((element_id_t)(1 - element_id), n_total,
                       &tx, &ty, &theading_unused);

    LOG_INF("CONVERGE TEST id=%u start=(%.1f,%.1f) target=(%.1f,%.1f)",
            (unsigned)element_id, (double)sx, (double)sy,
            (double)tx, (double)ty);

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.orientation = orientation_identity();
    own_state.position.x  = sx;
    own_state.position.y  = sy;
    transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);

    demo_odometry_t odo;
    demo_odometry_init(&odo, sx, sy);
    odo.heading = shead;

    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;   /* send immediately on first tick */

    /*
     * Convergence hold — same purpose and same pattern as the swarm
     * branch's barrier below (see DEMO_SYNC_LOG_INTERVAL_MS's doc,
     * top of file, for why it's unbounded), reused here because
     * omitting it is exactly what turned two power-timing tests into
     * collisions: successful ID negotiation only means both robots
     * agreed on element_id/n_total once, at negotiation time — it does
     * NOT mean this robot has yet received a single gossip frame with
     * the peer's actual position. Without this wait, demo_track_target()
     * spends its first however-many ticks computing repulsion against
     * an EMPTY world model (peer not fresh yet, so its own-position
     * gossip hasn't arrived/settled), and at this test's short 253 mm
     * starting distance and ~238 mm/s closing speed, that opening window
     * is not negligible — it can be most of the approach.
     */
    uint32_t peers_ready_ms = 0;
    for (uint32_t waited_ms = 0; ; waited_ms += WM_CYCLE_MS) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);
        substrate_set_signal(SUBSTRATE_SIGNAL_NONE);

        int fresh = 0;
        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm.entries[i];
            if (e->is_active && !e->is_self && !e->is_stale) {
                fresh++;
            }
        }
        if (fresh >= n_total - 1) {
            peers_ready_ms += WM_CYCLE_MS;
            if (peers_ready_ms >= DEMO_SYNC_SETTLE_MS) {
                break;
            }
        } else {
            peers_ready_ms = 0;
        }

        if (waited_ms % DEMO_SYNC_LOG_INTERVAL_MS == 0) {
            LOG_INF("id=%u still waiting for peer: %d/%d fresh (%u ms, "
                    "stable %u/%u ms)", (unsigned)element_id, fresh,
                    n_total - 1, (unsigned)waited_ms,
                    (unsigned)peers_ready_ms, (unsigned)DEMO_SYNC_SETTLE_MS);
        }

        /* Advance the Lamport clock while waiting — receivers reject any
         * frame whose logical_clock is not strictly newer, so a frozen
         * clock makes each peer flip fresh/stale on a ~5 s cycle (see the
         * swarm branch's identical barrier below for the full story). */
        wm_update_self(&wm, &own_state);

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }
        k_msleep(WM_CYCLE_MS);
    }

    LOG_INF("id=%u sync hold done — entering converge loop", (unsigned)element_id);

    uint32_t trace_accum  = 0;
#ifdef CONFIG_I2C
    uint32_t trace_left_edges  = 0;
    uint32_t trace_right_edges = 0;
#endif

    while (true) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        float speed_cmd = 0.0f;
        float rate_cmd  = 0.0f;
        demo_track_target(&wm, &odo, tx, ty, &speed_cmd, &rate_cmd);

        demo_odometry_update(&odo, speed_cmd, rate_cmd, WM_CYCLE_MS);

#ifdef CONFIG_I2C
        cutebot_line_sample_t line;
        cutebot_line_poll(&line);
#ifdef CONFIG_DEMO_GRID_CORRECTION
        demo_grid_correct(&odo, (line.left_edges > 0) || (line.right_edges > 0));
        demo_grid_heading_correct(&odo, speed_cmd,
                                   line.left_entered, line.left_entry_ms,
                                   line.right_entered, line.right_entry_ms);
#endif
        trace_left_edges  += line.left_edges;
        trace_right_edges += line.right_edges;
#endif

        own_state.position.x = odo.x;
        own_state.position.y = odo.y;
        wm_update_self(&wm, &own_state);

        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);

        substrate_set_signal(fabsf(speed_cmd) > 0.001f
                                  ? SUBSTRATE_SIGNAL_ACTIVE
                                  : SUBSTRATE_SIGNAL_IDLE);

        trace_accum += WM_CYCLE_MS;
        if (trace_accum >= 1000) {
            trace_accum = 0;

            /* Distance to the (one, by construction) fresh peer — the
             * number to actually watch: does it bottom out near
             * DEMO_TRACK_MIN_SEP and recover, or keep closing. */
            float peer_dist = -1.0f;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self && !e->is_stale) {
                    float dx = e->state.position.x - odo.x;
                    float dy = e->state.position.y - odo.y;
                    peer_dist = sqrtf(dx * dx + dy * dy);
                }
            }
#ifdef CONFIG_I2C
            LOG_INF("id=%u est=(%.2f,%.2f) tgt=(%.1f,%.1f) peer_dist=%.2f "
                    "repel=%d spd=%.2f rate=%.2f edges(1s) L=%u R=%u",
                    (unsigned)element_id, (double)odo.x, (double)odo.y,
                    (double)tx, (double)ty, (double)peer_dist,
                    (int)(peer_dist >= 0.0f && peer_dist < DEMO_TRACK_MIN_SEP),
                    (double)speed_cmd, (double)rate_cmd,
                    (unsigned)trace_left_edges, (unsigned)trace_right_edges);
            trace_left_edges  = 0;
            trace_right_edges = 0;
#else
            LOG_INF("id=%u est=(%.2f,%.2f) tgt=(%.1f,%.1f) peer_dist=%.2f "
                    "repel=%d spd=%.2f rate=%.2f",
                    (unsigned)element_id, (double)odo.x, (double)odo.y,
                    (double)tx, (double)ty, (double)peer_dist,
                    (int)(peer_dist >= 0.0f && peer_dist < DEMO_TRACK_MIN_SEP),
                    (double)speed_cmd, (double)rate_cmd);
#endif
        }

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }

        k_msleep(WM_CYCLE_MS);
    }
#elif defined(CONFIG_DEMO_MODE_WHEEL_CHARACTERIZE)
    /*
     * Per-wheel actuator characterization — see Kconfig's help text.
     * Deliberately bypasses formation.c's whole force model (no odometry,
     * no grid correction, no arena fence): the point is to observe the
     * REAL physical response with your eyes/a ruler, uncontaminated by
     * anything the firmware infers about its own position or heading —
     * an odometry-based judgment of "did it go straight" would just fold
     * the same wheel asymmetry this test exists to find into a
     * self-consistent (wrong) heading estimate instead of surfacing it.
     *
     * raw_wheel_twist() below is the exact algebraic inverse of
     * substrate_cutebot.c's to_pct() mapping (left_pct = (linear.x -
     * angular.z)*100, right_pct = (linear.x + angular.z)*100), so the
     * left_pct/right_pct requested here are what actually reaches
     * cutebot_drive(), not an approximation of it.
     *
     * NOT run through the arena fence — this can drive a robot well past
     * the chessboard's edge at the higher sweep percentages. Run this on
     * a long clear stretch of floor, not necessarily the board, and stay
     * ready to pick the robot up.
     */
    LOG_INF("WHEEL CHARACTERIZATION MODE — no fence, no odometry, raw "
            "left/right percentages only. Clear floor, not the board.");

    typedef struct {
        const char *label;
        float       left_pct;
        float       right_pct;
        uint32_t    hold_ms;
    } wheel_test_step_t;

    /* Phase 1: per-wheel stiction ramp, one wheel at a time (the OTHER
     * wheel held at exactly 0) — watch for the exact step at which THAT
     * wheel visibly starts to turn/creep. Low percentages, short holds:
     * at this level any real motion is a pivot around the stationary
     * wheel, not a long straight run, so this phase is safe indoors. */
    static const wheel_test_step_t k_steps[] = {
        { "L-stiction  8", 8,  0, 1200 }, { "L-stiction 12", 12,  0, 1200 },
        { "L-stiction 16", 16, 0, 1200 }, { "L-stiction 20", 20,  0, 1200 },
        { "L-stiction 24", 24, 0, 1200 }, { "L-stiction 28", 28,  0, 1200 },
        { "L-stiction 30", 30, 0, 1200 }, { "L-stiction 32", 32,  0, 1200 },
        { "L-stiction 34", 34, 0, 1200 },
        { "R-stiction  8", 0,  8, 1200 }, { "R-stiction 12", 0, 12, 1200 },
        { "R-stiction 16", 0, 16, 1200 }, { "R-stiction 20", 0, 20, 1200 },
        { "R-stiction 24", 0, 24, 1200 }, { "R-stiction 28", 0, 28, 1200 },
        { "R-stiction 30", 0, 30, 1200 }, { "R-stiction 32", 0, 32, 1200 },
        { "R-stiction 34", 0, 34, 1200 },

        /* Phase 2: straight-line sweep, both wheels equal, increasing
         * speed. Short holds even at low %, since nothing here bounds
         * travel distance — measure lateral drift + distance physically
         * (a ruler on the floor, or the chessboard's own gridlines) after
         * each step while the robot is stopped in the pause that follows. */
        { "straight 22", 22, 22, 1000 },
        { "straight 35", 35, 35, 1000 },
        { "straight 50", 50, 50,  800 },

        /* Phase 3: in-place rotation, both directions — a DIFFERENT
         * asymmetry axis than straight-line drift: does turning speed
         * match between CW and CCW for the same equal-and-opposite
         * command? */
        { "rotate CW ",  40, -40, 1000 },
        { "rotate CCW",  -40, 40, 1000 },
    };

    for (size_t i = 0; i < ARRAY_SIZE(k_steps); i++) {
        const wheel_test_step_t *s = &k_steps[i];

        LOG_INF("step %u/%u: %s  L=%.0f%% R=%.0f%%  hold=%u ms",
                (unsigned)(i + 1), (unsigned)ARRAY_SIZE(k_steps), s->label,
                (double)s->left_pct, (double)s->right_pct,
                (unsigned)s->hold_ms);

        substrate_twist_t twist = {
            .linear  = { .x = (s->left_pct + s->right_pct) / 200.0f },
            .angular = { .z = (s->right_pct - s->left_pct) / 200.0f },
        };
        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
        substrate_move(&twist);
        k_msleep(s->hold_ms);

        substrate_twist_t stop = {0};
        substrate_move(&stop);
        substrate_set_signal(SUBSTRATE_SIGNAL_IDLE);

        /* Pause between steps — time to note/measure the result and, for
         * the straight-line sweep, reposition before the next step, WITHOUT
         * the robot drifting on a nonzero residual command. */
        k_msleep(2000);
    }

    LOG_INF("WHEEL CHARACTERIZATION COMPLETE — motors off, halting.");
    substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);   /* red: done, not an error */
    while (true) {
        k_msleep(1000);
    }
#elif defined(CONFIG_DEMO_MODE_LINE_SENSOR_BENCH)
    /*
     * Raw line-sensor bring-up — see Kconfig's help text. No motors, no
     * odometry, no interrupts: just log what P13/P14 actually read,
     * unfiltered, so materials/angles/heights can be tested by eye
     * against a live log instead of inferring anything from driving
     * behavior.
     */
    if (cutebot_line_bench_init() != 0) {
        LOG_ERR("cutebot_line_bench_init failed — P13/P14 not ready. "
                "Check bbc_microbit_v2.overlay matches this board's "
                "actual wiring.");
        substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);
        while (true) {
            k_msleep(1000);
        }
    }

    LOG_INF("LINE SENSOR BENCH MODE — logging P13/P14 raw levels every "
            "500 ms. No inversion: 1/0 exactly as the pin reads.");

    while (true) {
        cutebot_line_bench_sample_t s;
        cutebot_line_bench_read(&s);

        LOG_INF("P13: %d  P14: %d", s.p13, s.p14);

        k_msleep(500);
    }
#else /* !CONFIG_DEMO_MODE_STRAIGHT_LINE / !CONFIG_DEMO_MODE_CONVERGE_TEST /
       * !CONFIG_DEMO_MODE_WHEEL_CHARACTERIZE /
       * !CONFIG_DEMO_MODE_LINE_SENSOR_BENCH — the normal swarm demo */

    if (transport_init() != 0) {
        LOG_WRN("transport init failed — no peer awareness");
    }

    int n_total;
    element_id_t element_id = transport_negotiate_id(&n_total);

    if (n_total < 2) {
        /* Auto-ID heard nobody. Since hex files are flashed to USB mass
         * storage sequentially, staggered reboots are the NORMAL case
         * here, not a rare fault — without recovery this robot would sit
         * isolated permanently (red headlights, no motion, Choreo frozen
         * on quorum loss) until power-cycled by hand. See transport_
         * negotiate_id_retry()'s doc for the two ways this resolves; no
         * solo-allowed escape hatch here (unlike cf21bl-formation) since a
         * grid formation with one missing robot is a visibly different
         * demo, not a silent degradation worth shipping quietly. */
        LOG_ERR("id=%u auto-ID heard NO peers — grounded self-healing "
                "recovery (renegotiates until a peer is found)",
                (unsigned)element_id);
        substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);

        element_id = transport_negotiate_id_retry(element_id, &n_total);

        LOG_WRN("id=%u recovered — n_total=%d, proceeding",
                (unsigned)element_id, n_total);
        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
    }

    /*
     * Announce the negotiated id BEFORE compute_start_pos() seeds this
     * robot's position/heading from it and broadcasts that seed as
     * ground truth (own_state below) — placement has to happen while
     * the id is still just a blink pattern, not after.  This replaces
     * the old "place all four by compass heading before power-on, then
     * separately identify one robot over serial and sticker it" flow —
     * id is unknowable before boot (FICR nonce rank), so that flow
     * required a one-time-per-robot calibration step done in advance.
     * Now: power on all four together, watch each one blink its own id
     * (element_id + 1 white flashes, repeated), and place/orient each
     * while it's blinking. See the README's placement table for which
     * heading each id faces. */
    substrate_identify(element_id);

    float sx, sy, shead;
    compute_start_pos(element_id, n_total, &sx, &sy, &shead);

    LOG_INF("Demo — element %u  start (%.1f, %.1f)  heading=%.2f rad  target_spacing=%.1f",
            (unsigned)element_id, (double)sx, (double)sy,
            (double)shead, (double)DEMO_TARGET_SPACING);

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.orientation = orientation_identity();  /* ground rover, no attitude sensing */
    own_state.position.x  = sx;
    own_state.position.y  = sy;

    transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);   /* pure AP — never freeze */

    demo_odometry_t odo;
    demo_odometry_init(&odo, sx, sy);
    odo.heading = shead;

    /* Real L5 SCR. quorum_min/quorum_target default to 1/1 (Kconfig) —
     * this script only ever needs one fresh peer, same threshold
     * cf21bl-formation and webots-formation use for the same reason.
     * SCR_CAP_ACTUATOR satisfies the script's CHOREO_CAP_LOCOMOTION
     * requirement. SCR_CAP_ABS_POSITION satisfies FORM's derived
     * CHOREO_CAP_ABS_POSITION requirement (frame=absolute) — see
     * ring.choreo.toml's own comment for why this is an honest but
     * weaker claim than cf21bl-formation's real lighthouse fix: cutebot's
     * "absolute position" is dead reckoning from compute_start_pos()'s
     * shared seed formula, not a real absolute sensor, and drifts over a
     * long mission. */
    scr_state_t scr;
    scr_init(&scr, element_id,
        (uint8_t)CONFIG_TAPESTRY_QUORUM_MIN,
        (uint8_t)CONFIG_TAPESTRY_QUORUM_TARGET,
        SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
    scr_set_quorum_hold_ms(&scr, QUORUM_UP_MS);

    choreo_init(element_id);
    choreo_register_scr(&scr);
    if (choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN) != 0) {
        LOG_ERR("id=%u choreo script rejected — staying grounded",
                (unsigned)element_id);
        substrate_set_power(SUBSTRATE_POWER_SLEEP);
        return -1;
    }
    LOG_INF("id=%u choreo \"%s\" loaded — %u steps, time bound %u s",
            (unsigned)element_id, CHOREO_NAME,
            (unsigned)CHOREO_SCRIPT_LEN,
            (unsigned)(CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u));

    float    speed_cmd    = 0.0f;
    float    rate_cmd     = 0.0f;
    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;   /* send immediately on first tick */

    /*
     * Convergence hold: wait until all n_total-1 expected peers are visible
     * and fresh in the world model before starting movement — a true
     * barrier, unbounded (see DEMO_SYNC_LOG_INTERVAL_MS's doc, top of
     * file, for why the old fixed cap here was exactly what let robots
     * that finished early start moving while later ones were still
     * negotiating/gossiping). Choreo isn't ticked yet, so there is
     * nothing mode-specific here.
     */
    uint32_t peers_ready_ms = 0;
    for (uint32_t waited_ms = 0; ; waited_ms += WM_CYCLE_MS) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);
        demo_set_leds(&wm, SUBSTRATE_SIGNAL_NONE);

        int fresh = 0;
        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm.entries[i];
            if (e->is_active && !e->is_self && !e->is_stale) {
                fresh++;
            }
        }
        if (fresh >= n_total - 1) {
            peers_ready_ms += WM_CYCLE_MS;
            if (peers_ready_ms >= DEMO_SYNC_SETTLE_MS) {
                break;
            }
        } else {
            peers_ready_ms = 0;
        }

        if (waited_ms % DEMO_SYNC_LOG_INTERVAL_MS == 0) {
            LOG_INF("id=%u still waiting for peers: %d/%d fresh (%u ms, "
                    "stable %u/%u ms)", (unsigned)element_id, fresh,
                    n_total - 1, (unsigned)waited_ms,
                    (unsigned)peers_ready_ms, (unsigned)DEMO_SYNC_SETTLE_MS);
        }

        /* Advance this element's Lamport clock every tick, as the main loop
         * does.  A receiver (wm_receive_gossip) drops any frame whose
         * logical_clock is not strictly newer than the one it holds, and
         * wm_update_self() is the only thing that advances it — without
         * this call every frame sent during the barrier carries the SAME
         * clock, so each peer is accepted once and then rejected until it
         * ages out (WM_EXPIRE_THRESHOLD_MS) and is re-accepted from
         * scratch.  Each peer therefore flips fresh/stale on a ~5 s cycle
         * (yellow LEDs "ping-ponging" between robots) and the barrier only
         * completes when the cycles happen to overlap.  Host repro:
         * 6 of 60 frames accepted, peer fresh 58% of the time. */
        wm_update_self(&wm, &own_state);

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }
        k_msleep(WM_CYCLE_MS);
    }

    LOG_INF("Demo ready — entering main loop");

    while (true) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        demo_odometry_update(&odo, speed_cmd, rate_cmd, WM_CYCLE_MS);

#ifdef CONFIG_I2C
        /* Grid-based drift correction — it only touches odo, nothing
         * goal-specific, before the corrected estimate is broadcast
         * below. cutebot_line_poll() is read-and-clear and interrupt-driven
         * (see cutebot_line.h), so this is exact regardless of
         * WM_CYCLE_MS — it never misses a crossing between polls. */
        cutebot_line_sample_t line;
        cutebot_line_poll(&line);
#ifdef CONFIG_DEMO_GRID_CORRECTION
        demo_grid_correct(&odo, (line.left_edges > 0) || (line.right_edges > 0));
        demo_grid_heading_correct(&odo, speed_cmd,
                                   line.left_entered, line.left_entry_ms,
                                   line.right_entered, line.right_entry_ms);
#endif

        /* Accumulated for the 1 Hz trace below — a single 100 ms poll
         * usually shows 0 edges even seconds after a real crossing
         * (crossings are brief), so the per-poll count alone is a poor
         * "is this sensor even working" signal over serial. These sum
         * across the whole 1 s window and are cleared when it prints. */
        static uint32_t trace_left_edges;
        static uint32_t trace_right_edges;
        trace_left_edges  += line.left_edges;
        trace_right_edges += line.right_edges;
#endif

        own_state.position.x = odo.x;
        own_state.position.y = odo.y;
        wm_update_self(&wm, &own_state);

        /* Real L5: recompute quorum from the actual world model.
         * scr_set_quorum_hold_ms() above means scr.quorum_state is
         * already the held view — see that call site's comment. */
        scr_tick(&scr, &wm);
        bool quorum_up = scr.quorum_state != SCR_QUORUM_LOST;

        /* HARD_RT gossip on the quorum-loss edge — same pattern as
         * cf21bl-formation's/webots-formation's main.c: fires once per
         * outage (the NONE/CLEARED -> TRIGGERED edge), not every tick it
         * persists. Mirrors tapestry-os/subsys/runtime/runtime.c's step 4b. */
        static scr_abort_state_t last_abort_state;
        scr_abort_state_t abort_state = scr_get_abort_state(&scr);
        if (abort_state == SCR_ABORT_TRIGGERED &&
            last_abort_state != SCR_ABORT_TRIGGERED) {
            own_state.update_seq++;
            choreo_publish_state(&own_state);
            LOG_WRN("id=%u quorum LOST — sending HARD_RT gossip now",
                    (unsigned)element_id);
            transport_send(&own_state, TAPESTRY_QOS_HARD_RT);
            gossip_accum = 0;
        }
        last_abort_state = abort_state;

        choreo_tick(&wm, &scr);
        /* Publish this element's own-goal achievement so peers can
         * aggregate the FORM step's scope="all" collective predicate
         * (choreo_collective_achieved()) — every element main loop that
         * gossips has to call this once per tick or the bit stays 0
         * forever and scope="all" can never advance (see this function's
         * own doc). */
        choreo_publish_state(&own_state);

        static int last_step = -2;
        if (choreo_script_step() != last_step) {
            last_step = choreo_script_step();
            LOG_INF("id=%u choreo step %d %s", (unsigned)element_id,
                    last_step,
                    choreo_goal_status() == CHOREO_STATE_SUSPENDED
                        ? "(suspended)" : "");
        }

        const tapestry_bse_directive_t *dir = choreo_get_directive();
        /* Per-goal quorum at the tracking layer: HOLD's directive
         * references only this robot's own captured station, so it is
         * tracked even with quorum lost (a solo robot station-keeps
         * properly); FORM is peer-referential (reads task_slot/swarm_size
         * from scr) and freezes while LOST. Same idiom as
         * cf21bl-formation's/webots-formation's main.c. */
        bool self_referential = choreo_current_goal_type() == CHOREO_GOAL_HOLD;
        if ((quorum_up || self_referential) &&
            dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
            demo_track_target(&wm, &odo, dir->target.x, dir->target.y,
                              &speed_cmd, &rate_cmd);
        } else {
            /* IDLE (script complete) or a peer-referential goal frozen by
             * quorum loss: station-keep by commanding nothing rather than
             * chasing a stale target. */
            speed_cmd = 0.0f;
            rate_cmd  = 0.0f;
        }

        /*
         * Throttled position/command trace, at LOG_INF so it is visible
         * with zero config change over a plain wired serial connection —
         * formation.c's existing per-tick traces are LOG_DBG, which
         * CONFIG_LOG_DEFAULT_LEVEL=3 (prj.conf) silently drops at
         * runtime even though they're compiled in. 1 Hz is dense enough
         * to see whether a robot is chasing a target, holding, or
         * (the failure mode this exists to catch) still commanding
         * outward motion after its own position estimate has pinned at
         * a WORLD_SIZE edge — watch for est==0 or est==100 alongside a
         * nonzero spd persisting for more than a couple of lines.
         */
        static uint32_t trace_accum = 0;
        trace_accum += WM_CYCLE_MS;
        if (trace_accum >= 1000) {
            trace_accum = 0;
            if (dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
                LOG_INF("id=%u step=%d est=(%.1f,%.1f) tgt=(%.1f,%.1f) "
                        "spd=%.2f rate=%.2f",
                        (unsigned)element_id, choreo_script_step(),
                        (double)odo.x, (double)odo.y,
                        (double)dir->target.x, (double)dir->target.y,
                        (double)speed_cmd, (double)rate_cmd);
            } else {
                LOG_INF("id=%u step=%d est=(%.1f,%.1f) dir=%d "
                        "spd=%.2f rate=%.2f",
                        (unsigned)element_id, choreo_script_step(),
                        (double)odo.x, (double)odo.y, (int)dir->type,
                        (double)speed_cmd, (double)rate_cmd);
            }
#ifdef CONFIG_I2C
            /* Sanity signal for bench-verifying the sensor itself before
             * trusting it in a full run: drag a robot BY HAND across a
             * few gridlines with the trace running and confirm L/R flip
             * as expected (see the overlay comment for polarity) and
             * edges increments — separately from whether the demo's
             * driving behavior looks right. */
            LOG_INF("id=%u line L=%d R=%d edges(1s) L=%u R=%u",
                    (unsigned)element_id, (int)line.left, (int)line.right,
                    (unsigned)trace_left_edges, (unsigned)trace_right_edges);
            trace_left_edges  = 0;
            trace_right_edges = 0;
#endif
        }

        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);
        demo_set_leds(&wm, choreo_current_indicator());
        demo_display_position(&odo);

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
#endif /* CONFIG_DEMO_MODE_STRAIGHT_LINE */
}
