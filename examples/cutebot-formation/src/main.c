/*
 * main.c — Tapestry Demo: Collective Formation (Cutebot ground rover)
 *
 * Build modes (Kconfig choice DEMO_MODE, see Kconfig):
 *
 *   DEMO_MODE_CHOREO (default) — the L5/L6/L7 path. ONE BINARY for all
 *     robots: element IDs are negotiated at boot (transport_negotiate_id,
 *     unchanged from the original design). A real L5 SCR (scr_init()/
 *     scr_tick() below) drives quorum from the actual world model — this
 *     app previously had ZERO L5, not even a synthetic-quorum stand-in.
 *     A declarative L7 Choreo script (../form-grid.choreo.toml) drives the
 *     robots through the L6 BSE:
 *       1. hold — station-keep at the boot-time position (coordinate-free)
 *       2. form (shape=grid) — arrange into a near-square grid centered on
 *          the arena, via demo_track_target()'s differential-drive
 *          go-to-point controller (formation.c) — the FIRST real-hardware
 *          consumer of Choreo's FORM goal anywhere in this repo (every
 *          other example either doesn't use it or only exercises it in
 *          host-side unit tests)
 *       3. hold — settle on the finished grid
 *     Script completion → directive IDLE → quiescence: this platform maps
 *     it to simply holding still (no takeoff/landing concept for a ground
 *     rover, unlike cf21bl-formation).
 *
 *   DEMO_MODE_SHOWCASE — the original, hardware-validated L4-only spring
 *     field (demo_compute_drive(), unchanged): pure emergent behavior, no
 *     SCR/L6/L7. Kept as a fallback — see Kconfig's help text.
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
 * DEMO_MODE_CHOREO ran end-to-end on four physical robots on 2026-08-24
 * (hold -> form(grid) -> hold, BLE gossip, real L5 quorum); approach speed
 * is untuned.  The tests/ suite covers the same path host-side.  The
 * boot/negotiation/sync-hold path and DEMO_MODE_SHOWCASE are unchanged
 * from the hardware-validated original — only the choice of what drives
 * speed_cmd/rate_cmd in the main loop is new. See the README's "Known
 * limitations" for the FORM/abs_position caveat (dead-reckoning drift).
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
#endif

#include "formation.h"

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

#define M_PI_F       3.14159265f

/*
 * Convergence hold, shared by the swarm branch and
 * DEMO_MODE_CONVERGE_TEST: wait until fresh peers actually show up in
 * the world model (a real gossip frame received and not yet stale)
 * before starting to move — successful auto-ID negotiation alone does
 * NOT mean that has happened yet. Caps at this so a genuinely missing
 * robot doesn't stall the demo forever.
 */
#define DEMO_SYNC_GRACE_MS 4000

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
 * The heading is set to the outward angle so every robot's spring force
 * projects fully forward on tick 1.  Without this, robots whose force
 * points opposite to heading 0 drive backward and collide with neighbors.
 * DEMO_MODE_CHOREO's FORM step doesn't need this property (demo_track_
 * target's go-to-point law works from any starting heading), but the
 * shared boot/sync-hold path runs before Choreo is submitted, so both
 * modes still benefit from a non-degenerate start.
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

    while (true) {
        float speed_cmd = 0.0f;
        float rate_cmd  = 0.0f;
        demo_drive_straight(&odo, &speed_cmd, &rate_cmd);

        demo_odometry_update(&odo, speed_cmd, rate_cmd, WM_CYCLE_MS);

#ifdef CONFIG_I2C
        cutebot_line_sample_t line;
        cutebot_line_poll(&line);
        demo_grid_correct(&odo, (line.left_edges > 0) || (line.right_edges > 0));
        trace_left_edges  += line.left_edges;
        trace_right_edges += line.right_edges;
#endif

        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);

        /* Green while the fence still lets it drive; blue once the
         * fence has vetoed the forward force and it's effectively
         * parked (the visible "did it stop at the border" signal). */
        substrate_set_signal(fabsf(speed_cmd) > 0.001f
                                  ? SUBSTRATE_SIGNAL_ACTIVE
                                  : SUBSTRATE_SIGNAL_IDLE);

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
     * branch's DEMO_SYNC_GRACE_MS loop below, reused here because
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
    for (uint32_t waited = 0; waited < DEMO_SYNC_GRACE_MS; waited += WM_CYCLE_MS) {
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
            break;
        }

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
        demo_grid_correct(&odo, (line.left_edges > 0) || (line.right_edges > 0));
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
#else /* !CONFIG_DEMO_MODE_STRAIGHT_LINE / !CONFIG_DEMO_MODE_CONVERGE_TEST
       * — the normal swarm demo */

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

#ifdef CONFIG_DEMO_MODE_CHOREO
    /* Real L5 SCR. quorum_min/quorum_target default to 1/1 (Kconfig) —
     * this script only ever needs one fresh peer, same threshold
     * cf21bl-formation and webots-formation use for the same reason.
     * SCR_CAP_ACTUATOR satisfies the script's CHOREO_CAP_LOCOMOTION
     * requirement. SCR_CAP_ABS_POSITION satisfies FORM's derived
     * CHOREO_CAP_ABS_POSITION requirement (frame=absolute, the default —
     * see choreo.c's derived_caps()) — see form-grid.choreo.toml's own
     * comment for why this is an honest but weaker claim than
     * cf21bl-formation's real lighthouse fix: cutebot's "absolute
     * position" is dead reckoning from compute_start_pos()'s shared seed
     * formula, not a real absolute sensor, and drifts over a long
     * mission. */
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
#endif

    float    speed_cmd    = 0.0f;
    float    rate_cmd     = 0.0f;
    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;   /* send immediately on first tick */

    /*
     * Convergence hold: wait until all n_total-1 expected peers are visible
     * and fresh in the world model before starting movement.  Prevents robots
     * that finished the ID window early from driving into a partial formation
     * while later robots are still exiting their own windows. See
     * DEMO_SYNC_GRACE_MS's doc (top of file, shared with
     * DEMO_MODE_CONVERGE_TEST) for the cap.  Runs identically in both
     * Choreo/Showcase modes — Choreo isn't ticked yet, so there is
     * nothing mode-specific here.
     */
    for (uint32_t waited = 0; waited < DEMO_SYNC_GRACE_MS; waited += WM_CYCLE_MS) {
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
            break;
        }

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
        /* Grid-based drift correction — runs in both DEMO_MODE_CHOREO
         * and DEMO_MODE_SHOWCASE (it only touches odo, nothing goal-
         * specific), before the corrected estimate is broadcast below.
         * cutebot_line_poll() is read-and-clear and interrupt-driven
         * (see cutebot_line.h), so this is exact regardless of
         * WM_CYCLE_MS — it never misses a crossing between polls. */
        cutebot_line_sample_t line;
        cutebot_line_poll(&line);
        demo_grid_correct(&odo, (line.left_edges > 0) || (line.right_edges > 0));

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

#ifdef CONFIG_DEMO_MODE_CHOREO
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
#else
        demo_compute_drive(&wm, &odo, &speed_cmd, &rate_cmd);
#endif

        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);
#ifdef CONFIG_DEMO_MODE_CHOREO
        demo_set_leds(&wm, choreo_current_indicator());
#else
        demo_set_leds(&wm, SUBSTRATE_SIGNAL_NONE);
#endif
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
