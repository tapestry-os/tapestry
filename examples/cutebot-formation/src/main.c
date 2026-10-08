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
 *   DEMO_MODE_DISCOVERY_BENCH — leak/spill discovery bench diagnostic:
 *     single robot, demo_drive_straight() (same as STRAIGHT_LINE) across
 *     the real printed target marker, running demo_discovery_detect()
 *     against the raw line-sensor reading every poll (no grid correction
 *     — not needed, see that function's doc) and logging every dark
 *     reading plus a clear line and solid green LED the instant
 *     discovery confirms. See Kconfig's help text.
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
#include <string.h>
#include <tapestry/csm.h>
#include <tapestry/transport.h>
#include <tapestry/substrate.h>
#ifdef CONFIG_DEMO_MODE_CHOREO
#include <tapestry/scr.h>      /* real L5 quorum */
#include <tapestry/choreo.h>
/* The show itself. GENERATED from one of ../{ring,leak,spill}.choreo.toml
 * (the file to edit) by sdk/tools/choreoc.py — see the regeneration command in its
 * banner. */
#include "choreo_script.h"
#endif
#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
/* Cutebot Pro: motor/encoder/LED/line-sensor are all one I2C peripheral —
 * see cutebot_pro.h. Drives demo_junction_correct() (formation.h) below,
 * not demo_grid_correct()/demo_grid_heading_correct() (Mini-only, need
 * cutebot_line.h's edge-timing data this hardware doesn't provide). */
#include "cutebot_pro.h"
#include "turn_calibration.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#elif defined(CONFIG_I2C)
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
#ifdef CONFIG_DEMO_BLE_DEBUG
#include "ble_debug.h"
#endif

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

/* A log line that is also a debug EVENT: with CONFIG_DEMO_BLE_DEBUG it is
 * mirrored into the wireless text stream (ble_debug.h) for untethered
 * runs. Kept to state changes — the 1 Hz trace stays serial-only (the
 * status packet carries its numbers). */
#ifdef CONFIG_DEMO_BLE_DEBUG
#define DEMO_EVT(level, ...)                 \
    do {                                     \
        LOG_##level(__VA_ARGS__);            \
        ble_debug_printf(__VA_ARGS__);       \
    } while (0)
#else
#define DEMO_EVT(level, ...) LOG_##level(__VA_ARGS__)
#endif

#define M_PI_F       3.14159265f

#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
/* Signed per-wheel distance (cm) since the previous call, from the raw
 * cumulative counters. Shared by the choreo loop's odometry and
 * DEMO_MODE_ENCODER_BENCH so the bench exercises the exact sign/scale
 * handling the demo uses. Direction is the counter's own sign: it counts
 * down in reverse (bench, 2026-10-08). Signing by the command instead
 * missed every wheel still rolling the old way after a reversal or a
 * stop — six CCW/CW pivot flips turned the robot 30 degrees that it
 * never saw (bench step 6). */
/* Encoder plausibility guard (2026-10-06): the Pro's replies carry no
 * checksum, so a read that completes is trusted. Twice in one run a robot
 * whose chassis was browning out returned a nonsense distance counter
 * (all-ones bytes as its controller dropped off the bus), and the
 * odometry turned it into a jump of 50-66 units while parked. A wheel
 * cannot beat the firmware's 50 cm/s ceiling, so any delta beyond
 * 1.5 x that over the interval, plus PRO_ENC_SLACK_CM, is a bad read:
 * it contributes no motion and the last good counter stays the baseline.
 * PRO_ENC_REBASE_STREAK implausible reads in a row are taken as a real
 * counter reset and become the new baseline. */
#define PRO_ENC_MAX_CMS        50.0f
#define PRO_ENC_SLACK_CM       3.0f
#define PRO_ENC_REBASE_STREAK  3u

typedef enum {
    PRO_ENC_OK = 0,
    PRO_ENC_REJECTED,     /* implausible: ignored, baseline kept */
    PRO_ENC_REBASED,      /* implausible PRO_ENC_REBASE_STREAK times: new baseline */
} pro_enc_result_t;

typedef struct {
    uint8_t  streak;      /* implausible reads in a row */
    uint32_t total;       /* implausible reads since start */
    int32_t  last_raw_l;  /* the latest delta, counts (for logging) */
    int32_t  last_raw_r;
} pro_enc_guard_t;

static __maybe_unused pro_enc_result_t pro_wheel_deltas_cm(const cutebot_pro_encoders_t *enc,
                                            int32_t *prev_l, int32_t *prev_r,
                                            uint32_t dt_ms, pro_enc_guard_t *g,
                                            float *dl_cm, float *dr_cm)
{
    int32_t raw_l = enc->left_distance  - *prev_l;
    int32_t raw_r = enc->right_distance - *prev_r;
    g->last_raw_l = raw_l;
    g->last_raw_r = raw_r;
    *dl_cm = 0.0f;
    *dr_cm = 0.0f;

    float max_counts = (PRO_ENC_MAX_CMS * 1.5f * (float)dt_ms * 0.001f +
                        PRO_ENC_SLACK_CM) * CUTEBOT_PRO_DIST_COUNTS_PER_CM;
    if (fabsf((float)raw_l) > max_counts || fabsf((float)raw_r) > max_counts) {
        g->total++;
        if (++g->streak < PRO_ENC_REBASE_STREAK) {
            return PRO_ENC_REJECTED;
        }
        g->streak = 0;
        *prev_l = enc->left_distance;
        *prev_r = enc->right_distance;
        return PRO_ENC_REBASED;
    }
    g->streak = 0;
    *prev_l = enc->left_distance;
    *prev_r = enc->right_distance;

    *dl_cm = (float)raw_l / CUTEBOT_PRO_DIST_COUNTS_PER_CM;
    *dr_cm = (float)raw_r / CUTEBOT_PRO_DIST_COUNTS_PER_CM;
    return PRO_ENC_OK;
}

/* This micro:bit's key in turn_calibration.h: the first four bytes of its
 * device id.  (Until 2026-10-07 this equalled the auto-ID nonce shown in
 * the observer's beacon events; the nonce now hashes the whole id, so read
 * the key from this robot's own log.) */
static __maybe_unused uint32_t demo_hw_id(void)
{
    uint8_t buf[4] = {0};
    if (hwinfo_get_device_id(buf, sizeof(buf)) < (ssize_t)sizeof(buf)) {
        return 0u;
    }
    uint32_t id;
    memcpy(&id, buf, sizeof(id));
    return id ? id : 1u;
}
#endif

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
 * VALUE IS TIGHTLY BOUNDED BY WM_STALE_THRESHOLD_MS (csm.h; 1500 ms when
 * this was tuned, 3000 ms now) and GOSSIP_INTERVAL_MS (500 ms) — first
 * attempt used 3000 ms (matching
 * CHOREO_MEMBERSHIP_HOLD_MS) and real hardware never satisfied it at
 * all: that's asking all 3 peer-links to each independently avoid a
 * >1500ms gap TWICE in a row, simultaneously, with zero misses across
 * any of them — a much harder bar than it looks on paper given real BLE
 * reception. Symptom was every robot's LED flickering yellow/green
 * (1/2 fresh) forever, never all four together — the barrier was
 * correctly refusing to release, just against a bar nothing could clear.
 * 800 ms was a little over half of one 1500 ms renewal window — enough
 * to rule out a single-instant fluke without demanding near-perfect
 * multi-second reception across three links at once.
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

/*
 * ONE finder or MANY?  A script that reads finds as shared facts
 * (leak.choreo.toml — choreoc emits CHOREO_USES_LANDMARKS) wants one marker
 * and one ring: once a TARGET fact is held a robot IGNORES its own dark hits.
 * Without this, a robot that pops up while braking or backing away (a
 * lifted reflectance sensor sees no surface, which reads as IR-dark — see
 * demo_discovery_in_safe_zone()) would latch and stop dead mid-ring.  A
 * second find within FACT_MERGE_RADIUS_TARGET of the first is merged into
 * it by L4 anyway (wm_fact_publish() reuses it); this gate keeps the robot
 * from stopping for it.  A per-robot script (spill.choreo.toml) wants the
 * opposite: EVERY robot that crosses the target stops on its own edge, so
 * nothing is suppressed there.
 */
#if defined(CHOREO_USES_DISCOVERY) && defined(CHOREO_USES_LANDMARKS)
#define DEMO_DISCOVERY_SINGLE_FINDER 1
#endif
#endif /* CONFIG_DEMO_MODE_CHOREO */

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

#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
    /* Cutebot Pro's line sensor is the same I2C peripheral substrate_init()
     * (-> cutebot_pro_init()) already brought up above — no separate init
     * call, unlike Mini's genuinely-separate GPIO line-sensor bus below. */
#elif defined(CONFIG_I2C)
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

    /* Real elapsed time between iterations, not the nominal WM_CYCLE_MS —
     * see the main CHOREO loop's identical pattern below for why: the
     * loop body's own work (I2C, logging, gossip) makes the real period
     * run visibly longer than the nominal tick, and integrating heading/
     * position against the wrong (too-small) dt silently desyncs the
     * estimate from reality every single tick. */
    int64_t last_tick_ms = k_uptime_get();

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

        uint32_t dt_ms = (uint32_t)k_uptime_delta(&last_tick_ms);
        demo_odometry_update(&odo, speed_cmd, rate_cmd, dt_ms);

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
#elif defined(CONFIG_DEMO_MODE_DISCOVERY_BENCH)
    /*
     * leak/spill discovery-detection bench — see Kconfig's
     * help text. Same isolated single-robot shape as STRAIGHT_LINE
     * (no transport/auto-ID/SCR/Choreo), but no grid correction at all:
     * demo_discovery_detect() needs none of it (see that function's own
     * doc — the board has no IR-dark feature besides the marker itself).
     */
    LOG_INF("DISCOVERY BENCH MODE — start=(%.1f,%.1f) heading=%.2f rad, "
            "no swarm, no auto-ID, no grid correction",
            (double)DEMO_TEST_START_X, (double)DEMO_TEST_START_Y,
            (double)DEMO_TEST_START_HEADING);

    demo_odometry_t odo;
    demo_odometry_init(&odo, DEMO_TEST_START_X, DEMO_TEST_START_Y);
    odo.heading = DEMO_TEST_START_HEADING;

#ifdef CONFIG_I2C
    demo_discovery_state_t discovery;
    demo_discovery_init(&discovery);
#endif

    uint32_t trace_accum = 0;
    int64_t  last_tick_ms = k_uptime_get();   /* see STRAIGHT_LINE mode's doc above */

    while (true) {
        float speed_cmd = 0.0f;
        float rate_cmd  = 0.0f;
        demo_drive_straight(&odo, &speed_cmd, &rate_cmd);

        uint32_t dt_ms = (uint32_t)k_uptime_delta(&last_tick_ms);
        demo_odometry_update(&odo, speed_cmd, rate_cmd, dt_ms);

#ifdef CONFIG_I2C
        cutebot_line_sample_t line;
        cutebot_line_poll(&line);

        bool was_discovered = discovery.discovered;
        bool dark_now       = line.left || line.right;
        bool in_safe_zone   = demo_discovery_in_safe_zone(&odo);
        /* A reflectance sensor cannot tell "dark surface" from "no
         * surface at all" (open air) — see demo_discovery_in_safe_zone()'s
         * doc, confirmed on the bench 2026-09-16 (lifting the robot reads
         * identically to the real marker). Gate what actually reaches
         * the debounce/latch on being safely away from the physical edge. */
        demo_discovery_detect(&discovery, dark_now && in_safe_zone);

        /* Every dark reading, not just a periodic trace — a single
         * confirming pass over the marker must never be lost between
         * 1 Hz status lines. Logged even when suppressed by the safe-zone
         * gate, so a bench run over the true edge is visible for what it
         * is instead of just silently not-confirming. */
        if (dark_now) {
            LOG_INF("est=(%.2f,%.2f) line L=%d R=%d dark_polls=%u%s%s",
                    (double)odo.x, (double)odo.y,
                    (int)line.left, (int)line.right,
                    (unsigned)discovery.consecutive_dark_polls,
                    discovery.discovered ? " CONFIRMED" : "",
                    in_safe_zone ? "" : " (IGNORED — outside safe zone, "
                                        "likely off-board not marker)");
        }
        if (!was_discovered && discovery.discovered) {
            LOG_INF("*** DISCOVERY CONFIRMED at est=(%.2f,%.2f) ***",
                    (double)odo.x, (double)odo.y);
        }
#endif

        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);

#ifdef CONFIG_I2C
        /* Solid green the instant discovery confirms, staying there for
         * the rest of the run (sticky, matching discovery.discovered
         * itself) — a bench-visible confirmation with no serial cable
         * needed while positioning the robot over the marker. Falls
         * back to the same green/blue fence-status signal
         * DEMO_MODE_STRAIGHT_LINE uses until then. */
        substrate_set_signal(discovery.discovered
                                  ? SUBSTRATE_SIGNAL_ACTIVE
                                  : (fabsf(speed_cmd) > 0.001f
                                         ? SUBSTRATE_SIGNAL_ACTIVE
                                         : SUBSTRATE_SIGNAL_IDLE));
#else
        substrate_set_signal(fabsf(speed_cmd) > 0.001f
                                  ? SUBSTRATE_SIGNAL_ACTIVE
                                  : SUBSTRATE_SIGNAL_IDLE);
#endif

        trace_accum += WM_CYCLE_MS;
        if (trace_accum >= 1000) {
            trace_accum = 0;
#ifdef CONFIG_I2C
            LOG_INF("est=(%.2f,%.2f) hdg=%.2f spd=%.2f rate=%.2f discovered=%d",
                    (double)odo.x, (double)odo.y, (double)odo.heading,
                    (double)speed_cmd, (double)rate_cmd,
                    (int)discovery.discovered);
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
    int64_t last_tick_ms = k_uptime_get();   /* see STRAIGHT_LINE mode's doc above */

    while (true) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        float speed_cmd = 0.0f;
        float rate_cmd  = 0.0f;
        demo_track_target(&wm, &odo, tx, ty, &speed_cmd, &rate_cmd);

        uint32_t dt_ms = (uint32_t)k_uptime_delta(&last_tick_ms);
        demo_odometry_update(&odo, speed_cmd, rate_cmd, dt_ms);

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
     * Sensor bring-up — see Kconfig's help text. No motors, no odometry:
     * just log the raw sensor state, unfiltered, so materials/angles/
     * heights can be tested by eye against a live log instead of
     * inferring anything from driving behavior.
     */
#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
    LOG_INF("LINE SENSOR BENCH MODE (Cutebot Pro) — logging the raw I2C "
            "4-way state AND per-channel raw reflectance (0-255, "
            "unthresholded) every 500 ms. bit0..bit3 = channel 1..4, "
            "1 = line/dark.");

    while (true) {
        uint8_t state = 0;
        int ret = cutebot_pro_read_line_state(&state);
        if (ret) {
            LOG_WRN("line state read failed: %d", ret);
        } else {
            uint8_t gray[4] = { 0 };
            for (uint8_t ch = 1; ch <= 4; ch++) {
                int gret = cutebot_pro_read_line_gray(ch, &gray[ch - 1]);
                if (gret) {
                    LOG_WRN("channel %u gray read failed: %d", (unsigned)ch, gret);
                }
            }
            LOG_INF("ch1=%d ch2=%d ch3=%d ch4=%d  (state=0x%X)  "
                    "gray: ch1=%u ch2=%u ch3=%u ch4=%u",
                    (int)(state & 0x1), (int)((state >> 1) & 0x1),
                    (int)((state >> 2) & 0x1), (int)((state >> 3) & 0x1),
                    (unsigned)state,
                    (unsigned)gray[0], (unsigned)gray[1],
                    (unsigned)gray[2], (unsigned)gray[3]);
        }
        k_msleep(500);
    }
#else
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
#endif
#elif defined(CONFIG_DEMO_MODE_ENCODER_BENCH)
    /*
     * Odometry bench (Cutebot Pro only). Each step drives until the
     * ENCODER ESTIMATE says it has arrived, then stops — so any error in
     * sign, distance scale, or wheel track shows up as a visible
     * mismatch on the floor. Uses pro_wheel_deltas_cm(), the same
     * sign/scale step the choreo loop uses. Clear floor, not the board.
     *
     * Before starting: mark the robot's start position and put a piece
     * of tape on the chassis pointing forward. Then per step:
     *   1 fwd 100 cm   — tape-measure the real distance travelled
     *   2 back to 0    — should return to the start mark; raw counters
     *                    in the log show whether they count DOWN in
     *                    reverse (signed) or keep climbing (magnitude)
     *   3 CCW 2 turns  — should end facing the start direction
     *   4 CW 2 turns   — back to heading 0, facing the start direction
     */
#define ODO_BENCH_LINEAR_CMS  30.0f
#define ODO_BENCH_SPIN_CMS    20.0f
#define ODO_BENCH_DIST_CM     100.0f
#define ODO_BENCH_TURNS       2.0f
#define ODO_BENCH_PAUSE_MS    15000u  /* time to tape-measure each step */
#define ODO_BENCH_TIMEOUT_MS  8000u   /* each step should take ~3-4 s */

    const float track_cm = CUTEBOT_PRO_WHEEL_TRACK_MM / 10.0f;
    const float spin_rad = ODO_BENCH_TURNS * 2.0f * M_PI_F;

    struct {
        const char *label;
        float       l_cms, r_cms;
        bool        linear;    /* true: stop on distance, false: on heading */
        float       goal;      /* cm, or rad (unwrapped) */
    } steps[] = {
        { "1 fwd 100cm",      ODO_BENCH_LINEAR_CMS,  ODO_BENCH_LINEAR_CMS,  true,  ODO_BENCH_DIST_CM },
        { "2 back to start", -ODO_BENCH_LINEAR_CMS, -ODO_BENCH_LINEAR_CMS,  true,  0.0f },
        { "3 CCW 2 turns",   -ODO_BENCH_SPIN_CMS,    ODO_BENCH_SPIN_CMS,    false, spin_rad },
        { "4 CW 2 turns",     ODO_BENCH_SPIN_CMS,   -ODO_BENCH_SPIN_CMS,    false, 0.0f },
    };

    if (transport_init() != 0) {
        LOG_WRN("transport init failed — results on serial only");
    }
#ifdef CONFIG_DEMO_BLE_DEBUG
    ble_debug_init();
#endif
    DEMO_EVT(INF, "ODOMETRY BENCH hw 0x%08x — counts/cm=%.2f track=%.1fmm. "
             "Mark start position and forward direction now; starting in %u s.",
             (unsigned)demo_hw_id(), (double)CUTEBOT_PRO_DIST_COUNTS_PER_CM,
             (double)CUTEBOT_PRO_WHEEL_TRACK_MM, ODO_BENCH_PAUSE_MS / 1000u);
    k_msleep(ODO_BENCH_PAUSE_MS);

    cutebot_pro_encoders_t enc = {0};
    (void)cutebot_pro_read_encoders(&enc);
    int32_t prev_l = enc.left_distance;
    int32_t prev_r = enc.right_distance;
    float dist_cm = 0.0f;   /* along the robot's own forward axis */
    float head    = 0.0f;   /* unwrapped, rad, CCW positive */

    pro_enc_guard_t enc_guard = {0};
    for (size_t i = 0; i < ARRAY_SIZE(steps); i++) {
        float start = steps[i].linear ? dist_cm : head;
        float sign  = (steps[i].goal > start) ? 1.0f : -1.0f;

        DEMO_EVT(INF, "step %s: raw L=%d R=%d  est dist=%.1fcm head=%.1fdeg",
                steps[i].label, (int)enc.left_distance, (int)enc.right_distance,
                (double)dist_cm, (double)(head * 180.0f / M_PI_F));

        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
        cutebot_pro_drive_cms(steps[i].l_cms, steps[i].r_cms);

        uint32_t elapsed = 0;
        uint32_t log_accum = 0;
        while (elapsed < ODO_BENCH_TIMEOUT_MS) {
            k_msleep(20);
            elapsed += 20;
            cutebot_pro_read_encoders(&enc);
            float dl, dr;
            pro_wheel_deltas_cm(&enc, &prev_l, &prev_r, 20u, &enc_guard,
                                &dl, &dr);
            dist_cm += 0.5f * (dl + dr);
            head    += (dr - dl) / track_cm;

            float now = steps[i].linear ? dist_cm : head;
            if ((now - steps[i].goal) * sign >= 0.0f) {
                break;
            }
            log_accum += 20;
            if (log_accum >= 200) {
                log_accum = 0;
                LOG_INF("  raw L=%d R=%d  est dist=%.1fcm head=%.1fdeg",
                        (int)enc.left_distance, (int)enc.right_distance,
                        (double)dist_cm, (double)(head * 180.0f / M_PI_F));
            }
        }
        cutebot_pro_drive_cms(0.0f, 0.0f);
        substrate_set_signal(SUBSTRATE_SIGNAL_IDLE);

        /* Let it coast to a stop, then account for any coast motion so
         * the next step starts from the true estimate. */
        k_msleep(500);
        cutebot_pro_read_encoders(&enc);
        float dl, dr;
        pro_wheel_deltas_cm(&enc, &prev_l, &prev_r, 520u, &enc_guard,
                            &dl, &dr);
        dist_cm += 0.5f * (dl + dr);
        head    += (dr - dl) / track_cm;

        DEMO_EVT(INF, "step %s DONE%s: raw L=%d R=%d  est dist=%.1fcm "
                "head=%.1fdeg — measure now",
                steps[i].label,
                (elapsed >= ODO_BENCH_TIMEOUT_MS) ? " (TIMEOUT)" : "",
                (int)enc.left_distance, (int)enc.right_distance,
                (double)dist_cm, (double)(head * 180.0f / M_PI_F));
        k_msleep(ODO_BENCH_PAUSE_MS);
    }

    /*
     * Steps 5-6: direction flips WITHOUT stopping, read at the demo loop's
     * own 100 ms cadence. Each interval's raw counter delta is turned into
     * motion two ways: with the counter's own sign (what the odometry
     * uses) and with the sign of the command in force (what it used
     * before 2026-10-08, kept here as the reference that fails). Each step
     * starts and ends at rest, so the tape measure says which is right;
     * on 2026-10-08 step 6 turned 30 degrees CCW, counter-sign 25.1,
     * command-sign -0.8.
     *   5 fwd/rev x4    — 1.5 s legs at 30 cm/s; measure the end position
     *                     along the line from the step-5 start mark
     *   6 CCW/CW x6     — 0.7 s pivots at 20 cm/s; measure the end heading
     *                     against the step-6 start heading
     */
#define ODO_FLIP_READ_MS 100u
    struct {
        const char *label;
        float       l_cms, r_cms;   /* first leg; legs alternate sign */
        uint32_t    leg_ms;
        int         legs;
    } flips[] = {
        { "5 fwd/rev flips", ODO_BENCH_LINEAR_CMS, ODO_BENCH_LINEAR_CMS, 1500u, 4 },
        { "6 CCW/CW flips", -ODO_BENCH_SPIN_CMS,   ODO_BENCH_SPIN_CMS,    700u, 6 },
    };
    for (size_t i = 0; i < ARRAY_SIZE(flips); i++) {
        float d_cmd = 0.0f, h_cmd = 0.0f;   /* command-sign estimate */
        float d_sgn = 0.0f, h_sgn = 0.0f;   /* counter-sign estimate */
        float l = 0.0f, r = 0.0f;           /* command in force */

        DEMO_EVT(INF, "step %s: mark position and heading now", flips[i].label);
        (void)cutebot_pro_read_encoders(&enc);
        prev_l = enc.left_distance;
        prev_r = enc.right_distance;

        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
        int total = flips[i].legs * (int)(flips[i].leg_ms / ODO_FLIP_READ_MS);
        for (int t = 0; t <= total + 5; t++) {   /* +5: 0.5 s coast */
            if (t < total && t % (int)(flips[i].leg_ms / ODO_FLIP_READ_MS) == 0) {
                float sgn = ((t / (int)(flips[i].leg_ms / ODO_FLIP_READ_MS)) % 2)
                            ? -1.0f : 1.0f;
                l = sgn * flips[i].l_cms;
                r = sgn * flips[i].r_cms;
                cutebot_pro_drive_cms(l, r);
            } else if (t == total) {
                cutebot_pro_drive_cms(0.0f, 0.0f);   /* keep l/r: the coast
                                                       * is signed by the
                                                       * last command, as
                                                       * the demo does */
            }
            k_msleep(ODO_FLIP_READ_MS);
            if (cutebot_pro_read_encoders(&enc) != 0) {
                continue;
            }
            float dl, dr;
            pro_enc_result_t eg = pro_wheel_deltas_cm(&enc, &prev_l, &prev_r,
                                                      ODO_FLIP_READ_MS,
                                                      &enc_guard, &dl, &dr);
            if (eg != PRO_ENC_OK) {
                continue;
            }
            d_sgn += 0.5f * (dl + dr);
            h_sgn += (dr - dl) / track_cm;
            float cl = (l < 0.0f) ? -fabsf(dl) : fabsf(dl);
            float cr = (r < 0.0f) ? -fabsf(dr) : fabsf(dr);
            d_cmd += 0.5f * (cl + cr);
            h_cmd += (cr - cl) / track_cm;
        }
        substrate_set_signal(SUBSTRATE_SIGNAL_IDLE);

        DEMO_EVT(INF, "step %s DONE: cmd-sign dist=%.1fcm head=%.1fdeg | "
                 "counter-sign dist=%.1fcm head=%.1fdeg — measure now",
                 flips[i].label, (double)d_cmd,
                 (double)(h_cmd * 180.0f / M_PI_F), (double)d_sgn,
                 (double)(h_sgn * 180.0f / M_PI_F));
        k_msleep(ODO_BENCH_PAUSE_MS);
    }

    DEMO_EVT(INF, "ODOMETRY BENCH COMPLETE — motors off.");
    substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);   /* red: done, not an error */
    while (true) {
        k_msleep(1000);
    }
#elif defined(CONFIG_DEMO_MODE_CIRCLE_BENCH)
    /*
     * Actuation bench (Cutebot Pro only) — see Kconfig's help text. Direct
     * cutebot_pro_drive_cms() calls, bypassing formation.c entirely, so a
     * passing run confirms the differential-preserving floor fix at the
     * driver level: straight for 2s, then the sharpest arc turn this
     * firmware can represent (20/50 cm/s, its 2.5:1 ceiling) for 8s,
     * then straight for 2s again. NOT run through the arena fence — long
     * clear floor, not the chessboard.
     */
#define CIRCLE_BENCH_STRAIGHT_CMS 30.0f
#define CIRCLE_BENCH_TURN_SLOW_CMS 20.0f
#define CIRCLE_BENCH_TURN_FAST_CMS 50.0f
#define CIRCLE_BENCH_STRAIGHT_MS   2000u
#define CIRCLE_BENCH_TURN_MS       8000u

    LOG_INF("CIRCLE BENCH MODE — straight %.0f cm/s for %us, then an arc "
            "turn (L=%.0f R=%.0f cm/s) for %us, then straight again. "
            "Clear floor, not the board.",
            (double)CIRCLE_BENCH_STRAIGHT_CMS, CIRCLE_BENCH_STRAIGHT_MS / 1000u,
            (double)CIRCLE_BENCH_TURN_SLOW_CMS, (double)CIRCLE_BENCH_TURN_FAST_CMS,
            CIRCLE_BENCH_TURN_MS / 1000u);

    LOG_INF("phase 1/3: straight");
    substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
    cutebot_pro_drive_cms(CIRCLE_BENCH_STRAIGHT_CMS, CIRCLE_BENCH_STRAIGHT_CMS);
    k_msleep(CIRCLE_BENCH_STRAIGHT_MS);

    LOG_INF("phase 2/3: arc turn — should visibly curve, NOT travel straight");
    substrate_set_signal(SUBSTRATE_SIGNAL_DEGRADED);
    cutebot_pro_drive_cms(CIRCLE_BENCH_TURN_SLOW_CMS, CIRCLE_BENCH_TURN_FAST_CMS);
    k_msleep(CIRCLE_BENCH_TURN_MS);

    LOG_INF("phase 3/3: straight again");
    substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
    cutebot_pro_drive_cms(CIRCLE_BENCH_STRAIGHT_CMS, CIRCLE_BENCH_STRAIGHT_CMS);
    k_msleep(CIRCLE_BENCH_STRAIGHT_MS);

    LOG_INF("CIRCLE BENCH COMPLETE — motors off, halting.");
    cutebot_pro_drive_cms(0.0f, 0.0f);
    substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);   /* red: done, not an error */
    while (true) {
        k_msleep(1000);
    }
#elif defined(CONFIG_DEMO_MODE_TURN_CALIBRATE)
    /*
     * Turn-scale calibration (Cutebot Pro only) — see Kconfig's help text.
     * Button A: spin CCW, button B: spin CW, CONFIG_DEMO_TURN_CAL_TURNS
     * full turns by the encoder each, at the wheel speed the demo turns
     * at. Reports the encoder total including the coast after stopping;
     * the operator measures the true end heading.
     */
#define TURN_CAL_SPIN_CMS  20.0f    /* the firmware's floor: demo turns */
#define TURN_CAL_HANDS_MS  1500u    /* after the press, before moving */
#define TURN_CAL_COAST_MS  1000u

    static const struct gpio_dt_spec btn_a = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
    static const struct gpio_dt_spec btn_b = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
    gpio_pin_configure_dt(&btn_a, GPIO_INPUT);
    gpio_pin_configure_dt(&btn_b, GPIO_INPUT);

    if (transport_init() != 0) {
        LOG_WRN("transport init failed — results on serial only");
    }
#ifdef CONFIG_DEMO_BLE_DEBUG
    ble_debug_init();
#endif
    uint32_t hw_id = demo_hw_id();
    const float track_cm = CUTEBOT_PRO_WHEEL_TRACK_MM / 10.0f;
    const int   turns    = CONFIG_DEMO_TURN_CAL_TURNS;
    const demo_turn_cal_t *cal = demo_turn_cal_lookup(hw_id);
    int runs[2] = {0};
    /* This robot's trims from turn_calibration.h (1.0 without an entry):
     * the spin is counted in trimmed degrees, so with correct trims it
     * ends where the report says. It stops early by the coast measured
     * on the previous run that way (17 degrees to start). */
    const float trim[2] = { cal ? cal->ccw : 1.0f, cal ? cal->cw : 1.0f };
    float coast_deg[2] = { 17.0f, 17.0f };

    substrate_set_signal(SUBSTRATE_SIGNAL_IDLE);
    uint32_t prompt_ms = 0;

    while (true) {
        /* The prompt repeats, so a logger started late still sees it. */
        if (k_uptime_get_32() - prompt_ms >= 15000u) {
            prompt_ms = k_uptime_get_32();
            DEMO_EVT(INF, "turn cal hw 0x%08x: A = CCW, B = CW, %d turns "
                     "(table %.4f/%.4f)", (unsigned)hw_id, turns,
                     (double)(cal ? cal->ccw : 1.0f),
                     (double)(cal ? cal->cw : 1.0f));
        }
        int dir = gpio_pin_get_dt(&btn_a) > 0 ? 0 :
                  gpio_pin_get_dt(&btn_b) > 0 ? 1 : -1;   /* 0 CCW, 1 CW */
        if (dir < 0) {
            k_msleep(20);
            continue;
        }

        substrate_set_signal(SUBSTRATE_SIGNAL_DEGRADED);
        k_msleep(TURN_CAL_HANDS_MS);

        float l = dir == 0 ? -TURN_CAL_SPIN_CMS : TURN_CAL_SPIN_CMS;
        float r = -l;
        cutebot_pro_encoders_t enc = {0};
        (void)cutebot_pro_read_encoders(&enc);
        int32_t prev_l = enc.left_distance, prev_r = enc.right_distance;
        pro_enc_guard_t enc_guard = {0};
        uint32_t last_read_ms = k_uptime_get_32();
        float   head = 0.0f;                 /* raw encoder, rad */
        const float goal = (2.0f * M_PI_F * (float)turns -
                            coast_deg[dir] * M_PI_F / 180.0f) / trim[dir];

        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
        cutebot_pro_drive_cms(l, r);
        uint32_t t0 = k_uptime_get_32();
        while (head < goal && k_uptime_get_32() - t0 < 60000u) {
            k_msleep(10);
            if (cutebot_pro_read_encoders(&enc) == 0) {
                float dl, dr;
                uint32_t now_ms = k_uptime_get_32();
                pro_wheel_deltas_cm(&enc, &prev_l, &prev_r,
                                    now_ms - last_read_ms, &enc_guard, &dl, &dr);
                last_read_ms = now_ms;
                head += fabsf(dr - dl) / track_cm;
            }
        }
        cutebot_pro_drive_cms(0.0f, 0.0f);
        uint32_t spin_ms = k_uptime_get_32() - t0;
        float at_stop = head;
        k_msleep(TURN_CAL_COAST_MS);
        if (cutebot_pro_read_encoders(&enc) == 0) {
            float dl, dr;
            pro_wheel_deltas_cm(&enc, &prev_l, &prev_r,
                                k_uptime_get_32() - last_read_ms, &enc_guard,
                                &dl, &dr);
            head += fabsf(dr - dl) / track_cm;   /* coast */
        }
        if (enc_guard.total > 0u) {
            DEMO_EVT(WRN, "turn cal: %u implausible encoder reads ignored "
                     "this run", (unsigned)enc_guard.total);
        }
        substrate_set_signal(SUBSTRATE_SIGNAL_IDLE);
        coast_deg[dir] = (head - at_stop) * trim[dir] * 180.0f / M_PI_F;

        float enc_deg  = head * 180.0f / M_PI_F;
        float trim_deg = enc_deg * trim[dir];
        runs[dir]++;
        DEMO_EVT(INF, "turn cal %s run %d: trim %.4f, expect %+.1f deg "
                 "(raw %.1f, %.1f s)", dir == 0 ? "CCW" : "CW", runs[dir],
                 (double)trim[dir], (double)(trim_deg - 360.0f * turns),
                 (double)enc_deg, (double)(spin_ms * 0.001f));
        DEMO_EVT(INF, "  scale = (%d + offset_deg) / %.1f", 360 * turns,
                 (double)enc_deg);

        /* Wait for release so one press is one run. */
        while (gpio_pin_get_dt(&btn_a) > 0 || gpio_pin_get_dt(&btn_b) > 0) {
            k_msleep(20);
        }
        prompt_ms = k_uptime_get_32();
    }
#else /* !CONFIG_DEMO_MODE_STRAIGHT_LINE / !CONFIG_DEMO_MODE_DISCOVERY_BENCH /
       * !CONFIG_DEMO_MODE_CONVERGE_TEST / !CONFIG_DEMO_MODE_WHEEL_CHARACTERIZE /
       * !CONFIG_DEMO_MODE_LINE_SENSOR_BENCH / !CONFIG_DEMO_MODE_ENCODER_BENCH /
       * !CONFIG_DEMO_MODE_CIRCLE_BENCH / !CONFIG_DEMO_MODE_TURN_CALIBRATE —
       * the normal swarm demo */

    if (transport_init() != 0) {
        LOG_WRN("transport init failed — no peer awareness");
    }
#ifdef CONFIG_DEMO_BLE_DEBUG
    ble_debug_init();
#endif

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
        DEMO_EVT(ERR, "id=%u auto-ID heard NO peers — grounded self-healing "
                 "recovery (renegotiates until a peer is found)",
                 (unsigned)element_id);
        substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);

        element_id = transport_negotiate_id_retry(element_id, &n_total);

        DEMO_EVT(WRN, "id=%u recovered — n_total=%d, proceeding",
                 (unsigned)element_id, n_total);
        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
    }
#ifdef CONFIG_DEMO_BLE_DEBUG
    ble_debug_set_id((uint8_t)element_id);
#endif

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

#ifdef CONFIG_I2C
    /* Discovery detection (leak/spill) — see formation.h.
     * Declared unconditionally on CONFIG_I2C (not gated on
     * CONFIG_DEMO_GRID_CORRECTION, which only concerns drift correction):
     * discovery has no dependency on grid correction at all. */
    demo_discovery_state_t discovery;
    demo_discovery_init(&discovery);
#if defined(CHOREO_USES_DISCOVERY) && !defined(DEMO_DISCOVERY_SINGLE_FINDER)
    /* Every robot's own crossing of a thin outline target counts (spill):
     * one entry edge is enough, no multi-poll confirmation. */
    discovery.confirm_polls = 1u;
#endif
#ifdef CHOREO_USES_LANDMARKS
    /* The script step this robot discovered on, and ticks since the script
     * left it — see the stop-on-discovery block in the loop. */
    int      discovery_step           = -1;
    uint32_t discovery_released_ticks = 0u;
    /* A find not yet in the fact table: wm_fact_publish() refuses until
     * the table has synced with a peer (FACT_SYNC_TIMEOUT_MS), so it is
     * retried every tick until it lands. */
    bool       target_pending = false;
    position_t target_at      = {0};
#endif
#endif

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
    /* FORM slots stay on the checkered play area: a ring around a marker
     * found near the border slides inward instead of parking a robot on
     * (or past) the fence. */
    bse_set_form_bounds(DEMO_SQUARE_UNITS, DEMO_SQUARE_UNITS,
                        WORLD_SIZE - DEMO_SQUARE_UNITS,
                        WORLD_SIZE - DEMO_SQUARE_UNITS);
    /*
     * choreoc.py emits one of two incompatible shapes depending on
     * whether the source .toml has [[steps]] (k_choreo_script/
     * CHOREO_SCRIPT_LEN — ring.choreo.toml, form-grid.choreo.toml,
     * spill.choreo.toml) or [[tracks]] (k_choreo_tracks/CHOREO_N_TRACKS
     * — no cutebot script today, §7) — see choreoc.py's _emit_steps_header()
     * vs _emit_tracks_header(). CHOREO_N_TRACKS only exists in the
     * tracks case, so it's the compile-time discriminator between the
     * two choreo_script.h shapes; whichever script is currently
     * compiled in, only one of these two branches ever actually builds.
     * choreo_submit_tracks() needs a `wm` snapshot to determine this
     * element's initial track — at this point in boot it holds only
     * self's own entry (no gossip received yet), which is exactly
     * right: every element starts this determination from its own
     * not-yet-discovered state, matching every track filter that reads
     * a self-only condition (see choreo_track_filter_t's doc).
     */
#ifdef CHOREO_N_TRACKS
    if (choreo_submit_tracks(&wm, k_choreo_tracks, CHOREO_N_TRACKS) != 0) {
        LOG_ERR("id=%u choreo tracks rejected — staying grounded",
                (unsigned)element_id);
        substrate_set_power(SUBSTRATE_POWER_SLEEP);
        return -1;
    }
    LOG_INF("id=%u choreo \"%s\" loaded — %u track(s), time bound %u s",
            (unsigned)element_id, CHOREO_NAME,
            (unsigned)CHOREO_N_TRACKS,
            (unsigned)(CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u));
#else
    if (choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN) != 0) {
        LOG_ERR("id=%u choreo script rejected — staying grounded",
                (unsigned)element_id);
        substrate_set_power(SUBSTRATE_POWER_SLEEP);
        return -1;
    }
    DEMO_EVT(INF, "id=%u choreo \"%s\" loaded — %u steps, time bound %u s",
            (unsigned)element_id, CHOREO_NAME,
            (unsigned)CHOREO_SCRIPT_LEN,
            (unsigned)(CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u));
#endif
    /* own_state.current_track is otherwise still 0 (zero-init) until the
     * main loop's first choreo_publish_state(): the sync barrier below
     * gossips own_state, so on a [[tracks]] script every peer would read
     * this robot as being on track 0 — possibly another track than its
     * own — and bse.c's track-scoped rank/count would drop it from their
     * FORM until the first loop gossip arrives. */
    choreo_publish_state(&own_state);

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
            DEMO_EVT(INF, "id=%u still waiting for peers: %d/%d fresh (%u ms, "
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

    DEMO_EVT(INF, "id=%u demo ready — entering main loop",
             (unsigned)element_id);

    int64_t last_tick_ms = k_uptime_get();   /* see STRAIGHT_LINE mode's doc above */

#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
    /* Baseline for the distance-delta odometry below — see that block's
     * doc. One throwaway read before the loop starts, same pattern as
     * last_tick_ms above, so the very first in-loop delta reflects real
     * motion since NOW, not since encoder power-on (whatever the counter
     * already held before this run started). */
    cutebot_pro_encoders_t enc_seed = {0};
    /* No good baseline yet if this read fails: the first good read in the
     * loop then only sets it (see enc_prev_valid there). */
    bool    enc_prev_valid  = cutebot_pro_read_encoders(&enc_seed) == 0;
    int32_t prev_left_dist  = enc_seed.left_distance;
    int32_t prev_right_dist = enc_seed.right_distance;

    /* Fast background line sampling — replaces the main loop's own
     * once-per-tick gray reads; see cutebot_pro_line_watch_start(). */
    uint32_t hw_id = demo_hw_id();
    const demo_turn_cal_t *cal = demo_turn_cal_lookup(hw_id);
    bool  turn_cal     = cal != NULL;
    float turn_cal_ccw = cal ? cal->ccw : 1.0f;
    float turn_cal_cw  = cal ? cal->cw  : 1.0f;
    if (turn_cal) {
        DEMO_EVT(INF, "id=%u turn cal hw 0x%08x: ccw %.4f cw %.4f",
                 (unsigned)element_id, (unsigned)hw_id,
                 (double)turn_cal_ccw, (double)turn_cal_cw);
    } else {
        DEMO_EVT(WRN, "id=%u no turn cal for hw 0x%08x — run "
                 "DEMO_MODE_TURN_CALIBRATE", (unsigned)element_id,
                 (unsigned)hw_id);
    }
    if (cutebot_pro_line_watch_start(CUTEBOT_PRO_TARGET_DARK_THRESHOLD,
                                     CUTEBOT_PRO_LINE_WATCH_PERIOD_MS) != 0) {
        LOG_ERR("line watch failed to start — no line sensing this run");
    }
#endif

    while (true) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        /* Real elapsed time since the last iteration, NOT the nominal
         * WM_CYCLE_MS this loop's own I2C/logging/gossip work routinely
         * overruns — confirmed directly from a real hardware log this
         * session: 1 Hz trace lines meant to fire every 1000 ms (10
         * ticks of the assumed 100 ms) actually landed ~1587 ms apart, a
         * consistent ~59% real-vs-assumed gap. Every demo_odometry_update()
         * call below was integrating against a dt ~37% smaller than what
         * actually elapsed, so the position/heading ESTIMATE tracked only
         * ~63% of the robot's real motion every tick — a controller
         * chasing that lagging estimate keeps commanding more turn/drive
         * long after the real robot has already gotten there, which is
         * exactly the "overturns and ends up displaced from where it
         * thinks it is" symptom id=2 hit on a real re-form turn. */
        uint32_t dt_ms = (uint32_t)k_uptime_delta(&last_tick_ms);

#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
        /* Closed-loop odometry via distance DELTAS, not sampled speed —
         * leaning harder into the encoder after real hardware showed
         * visible drift with grid correction as the backstop (2026-09-23
         * session: grid correction disabled entirely made no difference,
         * pointing at the dead-reckoning integration itself). A single
         * instantaneous speed sample per tick (the old approach) is
         * rectangle-rule integration: it represents the WHOLE interval by
         * one point, missing real motion between samples — worst exactly
         * during the accel/decel transients a turn or continuous spin
         * produces (both a running theme this session). The wheel's own
         * encoder integrates continuously in real hardware; reading its
         * cumulative distance and differencing against the LAST tick's
         * reading recovers the TRUE distance traveled over this exact
         * interval, with no sampling assumption at all. See
         * CUTEBOT_PRO_DIST_COUNTS_PER_CM's doc (formation.h) for the
         * scale factor (tape-checked 2026-10-08: 102.8 cm counted for
         * 102 cm driven). Direction is the counter's own sign — see
         * pro_wheel_deltas_cm(). */
        cutebot_pro_encoders_t enc = {0};
        int enc_ret = cutebot_pro_read_encoders(&enc);

        /* A failed read contributes no motion this tick (prev_* kept, so
         * the next good read measures the whole interval) — never a
         * delta against a garbage counter. */
        float left_delta_cm = 0.0f, right_delta_cm = 0.0f;
        if (enc_ret == 0 && !enc_prev_valid) {
            prev_left_dist  = enc.left_distance;    /* baseline only */
            prev_right_dist = enc.right_distance;
            enc_prev_valid  = true;
        } else if (enc_ret == 0) {
            /* The delta spans everything since the last accepted read —
             * several ticks after failed reads — so judge it against
             * that interval, not this tick's dt_ms. */
            static pro_enc_guard_t enc_guard;
            static uint32_t enc_good_ms;
            uint32_t now_ms_enc = k_uptime_get_32();
            uint32_t span_ms = enc_good_ms ? now_ms_enc - enc_good_ms : dt_ms;
            pro_enc_result_t eg = pro_wheel_deltas_cm(
                &enc, &prev_left_dist, &prev_right_dist, span_ms, &enc_guard,
                &left_delta_cm, &right_delta_cm);
            if (eg != PRO_ENC_REJECTED) {
                enc_good_ms = now_ms_enc;
            }
            /* First one, every 20th, and every re-baseline: a chassis
             * that keeps producing them is failing (browning out, loose
             * cable) and should be visible in the observer at once. */
            if (eg != PRO_ENC_OK &&
                (enc_guard.total == 1u || enc_guard.total % 20u == 0u ||
                 eg == PRO_ENC_REBASED)) {
                DEMO_EVT(WRN, "id=%u encoder read implausible (dL %d dR %d "
                         "counts in %u ms) — %s, %u so far",
                         (unsigned)element_id, (int)enc_guard.last_raw_l,
                         (int)enc_guard.last_raw_r, (unsigned)span_ms,
                         eg == PRO_ENC_REBASED ? "re-baselined" : "ignored",
                         (unsigned)enc_guard.total);
            }
        } else {
            static uint32_t enc_fail_count;
            if ((enc_fail_count++ % 50u) == 0u) {
                DEMO_EVT(WRN, "id=%u encoder read failed (%d), %u so far — "
                         "odometry held", (unsigned)element_id, enc_ret,
                         (unsigned)enc_fail_count);
            }
        }

        /* Recover an "effective average cm/s over this exact interval"
         * from the real measured distance, rather than a sampled
         * instantaneous value — demo_odometry_update() below multiplies
         * this back out by the SAME dt_ms, so the true measured delta is
         * reproduced EXACTLY (v*dt = (delta/dt)*dt = delta), not
         * approximated. dt_s>0 guard: the very first tick after the
         * seed read above could in principle see dt_ms==0. */
        float dt_s = (float)dt_ms * 0.001f;
        float left_meas_cms  = (dt_s > 0.0f) ? (left_delta_cm  / dt_s) : 0.0f;
        float right_meas_cms = (dt_s > 0.0f) ? (right_delta_cm / dt_s) : 0.0f;

        /* cm/s -> logical units/s (DEMO_MM converts mm; *10 is cm->mm),
         * then normalize by the SAME DEMO_MAX_SPEED/DEMO_MAX_OMEGA
         * demo_odometry_update() itself scales by — see formation.h.
         * CUTEBOT_PRO_WHEEL_TRACK (formation.h) is Pro's own real,
         * bench-measured chassis dimension — NOT Mini's DEMO_WHEEL_TRACK,
         * a visibly different (larger) chassis. */
        float v_center_units_s = DEMO_MM((left_meas_cms + right_meas_cms) * 5.0f);
        float omega_rad_s = DEMO_MM((right_meas_cms - left_meas_cms) * 10.0f) / CUTEBOT_PRO_WHEEL_TRACK;

        /* This robot's bench-measured turn scale (turn_calibration.h). */
        omega_rad_s *= (omega_rad_s >= 0.0f) ? turn_cal_ccw : turn_cal_cw;

        demo_odometry_update(&odo, v_center_units_s / DEMO_MAX_SPEED,
                             omega_rad_s / DEMO_MAX_OMEGA, dt_ms);

        /* Junction-based drift correction — a crossroads/T-junction pins
         * both axes at once; see formation.h's demo_junction_correct().
         * line_state/junction stay in scope (not gated by CONFIG_DEMO_
         * GRID_CORRECTION) for the 1 Hz trace below, same pattern the
         * Mini branch uses for `line`/edge counts regardless of whether
         * correction itself is enabled.
         *
         * Built from the line watcher's latest raw gray sample and an
         * app-level threshold (CUTEBOT_PRO_LINE_DARK_THRESHOLD,
         * formation.h) — NOT cutebot_pro_read_line_state()'s firmware-
         * thresholded bit, which real hardware showed never fires on this
         * board's printed ink (see that constant's doc for the bench
         * numbers). */
        cutebot_pro_line_sample_t lw;
        cutebot_pro_line_watch_poll(&lw);

        uint8_t line_state = 0;
        int     line_ret   = lw.valid ? 0 : -EIO;
        for (int i = 0; i < 4; i++) {
            if (lw.gray[i] >= CUTEBOT_PRO_LINE_DARK_THRESHOLD) {
                line_state |= (uint8_t)(1u << i);
            }
        }
        demo_junction_type_t junction = DEMO_JUNCTION_NONE;
#ifdef CONFIG_DEMO_GRID_CORRECTION
        if (line_ret == 0) {
            junction = demo_junction_correct(&odo, line_state);
        }
#endif
        /* Target (leak/spill) signals at CUTEBOT_PRO_TARGET_DARK_THRESHOLD
         * — the board's own black squares stay below it. seen_mask covers
         * every background sample since the last tick, so a 2 cm outline
         * crossed between ticks still counts (Mini's entry flags' job). */
        /* Only fresh, error-free samples may count as dark: a chassis that
         * stops answering must not keep its last reading (see the line
         * watcher) and declare a discovery from it. */
        bool line_fresh      = lw.valid && lw.samples > 0u && lw.errors == 0u;
        bool line_dark_level = line_fresh && lw.dark_mask != 0;
        bool line_dark_seen  = line_fresh && lw.seen_mask != 0;

        /* For the 1 Hz trace — see the Mini branch's edge counters. */
        static uint32_t trace_lw_samples;
        static uint32_t trace_lw_errors;
        static uint8_t  trace_lw_seen;
        trace_lw_samples += lw.samples;
        trace_lw_errors  += lw.errors;
        trace_lw_seen    |= lw.seen_mask;
#elif defined(CONFIG_I2C)
        demo_odometry_update(&odo, speed_cmd, rate_cmd, dt_ms);

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
        bool line_dark_level = line.left || line.right;
        bool line_dark_seen  = line.left_entered || line.right_entered;

        /* Accumulated for the 1 Hz trace below — a single 100 ms poll
         * usually shows 0 edges even seconds after a real crossing
         * (crossings are brief), so the per-poll count alone is a poor
         * "is this sensor even working" signal over serial. These sum
         * across the whole 1 s window and are cleared when it prints. */
        static uint32_t trace_left_edges;
        static uint32_t trace_right_edges;
        trace_left_edges  += line.left_edges;
        trace_right_edges += line.right_edges;
#else
        demo_odometry_update(&odo, speed_cmd, rate_cmd, dt_ms);
#endif

#ifdef CONFIG_I2C
        /* Discovery (leak/spill) — shared by both variants; each branch
         * above supplies line_dark_level (dark right now) and
         * line_dark_seen (went dark at any point since the last tick).
         *
         * Independent of CONFIG_DEMO_GRID_CORRECTION — see the
         * declaration site's comment. Gated on demo_discovery_in_safe_
         * zone(): a reflectance sensor cannot tell "dark surface" from
         * "no surface at all" — a robot that drifts off the physical
         * board edge during the search patrol must not self-declare
         * discovery from that ambiguity alone (confirmed on the bench
         * 2026-09-16 — see that function's doc). */
        bool was_discovered = discovery.discovered;

        /* Detection is armed only in scripts that actually react to a
         * find (choreoc emits CHOREO_USES_DISCOVERY) — the ring never
         * looks for one, so a false latch there must not freeze a robot
         * through the stop-on-discovery reflex below. */
        bool discovery_armed = false;
#ifdef CHOREO_USES_DISCOVERY
        discovery_armed = true;
#endif
#ifdef DEMO_DISCOVERY_SINGLE_FINDER
        fact_t known_target;
        bool   target_known = wm_fact_find(&wm, FACT_TYPE_TARGET, 0,
                                           &known_target);
        if (target_known) {
            discovery_armed = false;
        }
        {
            static bool ignored_logged;
            bool ignoring = target_known && !discovery.discovered &&
                            line_dark_level;
            if (ignoring && !ignored_logged) {
                DEMO_EVT(INF, "id=%u dark hit IGNORED — target %u/%04x/%u "
                         "at (%.1f,%.1f) is already known",
                         (unsigned)element_id,
                         (unsigned)known_target.key.origin,
                         (unsigned)known_target.key.epoch,
                         (unsigned)known_target.key.seq,
                         (double)known_target.pos.x,
                         (double)known_target.pos.y);
            }
            ignored_logged = ignoring;
        }
#endif

        bool dark_now = line_dark_level;
#ifndef DEMO_DISCOVERY_SINGLE_FINDER
        /* Outline target (spill): its boundary can be crossed in well under
         * one tick, so also accept "went dark since the last tick" (Mini:
         * interrupt entry flags; Pro: background line watcher) — see
         * demo_discovery_state_t::confirm_polls, set to 1 at init. */
        dark_now = dark_now || line_dark_seen;
#else
        /* leak: its 120 mm disc is crossed over several ticks, so it uses
         * the steady level with confirm_polls consecutive hits instead. */
        ARG_UNUSED(line_dark_seen);
#endif
        /* Forward motion only, and not within 0.5 s of a backup — a robot
         * that is carried, lifted or rocking back on its wheels sees no
         * surface, which reads dark.  speed_cmd is last tick's command. */
        bool motion_ok = demo_discovery_motion_ok(&discovery, speed_cmd,
                                                   WM_CYCLE_MS);
        demo_discovery_detect(&discovery,
                               discovery_armed && motion_ok && dark_now &&
                               demo_discovery_in_safe_zone(&odo));
        if (!was_discovered && discovery.discovered) {
            own_state.discovered = true;
#ifdef CHOREO_USES_LANDMARKS
            discovery_step = choreo_script_step();
            target_pending = true;
            target_at = (position_t){ odo.x, odo.y, 0.0f };
#endif
            DEMO_EVT(INF, "id=%u DISCOVERED the target at est=(%.1f,%.1f)",
                    (unsigned)element_id, (double)odo.x, (double)odo.y);
        }
#ifdef CHOREO_USES_LANDMARKS
        /* Publish the find as a TARGET fact — where THIS robot was when
         * its sensors confirmed it.  Immutable (the first report stands)
         * and top priority (never evicted for a lesser fact). */
        if (target_pending) {
            fact_key_t key;
            int rc = wm_fact_publish(&wm, FACT_TYPE_TARGET, &target_at, 0u,
                                     FACT_IMMUTABLE | FACT_PRIO(3), &key);
            if (rc != WM_FACT_ERR_UNSYNCED) {
                target_pending = false;
            }
            if (rc == WM_FACT_CREATED || rc == WM_FACT_REUSED) {
                DEMO_EVT(INF, "id=%u %s target %u/%04x/%u at (%.1f,%.1f)",
                         (unsigned)element_id,
                         rc == WM_FACT_CREATED ? "PUBLISHED" : "matched known",
                         (unsigned)key.origin, (unsigned)key.epoch,
                         (unsigned)key.seq,
                         (double)target_at.x, (double)target_at.y);
            } else if (rc != WM_FACT_ERR_UNSYNCED) {
                DEMO_EVT(ERR, "id=%u target fact NOT published (%d)",
                         (unsigned)element_id, rc);
            }
        }
#endif
#endif /* CONFIG_I2C — discovery */

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
            DEMO_EVT(WRN, "id=%u quorum LOST — sending HARD_RT gossip now",
                     (unsigned)element_id);
            transport_send(&own_state, TAPESTRY_QOS_HARD_RT);
            gossip_accum = 0;
        }
        if (abort_state == SCR_ABORT_CLEARED &&
            last_abort_state != SCR_ABORT_CLEARED) {
            DEMO_EVT(INF, "id=%u quorum recovered", (unsigned)element_id);
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
            DEMO_EVT(INF, "id=%u choreo track %u step %d %s", (unsigned)element_id,
                    (unsigned)choreo_current_track(), last_step,
                    choreo_goal_status() == CHOREO_STATE_SUSPENDED
                        ? "(suspended)" : "");
        }

        /* The target landmark every ring vertex is computed from (rank 0 of
         * the TARGET facts — the same on every robot holding the table).
         * One-shot log when it first appears or changes, so a rejoined
         * robot's log shows where it learned it and from how many frames. */
        fact_t target = {0};
        bool   target_valid = wm_fact_find(&wm, FACT_TYPE_TARGET, 0, &target);
        {
            static bool       logged_valid;
            static fact_key_t logged_key;
            if (target_valid &&
                (!logged_valid || !fact_key_equal(&logged_key, &target.key))) {
                logged_valid = true;
                logged_key   = target.key;
                DEMO_EVT(INF, "id=%u target landmark %u/%04x/%u at (%.1f,%.1f) "
                         "stamp %u, own est=(%.1f,%.1f), fact frames heard %u",
                         (unsigned)element_id, (unsigned)target.key.origin,
                         (unsigned)target.key.epoch, (unsigned)target.key.seq,
                         (double)target.pos.x, (double)target.pos.y,
                         (unsigned)target.stamp, (double)odo.x, (double)odo.y,
                         (unsigned)wm.facts.frames_heard);
            } else if (!target_valid && logged_valid) {
                logged_valid = false;
                DEMO_EVT(WRN, "id=%u target landmark GONE (%d tombstone(s))",
                         (unsigned)element_id,
                         wm_fact_tombstones(&wm, FACT_TYPE_TARGET));
            }
        }
        {
            static uint32_t logged_evictions;
            if (wm.facts.evictions != logged_evictions) {
                logged_evictions = wm.facts.evictions;
                const fact_key_t *k = &wm.facts.last_evicted;
                DEMO_EVT(WRN, "id=%u fact EVICTED (%s): type %u %u/%04x/%u "
                         "prio %u age %u ms, %u eviction(s) so far",
                         (unsigned)element_id,
                         wm.facts.last_evict_reason == WM_FACT_EVICT_PER_ORIGIN
                             ? "per-origin cap" : "capacity",
                         (unsigned)k->type, (unsigned)k->origin,
                         (unsigned)k->epoch, (unsigned)k->seq,
                         (unsigned)FACT_PRIO_OF(wm.facts.last_evicted_flags),
                         (unsigned)wm.facts.last_evicted_age_ms,
                         (unsigned)wm.facts.evictions);
            }
        }

        /* Fresh-peer count changes: what element_lost/joined and quorum
         * are computed from, and the first thing to look at when a robot
         * re-forms for no visible reason. */
        int fresh_peers = 0;
        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm.entries[i];
            if (e->is_active && !e->is_self && !e->is_stale) {
                fresh_peers++;
            }
        }
        {
            static int last_fresh = -1;
            if (fresh_peers != last_fresh) {
                DEMO_EVT(INF, "id=%u fresh peers %d -> %d",
                         (unsigned)element_id, last_fresh, fresh_peers);
                last_fresh = fresh_peers;
            }
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

#ifdef CONFIG_I2C
        /* A robot that has found the target STOPS, right where it stands
         * — before (not after) its Choreo step catches up: the event has
         * no debounce, but the new step's directive still lags one tick.
         * Once stopped here, the HOLD that finally captures its station
         * captures THIS position.
         *
         * leak (CHOREO_USES_LANDMARKS) instead wants the finder back in the
         * ring, so the stop lasts only until the script has left the step it
         * discovered on — plus one tick, since the tick the step changes
         * still carries the old step's directive (the new goal's first
         * bse_tick() is the next choreo_tick()). The ring's centre is the
         * published fact, not this robot, so it may then move freely. */
        bool discovery_stop = discovery.discovered;
#ifdef CHOREO_USES_LANDMARKS
        if (discovery_stop && choreo_script_step() != discovery_step) {
            discovery_stop = discovery_released_ticks < 1u;
            if (discovery_released_ticks < 1u) {
                discovery_released_ticks++;
            }
        }
#endif
        if (discovery_stop) {
            speed_cmd = 0.0f;
            rate_cmd  = 0.0f;
        }
#endif

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
                LOG_INF("id=%u trk=%u step=%d disc=%d est=(%.1f,%.1f) "
                        "tgt=(%.1f,%.1f) spd=%.2f rate=%.2f",
                        (unsigned)element_id,
                        (unsigned)choreo_current_track(),
                        choreo_script_step(),
                        (int)own_state.discovered,
                        (double)odo.x, (double)odo.y,
                        (double)dir->target.x, (double)dir->target.y,
                        (double)speed_cmd, (double)rate_cmd);
            } else {
                LOG_INF("id=%u trk=%u step=%d disc=%d est=(%.1f,%.1f) "
                        "dir=%d spd=%.2f rate=%.2f",
                        (unsigned)element_id,
                        (unsigned)choreo_current_track(),
                        choreo_script_step(),
                        (int)own_state.discovered,
                        (double)odo.x, (double)odo.y, (int)dir->type,
                        (double)speed_cmd, (double)rate_cmd);
            }
#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
            /* Sanity signal for bench-verifying the sensor itself before
             * trusting it in a full run — same purpose as the Mini trace
             * below, adapted to the I2C 4-way state instead of GPIO
             * edge counts. */
            LOG_INF("id=%u line_state=0x%X (read %s) junction=%d "
                    "gray=%u/%u/%u/%u target_seen(1s)=0x%X "
                    "samples(1s)=%u err=%u enc_L=%.1fcm/s enc_R=%.1fcm/s",
                    (unsigned)element_id, (unsigned)line_state,
                    (line_ret == 0) ? "ok" : "FAIL", (int)junction,
                    (unsigned)lw.gray[0], (unsigned)lw.gray[1],
                    (unsigned)lw.gray[2], (unsigned)lw.gray[3],
                    (unsigned)trace_lw_seen, (unsigned)trace_lw_samples,
                    (unsigned)trace_lw_errors,
                    (double)enc.left_speed_cms, (double)enc.right_speed_cms);
            trace_lw_samples = 0;
            trace_lw_errors  = 0;
            trace_lw_seen    = 0;
#elif defined(CONFIG_I2C)
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

#ifdef CONFIG_DEMO_BLE_DEBUG
        {
            ble_debug_status_t st = {
                .step         = (int8_t)choreo_script_step(),
                .track        = choreo_current_track(),
                .quorum       = (uint8_t)scr.quorum_state,
                .fresh_peers  = (uint8_t)fresh_peers,
                .suspended    = choreo_goal_status() == CHOREO_STATE_SUSPENDED,
                .discovered   = own_state.discovered,
                .point_valid  = target_valid,
                .target_valid = dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT,
                .x = odo.x, .y = odo.y, .heading = odo.heading,
                .tx = dir->target.x, .ty = dir->target.y,
                .px = target.pos.x, .py = target.pos.y,
            };
#if defined(CONFIG_CUTEBOT_VARIANT_PRO)
            for (int i = 0; i < 4; i++) {
                st.gray_max = MAX(st.gray_max, lw.gray[i]);
            }
#elif defined(CONFIG_I2C)
            st.gray_max = line_dark_level ? 255u : 0u;
#endif
            ble_debug_set_status(&st);
        }
#endif

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
#endif /* CONFIG_DEMO_MODE_STRAIGHT_LINE */
}
