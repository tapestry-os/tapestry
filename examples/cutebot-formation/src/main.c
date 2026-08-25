/*
 * main.c — Tapestry Demo: Collective Formation (Cutebot ground rover)
 *
 * Two build modes (Kconfig choice DEMO_MODE, see Kconfig):
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
 *   DEMO_MODE_SHOWCASE — the original, flight-validated L4-only spring
 *     field (demo_compute_drive(), unchanged): pure emergent behavior, no
 *     SCR/L6/L7. Kept as a fallback — see Kconfig's help text.
 *
 * DEMO_MODE_CHOREO ran end-to-end on four physical robots on 2026-08-24
 * (hold -> form(grid) -> hold, BLE gossip, real L5 quorum); approach speed
 * is untuned.  The tests/ suite covers the same path host-side.  The
 * boot/negotiation/sync-hold path and DEMO_MODE_SHOWCASE are unchanged
 * from the flight-validated original — only the choice of what drives
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

#include "formation.h"

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

#define M_PI_F       3.14159265f

#ifdef CONFIG_DEMO_MODE_CHOREO
/* Quorum-recovery hold, fed to scr_set_quorum_hold_ms() below — requires
 * a LOST -> >=DEGRADED recovery to be SUSTAINED before scr_tick() reports
 * it, filtering a single lucky gossip frame from flickering quorum up for
 * one cycle (see scr_set_quorum_hold_ms()'s own doc for the full
 * semantics). Same value cf21bl-formation and webots-formation use
 * (flight-tested there, 2026-07-19 flight 2) — kept as the best available
 * starting point for BLE gossip timing here too, NOT independently tuned
 * against real Cutebot/BLE loss rates (see this file's top-of-file
 * "UNVALIDATED" note). */
#define QUORUM_UP_MS 2000
#endif

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/*
 * Seed each robot near the arena center on a tiny 3-unit circle, and
 * return the outward-facing heading angle.
 *
 * The 3-unit radius keeps all robots visually clustered at start while
 * placing adjacent robots ~4 units apart — far below DEMO_TARGET_SPACING —
 * so spring repulsion immediately drives them outward.
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
    *x       = 50.0f + 3.0f * cosf(a);
    *y       = 50.0f + 3.0f * sinf(a);
    *heading = a;
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    if (substrate_init() != 0) {
        LOG_WRN("substrate init failed — movement and signal disabled");
    }

    if (transport_init() != 0) {
        LOG_WRN("transport init failed — no peer awareness");
    }

    int n_total;
    const element_id_t element_id = transport_negotiate_id(&n_total);

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
     * while later robots are still exiting their own windows.
     *
     * Caps at DEMO_SYNC_GRACE_MS so a missing robot doesn't stall the demo.
     * Runs identically in both modes — Choreo isn't ticked yet, so there is
     * nothing mode-specific here.
     */
#define DEMO_SYNC_GRACE_MS 4000
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
}
