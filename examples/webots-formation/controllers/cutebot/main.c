/*
 * main.c — ring.choreo.toml on a simulated Cutebot (Webots), the cutebot
 * substrate for the examples/webots-formation pattern.
 *
 * Webots counterpart to examples/cutebot-formation's DEMO_MODE_CHOREO
 * swarm branch: the exact same generated choreo_script.h (from
 * cutebot-formation's own ring.choreo.toml, via sdk/tools/choreoc.py)
 * drives the same L4/L6/L7 stack (world_model.c/bse.c/choreo.c) and the
 * same L3 gossip framing (gossip.c) — see ../../README.md for the
 * general pattern and ../cf21bl/main.c for the sibling substrate this
 * was modeled on.
 *
 * Element ID comes from Webots controllerArgs, NOT transport_negotiate_
 * id() — same shortcut ../cf21bl/main.c already takes. This is a
 * deliberate scope decision for this session's task, not an oversight:
 * on real hardware, auto-ID negotiation was already watched via each
 * robot's substrate_identify() LED blink and confirmed to assign
 * distinct, correct IDs, so re-exercising negotiation itself under
 * Webots isn't what this harness is for. What IS worth Webots here,
 * with IDs known-good, is reproducing the sync-hold timing symptom (one
 * robot moving before its peers finish their fresh-peer wait) and
 * getting fast, repeatable observability into scr.c/bse.c's task_slot ->
 * FORM-vertex rank assignment — a candidate for the "two robots at one
 * corner" hardware symptom that's independent of element_id itself and
 * fully exercised by the real gossip -> scr_tick -> bse.c path below
 * regardless of how element_id was obtained.
 *
 * Position/heading come from real Webots GPS/InertialUnit ground truth
 * (substrate_webots_cutebot_get_position()/get_yaw()), not dead
 * reckoning — demo_odometry_update() (integration from commanded speed)
 * is never called here, matching how ../cf21bl/main.c overwrites
 * own_state.position/orientation from its own substrate_webots_get_*()
 * every tick instead of propagating an internal estimate.
 *
 * Usage (set by the .wbt file's controllerArgs, not run manually):
 *   cutebot <element_id> <n_elements>
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <tapestry/csm.h>
#include <tapestry/scr.h>
#include <tapestry/choreo.h>
#include <tapestry/substrate.h>
#include "gossip.h"
#include "substrate_webots_cutebot.h"
#include "transceiver_udp_posix.h"
#include "formation.h"
#include "choreo_script.h"

/* UDP base port for the POSIX transceiver — distinct from cf21bl's 5900
 * (../cf21bl/main.c) purely so both examples' worlds could in principle
 * run side by side on one machine without a port clash; each world's
 * controllers only ever talk to their own siblings regardless. */
#define GOSSIP_BASE_PORT 5910

/* Real hardware's GOSSIP_INTERVAL_MS (csm.h, 500 ms) — kept unaccelerated
 * so the sync-hold/convergence timing this harness exists to reproduce
 * matches hardware cadence, not an artificially fast simulation rate. */

/* This script only ever needs one fresh partner to be HEALTHY — same
 * threshold cutebot-formation's real Kconfig default and
 * ../cf21bl/main.c's literal substitution both use. */
#define QUORUM_MIN    1
#define QUORUM_TARGET 1

/* Held-recovery window for scr_tick()'s quorum_state — see ../cf21bl/
 * main.c's identical constant and cutebot-formation/src/main.c's
 * QUORUM_UP_MS for the shared rationale (a LOST -> healthy recovery must
 * be SUSTAINED, not instantaneous, before it's reported; loss itself
 * stays immediate). */
#define QUORUM_UP_MS 2000u

/* Convergence hold before the main loop starts: wait until every
 * expected peer is fresh in the world model, so a robot that finishes
 * booting early doesn't start driving into a partial formation while a
 * later one is still starting up — the exact mechanism symptom #1 (id=0
 * moving before its peers finished this wait) needs to be inspected
 * against. Same cap as cutebot-formation/src/main.c's DEMO_SYNC_GRACE_MS. */
#define DEMO_SYNC_GRACE_MS 4000u

/* World-unit <-> Webots-meters conversion. ring.choreo.toml and every
 * constant demo_track_target() uses (formation.h) are expressed in the
 * abstract [0, WORLD_SIZE=100] unit space the whole L4-L7 stack already
 * shares — this is a pure substrate-level rendering choice with zero
 * effect on gossip/scr/bse/choreo logic. Deliberately NOT the real
 * chessboard's cramped 12.6625 mm/unit (formation.h's DEMO_MM_PER_UNIT)
 * — that scale is for formation.h's fence-margin arithmetic only; a
 * Webots-friendly 3 cm/unit (100 units = 3 m arena, ring radius 30 units
 * = 0.9 m) avoids physics/collision artifacts that would be Webots-scale
 * noise, not the bug this harness exists to chase. World-frame origin
 * (0,0) in Webots is abstract-unit center (50,50) — see world_to_units()/
 * units_to_world() below. */
#define METERS_PER_UNIT 0.03f
#define WORLD_ORIGIN_UNITS (WORLD_SIZE * 0.5f)

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: cutebot <element_id> <n_elements>\n");
        return -1;
    }
    element_id_t element_id = (element_id_t)atoi(argv[1]);
    uint8_t      n_total    = (uint8_t)atoi(argv[2]);

    if (substrate_init() != 0) {
        fprintf(stderr, "id=%u substrate_init failed — actuation disabled\n",
                (unsigned)element_id);
    }

    udp_posix_configure(element_id, n_total, GOSSIP_BASE_PORT);
    static const tapestry_transceiver_t *transceivers[1];
    transceivers[0] = &transceiver_udp_posix;
    gossip_register_transceivers(transceivers, 1);
    if (transceiver_udp_posix.init() != 0) {
        fprintf(stderr, "id=%u UDP transceiver init failed — no peer awareness\n",
                (unsigned)element_id);
    }

    int    timestep      = (int)wb_robot_get_basic_time_step();
    double coord_accum_ms = 0.0;

    /* First coordination tick's real position — substrate_webots_cutebot_
     * get_position() needs at least one wb_robot_step() to have run for
     * the GPS device to have a sample. */
    wb_robot_step(timestep);

    float px, py, pz, yaw;
    substrate_webots_cutebot_get_position(&px, &py, &pz);
    yaw = substrate_webots_cutebot_get_yaw();

    demo_odometry_t odo;
    demo_odometry_init(&odo, WORLD_ORIGIN_UNITS + px / METERS_PER_UNIT,
                             WORLD_ORIGIN_UNITS + py / METERS_PER_UNIT);
    odo.heading = yaw;

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.orientation = orientation_identity();  /* ground rover, no attitude sensing */
    own_state.position.x  = odo.x;
    own_state.position.y  = odo.y;

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);

    scr_state_t scr;
    scr_init(&scr, element_id, QUORUM_MIN, QUORUM_TARGET,
              SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
    scr_set_quorum_hold_ms(&scr, QUORUM_UP_MS);

    choreo_init(element_id);
    choreo_register_scr(&scr);
    if (choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN) != 0) {
        fprintf(stderr, "id=%u choreo script rejected — staying grounded\n",
                (unsigned)element_id);
        return -1;
    }
    printf("id=%u choreo \"%s\" loaded — %u steps, time bound %u s\n",
           (unsigned)element_id, CHOREO_NAME, (unsigned)CHOREO_SCRIPT_LEN,
           (unsigned)(CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u));

    float    speed_cmd    = 0.0f;
    float    rate_cmd     = 0.0f;
    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;   /* send immediately on first tick */

    /* ── Sync-hold: wait for every expected peer to be fresh ──────────────
     * Same coord_accum_ms accumulator (with carry, not a per-iteration
     * reset) the main loop below uses, so simulated time stays precise
     * across the sync-hold -> main-loop transition instead of drifting by
     * up to one timestep per WM_CYCLE_MS period. */
    bool     synced    = false;
    uint32_t waited_ms = 0;
    while (!synced && wb_robot_step(timestep) != -1) {
        coord_accum_ms += timestep;
        if (coord_accum_ms < (double)WM_CYCLE_MS) {
            continue;
        }
        coord_accum_ms -= (double)WM_CYCLE_MS;
        waited_ms      += WM_CYCLE_MS;

        gossip_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        int fresh = 0;
        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm.entries[i];
            if (e->is_active && !e->is_self && !e->is_stale) {
                fresh++;
            }
        }
        if (fresh >= n_total - 1 || waited_ms >= DEMO_SYNC_GRACE_MS) {
            synced = true;
            break;
        }

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            gossip_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }
    }

    printf("id=%u sync hold done — entering main loop\n", (unsigned)element_id);

    int      last_step        = -2;
    uint32_t trace_accum_ms   = 0;
    scr_abort_state_t last_abort_state = SCR_ABORT_NONE;

    while (wb_robot_step(timestep) != -1) {
        coord_accum_ms += timestep;
        if (coord_accum_ms < (double)WM_CYCLE_MS) {
            continue;
        }
        coord_accum_ms -= (double)WM_CYCLE_MS;

        /* ── Coordination tick (WM_CYCLE_MS cadence) ─────────────────── */
        gossip_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        substrate_webots_cutebot_get_position(&px, &py, &pz);
        odo.x       = WORLD_ORIGIN_UNITS + px / METERS_PER_UNIT;
        odo.y       = WORLD_ORIGIN_UNITS + py / METERS_PER_UNIT;
        odo.heading = substrate_webots_cutebot_get_yaw();

        own_state.position.x = odo.x;
        own_state.position.y = odo.y;
        wm_update_self(&wm, &own_state);

        scr_tick(&scr, &wm);
        bool quorum_up = scr.quorum_state != SCR_QUORUM_LOST;

        /* HARD_RT gossip on the quorum-loss edge — same idiom as
         * ../cf21bl/main.c and cutebot-formation/src/main.c. */
        scr_abort_state_t abort_state = scr_get_abort_state(&scr);
        if (abort_state == SCR_ABORT_TRIGGERED &&
            last_abort_state != SCR_ABORT_TRIGGERED) {
            own_state.update_seq++;
            choreo_publish_state(&own_state);
            printf("id=%u quorum LOST — sending HARD_RT gossip now\n",
                   (unsigned)element_id);
            gossip_send(&own_state, TAPESTRY_QOS_HARD_RT);
            gossip_accum = 0;
        }
        last_abort_state = abort_state;

        choreo_tick(&wm, &scr);
        choreo_publish_state(&own_state);

        if (choreo_script_step() != last_step) {
            last_step = choreo_script_step();
            printf("id=%u choreo step %d %s\n", (unsigned)element_id, last_step,
                   choreo_goal_status() == CHOREO_STATE_SUSPENDED ? "(suspended)" : "");
        }

        const tapestry_bse_directive_t *dir = choreo_get_directive();
        bool self_referential = choreo_current_goal_type() == CHOREO_GOAL_HOLD;
        if ((quorum_up || self_referential) &&
            dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
            demo_track_target(&wm, &odo, dir->target.x, dir->target.y,
                              &speed_cmd, &rate_cmd);
        } else {
            speed_cmd = 0.0f;
            rate_cmd  = 0.0f;
        }

        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);

        trace_accum_ms += WM_CYCLE_MS;
        if (trace_accum_ms >= 1000u) {
            trace_accum_ms = 0;
            int fresh = 0, active = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self) {
                    active++;
                    if (!e->is_stale) { fresh++; }
                }
            }
            printf("id=%u peers %d/%d est=(%.1f,%.1f) tgt=(%.1f,%.1f) "
                   "spd=%.2f rate=%.2f step=%d q=%c\n",
                   (unsigned)element_id, fresh, active,
                   (double)odo.x, (double)odo.y,
                   (double)dir->target.x, (double)dir->target.y,
                   (double)speed_cmd, (double)rate_cmd,
                   choreo_script_step(), quorum_up ? 'H' : 'L');
        }

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            choreo_publish_state(&own_state);
            gossip_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }
    }

    return 0;
}
