/*
 * main.c — change-partners choreo on a simulated Crazyflie (Webots),
 * the cf21bl substrate for the examples/webots-formation pattern
 *
 * Webots counterpart to examples/cf21bl-formation's Choreo mode: the exact
 * same generated choreo_script.h (from cf21bl-formation's own
 * change-partners.choreo.toml, via sdk/tools/choreoc.py) drives the same
 * L4/L6/L7 stack (world_model.c/bse.c/choreo.c, unmodified) and the same
 * L3 gossip framing (gossip.c, unmodified — see ../common/zephyr_shim/).
 * Only L1 (this file's flight-state machine + substrate_webots.c, both
 * cf21bl-specific) is new to this substrate; L3's transceiver
 * (../common/transceiver_udp_posix.c) is shared across every substrate in
 * this example. See ../../README.md for the full architecture, the
 * "Porting to a different element" section for what a new substrate needs to
 * write, and the deliberate scope cuts noted inline below.
 *
 * One controller process per drone (Webots' standard multi-robot model,
 * same as one OS process per physical element on real hardware). Element
 * ID and swarm size come from Webots controllerArgs (see
 * ../../worlds/change_partners.wbt) rather than the radio auto-ID
 * negotiation cf21bl-formation uses — Webots starts every controller
 * deterministically together, so there is no discovery problem to solve.
 *
 * Usage (set by the .wbt file's controllerArgs, not run manually):
 *   cf21bl <element_id> <n_elements>
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <tapestry/csm.h>
#include <tapestry/wire.h>
#include <tapestry/scr.h>
#include <tapestry/choreo.h>
#include <tapestry/substrate.h>
#include "gossip.h"
#include "substrate_webots.h"
#include "transceiver_udp_posix.h"
#include "tracker.h"
#include "choreo_script.h"
#include "choreo_telemetry.h"

/* Per-drone cruise altitude, staggered by element_id — same rationale as
 * cf21bl-formation (reduces downwash interaction), widened a bit beyond
 * hardware's 0.20 m stagger since Webots physics doesn't need the tighter
 * hardware battery/downwash tradeoff that set that value. Real margin, but
 * NOT what fixed the earlier slow-swap issue — see the "Known limitations"
 * section of ../../README.md, and the note in
 * examples/cf21bl-formation/change-partners.choreo.toml itself. */
#define ALT_BASE_M        0.30f
#define ALT_STEP_PER_ID_M 0.40f

#define LAND_TOUCHDOWN_M  0.05f

/* Approach gain converting world-frame position error (meters) into a
 * normalized [-1,1] twist command — substrate_webots.c scales the result
 * by its own MAX_SPEED_MPS. 2.0 reaches the twist's unit cap at a 0.5 m
 * error, well inside the DEMO_TARGET_LEASH_M tracker already enforces. */
#define TRACK_KP 2.0f

/* Synthetic L5 quorum debounce — see cf21bl-formation/src/main.c's
 * QUORUM_UP_MS comment (2026-07-19 flight 2) for why quorum requires
 * SUSTAINED freshness, not just an instantaneous fresh peer. */
#define QUORUM_UP_MS 2000u

/* UDP base port for the POSIX transceiver — see transceiver_udp_posix.h. */
#define GOSSIP_BASE_PORT 5900

#define DEMO_GOSSIP_MS 200u

typedef enum {
    FLIGHT_RAMPING,
    FLIGHT_FLYING,
    FLIGHT_LANDING,
    FLIGHT_LANDED,
} flight_state_t;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: cf21bl <element_id> <n_elements>\n");
        return -1;
    }
    element_id_t element_id = (element_id_t)atoi(argv[1]);
    uint8_t      n_elements = (uint8_t)atoi(argv[2]);

    /* substrate_init() calls wb_robot_init() — must run before any other
     * Webots API call, including reading controllerArgs-derived state
     * back through the device tree. */
    if (substrate_init() != 0) {
        fprintf(stderr, "id=%u substrate_init failed — actuation disabled\n",
                (unsigned)element_id);
    }

    udp_posix_configure(element_id, n_elements, GOSSIP_BASE_PORT);
    static const tapestry_transceiver_t *transceivers[1];
    transceivers[0] = &transceiver_udp_posix;
    gossip_register_transceivers(transceivers, 1);
    if (transceiver_udp_posix.init() != 0) {
        fprintf(stderr, "id=%u UDP transceiver init failed — no peer awareness\n",
                (unsigned)element_id);
    }

    choreo_init(element_id);
    if (choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN) != 0) {
        fprintf(stderr, "id=%u choreo script rejected — staying grounded\n",
                (unsigned)element_id);
        return -1;
    }
    printf("id=%u choreo \"%s\" loaded — %u steps, time bound %u s\n",
           (unsigned)element_id, CHOREO_NAME, (unsigned)CHOREO_SCRIPT_LEN,
           (unsigned)(CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u));

    /* Offline replay capture — see choreo_telemetry.h. NULL (no-op) unless
     * TAPESTRY_TELEMETRY_DIR is set. */
    choreo_telemetry_t *telemetry = choreo_telemetry_open(element_id);
    uint32_t telemetry_tick = 0;

    scr_state_t scr_synth = {0};
    scr_synth.own_id       = element_id;
    scr_synth.quorum_state = SCR_QUORUM_LOST;   /* until peers are fresh */

    element_state_t own_state = {0};
    own_state.id = element_id;

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);

    demo_setpoint_t target      = {0};
    bool            target_init = false;

    const float cruise_alt_m = ALT_BASE_M + (float)element_id * ALT_STEP_PER_ID_M;

    flight_state_t state           = FLIGHT_RAMPING;
    uint32_t        gossip_accum_ms = DEMO_GOSSIP_MS;
    uint32_t        fresh_streak_ms = 0;
    bool            log_quorum_up   = false;
    int             last_step       = -2;

    int    timestep      = (int)wb_robot_get_basic_time_step();
    double coord_accum_ms = 0.0;
    uint32_t log_accum_ms = 0;
    float  last_dir_tx = 0.0f, last_dir_ty = 0.0f;   /* debug: real L6 goal */

    while (wb_robot_step(timestep) != -1) {
        substrate_webots_step((double)timestep / 1000.0);

        coord_accum_ms += timestep;
        if (coord_accum_ms < (double)WM_CYCLE_MS) {
            continue;
        }
        coord_accum_ms -= (double)WM_CYCLE_MS;

        /* ── Coordination tick (WM_CYCLE_MS cadence) ─────────────────── */
        gossip_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        float px, py, pz;
        substrate_webots_get_position(&px, &py, &pz);
        position_t own_pos_m = { px, py };
        own_state.position = own_pos_m;
        wm_update_self(&wm, &own_state);

        if (!target_init) {
            demo_setpoint_init(&target, own_pos_m.x, own_pos_m.y);
            target_init = true;
        }

        substrate_twist_t sp = {0};

        switch (state) {
        case FLIGHT_RAMPING:
            if (pz < cruise_alt_m - 0.02f) {
                sp.linear.z = 1.0f;
            } else {
                state = FLIGHT_FLYING;
                printf("id=%u cruise altitude reached — formation control active\n",
                       (unsigned)element_id);
            }
            break;

        case FLIGHT_FLYING: {
            /* Proportional altitude hold — substrate_webots.c's twist.z
             * is a climb-RATE command (see substrate_webots.h), so cruise
             * altitude is held by commanding a rate proportional to the
             * error, not a one-shot absolute setpoint. */
            sp.linear.z = clampf((cruise_alt_m - pz) * 2.0f, -1.0f, 1.0f);

            /* Synthetic L5 quorum from L4 freshness, debounced on the way
             * up — see QUORUM_UP_MS above. */
            int fresh_peers = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self && !e->is_stale) {
                    fresh_peers++;
                }
            }
            if (fresh_peers >= 1) {
                if (fresh_streak_ms < QUORUM_UP_MS) {
                    fresh_streak_ms += WM_CYCLE_MS;
                }
            } else {
                fresh_streak_ms = 0;
            }
            bool quorum_up = fresh_streak_ms >= QUORUM_UP_MS;
            log_quorum_up = quorum_up;
            scr_synth.fresh_count  = (uint8_t)fresh_peers;
            scr_synth.quorum_state = quorum_up ? SCR_QUORUM_HEALTHY : SCR_QUORUM_LOST;

            choreo_tick(&wm, &scr_synth);

            const tapestry_bse_directive_t *dir = choreo_get_directive();
            choreo_telemetry_write(telemetry, telemetry_tick,
                                   (double)telemetry_tick * WM_CYCLE_MS / 1000.0,
                                   &wm, &scr_synth, dir);
            telemetry_tick++;

            if (choreo_script_step() != last_step) {
                last_step = choreo_script_step();
                printf("id=%u choreo step %d %s\n", (unsigned)element_id, last_step,
                       choreo_goal_status() == CHOREO_STATE_SUSPENDED ? "(suspended)" : "");
            }

            if (choreo_script_complete()) {
                printf("id=%u choreo complete — resting: landing in place at (%.2f, %.2f)\n",
                       (unsigned)element_id, (double)own_pos_m.x, (double)own_pos_m.y);
                state = FLIGHT_LANDING;
                break;
            }

            bool self_referential = choreo_current_goal_type() == CHOREO_GOAL_HOLD;
            float min_dist_m = -1.0f;
            if (dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
                last_dir_tx = dir->target.x;
                last_dir_ty = dir->target.y;
            }
            if ((quorum_up || self_referential) &&
                dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
                min_dist_m = demo_choreo_track(&wm, &own_pos_m, &target,
                                               dir->target.x, dir->target.y,
                                               WM_CYCLE_MS, element_id);
            }
            if (min_dist_m >= 0.0f && min_dist_m < DEMO_MIN_SEP_M) {
                printf("id=%u separation violation: nearest peer %.2f m (min %.2f m)\n",
                       (unsigned)element_id, (double)min_dist_m, (double)DEMO_MIN_SEP_M);
            }

            /* World-frame position error -> body-frame velocity twist. */
            float ex = target.x - own_pos_m.x;
            float ey = target.y - own_pos_m.y;
            float world_vx = clampf(ex * TRACK_KP, -1.0f, 1.0f);
            float world_vy = clampf(ey * TRACK_KP, -1.0f, 1.0f);
            float yaw = substrate_webots_get_yaw();
            sp.linear.x =  world_vx * cosf(yaw) + world_vy * sinf(yaw);
            sp.linear.y = -world_vx * sinf(yaw) + world_vy * cosf(yaw);
            break;
        }

        case FLIGHT_LANDING:
            sp.linear.z = -1.0f;
            if (pz <= LAND_TOUCHDOWN_M) {
                state = FLIGHT_LANDED;
                printf("id=%u landed\n", (unsigned)element_id);
            }
            break;

        case FLIGHT_LANDED:
        default:
            sp.linear.z = -1.0f;
            break;
        }

        substrate_move(&sp);
        demo_set_leds(&wm);

        log_accum_ms += WM_CYCLE_MS;
        if (log_accum_ms >= 1000u) {
            log_accum_ms = 0;
            int fresh = 0, active = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self) {
                    active++;
                    if (!e->is_stale) { fresh++; }
                }
            }
            printf("id=%u %s peers %d/%d pos=(%.2f,%.2f) tgt=(%.2f,%.2f) goal=(%.2f,%.2f) alt=%.2f "
                   "step=%d q=%c%s\n",
                   (unsigned)element_id,
                   state == FLIGHT_RAMPING ? "RAMPING" :
                   state == FLIGHT_FLYING  ? "FLYING"  :
                   state == FLIGHT_LANDING ? "LANDING" : "LANDED",
                   fresh, active, (double)own_pos_m.x, (double)own_pos_m.y,
                   (double)target.x, (double)target.y,
                   (double)last_dir_tx, (double)last_dir_ty, (double)pz,
                   choreo_script_step(), log_quorum_up ? 'H' : 'L',
                   choreo_goal_status() == CHOREO_STATE_SUSPENDED ? "(susp)" : "");
        }

        /* Keep gossiping after landing — a 2-drone script has no "departs
         * and is healed around" beat; both drones finish. Silencing on
         * landing would age the still-airborne partner's only peer out of
         * quorum mid-EXCHANGE. Same rationale as cf21bl-formation's
         * main.c (2026-07-19 flight 12 deadlock). */
        gossip_accum_ms += WM_CYCLE_MS;
        if (gossip_accum_ms >= DEMO_GOSSIP_MS) {
            gossip_accum_ms = 0;
            own_state.update_seq++;
            gossip_send(&own_state, TAPESTRY_QOS_SOFT_RT);
        }
    }

    choreo_telemetry_close(telemetry);
    return 0;
}
