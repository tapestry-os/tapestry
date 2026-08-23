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

/* Default/idle altitude — the ramp target before Choreo has issued a
 * directive at all (boot). Once FLIGHT_FLYING starts ticking Choreo, real
 * altitude is Choreo-commanded (directive.target.z) instead: the automatic
 * per-element stagger this constant used to carry is gone — vertical
 * separation, if wanted, is now something a script expresses explicitly
 * (e.g. distinct z per track/element), and the physical safety net for
 * elements sharing an altitude is formation.c's existing repulsion/
 * emergency-spring math (still not itself flight-validated in 3D — see
 * the "Known limitations" section of ../../README.md). */
#define ALT_BASE_M        0.30f

#define LAND_TOUCHDOWN_M  0.05f

/* Geofence backstop: land immediately if the drone's ACTUAL measured
 * position strays past this distance from the world origin — independent
 * of, and in addition to, tracker.c's target-leash/arena-clamp above
 * (those bound the COMMANDED target, not where the airframe actually is;
 * defense in depth against a runaway tracking error, not a substitute for
 * this check). Hardware uses a room-scaled GEOFENCE_RADIUS_M=2.0m; reusing
 * DEMO_ARENA_LIMIT_M here keeps this backstop consistent with this
 * example's own arena scale rather than the hardware room's. */
#define GEOFENCE_RADIUS_M DEMO_ARENA_LIMIT_M

/* Mission-duration backstop: land unconditionally if FLIGHT_FLYING runs
 * longer than the script's own time bound plus margin — the robustness
 * net if the script stalls (e.g. stuck SUSPENDED with no peer ever
 * returning). Same formula as cf21bl-formation/src/main.c's
 * MISSION_DURATION_S. mission_elapsed_ms (below) is latched from the
 * FLIGHT_RAMPING -> FLIGHT_FLYING transition, not wall-clock/arm time —
 * there is no separate "arm" event in this substrate. */
#define MISSION_DURATION_S (CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u + 40u)

/* Approach gain converting world-frame position error (meters) into a
 * normalized [-1,1] twist command — substrate_webots.c scales the result
 * by its own MAX_SPEED_MPS. 2.0 reaches the twist's unit cap at a 0.5 m
 * error, well inside the DEMO_TARGET_LEASH_M tracker already enforces. */
#define TRACK_KP 2.0f

/* Quorum-recovery hold, fed to scr_set_quorum_hold_ms() below — see
 * cf21bl-formation/src/main.c's QUORUM_UP_MS comment (2026-07-19 flight
 * 2) for why quorum recovery requires SUSTAINED freshness, not just an
 * instantaneous fresh peer. */
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

    /* Real L5 SCR — quorum_min=quorum_target=1: this script only ever
     * needs one fresh partner (same threshold the old synthetic quorum
     * used). SCR_CAP_ACTUATOR satisfies the script's CHOREO_CAP_LOCOMOTION
     * requirement, enforced below by choreo_submit_script() now that a
     * real scr is registered. */
    scr_state_t scr;
    scr_init(&scr, element_id, 1, 1, SCR_CAP_ACTUATOR);
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

    /* Offline replay capture — see choreo_telemetry.h. NULL (no-op) unless
     * TAPESTRY_TELEMETRY_DIR is set. */
    choreo_telemetry_t *telemetry = choreo_telemetry_open(element_id);
    uint32_t telemetry_tick = 0;

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.orientation = orientation_identity();  /* overwritten with real
                                                       * ground truth below,
                                                       * once substrate_webots_
                                                       * step() has run once */

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);

    demo_setpoint_t target      = {0};
    bool            target_init = false;

    /* Choreo-commanded altitude setpoint — starts at the pre-Choreo
     * default and is updated to directive.target.z below, once
     * choreo_tick() has produced a MOVE_TO_POINT directive (see the
     * FLIGHT_FLYING case). Rate-limited to it the same way x/y target
     * tracking is rate-limited (TRACK_KP / the proportional law below),
     * not jumped to. */
    float z_setpoint_m = ALT_BASE_M;

    flight_state_t state           = FLIGHT_RAMPING;
    uint32_t        gossip_accum_ms = DEMO_GOSSIP_MS;
    uint32_t        mission_elapsed_ms = 0;   /* counts up once FLYING starts */
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
        position_t own_pos_m = { px, py, pz };
        own_state.position = own_pos_m;
        /* Real ground-truth orientation (Webots InertialUnit), not a
         * fabricated estimate — see substrate_webots_get_orientation()'s
         * comment. */
        substrate_quat_t webots_q;
        substrate_webots_get_orientation(&webots_q);
        own_state.orientation.w = webots_q.w;
        own_state.orientation.x = webots_q.x;
        own_state.orientation.y = webots_q.y;
        own_state.orientation.z = webots_q.z;
        wm_update_self(&wm, &own_state);

        if (!target_init) {
            demo_setpoint_init(&target, own_pos_m.x, own_pos_m.y);
            target_init = true;
        }

        substrate_twist_t sp = {0};

        switch (state) {
        case FLIGHT_RAMPING:
            if (pz < z_setpoint_m - 0.02f) {
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
            sp.linear.z = clampf((z_setpoint_m - pz) * 2.0f, -1.0f, 1.0f);

            /* Deliberately horizontal-only, re-examined (not left stale)
             * now that z is Choreo-commanded and genuinely varies rather
             * than a fixed per-ID constant: kept as a radius, not a
             * sphere, since this room has a flat floor and no modeled
             * ceiling — a horizontal-only boundary is still the more
             * natural safety envelope. Same rationale as
             * cf21bl-formation's main.c, which carries the full comment. */
            float origin_dist = sqrtf(own_pos_m.x * own_pos_m.x
                                       + own_pos_m.y * own_pos_m.y);
            if (origin_dist > GEOFENCE_RADIUS_M) {
                printf("id=%u geofence breach (%.2f m > %.2f m) — landing\n",
                       (unsigned)element_id, (double)origin_dist,
                       (double)GEOFENCE_RADIUS_M);
                state = FLIGHT_LANDING;
                break;
            }

            mission_elapsed_ms += WM_CYCLE_MS;
            if (mission_elapsed_ms > (uint32_t)MISSION_DURATION_S * 1000u) {
                printf("id=%u mission duration elapsed — landing\n",
                       (unsigned)element_id);
                state = FLIGHT_LANDING;
                break;
            }

            /* Real L5: recompute quorum/role/task_slot/abort from the
             * actual world model. scr_set_quorum_hold_ms() (below main())
             * means scr.quorum_state (and everything scr_tick() derives
             * from it — abort_state, leader, role, task_slot) is already
             * the held view: a LOST -> >=DEGRADED recovery must be
             * SUSTAINED for QUORUM_UP_MS before it's reported. Quorum
             * LOSS remains immediate regardless — see scr_set_quorum_
             * hold_ms()'s doc; same rationale as cf21bl-formation's
             * main.c, which carries the full comment. */
            scr_tick(&scr, &wm);
            bool quorum_up = scr.quorum_state != SCR_QUORUM_LOST;
            log_quorum_up = quorum_up;

            choreo_tick(&wm, &scr);

            /* HARD_RT gossip on the quorum-loss edge. Not delayed by
             * QUORUM_UP_MS despite scr's held view above: loss is always
             * reported immediately (scr_set_quorum_hold_ms() only ever
             * holds the RECOVERY edge). Fires once per outage, not every
             * tick it persists — scr_get_abort_state() is a level held
             * for the whole LOST period (see scr.h), so this checks the
             * NONE/CLEARED -> TRIGGERED edge specifically. Same rationale
             * as cf21bl-formation's main.c, which carries the full
             * comment; mirrors tapestry-os/subsys/runtime/runtime.c's
             * step 4b. */
            {
                static scr_abort_state_t last_abort_state;
                scr_abort_state_t abort_state = scr_get_abort_state(&scr);

                if (abort_state == SCR_ABORT_TRIGGERED &&
                    last_abort_state != SCR_ABORT_TRIGGERED) {
                    own_state.update_seq++;
                    own_state.goal_achieved = choreo_goal_achieved();
                    own_state.current_track = choreo_current_track();
                    printf("id=%u quorum LOST — sending HARD_RT gossip now\n",
                           (unsigned)element_id);
                    gossip_send(&own_state, TAPESTRY_QOS_HARD_RT);
                    gossip_accum_ms = 0;
                }
                last_abort_state = abort_state;
            }

            const tapestry_bse_directive_t *dir = choreo_get_directive();
            choreo_telemetry_write(telemetry, telemetry_tick,
                                   (double)telemetry_tick * WM_CYCLE_MS / 1000.0,
                                   &wm, &scr, dir);
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
                z_setpoint_m = dir->target.z;
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
        demo_set_leds(&wm, choreo_current_indicator());

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
            /* Publish this element's own-goal achievement so peers can
             * aggregate the scope="all" collective predicate
             * (choreo_collective_achieved()). Refreshed here, at the send,
             * rather than beside choreo_tick(): this loop keeps gossiping
             * after landing (see above), and a landed element must report
             * the achievement state it actually finished on, not the one it
             * held on its last airborne tick. Every element main loop that
             * gossips has to do this — without it the bit is permanently 0
             * and a scope="all" step can never advance on achievement. */
            own_state.goal_achieved = choreo_goal_achieved();
            own_state.current_track = choreo_current_track();
            gossip_send(&own_state, TAPESTRY_QOS_SOFT_RT);
        }
    }

    choreo_telemetry_close(telemetry);
    return 0;
}
