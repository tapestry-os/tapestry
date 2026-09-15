/*
 * main.c — Tapestry warehouse AMR, Webots ground-vehicle substrate
 *
 * The counterpart to examples/webots-formation/controllers/cf21bl/main.c
 * for a differential-drive ground element. The L3-L7 stack compiled in here
 * is byte-for-byte the same code that runs on real cf21bl and cutebot
 * hardware — world_model.c (L4), scr.c (L5), bse.c (L6), choreo.c (L7), and
 * gossip.c (L3 framing) are pulled in unmodified by sources.mk. This file,
 * substrate_webots.c, spring_track.c and rf_occlusion.c are the entire
 * example-specific surface.
 *
 * One controller process per robot (Webots' standard multi-robot model,
 * mirroring one OS process per physical element on real hardware). No
 * process can see another's memory; everything a robot knows about its
 * peers arrived as a UDP datagram carrying a tapestry_gossip_frame_t.
 *
 * Usage (set by each world's controllerArgs, not run by hand):
 *   rover <element_id> <n_elements> <scene> [fail_at_s]
 *
 *   scene      1, 2 or 3 — selects the Choreo script (scene.h) and whether
 *              the RF occlusion model is live (scene 2 only).
 *   fail_at_s  optional; scene 3's injected failure. Seconds after the
 *              script starts, after which this element hard-fails.
 *
 * Environment:
 *   CONSISTENCY_BIAS   0.0 (default, AP) .. 1.0 (CP). Same dial, same
 *                      meaning, and read the same way as
 *                      tapestry-csm-sim/zephyr/element/src/main.c. See the
 *                      CP freeze below and the README's scene 2 section.
 *   TAPESTRY_TELEMETRY_DIR  if set, per-tick CSV capture for offline replay.
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
#include "spring_track.h"
#include "avoid.h"
#include "rf_occlusion.h"
#include "status_tx.h"
#include "scene.h"

/* UDP base port — deliberately different from webots-formation's 5900 so
 * both examples can run simultaneously without cross-talk. */
#define GOSSIP_BASE_PORT 5800

#define DEMO_GOSSIP_MS 200u

/*
 * Quorum thresholds, in PEER counts (self is not counted — see scr.c).
 * min=1/target=2 with a fleet of 8 is chosen so that scene 2's 5/3 island
 * split leaves BOTH islands >= HEALTHY: partition tolerance has to be shown
 * by the fleet continuing to work, and a threshold that dropped the
 * 3-element island straight to LOST would only show it suspending. The
 * threshold is what encodes "how small a group is still a working group",
 * and in a warehouse a 3-robot island genuinely is one.
 */
#define QUORUM_MIN    1
#define QUORUM_TARGET 2

/* Quorum-recovery hold — a LOST -> HEALTHY recovery must be SUSTAINED this
 * long before it is reported (loss is always immediate). Directly relevant
 * here: elements transiting the deck band flap their links for a second or
 * two, and without this the fleet would re-elect and reflow on every flap.
 * Same lesson as cf21bl-formation's 2026-07-19 flight 2. */
#define QUORUM_UP_MS 2000u

/* ── Steering law (unicycle) ─────────────────────────────────────────────
 * The tracker produces a world-frame setpoint; a differential drive cannot
 * strafe to it, so it must turn toward it and then drive. */

/* Stop band: inside this the element is "there" and both wheels are
 * commanded to zero. Without it, GPS/IMU noise on a station-keeping robot
 * produces a meaningless heading error and the robot spins in place. */
#define ARRIVE_M    0.06f

/* Distance error -> forward speed fraction. Reaches full speed at a 0.67 m
 * error, inside the 1.5 m target leash the tracker enforces. */
#define TRACK_KP    1.5f

/* Heading error -> yaw rate fraction. Reaches full rate at ~40 degrees. */
#define YAW_KP      1.4f

/* Rectangular geofence backstop, half-extents in meters. Shared by every
 * scene (one rover binary), so sized to comfortably clear the TIGHTEST
 * wall layout in use (scene 1's 24x24 arena — dock wall face at 11.6 m,
 * racking faces at 12.0 m) while still being well outside every other
 * scene's own legitimate operating range (scene 3's ring reaches x=+/-9,
 * scene 2's deck crossing reaches x=+/-10). Rectangular and not a
 * radius, unlike the cf21bl substrate's GEOFENCE_RADIUS_M: a warehouse
 * floor is a rectangle, and a circular fence inscribed in it would forbid
 * the corners this example's own scripts legitimately use. This is
 * defense in depth behind the tracker's own arena clamp — it bounds where
 * the ROBOT actually is, not where its setpoint is.
 *
 * THIS VALUE HAS TO BE KEPT IN SYNC WITH EVERY SCENE'S FLOOR/WALL LAYOUT
 * BY HAND — nothing checks it at compile time, and getting it wrong is a
 * silent, permanent failure mode: a robot that measures past this
 * backstop enters ROVER_STOPPED with no recovery path, motionless for the
 * rest of the run, while its own L6 goal keeps updating normally the
 * whole time (so the status log looks completely healthy right up until
 * "geofence breach"). When scene 1's floor grew from 24x18 to 28x28
 * without this being updated to match, five of eight robots hit exactly
 * this and froze — easy to mistake for a physical collision or a
 * disperse-dynamics bug, since nothing about the symptom points at this
 * constant specifically; the `printf` right below is the only tell. */
#define GEOFENCE_HALF_X_M  11.4f
#define GEOFENCE_HALF_Y_M  11.4f

typedef enum {
    ROVER_RUNNING,
    ROVER_FAILED,     /* scene 3's injected hard failure */
    ROVER_STOPPED,    /* script complete, or a backstop fired */
} rover_state_t;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Wrap to [-pi, pi]. */
static float wrap_pi(float a)
{
    while (a >  (float)M_PI) { a -= 2.0f * (float)M_PI; }
    while (a < -(float)M_PI) { a += 2.0f * (float)M_PI; }
    return a;
}

static float env_float(const char *name, float dflt)
{
    const char *v = getenv(name);
    if (v == NULL || *v == '\0') {
        return dflt;
    }
    return (float)atof(v);
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr,
                "usage: rover <element_id> <n_elements> <scene> [fail_at_s]\n");
        return -1;
    }
    element_id_t element_id = (element_id_t)atoi(argv[1]);
    uint8_t      n_elements = (uint8_t)atoi(argv[2]);
    int          scene      = atoi(argv[3]);
    /* < 0 means "never fail" — the normal case for every element in every
     * scene except the one element scene 3 singles out. */
    float        fail_at_s  = (argc > 4) ? (float)atof(argv[4]) : -1.0f;

    scene_script_t script;
    if (!scene_get_script(scene, &script)) {
        fprintf(stderr, "id=%u unknown scene %d (expected 1, 2 or 3)\n",
                (unsigned)element_id, scene);
        return -1;
    }

    /* substrate_init() calls wb_robot_init() — must run before any other
     * Webots API call. */
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
    status_tx_init(GOSSIP_BASE_PORT);

    /* Install the RF obstruction model. Inert unless this scene's world
     * actually contains the mezzanine deck — see rf_occlusion.h. */
    rf_occlusion_configure(scene_has_rf_deck(scene));

    /*
     * SCR_CAP_ABS_POSITION is an honest claim on this platform: position
     * comes from a Webots GPS, which is real absolute ground truth. That is
     * what lets the scripts' frame = "absolute" targets mean the same thing
     * to every element. Contrast examples/cutebot-formation, whose elements
     * declare the same capability on top of dead reckoning from a shared
     * seed formula, and whose formation therefore drifts off-center over a
     * long mission.
     */
    scr_state_t scr;
    scr_init(&scr, element_id, QUORUM_MIN, QUORUM_TARGET,
             SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
    scr_set_quorum_hold_ms(&scr, QUORUM_UP_MS);

    choreo_init(element_id);
    choreo_register_scr(&scr);
    if (choreo_submit_script(script.steps, script.len) != 0) {
        fprintf(stderr, "id=%u choreo script rejected — staying parked\n",
                (unsigned)element_id);
        return -1;
    }

    const float consistency_bias = env_float("CONSISTENCY_BIAS", 0.0f);

    printf("id=%u scene %d choreo \"%s\" — %u steps, time bound %u s, "
           "consistency_bias %.2f%s\n",
           (unsigned)element_id, scene, script.name, (unsigned)script.len,
           (unsigned)(script.total_timeout_ms / 1000u),
           (double)consistency_bias,
           fail_at_s >= 0.0f ? " [WILL FAIL]" : "");

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.orientation = orientation_identity();

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, consistency_bias);

    demo_setpoint_t target      = {0};
    bool            target_init = false;

    rover_state_t state = ROVER_RUNNING;

    int      timestep       = (int)wb_robot_get_basic_time_step();
    double   coord_accum_ms = 0.0;
    uint32_t gossip_accum_ms = DEMO_GOSSIP_MS;
    uint32_t log_accum_ms   = 0;
    uint32_t elapsed_ms     = 0;
    int      last_step      = -2;
    bool     last_cp_frozen = false;
    uint8_t  last_blocked   = 0;

    while (wb_robot_step(timestep) != -1) {
        substrate_webots_step((double)timestep / 1000.0);

        coord_accum_ms += timestep;
        if (coord_accum_ms < (double)WM_CYCLE_MS) {
            continue;
        }
        coord_accum_ms -= (double)WM_CYCLE_MS;

        /* ── Coordination tick (WM_CYCLE_MS cadence) ───────────────────── */

        /* A failed element is dark: it does not drain, tick, gossip or
         * steer. Everything below is skipped so that from every peer's
         * point of view it has simply stopped existing — which is the
         * whole point of scene 3. It keeps calling wb_robot_step() only so
         * the robot stays in the world as a physical obstacle rather than
         * vanishing. */
        if (state == ROVER_FAILED) {
            continue;
        }

        /* Pose first, before draining: the RF receive filter evaluates each
         * arriving frame against this element's OWN live position, so it has
         * to be current before any frame is accepted or dropped. */
        float px, py, pz;
        substrate_webots_get_position(&px, &py, &pz);
        position_t own_pos_m = { px, py, pz };
        rf_occlusion_set_own_position(&own_pos_m);

        gossip_drain(&wm, element_id);

        /* Remote L6 directive path (wire.h v5) — fed in before
         * wm_tick/scr_tick/choreo_tick, the same ordering as
         * tapestry-os/subsys/runtime/runtime.c step 1b. No scene uses it;
         * it is wired here so this substrate is a complete element rather
         * than one that quietly lacks a path the others have. */
        tapestry_directive_frame_t dirf;
        if (gossip_poll_directive(&dirf, element_id)) {
            tapestry_bse_directive_t d = {
                .type     = (tapestry_bse_directive_type_t)dirf.type,
                .target   = { .x = dirf.x, .y = dirf.y, .z = dirf.z },
                .spring_k = dirf.spring_k,
                .spacing  = dirf.spacing,
            };
            choreo_remote_directive(&d, dirf.goal_id, dirf.src_id);
        }

        wm_tick(&wm, WM_CYCLE_MS);

        own_state.position = own_pos_m;

        substrate_quat_t q;
        substrate_webots_get_orientation(&q);
        own_state.orientation.w = q.w;
        own_state.orientation.x = q.x;
        own_state.orientation.y = q.y;
        own_state.orientation.z = q.z;
        wm_update_self(&wm, &own_state);

        /* Peers whose frames the deck swallowed since the last tick. Read
         * (and cleared) here purely for the log and the status feed. */
        uint8_t blocked = rf_occlusion_blocked_count();
        if (blocked != last_blocked) {
            printf("id=%u RF: %u of %u peers unheard behind the deck\n",
                   (unsigned)element_id, (unsigned)blocked,
                   (unsigned)(n_elements - 1));
            last_blocked = blocked;
        }

        elapsed_ms += WM_CYCLE_MS;

        /* ── Scene 3: injected hard failure ───────────────────────────────
         * A hard failure is SILENCE, not a goodbye. This deliberately does
         * NOT set ELEMENT_HEALTH_DEPARTED and does NOT send a final frame:
         * a self-declared departure is a different mechanism with different
         * semantics (csm.h; choreo.h's departure policies), and using it
         * here would prove the wrong thing. Peers must infer the loss from
         * silence alone, which they do at WM_STALE_THRESHOLD_MS. */
        if (fail_at_s >= 0.0f && state == ROVER_RUNNING &&
            (float)elapsed_ms * 0.001f >= fail_at_s) {
            state = ROVER_FAILED;
            substrate_webots_halt();
            printf("id=%u *** HARD FAILURE at t=%.1fs — going dark "
                   "(no departure flag, no final gossip). Peers should drop "
                   "it from swarm_size %ums later. ***\n",
                   (unsigned)element_id, (double)elapsed_ms * 0.001,
                   (unsigned)WM_STALE_THRESHOLD_MS);
            {
                wh_status_t st = {0};
                st.magic      = WH_STATUS_MAGIC;
                st.element_id = element_id;
                st.scene      = (uint8_t)scene;
                st.flags      = WH_STATUS_F_FAILED;
                st.x          = own_pos_m.x;
                st.y          = own_pos_m.y;
                status_tx_send(&st);
            }
            continue;
        }

        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);

        /* HARD_RT gossip on the quorum-loss edge — loss is reported
         * immediately even though recovery is held (QUORUM_UP_MS). Fires
         * once per outage, not every tick it persists. Mirrors
         * runtime.c step 4b. */
        {
            static scr_abort_state_t last_abort_state;
            scr_abort_state_t abort_state = scr_get_abort_state(&scr);
            if (abort_state == SCR_ABORT_TRIGGERED &&
                last_abort_state != SCR_ABORT_TRIGGERED) {
                own_state.update_seq++;
                choreo_publish_state(&own_state);
                printf("id=%u quorum LOST — sending HARD_RT gossip now\n",
                       (unsigned)element_id);
                gossip_send(&own_state, TAPESTRY_QOS_HARD_RT);
                gossip_accum_ms = 0;
            }
            last_abort_state = abort_state;
        }

        if (choreo_script_step() != last_step) {
            last_step = choreo_script_step();
            printf("id=%u choreo step %d%s\n", (unsigned)element_id, last_step,
                   choreo_goal_status() == CHOREO_STATE_SUSPENDED
                       ? " (suspended)" : "");
        }

        if (choreo_script_complete() && state == ROVER_RUNNING) {
            state = ROVER_STOPPED;
            printf("id=%u choreo complete — parked at (%.2f, %.2f)\n",
                   (unsigned)element_id, (double)own_pos_m.x,
                   (double)own_pos_m.y);
        }

        /* Script-duration backstop: stop unconditionally if the script has
         * run past its own time bound plus margin. The net for a script
         * stuck SUSPENDED with no peer ever returning. */
        if (state == ROVER_RUNNING &&
            elapsed_ms > script.total_timeout_ms + 40000u) {
            state = ROVER_STOPPED;
            printf("id=%u script time bound exceeded — stopping\n",
                   (unsigned)element_id);
        }

        /* Geofence backstop on the MEASURED position. */
        if (state == ROVER_RUNNING &&
            (fabsf(own_pos_m.x) > GEOFENCE_HALF_X_M ||
             fabsf(own_pos_m.y) > GEOFENCE_HALF_Y_M)) {
            state = ROVER_STOPPED;
            printf("id=%u geofence breach at (%.2f, %.2f) — stopping\n",
                   (unsigned)element_id, (double)own_pos_m.x,
                   (double)own_pos_m.y);
        }

        /*
         * ── L4 consistency dial (AP vs CP) ───────────────────────────────
         * wm.metric.degraded is set when the fraction of this element's
         * ACTIVE peers that are still FRESH falls below
         * consistency_bias * WM_QUORUM_FRACTION (world_model.c). At the
         * default bias of 0.0 the threshold is 0 and this is never true —
         * pure AP, the element keeps working through any partition. At
         * bias 1.0 the threshold is 0.5, so an element in a minority island
         * freezes until the majority's entries EXPIRE and stop counting
         * against it.
         *
         * L4 only reports; freezing is the application's choice, exactly as
         * in tapestry-csm-sim/zephyr/element/src/main.c. Scene 2 is the
         * comparison this exists for.
         */
        bool cp_frozen = (consistency_bias > 0.0f) && wm.metric.degraded;
        if (cp_frozen != last_cp_frozen) {
            printf("id=%u CP consistency freeze %s (fresh %u/%u active)\n",
                   (unsigned)element_id, cp_frozen ? "ENGAGED" : "released",
                   (unsigned)wm.metric.active_fresh,
                   (unsigned)wm.metric.active_total);
            last_cp_frozen = cp_frozen;
        }

        /* ── Steering ────────────────────────────────────────────────────── */

        substrate_twist_t sp = {0};

        if (!target_init) {
            demo_setpoint_init(&target, own_pos_m.x, own_pos_m.y);
            target_init = true;
        }

        const tapestry_bse_directive_t *dir = choreo_get_directive();

        /* HOLD is self-referential — it needs no peers, so it is allowed to
         * keep steering while quorum is LOST (choreo.h's per-goal quorum
         * note). MOVE joins it for a different reason: bse.c captures its
         * target as a fixed offset from the participant centroid ONCE at
         * activation (see scene2-partition.choreo.toml's step 3 comment),
         * so once underway it needs no peer either — freezing it on raw
         * isolation only strands an element exactly where it can never
         * regain one (scene 2's deck crossing: an isolated straggler stops
         * dead under the RF-dead band and can then never leave it).
         *
         * FORM joins them too, but for a subtler reason: unlike MOVE it does
         * NOT capture a fixed target on purpose — it recomputes one from
         * swarm_size every tick, by design (that's what produces the
         * 5-cell/3-cell/8-cell reflow scene 2 exists to show). But the
         * instant quorum is lost, choreo.c drops into CHOREO_STATE_SUSPENDED
         * and stops calling bse_tick() for any non-HOLD goal (see its
         * per-goal quorum note) — so the target IS already frozen at
         * whatever it last validly computed, same as MOVE's, just by an
         * incidental path rather than a deliberate one. Gating steering on
         * top of that frozen target strands a straggler exactly like MOVE
         * did: a scene 2 run with a slow last-mover crossing back after
         * everyone else already parked left it isolated with zero fresh
         * peers, frozen forever a few meters short, in this exact step.
         * That frozen target may be stale (computed for a smaller apparent
         * swarm than the true one) rather than provably correct, but
         * driving toward a plausible nearby point and reconciling on
         * arrival beats never arriving.
         *
         * This bypasses only the raw quorum_up floor for both. cp_frozen
         * (the actual AP/CP dial, world_model.c's consistency_bias *
         * WM_QUORUM_FRACTION) still applies to every goal including these —
         * at bias 0.0 it never engages, so this changes nothing for pure
         * AP's failure mode; at bias > 0 it engages before raw isolation
         * even occurs, so both goals still slow/freeze proportionally to
         * the dial. */
        bool quorum_up        = scr.quorum_state != SCR_QUORUM_LOST;
        choreo_goal_type_t goal = choreo_current_goal_type();
        bool self_referential = (goal == CHOREO_GOAL_HOLD) ||
                                (goal == CHOREO_GOAL_MOVE) ||
                                (goal == CHOREO_GOAL_FORM);
        bool may_steer        = (state == ROVER_RUNNING) && !cp_frozen &&
                                (quorum_up || self_referential);

        float min_dist_m = -1.0f;
        if (may_steer) {
            if (dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
                min_dist_m = demo_choreo_track(&wm, &own_pos_m, &target,
                                               dir->target.x, dir->target.y,
                                               WM_CYCLE_MS, element_id);
            } else if (dir->type == TAPESTRY_BSE_DIRECTIVE_MAINTAIN_SPRING) {
                /* The directive owns spacing and gain — see spring_track.h. */
                min_dist_m = demo_spring_track(&wm, &own_pos_m, &target,
                                               dir->spacing, dir->spring_k,
                                               WM_CYCLE_MS);
            }
        }

        /* Platform-level avoidance reflex, applied to the setpoint AFTER
         * L6's directive has produced one. Non-holonomic platforms need a
         * tangential term that tracker.c's holonomic repulsion does not
         * provide — see avoid.h. */
        if (may_steer) {
            avoid_apply(&wm, &own_pos_m, &target);
        }

        if (min_dist_m >= 0.0f && min_dist_m < DEMO_MIN_SEP_M) {
            printf("id=%u separation violation: nearest peer %.2f m "
                   "(min %.2f m)\n", (unsigned)element_id, (double)min_dist_m,
                   (double)DEMO_MIN_SEP_M);
        }

        if (may_steer) {
            /* World-frame setpoint error -> unicycle (forward, yaw rate).
             * A differential drive cannot strafe, so it turns toward the
             * setpoint and drives; the cos() factor keeps it from driving
             * forward while badly misaligned, which would otherwise carve a
             * wide arc through a neighbour's zone. */
            float ex   = target.x - own_pos_m.x;
            float ey   = target.y - own_pos_m.y;
            float dist = sqrtf(ex * ex + ey * ey);

            if (dist > ARRIVE_M) {
                float yaw_err = wrap_pi(atan2f(ey, ex) -
                                        substrate_webots_get_yaw());
                float align   = cosf(yaw_err);
                sp.angular.z = clampf(YAW_KP * yaw_err, -1.0f, 1.0f);
                sp.linear.x  = (align > 0.0f)
                               ? clampf(dist * TRACK_KP, 0.0f, 1.0f) * align
                               : 0.0f;
            }
        }

        substrate_move(&sp);
        demo_set_leds(&wm, choreo_current_indicator());

        /* ── Supervisor status feed (presentation only — status_tx.h) ───── */
        {
            int fresh = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self && !e->is_stale) { fresh++; }
            }

            wh_status_t st = {0};
            st.magic         = WH_STATUS_MAGIC;
            st.element_id    = element_id;
            st.scene         = (uint8_t)scene;
            st.quorum        = (uint8_t)scr.quorum_state;
            st.role          = (uint8_t)scr_get_role(&scr);
            st.swarm_size    = scr_get_swarm_size(&scr);
            st.task_slot     = scr_get_task_slot(&scr);
            st.fresh_peers   = (uint8_t)fresh;
            st.blocked_peers = blocked;
            st.step          = (uint8_t)choreo_script_step();
            st.x             = own_pos_m.x;
            st.y             = own_pos_m.y;
            if (choreo_goal_status() == CHOREO_STATE_SUSPENDED) {
                st.flags |= WH_STATUS_F_SUSPENDED;
            }
            if (cp_frozen) { st.flags |= WH_STATUS_F_CP_FROZEN; }
            if (blocked)   { st.flags |= WH_STATUS_F_RF_BLOCKED; }
            status_tx_send(&st);
        }

        log_accum_ms += WM_CYCLE_MS;
        if (log_accum_ms >= 1000u) {
            log_accum_ms = 0;
            /* MAINTAIN_SPRING carries no goal point — printing dir->target
             * for it shows a stale leftover from the previous directive.
             * Report what the directive actually says instead. */
            char goal_desc[40];
            if (dir->type == TAPESTRY_BSE_DIRECTIVE_MAINTAIN_SPRING) {
                snprintf(goal_desc, sizeof(goal_desc), "spring@%.1fm",
                         (double)dir->spacing);
            } else {
                snprintf(goal_desc, sizeof(goal_desc), "(%6.2f,%6.2f)",
                         (double)dir->target.x, (double)dir->target.y);
            }
            printf("id=%u t=%5.1f pos=(%6.2f,%6.2f) goal=%-15s "
                   "slot=%u/%u q=%c step=%d%s%s\n",
                   (unsigned)element_id, (double)elapsed_ms * 0.001,
                   (double)own_pos_m.x, (double)own_pos_m.y,
                   goal_desc,
                   (unsigned)scr_get_task_slot(&scr),
                   (unsigned)scr_get_swarm_size(&scr),
                   scr.quorum_state == SCR_QUORUM_HEALTHY  ? 'H' :
                   scr.quorum_state == SCR_QUORUM_DEGRADED ? 'D' : 'L',
                   choreo_script_step(),
                   cp_frozen ? " CP-FROZEN" : "",
                   blocked ? " RF-BLOCKED" : "");
        }

        /* Keep gossiping after the script completes — a parked element must
         * stay visible to peers still working, or it would age out of their
         * quorum and swarm_size and silently reflow their formation. Same
         * rationale as cf21bl-formation's flight-12 deadlock fix. The one
         * element that stops gossiping is the one scene 3 fails on purpose,
         * which returns above before reaching this. */
        gossip_accum_ms += WM_CYCLE_MS;
        if (gossip_accum_ms >= DEMO_GOSSIP_MS) {
            gossip_accum_ms = 0;
            own_state.update_seq++;
            /* Publishes this element's own goal_achieved bit so peers can
             * aggregate the scope="all" collective predicate. Every element
             * loop that gossips must call this — without it the bit is
             * permanently 0 and no scope="all" step can ever advance. */
            choreo_publish_state(&own_state);
            gossip_send(&own_state, TAPESTRY_QOS_SOFT_RT);
        }
    }

    return 0;
}
