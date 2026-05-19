/*
 * main.c — Tapestry SCR simulation element entry point
 *
 * Runs both L4 (CSM) and L5 (SCR) on a Zephyr native_sim process.
 *
 * Environment variables:
 *   ELEMENT_ID        integer element ID [0..MAX_ELEMENTS)  default 0
 *   ORCH_PORT         orchestrator UDP port                  default 5100
 *   CONSISTENCY_BIAS  L4 AP/CP dial [0.0–1.0]               default 0.0
 *   QUORUM_MIN        SCR minimum fresh peers for DEGRADED   default 1
 *   QUORUM_TARGET     SCR minimum fresh peers for HEALTHY    default 2
 *
 * Main loop (WM_CYCLE_MS per iteration):
 *   1. comms_drain_inbox  — gossip + control messages
 *   2. wm_tick            — age L4 entries, recompute consistency
 *   3. scr_tick           — recompute role and quorum from wm snapshot
 *   4. Movement           — random walk + repulsion (if active and not degraded)
 *   5. wm_update_self     — always; keeps logical clock advancing
 *   6. choreo_tick        — synthesise per-element directive against fresh own state
 *   7. Send L4 gossip     — every GOSSIP_INTERVAL_MS
 *   8. Send L4 metric     — every cycle
 *   9. Send L5 SCR metric — every cycle
 *  10. k_msleep           — rest of cycle
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/scr.h>
#include <tapestry/choreo.h>
#include "movement.h"
#include "comms.h"
#include "comms_scr.h"
#include "sim_protocol.h"

LOG_MODULE_REGISTER(element, LOG_LEVEL_INF);

extern char   *getenv(const char *name);
extern int     atoi(const char *s);
extern double  atof(const char *s);

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static int env_int(const char *name, int default_val)
{
    const char *v = getenv(name);
    return (v != NULL) ? atoi(v) : default_val;
}

static float env_float(const char *name, float default_val)
{
    const char *v = getenv(name);
    return (v != NULL) ? (float)atof(v) : default_val;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    const int   element_id       = env_int("ELEMENT_ID",        0);
    const int   orch_port        = env_int("ORCH_PORT",         SIM_ORCH_PORT);
    const float consistency_bias = env_float("CONSISTENCY_BIAS", 0.0f);
    const int   quorum_min       = env_int("QUORUM_MIN",        1);
    const int   quorum_target    = env_int("QUORUM_TARGET",     2);

    /* ── Initialise own state ──────────────────────────────────────────── */
    element_state_t own_state = {0};
    own_state.id               = (element_id_t)element_id;
    own_state.partition_island = 0;
    own_state.logical_clock    = 0;
    own_state.update_seq       = 0;
    own_state.position.x       = 10.0f + (float)((element_id * 17) % 80);
    own_state.position.y       = 10.0f + (float)((element_id * 23) % 80);

    LOG_INF("element %d starting at (%.1f, %.1f) bias=%.2f qmin=%d qtgt=%d",
            element_id,
            (double)own_state.position.x,
            (double)own_state.position.y,
            (double)consistency_bias,
            quorum_min, quorum_target);

    /* ── Initialise L4 world model ──────────────────────────────────────── */
    world_model_t wm;
    wm_init(&wm, own_state.id, &own_state, consistency_bias);

    /* ── Initialise L5 SCR ──────────────────────────────────────────────── */
    scr_state_t scr;
    scr_init(&scr, own_state.id,
             (uint8_t)quorum_min,
             (uint8_t)quorum_target,
             SCR_CAP_NONE);

    /* ── Initialise L7 choreo ───────────────────────────────────────────── */
    choreo_init(own_state.id);
    {
        choreo_goal_t goal = {
            .type   = CHOREO_GOAL_FORM,
            .target = { .x = 50.0f, .y = 50.0f },
            .radius = 30.0f,
            .shape  = TAPESTRY_BSE_SHAPE_CIRCLE,
        };
        choreo_submit_goal(&goal);
    }

    /* ── Initialise comms ───────────────────────────────────────────────── */
    comms_t comms;
    if (comms_init(&comms, own_state.id, (uint16_t)orch_port) != 0) {
        LOG_ERR("comms_init failed — aborting");
        return -1;
    }

    /* ── Main loop ──────────────────────────────────────────────────────── */
    uint32_t     gossip_accum_ms = 0;
    uint32_t     election_count  = 0;
    element_id_t last_leader     = ELEMENT_ID_INVALID;

    while (!comms.shutdown) {

        /* 1. Process inbound messages */
        comms_drain_inbox(&comms, &wm, &own_state);

        /* 2. Age L4 entries and recompute consistency */
        wm_tick(&wm, WM_CYCLE_MS);

        /* 3. Recompute L5 role and quorum */
        scr_tick(&scr, &wm);

        /* Track leadership changes for election_count telemetry */
        if (scr.leader_id != last_leader) {
            if (last_leader != ELEMENT_ID_INVALID ||
                scr.leader_id != ELEMENT_ID_INVALID) {
                election_count++;
            }
            last_leader = scr.leader_id;
        }

        /* 4. Update position — only when active and L4 not frozen. */
        const wm_consistency_metric_t *metric = wm_get_metric(&wm);
        if (!metric->degraded) {
            movement_tick(&own_state, &wm);
        }
        wm_update_self(&wm, &own_state);

        /* 5. Choreo tick — synthesise per-element directive against fresh own state */
        choreo_tick(&wm, &scr);

        /* 6. Send gossip on interval */
        gossip_accum_ms += WM_CYCLE_MS;
        if (gossip_accum_ms >= GOSSIP_INTERVAL_MS) {
            comms_send_gossip(&comms, &own_state);
            gossip_accum_ms = 0;
        }

        /* 7. Send L4 consistency metric every cycle */
        comms_send_metric(&comms, &wm, own_state.id);

        /* 8. Send L5 SCR metric every cycle */
        comms_send_scr_metric(&comms, &scr, election_count);

        /* 9. Sleep for the rest of the cycle */
        k_msleep(WM_CYCLE_MS);
    }

    LOG_INF("element %d shutting down (leader was %d, elections: %u)",
            element_id, last_leader, election_count);
    return 0;
}
