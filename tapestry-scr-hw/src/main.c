/*
 * main.c — Tapestry hardware element entry point (L4 CSM + L5 SCR)
 *
 * Runs on ESP-WROVER-KIT (element 0, WiFi) and EK-RA8D1 (element 1,
 * Ethernet).  All configuration is baked in at build time via Kconfig.
 *
 * Startup sequence:
 *   1. net_connect()    — join WiFi AP or wait for Ethernet DHCP
 *   2. comms_init()     — open gossip broadcast socket + metric socket
 *   3. wm_init()        — initialise L4 world model
 *   4. scr_init()       — initialise L5 quorum and role tracker
 *   5. main loop
 *
 * Main loop (WM_CYCLE_MS per iteration):
 *   1. comms_drain_inbox  — receive foreign gossip broadcasts
 *   2. wm_tick            — age L4 entries, recompute consistency
 *   3. scr_tick           — recompute role and quorum from wm snapshot
 *   4. wm_update_self     — refresh own entry with latest state
 *   5. comms_send_gossip  — broadcast own state (every GOSSIP_INTERVAL_MS)
 *   6. comms_send_metric  — send L4 metric to telemetry collector
 *   7. comms_send_scr_metric — send L5 metric to telemetry collector
 *   8. k_msleep           — sleep for rest of cycle
 *
 * There is no movement: position is fixed at startup.
 * There are no control messages from a central orchestrator.
 * Gossip flows directly between elements via UDP broadcast.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/scr.h>

#include "net_init.h"
#include "comms.h"
#include "sim_protocol.h"
#include "scr_protocol.h"

LOG_MODULE_REGISTER(element, LOG_LEVEL_INF);

/* Fixed logical positions (units in a 100×100 world).
 * These are the values gossipped to peers — they do not affect L4/L5
 * protocol behaviour, but give the telemetry plotter meaningful geometry. */
static const float ELEMENT_POS_X[] = { 10.0f, 50.0f, 30.0f, 70.0f, 90.0f };
static const float ELEMENT_POS_Y[] = { 10.0f, 50.0f, 80.0f, 20.0f, 60.0f };

int main(void)
{
    const element_id_t element_id   = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const float        pos_x        = ELEMENT_POS_X[element_id];
    const float        pos_y        = ELEMENT_POS_Y[element_id];

    LOG_INF("Tapestry element %u starting", (unsigned)element_id);
    LOG_INF("position (%.1f, %.1f)  quorum_min=%d  quorum_target=%d",
            (double)pos_x, (double)pos_y,
            CONFIG_TAPESTRY_QUORUM_MIN,
            CONFIG_TAPESTRY_QUORUM_TARGET);

    /* ── Network bring-up ──────────────────────────────────────────────── */
    if (net_connect() != 0) {
        LOG_ERR("network bring-up failed — halting");
        return -1;
    }

    /* ── Initialise own state ──────────────────────────────────────────── */
    element_state_t own_state = {0};
    own_state.id            = element_id;
    own_state.power_state   = POWER_ACTIVE;
    own_state.logical_clock = 0;
    own_state.update_seq    = 0;
    own_state.position.x    = pos_x;
    own_state.position.y    = pos_y;

    /* ── Initialise L4 world model ─────────────────────────────────────── */
    world_model_t wm;
    wm_init(&wm, element_id, &own_state,
            /* consistency_bias */ 0.0f);   /* pure AP: never freeze */

    /* ── Initialise L5 SCR ─────────────────────────────────────────────── */
    scr_state_t scr;
    scr_init(&scr, element_id,
             (uint8_t)CONFIG_TAPESTRY_QUORUM_MIN,
             (uint8_t)CONFIG_TAPESTRY_QUORUM_TARGET);

    /* ── Initialise comms ──────────────────────────────────────────────── */
    hw_comms_t comms;
    if (comms_init(&comms) != 0) {
        LOG_ERR("comms_init failed — halting");
        return -1;
    }

    LOG_INF("element %u ready — entering main loop", (unsigned)element_id);

    /* ── Main loop ─────────────────────────────────────────────────────── */
    uint32_t     gossip_accum_ms = 0;
    uint32_t     election_count  = 0;
    element_id_t last_leader     = ELEMENT_ID_INVALID;

    while (true) {

        /* 1. Receive gossip from peers */
        comms_drain_inbox(&comms, &wm, element_id);

        /* 2. Age L4 entries and recompute consistency */
        wm_tick(&wm, WM_CYCLE_MS);

        /* 3. Recompute L5 role and quorum */
        scr_tick(&scr, &wm);

        /* Track leadership changes for election_count telemetry. */
        if (scr.leader_id != last_leader) {
            if (last_leader     != ELEMENT_ID_INVALID ||
                scr.leader_id   != ELEMENT_ID_INVALID) {
                election_count++;
            }
            last_leader = scr.leader_id;

            LOG_INF("election #%u: leader=%u role=%u quorum=%u",
                    election_count,
                    (unsigned)scr.leader_id,
                    (unsigned)scr.role,
                    (unsigned)scr.quorum_state);
        }

        /* 4. Update own entry in world model */
        wm_update_self(&wm, &own_state);

        /* 5. Broadcast own state on interval */
        gossip_accum_ms += WM_CYCLE_MS;
        if (gossip_accum_ms >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            comms_send_gossip(&comms, &own_state);
            gossip_accum_ms = 0;
        }

        /* 6. Send L4 metric to telemetry collector */
        comms_send_metric(&comms, &wm, element_id);

        /* 7. Send L5 SCR metric to telemetry collector */
        comms_send_scr_metric(&comms, &scr, election_count);

        /* 8. Sleep for the rest of the cycle */
        k_msleep(WM_CYCLE_MS);
    }

    return 0;   /* unreachable */
}
