/*
 * main.c — Tapestry hardware element (L4 CSM + L5 SCR)
 *
 * One source file builds for all board targets.  Transport and actuation
 * are selected by the build system (see CMakeLists.txt):
 *
 *   BBC micro:bit V2  — BLE gossip, Cutebot motors/LEDs, serial telemetry
 *   EK-RA8D1          — UDP gossip, no actuators, UDP telemetry
 *   ESP-WROVER-KIT    — BLE + UDP gossip bridge, no actuators, UDP telemetry
 *
 * Startup sequence:
 *   1. substrate_init() — stop motors, signal off (no-op on non-Cutebot boards)
 *   2. transport_init() — network + BLE bring-up, open sockets
 *   3. wm_init()        — initialise L4 world model
 *   4. scr_init()       — initialise L5 quorum and role tracker
 *   5. tapestry_init()  — initialise L6/L7 SDK
 *   6. main loop
 *
 * Main loop (WM_CYCLE_MS per iteration):
 *   1. transport_drain()          — receive gossip from all transports
 *   2. wm_tick()                  — age L4 entries, recompute consistency
 *   3. scr_tick()                 — recompute role and quorum
 *   4. wm_update_self()           — refresh own entry
 *   5. tapestry_tick()            — BSE: synthesise per-element directive
 *   6. transport_send()           — broadcast gossip on interval
 *   7. substrate_set_signal/move() — drive signal and motors from L5 state
 *   8. transport_send_telemetry() — metric frames to collector (UDP or serial)
 *   9. k_msleep()
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/scr.h>
#include <tapestry/transport.h>
#include <tapestry/substrate.h>
#include <tapestry/app.h>

LOG_MODULE_REGISTER(element, LOG_LEVEL_INF);

/* Fixed logical positions (units in a 100×100 world). */
static const float ELEMENT_POS_X[] = { 10.0f, 50.0f, 30.0f, 70.0f, 90.0f };
static const float ELEMENT_POS_Y[] = { 10.0f, 50.0f, 80.0f, 20.0f, 60.0f };

/* ── Entry point ──────────────────────────────────────────────────────────── */

int main(void)
{
    const element_id_t element_id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const float        pos_x      = ELEMENT_POS_X[element_id];
    const float        pos_y      = ELEMENT_POS_Y[element_id];

    LOG_INF("Tapestry element %u starting", (unsigned)element_id);
    LOG_INF("position (%.1f, %.1f)  quorum_min=%d  quorum_target=%d",
            (double)pos_x, (double)pos_y,
            CONFIG_TAPESTRY_QUORUM_MIN,
            CONFIG_TAPESTRY_QUORUM_TARGET);

    if (substrate_init() != 0) {
        LOG_WRN("substrate init failed — movement and signal disabled");
    }

    if (transport_init() != 0) {
        LOG_ERR("transport init failed — halting");
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
    wm_init(&wm, element_id, &own_state, /* consistency_bias */ 0.0f);

    /* ── Initialise L5 SCR ─────────────────────────────────────────────── */
    scr_state_t scr;
    scr_init(&scr, element_id,
             (uint8_t)CONFIG_TAPESTRY_QUORUM_MIN,
             (uint8_t)CONFIG_TAPESTRY_QUORUM_TARGET);

    /* ── Initialise L6/L7 SDK ──────────────────────────────────────────── */
    tapestry_init(element_id);
    {
        tapestry_goal_t goal = {
            .type   = TAPESTRY_GOAL_FORM,
            .target = { .x = 50.0f, .y = 50.0f },
            .radius = 30.0f,
            .shape  = TAPESTRY_BSE_SHAPE_CIRCLE,
        };
        tapestry_submit_goal(&goal);
        LOG_INF("default goal: FORM circle r=30 @ (50,50)");
    }

    LOG_INF("element %u ready — entering main loop", (unsigned)element_id);

    /* ── Main loop ─────────────────────────────────────────────────────── */
    uint32_t     gossip_accum_ms = 0;
    uint32_t     election_count  = 0;
    element_id_t last_leader     = ELEMENT_ID_INVALID;

    while (true) {

        /* 1. Receive gossip from all transports */
        transport_drain(&wm, element_id);

        /* 2. Age L4 entries and recompute consistency */
        wm_tick(&wm, WM_CYCLE_MS);

        /* 3. Recompute L5 role and quorum */
        scr_tick(&scr, &wm);

        if (scr.leader_id != last_leader) {
            if (last_leader   != ELEMENT_ID_INVALID ||
                scr.leader_id != ELEMENT_ID_INVALID) {
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

        /* 5. BSE tick — synthesise per-element behavioural directive */
        tapestry_tick(&wm, &scr);

        /* 6. Send gossip on interval */
        gossip_accum_ms += WM_CYCLE_MS;
        if (gossip_accum_ms >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state);
            gossip_accum_ms = 0;
        }

        /* 7. Signal quorum state and drive motors from L5 role */
        substrate_signal_t sig;
        if      (scr.quorum_state == SCR_QUORUM_HEALTHY)  sig = SUBSTRATE_SIGNAL_ACTIVE;
        else if (scr.quorum_state == SCR_QUORUM_DEGRADED) sig = SUBSTRATE_SIGNAL_DEGRADED;
        else                                               sig = SUBSTRATE_SIGNAL_FAILED;
        substrate_set_signal(sig);

        substrate_twist_t twist = {0};
        if (scr.quorum_state == SCR_QUORUM_HEALTHY) {
            twist.linear.x = (scr.role == SCR_ROLE_LEADER) ? 0.7f : 0.5f;
        }
        substrate_move(&twist);

        /* 8. Send telemetry (UDP unicast or serial CSV, per transport) */
        transport_send_telemetry(&wm, element_id, &scr, election_count);

        /* 9. Sleep for the rest of the cycle */
        k_msleep(WM_CYCLE_MS);
    }

    return 0;   /* unreachable */
}
