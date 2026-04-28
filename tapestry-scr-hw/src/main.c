/*
 * main.c — Tapestry hardware element (L4 CSM + L5 SCR)
 *
 * Three board targets share this source tree.  Transport and output are
 * selected at compile time by the board overlay:
 *
 *   ESP-WROVER-KIT  (elem 0)  CONFIG_NETWORKING + CONFIG_BT
 *     → UDP gossip to RA8D1 over WiFi
 *     → BLE advertising gossip to micro:bit elements
 *     → UDP unicast metrics to collect.py
 *
 *   EK-RA8D1        (elem 1)  CONFIG_NETWORKING only
 *     → UDP gossip over Ethernet
 *     → UDP unicast metrics to collect.py
 *
 *   BBC micro:bit V2 (elem N)  CONFIG_BT + CONFIG_I2C
 *     → BLE advertising gossip to any other element in range
 *     → Cutebot motors and LEDs driven from L5 role/quorum state
 *     → METRIC CSV lines over USB serial to collect_serial.py
 *
 * Startup sequence:
 *   1. cutebot_init()     — (I2C) stop motors, LEDs off
 *   2. net_connect()      — (NETWORKING) acquire IPv4 via DHCP/WiFi
 *   3. udp_gossip_init()  — (NETWORKING) open gossip + metric sockets
 *   4. ble_gossip_init()  — (BT) enable BT, start scan + advertising
 *   5. wm_init()          — initialise L4 world model
 *   6. scr_init()         — initialise L5 quorum and role tracker
 *   7. main loop
 *
 * Main loop (WM_CYCLE_MS per iteration):
 *   1. drain gossip       — UDP inbox + BLE queue (whichever are compiled in)
 *   2. wm_tick            — age L4 entries, recompute consistency
 *   3. scr_tick           — recompute role and quorum
 *   4. wm_update_self     — refresh own entry
 *   5. send gossip        — UDP broadcast + BLE adv update (on interval)
 *   6. cutebot_update     — (I2C) set motors and LEDs from L5 state
 *   7. send metrics       — UDP unicast (NETWORKING) or serial CSV (else)
 *   8. k_msleep
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/scr.h>
#include <tapestry/app.h>

#include "net_init.h"
#include "udp_gossip.h"
#include "ble_gossip.h"
#include "cutebot.h"

LOG_MODULE_REGISTER(element, LOG_LEVEL_INF);

/* Fixed logical positions (units in a 100×100 world). */
static const float ELEMENT_POS_X[] = { 10.0f, 50.0f, 30.0f, 70.0f, 90.0f };
static const float ELEMENT_POS_Y[] = { 10.0f, 50.0f, 80.0f, 20.0f, 60.0f };

/* ── Serial metric output (micro:bit / no-network path) ────────────────────── */

#ifndef CONFIG_NETWORKING
/*
 * Emit one CSV metric line to stdout (UART → USB serial bridge).
 * collect_serial.py matches lines starting with "METRIC,".
 * Format: METRIC,<uptime_ms>,<elem_id>,<fresh_ratio>,<quorum>,<role>,
 *                <leader_id>,<election_count>,<mean_age_ms>
 */
static void emit_serial_metric(element_id_t element_id,
                                const world_model_t *wm,
                                const scr_state_t   *scr,
                                uint32_t             election_count)
{
    const wm_consistency_metric_t *m = wm_get_metric(wm);
    float age_sum = 0.0f;
    uint8_t age_cnt = 0;

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->state.id == ELEMENT_ID_INVALID || e->is_self || !e->is_active) {
            continue;
        }
        age_sum += (float)e->age_ms;
        age_cnt++;
    }
    float mean_age = age_cnt > 0 ? age_sum / (float)age_cnt : 0.0f;
    uint8_t leader_id = scr->leader_valid ? scr->leader_id : 0xFFu;

    printk("METRIC,%u,%u,%.4f,%u,%u,%u,%u,%.1f\n",
           (unsigned)k_uptime_get_32(),
           (unsigned)element_id,
           (double)m->fresh_ratio,
           (unsigned)scr->quorum_state,
           (unsigned)scr->role,
           (unsigned)leader_id,
           (unsigned)election_count,
           (double)mean_age);
}
#endif /* !CONFIG_NETWORKING */

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

    /* ── Cutebot (micro:bit only) ──────────────────────────────────────── */
#ifdef CONFIG_I2C
    if (cutebot_init() != 0) {
        LOG_WRN("Cutebot not found — physical movement disabled");
    }
#endif

    /* ── Network bring-up (ESP32 + RA8D1) ─────────────────────────────── */
#ifdef CONFIG_NETWORKING
    if (net_connect() != 0) {
        LOG_ERR("network bring-up failed — halting");
        return -1;
    }
#endif

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

    /* ── UDP comms (ESP32 + RA8D1) ─────────────────────────────────────── */
#ifdef CONFIG_NETWORKING
    udp_gossip_ctx_t comms;
    if (udp_gossip_init(&comms) != 0) {
        LOG_ERR("udp_gossip_init failed — halting");
        return -1;
    }
#endif

    /* ── BLE gossip (ESP32 + micro:bit) ────────────────────────────────── */
#ifdef CONFIG_BT
    if (ble_gossip_init() != 0) {
        LOG_WRN("BLE gossip init failed — BLE peers will not be heard");
    }
#endif

#ifndef CONFIG_NETWORKING
    printk("HEADER,uptime_ms,element_id,fresh_ratio,quorum_state,role,"
           "leader_id,election_count,mean_age_ms\n");
#endif

    LOG_INF("element %u ready — entering main loop", (unsigned)element_id);

    /* ── Main loop ─────────────────────────────────────────────────────── */
    uint32_t     gossip_accum_ms  = 0;
    uint32_t     election_count   = 0;
#ifndef CONFIG_NETWORKING
    uint32_t     metric_accum_ms  = 0;
#endif
    element_id_t last_leader      = ELEMENT_ID_INVALID;

    while (true) {

        /* 1. Receive gossip */
#ifdef CONFIG_NETWORKING
        udp_gossip_drain(&comms, &wm, element_id);
#endif
#ifdef CONFIG_BT
        ble_gossip_drain(&wm, element_id);
#endif

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

        /* 5. BSE tick — synthesise per-element behavioral directive */
        tapestry_tick(&wm, &scr);

        /* 6. Send gossip on interval */
        gossip_accum_ms += WM_CYCLE_MS;
        if (gossip_accum_ms >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
#ifdef CONFIG_NETWORKING
            udp_gossip_send(&comms, &own_state);
#endif
#ifdef CONFIG_BT
            ble_gossip_send(&own_state);
#endif
            gossip_accum_ms = 0;
        }

        /* 7. Drive Cutebot from L5 state (micro:bit only).
         * tapestry_get_directive() is available here for future odometry-
         * based motor control; see examples/collective-formation for the
         * dead-reckoning + spring-field integration pattern.
         *
         * TODO: replace board-specific #ifdef blocks in main.c with a thin
         * actuation HAL (actuation_init / actuation_update) so main.c
         * contains only application logic.  actuation_update() would accept
         * tapestry_get_directive() and translate it to cutebot / null
         * commands via CMakeLists-selected source files.  Prerequisite:
         * odometry integration for directive-driven motor control. */
#ifdef CONFIG_I2C
        cutebot_update(scr.role, scr.quorum_state);
#endif

        /* 8. Send metrics */
#ifdef CONFIG_NETWORKING
        udp_gossip_send_metric(&comms, &wm, element_id);
        udp_gossip_send_scr_metric(&comms, &scr, election_count);
#else
        metric_accum_ms += WM_CYCLE_MS;
        if (metric_accum_ms >= GOSSIP_INTERVAL_MS) {
            emit_serial_metric(element_id, &wm, &scr, election_count);
            metric_accum_ms = 0;
        }
#endif

        /* 8. Sleep for the rest of the cycle */
        k_msleep(WM_CYCLE_MS);
    }

    return 0;   /* unreachable */
}
