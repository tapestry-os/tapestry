/*
 * main.c — Tapestry hardware element (L4 CSM + L5 SCR + L6 BSE)
 *
 * One source file builds for all board targets.  Transport and actuation
 * are selected by the build system (see CMakeLists.txt):
 *
 *   BBC micro:bit V2  — BLE gossip, Cutebot motors/LEDs, serial telemetry
 *   EK-RA8D1          — UDP gossip, no actuators, UDP telemetry
 *   ESP-WROVER-KIT    — BLE + UDP gossip bridge, no actuators, UDP telemetry
 *
 * Startup sequence:
 *   tapestry_runtime_init() — substrate, transport, L4/L5/L6/L7 init
 *   choreo_submit_goal()    — set swarm goal
 *   main loop               — tick runtime, drive substrate from SCR state
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/runtime.h>
#include <tapestry/substrate.h>
#include <tapestry/choreo.h>

LOG_MODULE_REGISTER(element, LOG_LEVEL_INF);

/* Fixed logical positions (units in a 100×100 world). */
static const float ELEMENT_POS_X[] = { 10.0f, 50.0f, 30.0f, 70.0f, 90.0f };
static const float ELEMENT_POS_Y[] = { 10.0f, 50.0f, 80.0f, 20.0f, 60.0f };

/* ── Entry point ──────────────────────────────────────────────────────────── */

int main(void)
{
    const element_id_t element_id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;

    tapestry_runtime_config_t cfg = {
        .self_id          = element_id,
        .pos_x            = ELEMENT_POS_X[element_id],
        .pos_y            = ELEMENT_POS_Y[element_id],
        .consistency_bias = 0.0f,
        .quorum_min       = (uint8_t)CONFIG_TAPESTRY_QUORUM_MIN,
        .quorum_target    = (uint8_t)CONFIG_TAPESTRY_QUORUM_TARGET,
    };

    LOG_INF("element %u starting  pos=(%.1f, %.1f)  quorum=%d/%d",
            (unsigned)element_id,
            (double)cfg.pos_x, (double)cfg.pos_y,
            CONFIG_TAPESTRY_QUORUM_MIN, CONFIG_TAPESTRY_QUORUM_TARGET);

    if (tapestry_runtime_init(&cfg) != 0) {
        return -1;
    }

    choreo_goal_t goal = {
        .type   = CHOREO_GOAL_FORM,
        .target = { .x = 50.0f, .y = 50.0f },
        .radius = 30.0f,
        .shape  = TAPESTRY_BSE_SHAPE_CIRCLE,
    };
    choreo_submit_goal(&goal);
    LOG_INF("element %u ready — default goal: FORM circle r=30 @ (50,50)",
            (unsigned)element_id);

    /* ── Main loop ─────────────────────────────────────────────────────── */

    while (true) {
        tapestry_runtime_tick();

        const scr_state_t *scr = tapestry_runtime_scr();

        substrate_signal_t sig;
        if      (scr->quorum_state == SCR_QUORUM_HEALTHY)  sig = SUBSTRATE_SIGNAL_ACTIVE;
        else if (scr->quorum_state == SCR_QUORUM_DEGRADED) sig = SUBSTRATE_SIGNAL_DEGRADED;
        else                                                sig = SUBSTRATE_SIGNAL_FAILED;
        substrate_set_signal(sig);

        substrate_twist_t twist = {0};
        if (scr->quorum_state == SCR_QUORUM_HEALTHY) {
            twist.linear.x = (scr->role == SCR_ROLE_LEADER) ? 0.7f : 0.5f;
        }
        substrate_move(&twist);

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
}
