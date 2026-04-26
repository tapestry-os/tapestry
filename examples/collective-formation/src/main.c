/*
 * main.c — Tapestry Demo: Collective Formation (L4 only)
 *
 * Four Cutebot Mini robots running L4 CSM.  Each robot:
 *   1. Advertises its own dead-reckoning position via BLE gossip.
 *   2. Receives peer positions into its local L4 world model.
 *   3. Computes a spring-field drive command (repulsion/attraction).
 *   4. Drives toward the formation equilibrium.
 *   5. Sets LEDs to reflect how many fresh peers are currently visible.
 *
 * No L5 SCR — formation is a pure L4 behaviour.
 *
 * Starting positions (logical 100×100 world — corners):
 *   Element 0: (15, 15)   Element 1: (85, 15)
 *   Element 2: (15, 85)   Element 3: (85, 85)
 *
 * Build one binary per robot, each with its own element ID:
 *   west build -b bbc_microbit_v2 tapestry/examples/collective-formation
 *   west build -b bbc_microbit_v2 tapestry/examples/collective-formation \
 *       -- -DCONFIG_TAPESTRY_ELEMENT_ID=1
 *   ... (repeat for IDs 2 and 3)
 *
 * See formation.h for physical calibration constants.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/csm.h>

#include "ble_gossip.h"
#include "cutebot.h"
#include "formation.h"
#include "sim_protocol.h"

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

/*
 * Robots start at their equilibrium positions (square side ≈ 30 units,
 * centred at 50,50).  Spring forces are near-zero at boot so there is no
 * violent initial spread.  The Lamport-clock rejoin fix (wm_receive_gossip)
 * makes these spread starts safe even after a reboot.
 */
static const float START_X[] = { 25.0f, 75.0f, 25.0f, 75.0f };
static const float START_Y[] = { 25.0f, 25.0f, 75.0f, 75.0f };

int main(void)
{
    const element_id_t element_id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const float        sx         = START_X[element_id];
    const float        sy         = START_Y[element_id];

    LOG_INF("Demo — element %u  start (%.1f, %.1f)  target_spacing=%.1f",
            (unsigned)element_id, (double)sx, (double)sy,
            (double)DEMO_TARGET_SPACING);

    if (cutebot_init() != 0) {
        LOG_WRN("Cutebot not found — movement and LEDs disabled");
    }

    if (ble_gossip_init() != 0) {
        LOG_WRN("BLE gossip init failed — no peer awareness");
    }

    element_state_t own_state = {0};
    own_state.id           = element_id;
    own_state.power_state  = POWER_ACTIVE;
    own_state.position.x   = sx;
    own_state.position.y   = sy;

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);   /* pure AP — never freeze */

    demo_odometry_t odo;
    demo_odometry_init(&odo, sx, sy);

    int      left_cmd     = 0;
    int      right_cmd    = 0;
    uint32_t gossip_accum = 0;

    LOG_INF("Demo ready — entering main loop");

    while (true) {

        /* 1. Receive peer positions from BLE gossip queue */
        ble_gossip_drain(&wm, element_id);

        /* 2. Age world model entries, recompute consistency */
        wm_tick(&wm, WM_CYCLE_MS);

        /* 3. Update own position estimate via dead reckoning */
        demo_odometry_update(&odo, left_cmd, right_cmd, WM_CYCLE_MS);
        own_state.position.x = odo.x;
        own_state.position.y = odo.y;

        /* 4. Push updated position into world model */
        wm_update_self(&wm, &own_state);

        /* 5. Compute formation drive from world model */
        demo_compute_drive(&wm, &odo, &left_cmd, &right_cmd);

        /* 6. Apply motor command */
        cutebot_drive(left_cmd, right_cmd);

        /* 7. Set LEDs from world model peer visibility */
        demo_set_leds(&wm);

        /* 8. Show dead-reckoning position on micro:bit LED matrix */
        demo_display_position(&odo);

        /* 9. Broadcast own position on gossip interval */
        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            ble_gossip_send(&own_state);
            gossip_accum = 0;
        }

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
}
