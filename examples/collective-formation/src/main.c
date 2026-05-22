/*
 * main.c — Tapestry Demo: Collective Formation (L4 only)
 *
 * Any number of robots running L4 CSM.  Each robot:
 *   1. Negotiates a unique element ID over transport during a 4-second boot window.
 *   2. Advertises its own dead-reckoning position via gossip.
 *   3. Receives peer positions into its local L4 world model.
 *   4. Computes a spring-field drive command (repulsion/attraction).
 *   5. Drives toward the formation equilibrium.
 *   6. Sets LEDs to reflect how many fresh peers are currently visible.
 *
 * No L5 SCR — formation is a pure L4 behavior.
 *
 * ID assignment is handled by transport_negotiate_id() — see transport.h and
 * CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS for the auto-ID protocol details.
 *
 * One binary for all robots — no per-robot build flags needed.
 *
 * See formation.h for physical calibration constants.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <tapestry/csm.h>
#include <tapestry/transport.h>
#include <tapestry/substrate.h>

#include "formation.h"

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

#define M_PI_F       3.14159265f

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static float start_clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * Place robot 'id' on a regular n_total-gon whose side equals
 * DEMO_TARGET_SPACING — the spring equilibrium radius.
 */
static void compute_start_pos(element_id_t id, int n_total, float *x, float *y)
{
    if (n_total <= 1) {
        *x = 50.0f;
        *y = 50.0f;
        return;
    }
    float n = (float)n_total;
    float R = DEMO_TARGET_SPACING / (2.0f * sinf(M_PI_F / n));
    float a = 2.0f * M_PI_F * (float)id / n;
    *x = start_clampf(50.0f + R * cosf(a), 5.0f, 95.0f);
    *y = start_clampf(50.0f + R * sinf(a), 5.0f, 95.0f);
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    if (substrate_init() != 0) {
        LOG_WRN("substrate init failed — movement and signal disabled");
    }

    if (transport_init() != 0) {
        LOG_WRN("transport init failed — no peer awareness");
    }

    int n_total;
    const element_id_t element_id = transport_negotiate_id(&n_total);

    float sx, sy;
    compute_start_pos(element_id, n_total, &sx, &sy);

    LOG_INF("Demo — element %u  start (%.1f, %.1f)  target_spacing=%.1f",
            (unsigned)element_id, (double)sx, (double)sy,
            (double)DEMO_TARGET_SPACING);

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.position.x  = sx;
    own_state.position.y  = sy;

    transport_send(&own_state);

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);   /* pure AP — never freeze */

    demo_odometry_t odo;
    demo_odometry_init(&odo, sx, sy);

    float    speed_cmd    = 0.0f;
    float    rate_cmd     = 0.0f;
    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;   /* send immediately on first tick */

    LOG_INF("Demo ready — entering main loop");

    while (true) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        demo_odometry_update(&odo, speed_cmd, rate_cmd, WM_CYCLE_MS);
        own_state.position.x = odo.x;
        own_state.position.y = odo.y;
        wm_update_self(&wm, &own_state);

        demo_compute_drive(&wm, &odo, &speed_cmd, &rate_cmd);
        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);
        demo_set_leds(&wm);
        demo_display_position(&odo);

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state);
            gossip_accum = 0;
        }

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
}
