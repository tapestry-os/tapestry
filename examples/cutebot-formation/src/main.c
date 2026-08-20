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

/*
 * Seed each robot near the arena center on a tiny 3-unit circle, and
 * return the outward-facing heading angle.
 *
 * The 3-unit radius keeps all robots visually clustered at start while
 * placing adjacent robots ~4 units apart — far below DEMO_TARGET_SPACING —
 * so spring repulsion immediately drives them outward.
 *
 * The heading is set to the outward angle so every robot's spring force
 * projects fully forward on tick 1.  Without this, robots whose force
 * points opposite to heading 0 drive backward and collide with neighbors.
 *
 * Physical placement: orient each robot so its physical forward direction
 * matches its assigned heading (see README for per-ID compass directions).
 */
static void compute_start_pos(element_id_t id, int n_total,
                               float *x, float *y, float *heading)
{
    float a = 2.0f * M_PI_F * (float)id / (float)(n_total > 1 ? n_total : 1);
    *x       = 50.0f + 3.0f * cosf(a);
    *y       = 50.0f + 3.0f * sinf(a);
    *heading = a;
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

    float sx, sy, shead;
    compute_start_pos(element_id, n_total, &sx, &sy, &shead);

    LOG_INF("Demo — element %u  start (%.1f, %.1f)  heading=%.2f rad  target_spacing=%.1f",
            (unsigned)element_id, (double)sx, (double)sy,
            (double)shead, (double)DEMO_TARGET_SPACING);

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.orientation = orientation_identity();  /* ground rover, no attitude sensing */
    own_state.position.x  = sx;
    own_state.position.y  = sy;

    transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);   /* pure AP — never freeze */

    demo_odometry_t odo;
    demo_odometry_init(&odo, sx, sy);
    odo.heading = shead;

    float    speed_cmd    = 0.0f;
    float    rate_cmd     = 0.0f;
    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;   /* send immediately on first tick */

    /*
     * Convergence hold: wait until all n_total-1 expected peers are visible
     * and fresh in the world model before starting movement.  Prevents robots
     * that finished the ID window early from driving into a partial formation
     * while later robots are still exiting their own windows.
     *
     * Caps at DEMO_SYNC_GRACE_MS so a missing robot doesn't stall the demo.
     */
#define DEMO_SYNC_GRACE_MS 4000
    for (uint32_t waited = 0; waited < DEMO_SYNC_GRACE_MS; waited += WM_CYCLE_MS) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);
        demo_set_leds(&wm);

        int fresh = 0;
        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm.entries[i];
            if (e->is_active && !e->is_self && !e->is_stale) {
                fresh++;
            }
        }
        if (fresh >= n_total - 1) {
            break;
        }

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }
        k_msleep(WM_CYCLE_MS);
    }

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
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
}
