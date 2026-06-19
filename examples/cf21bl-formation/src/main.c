/*
 * main.c — CF21BL collective formation demo (L4 CSM + syslink P2P radio)
 *
 * Three Crazyflie 2.1 brushless drones flying a spring-field formation.
 * No tapestry runtime / SCR — pure L4 gossip, identical to the Cutebot demo.
 *
 * Differences from examples/collective-formation:
 *   - Transport: syslink P2P broadcast over nRF51 ESB radio (no BLE)
 *   - Substrate: CF21BL attitude + altitude hold (linear.z = 0 → hover 1 m)
 *   - Velocity feedforward: speed_cmd → linear.x → pitch tilt → horizontal
 *     motion (handled by the stabilizer's velocity feedforward mapping)
 *   - No micro:bit LED matrix
 *
 * Build (one per drone — or use transport_negotiate_id() for auto-ID):
 *   west build -p always -b crazyflie21bl tapestry/examples/cf21bl-formation \
 *     -- -DCONFIG_TAPESTRY_ELEMENT_ID=0   # 1, 2 for other drones
 * Flash:
 *   cfloader flash build/zephyr/zephyr.bin stm32-dfu
 * Console:
 *   minicom -D /dev/ttyUSB0 -b 115200   (USART3 wired, NOT CRTP radio)
 *
 * Flight checklist:
 *   1. Flash all three drones with IDs 0, 1, 2.
 *   2. Place in a ~3 m arena on the ground, separated by at least 0.5 m.
 *   3. Power on simultaneously (or within the 4 s boot window).
 *   4. Drones arm, calibrate baro (~1 s), then wait for peer gossip.
 *   5. Once all peers are fresh, altitude hold engages and drones hover.
 *   6. Spring field drives them to a 30-unit equilateral triangle at ~1.5 m
 *      separation (tune DEMO_TARGET_SPACING and arena scale as needed).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include <tapestry/csm.h>
#include <tapestry/transport.h>
#include <tapestry/substrate.h>

#include "formation.h"

LOG_MODULE_REGISTER(cf21bl_formation, LOG_LEVEL_INF);

#define M_PI_F  3.14159265f

/* ── Start positions ─────────────────────────────────────────────────────── */

static void compute_start_pos(element_id_t id, int n_total,
                               float *x, float *y, float *heading)
{
    float a = 2.0f * M_PI_F * (float)id / (float)(n_total > 1 ? n_total : 1);
    *x       = 50.0f + 3.0f * cosf(a);
    *y       = 50.0f + 3.0f * sinf(a);
    *heading = a;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    /* substrate_init() → cf21bl_init(): ESC arming (3 s) + stabilizer start
     * (includes 1 s baro baseline averaging).  During this time motors are
     * silent and the world model is not yet running. */
    if (substrate_init() != 0) {
        LOG_WRN("substrate_init failed — actuation disabled");
    }

    if (transport_init() != 0) {
        LOG_WRN("transport_init failed — no peer awareness");
    }

    /* Use the build-time element ID directly.  Auto-ID (transport_negotiate_id)
     * requires BLE for nonce exchange and is not wired to syslink P2P — both
     * drones would end up with id=0 and discard each other's gossip as "self".
     * Build each drone with -DCONFIG_TAPESTRY_ELEMENT_ID=N (0, 1, 2). */
    const element_id_t element_id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const int n_total = CONFIG_TAPESTRY_ELEMENT_COUNT;

    float sx, sy, shead;
    compute_start_pos(element_id, n_total, &sx, &sy, &shead);

    LOG_INF("CF21BL formation — element %u  start=(%.1f,%.1f)  "
            "n_total=%d  target_spacing=%.1f",
            (unsigned)element_id, (double)sx, (double)sy,
            n_total, (double)DEMO_TARGET_SPACING);

    element_state_t own_state = {0};
    own_state.id         = element_id;
    own_state.position.x = sx;
    own_state.position.y = sy;

    transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);

    demo_odometry_t odo;
    demo_odometry_init(&odo, sx, sy);
    odo.heading = shead;

    float    speed_cmd    = 0.0f;
    float    rate_cmd     = 0.0f;
    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;

    /* Convergence hold: wait until all expected peers are fresh. */
#define DEMO_SYNC_GRACE_MS 5000
    for (uint32_t waited = 0; waited < DEMO_SYNC_GRACE_MS; waited += WM_CYCLE_MS) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);
        /* demo_set_leds removed — console output is the feedback mechanism */

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

    LOG_INF("Formation ready — arming and entering main loop");

    substrate_set_power(SUBSTRATE_POWER_ACTIVE);

    while (true) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        demo_odometry_update(&odo, speed_cmd, rate_cmd, WM_CYCLE_MS);
        own_state.position.x = odo.x;
        own_state.position.y = odo.y;
        wm_update_self(&wm, &own_state);

        demo_compute_drive(&wm, &odo, &speed_cmd, &rate_cmd);

        /* Log peer count and motion command at ~1 Hz. */
        static uint32_t log_accum;
        log_accum += WM_CYCLE_MS;
        if (log_accum >= 1000) {
            log_accum = 0;
            int fresh = 0, active = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self) {
                    active++;
                    if (!e->is_stale) { fresh++; }
                }
            }
            LOG_INF("peers %d/%d  spd=%.2f rate=%.2f  pos=(%.1f,%.1f)",
                    fresh, active,
                    (double)speed_cmd, (double)rate_cmd,
                    (double)odo.x, (double)odo.y);
        }

        /* Substrate command:
         *   linear.x = speed_cmd → pitch tilt → horizontal forward motion
         *   angular.z = rate_cmd → yaw rate
         *   linear.z  = 0.0     → altitude hold at 1 m above boot altitude
         * The CF21BL stabilizer's velocity feedforward maps linear.x to a
         * pitch angle setpoint via P = angular.y − linear.x in the mix. */
        substrate_twist_t twist = {
            .linear  = { .x = speed_cmd, .z = 0.0f },
            .angular = { .z = rate_cmd  },
        };
        substrate_move(&twist);
        /* demo_set_leds removed — console output is the feedback mechanism */

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
