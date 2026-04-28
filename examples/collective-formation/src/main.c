/*
 * main.c — Tapestry Demo: Collective Formation (L4 only)
 *
 * Any number of Cutebot Mini robots running L4 CSM.  Each robot:
 *   1. Negotiates a unique element ID over BLE during a 4-second boot window.
 *   2. Advertises its own dead-reckoning position via BLE gossip.
 *   3. Receives peer positions into its local L4 world model.
 *   4. Computes a spring-field drive command (repulsion/attraction).
 *   5. Drives toward the formation equilibrium.
 *   6. Sets LEDs to reflect how many fresh peers are currently visible.
 *
 * No L5 SCR — formation is a pure L4 behaviour.
 *
 * Auto-ID protocol (DISCOVER_MS boot window):
 *   Each robot advertises its FICR hardware nonce in a discovery beacon
 *   (gossip packet with id=ELEMENT_ID_INVALID).  After the window:
 *     - Nonce rank among co-booting peers → candidate rank
 *     - Rank-th unclaimed ID (not seen in live gossip) → element_id
 *   A rebooting robot sees running peers' gossip IDs, avoids conflicts,
 *   and reclaims its original slot.
 *
 * One binary for all robots — no per-robot build flags needed.
 *
 * See formation.h for physical calibration constants.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>
#include <math.h>
#include <string.h>
#include <tapestry/csm.h>

#include "ble_gossip.h"
#include "cutebot.h"
#include "formation.h"

LOG_MODULE_REGISTER(demo, LOG_LEVEL_INF);

#define M_PI_F       3.14159265f
#define DISCOVER_MS  4000u   /* boot window for nonce exchange */

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static float start_clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Read a stable per-board hardware nonce from FICR via Zephyr hwinfo. */
static uint32_t hw_nonce(void)
{
    uint8_t buf[4] = {0};
    if (hwinfo_get_device_id(buf, sizeof(buf)) < (ssize_t)sizeof(buf)) {
        /* hwinfo unavailable — salt with uptime to avoid all-zero ties */
        return (uint32_t)k_uptime_get() ^ 0xA5A5A5A5u;
    }
    uint32_t n = 0;
    memcpy(&n, buf, 4);
    return n ? n : 1u;   /* 0 would sort first — shift to 1 */
}

/*
 * Place robot 'id' on a regular n_total-gon whose side equals
 * DEMO_TARGET_SPACING — the spring equilibrium radius.
 * Robots start at their equilibrium positions so initial forces are minimal.
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

/*
 * Negotiate a unique element_id using BLE nonce exchange.
 *
 * During DISCOVER_MS all co-booting robots advertise id=ELEMENT_ID_INVALID
 * with their hardware nonce in update_seq.  Already-running robots continue
 * advertising normal gossip.  After the window:
 *
 *   own_rank  = count of co-booting peers whose nonce < own_nonce
 *   element_id = own_rank-th ID not already claimed by a running robot
 *
 * This guarantees unique IDs when robots boot together, and lets a rebooting
 * robot reclaim the lowest available slot without conflicting with live peers.
 */
static element_id_t auto_assign_id(uint32_t own_nonce, int *n_total_out)
{
    uint32_t peer_nonces[MAX_ELEMENTS];
    bool     claimed[MAX_ELEMENTS];
    int      n_peers = 0;

    memset(peer_nonces, 0, sizeof(peer_nonces));
    memset(claimed,     0, sizeof(claimed));

    ble_gossip_advertise_nonce(own_nonce);
    LOG_INF("auto_id: nonce=0x%08x  window=%u ms", own_nonce, DISCOVER_MS);

    for (uint32_t elapsed = 0; elapsed < DISCOVER_MS; elapsed += WM_CYCLE_MS) {
        /* Collect nonces from co-booting peers */
        uint32_t batch[8];
        int got = ble_gossip_drain_nonces(batch, ARRAY_SIZE(batch));
        for (int i = 0; i < got; i++) {
            if (batch[i] == own_nonce) {
                continue;   /* own echo — nRF RPA does not suppress self-rx */
            }
            bool dup = false;
            for (int j = 0; j < n_peers; j++) {
                if (peer_nonces[j] == batch[i]) { dup = true; break; }
            }
            if (!dup && n_peers < MAX_ELEMENTS) {
                peer_nonces[n_peers++] = batch[i];
            }
        }

        /* Learn which IDs are claimed by already-running robots */
        ble_gossip_drain_claimed(claimed, MAX_ELEMENTS);

        k_msleep(WM_CYCLE_MS);
    }

    /* Count already-running peers for n_total estimate */
    int n_running = 0;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        if (claimed[i]) { n_running++; }
    }

    /* Rank own nonce among co-booting peers (lower nonce → lower rank) */
    int own_rank = 0;
    for (int i = 0; i < n_peers; i++) {
        if (peer_nonces[i] < own_nonce) { own_rank++; }
    }

    /* Pick the own_rank-th unclaimed ID */
    element_id_t id     = (element_id_t)own_rank;   /* safe fallback */
    int          seen   = 0;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        if (!claimed[i]) {
            if (seen == own_rank) { id = (element_id_t)i; break; }
            seen++;
        }
    }

    *n_total_out = n_peers + 1 + n_running;
    LOG_INF("auto_id: rank=%d co_booting=%d running=%d -> id=%u n_total=%d",
            own_rank, n_peers, n_running, (unsigned)id, *n_total_out);
    return id;
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    if (cutebot_init() != 0) {
        LOG_WRN("Cutebot not found — movement and LEDs disabled");
    }

    if (ble_gossip_init() != 0) {
        LOG_WRN("BLE gossip init failed — no peer awareness");
    }

    /* Auto-ID: negotiate element_id over BLE, then switch to normal gossip */
    int n_total;
    const uint32_t      nonce      = hw_nonce();
    const element_id_t  element_id = auto_assign_id(nonce, &n_total);

    float sx, sy;
    compute_start_pos(element_id, n_total, &sx, &sy);

    LOG_INF("Demo — element %u  start (%.1f, %.1f)  target_spacing=%.1f",
            (unsigned)element_id, (double)sx, (double)sy,
            (double)DEMO_TARGET_SPACING);

    element_state_t own_state = {0};
    own_state.id          = element_id;
    own_state.power_state = POWER_ACTIVE;
    own_state.position.x  = sx;
    own_state.position.y  = sy;

    /* Switch advertisement from discovery beacon to real gossip immediately */
    ble_gossip_send(&own_state);

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);   /* pure AP — never freeze */

    demo_odometry_t odo;
    demo_odometry_init(&odo, sx, sy);

    int      left_cmd     = 0;
    int      right_cmd    = 0;
    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;   /* send immediately on first tick */

    LOG_INF("Demo ready — entering main loop");

    while (true) {
        ble_gossip_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        demo_odometry_update(&odo, left_cmd, right_cmd, WM_CYCLE_MS);
        own_state.position.x = odo.x;
        own_state.position.y = odo.y;
        wm_update_self(&wm, &own_state);

        demo_compute_drive(&wm, &odo, &left_cmd, &right_cmd);
        cutebot_drive(left_cmd, right_cmd);
        demo_set_leds(&wm);
        demo_display_position(&odo);

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
