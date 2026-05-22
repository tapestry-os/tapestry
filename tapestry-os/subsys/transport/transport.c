/*
 * tapestry-os/subsys/transport/transport.c
 * Tapestry L3 transport multiplexer
 *
 * Implements <tapestry/transport.h>.  Registers the active transceiver
 * backends into the gossip framing layer and routes telemetry to the
 * appropriate channel.
 *
 * To add a new medium: create transceiver_<medium>.c, declare the vtable in
 * transceiver_<medium>.h, and register it below under the appropriate Kconfig
 * guard.  No changes to transport.h or any caller.
 *
 *   CONFIG_BT              → transceiver_ble.c  (BLE advertising / scan)
 *   CONFIG_NETWORKING      → transceiver_udp.c + net_init.c  (UDP broadcast)
 *   Both                   → both backends; gossip_drain merges frames
 * Telemetry routing:
 *   CONFIG_NETWORKING → UDP unicast to collector (every cycle)
 *   BLE-only          → serial CSV to stdout (throttled to GOSSIP_INTERVAL_MS)
 */

#include <tapestry/transport.h>
#include <tapestry/csm.h>
#include "gossip.h"

#ifdef CONFIG_BT
#include <zephyr/drivers/hwinfo.h>
#endif

#include <string.h>

#ifdef CONFIG_NETWORKING
#include "transceiver_udp.h"
#include "net_init.h"
#endif

#ifdef CONFIG_BT
#include "transceiver_ble.h"
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(transport, LOG_LEVEL_INF);

/* ── Transceiver registry ────────────────────────────────────────────────── */

#define MAX_TRANSCEIVERS 8

static const tapestry_transceiver_t *active[MAX_TRANSCEIVERS];
static int n_active;

/* ── Private state ───────────────────────────────────────────────────────── */

#ifndef CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS
#define CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS 4000
#endif

#ifndef CONFIG_NETWORKING
static uint32_t serial_metric_accum_ms;
#endif

/* ── transport_init ──────────────────────────────────────────────────────── */

int transport_init(void)
{
    n_active = 0;

#ifdef CONFIG_NETWORKING
    if (net_connect() != 0) {
        LOG_ERR("network bring-up failed");
        return -1;
    }
    active[n_active++] = &transceiver_udp;
#endif

#ifdef CONFIG_BT
    active[n_active++] = &transceiver_ble;
#endif



    for (int i = 0; i < n_active; i++) {
        int ret = active[i]->init();
        if (ret != 0) {
            if (active[i]->type == TRANSCEIVER_TYPE_UDP) {
                LOG_ERR("UDP transceiver init failed: %d", ret);
                return ret;
            }
            LOG_WRN("transceiver type=%d init failed: %d — peers on this medium "
                    "will not be heard", (int)active[i]->type, ret);
        }
    }

    gossip_register_transceivers(active, n_active);

#ifndef CONFIG_NETWORKING
    printk("HEADER,uptime_ms,element_id,fresh_ratio,quorum_state,role,"
           "leader_id,election_count,mean_age_ms\n");
    serial_metric_accum_ms = GOSSIP_INTERVAL_MS;
#endif

    return 0;
}

/* ── transport_send ──────────────────────────────────────────────────────── */

void transport_send(const element_state_t *own_state, uint8_t qos_tier)
{
    gossip_send(own_state, qos_tier);
}

/* ── transport_drain ─────────────────────────────────────────────────────── */

int transport_drain(world_model_t *wm, element_id_t own_id)
{
    int n = gossip_drain(wm, own_id);
    gossip_relay_flush();
    return n;
}

/* ── transport_negotiate_id ──────────────────────────────────────────────── */

static uint32_t hw_nonce(void)
{
#ifdef CONFIG_BT
    uint8_t buf[4] = {0};
    if (hwinfo_get_device_id(buf, sizeof(buf)) >= (ssize_t)sizeof(buf)) {
        uint32_t n = 0;
        memcpy(&n, buf, 4);
        return n ? n : 1u;   /* 0 would sort first — shift to 1 */
    }
#endif
    /* hwinfo unavailable — salt with uptime to avoid all-zero ties */
    return (uint32_t)k_uptime_get() ^ 0xA5A5A5A5u;
}

element_id_t transport_negotiate_id(int *n_total_out)
{
    uint32_t own_nonce = hw_nonce();
    uint32_t peer_nonces[MAX_ELEMENTS];
    bool     claimed[MAX_ELEMENTS];
    int      n_peers = 0;

    memset(peer_nonces, 0, sizeof(peer_nonces));
    memset(claimed,     0, sizeof(claimed));

    transport_advertise_nonce(own_nonce);
    LOG_INF("auto_id: nonce=0x%08x  window=%u ms",
            own_nonce, CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS);

    for (uint32_t elapsed = 0;
         elapsed < CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS;
         elapsed += WM_CYCLE_MS) {

        uint32_t batch[8];
        int got = transport_drain_nonces(batch, ARRAY_SIZE(batch));
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

        transport_drain_claimed(claimed, MAX_ELEMENTS);
        k_msleep(WM_CYCLE_MS);
    }

    int n_running = 0;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        if (claimed[i]) { n_running++; }
    }

    int own_rank = 0;
    for (int i = 0; i < n_peers; i++) {
        if (peer_nonces[i] < own_nonce) { own_rank++; }
    }

    element_id_t id   = (element_id_t)own_rank;
    int          seen = 0;
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

/* ── Auto-ID (BLE discovery window) ─────────────────────────────────────── */

void transport_advertise_nonce(uint32_t nonce)
{
#ifdef CONFIG_BT
    ble_transceiver_advertise_nonce(nonce);
#else
    ARG_UNUSED(nonce);
#endif
}

int transport_drain_nonces(uint32_t *out, int max)
{
#ifdef CONFIG_BT
    return ble_transceiver_drain_nonces(out, max);
#else
    ARG_UNUSED(out);
    ARG_UNUSED(max);
    return 0;
#endif
}

int transport_drain_claimed(bool *claimed_out, int max_id)
{
#ifdef CONFIG_BT
    return ble_transceiver_drain_claimed(claimed_out, max_id);
#else
    ARG_UNUSED(claimed_out);
    ARG_UNUSED(max_id);
    return 0;
#endif
}

/* ── transport_send_telemetry ────────────────────────────────────────────── */

#ifdef CONFIG_NETWORKING

void transport_send_telemetry(const world_model_t *wm, element_id_t element_id,
                              const scr_state_t *scr, uint32_t election_count)
{
    udp_transceiver_send_metric(wm, element_id);
    if (scr != NULL) {
        udp_transceiver_send_scr_metric(scr, election_count);
    }
}

#else /* BLE-only: emit serial CSV */

static void emit_serial_metric(element_id_t element_id,
                                const world_model_t *wm,
                                const scr_state_t   *scr,
                                uint32_t             election_count)
{
    const wm_consistency_metric_t *m = wm_get_metric(wm);

    float   age_sum = 0.0f;
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

    uint8_t  quorum_state = 0;
    uint8_t  role         = 0;
    uint8_t  leader_id    = 0xFFu;

    if (scr != NULL) {
        quorum_state = (uint8_t)scr->quorum_state;
        role         = (uint8_t)scr->role;
        leader_id    = scr->leader_valid ? scr->leader_id : 0xFFu;
    }

    printk("METRIC,%u,%u,%.4f,%u,%u,%u,%u,%.1f\n",
           (unsigned)k_uptime_get_32(),
           (unsigned)element_id,
           (double)m->fresh_ratio,
           (unsigned)quorum_state,
           (unsigned)role,
           (unsigned)leader_id,
           (unsigned)election_count,
           (double)mean_age);
}

void transport_send_telemetry(const world_model_t *wm, element_id_t element_id,
                              const scr_state_t *scr, uint32_t election_count)
{
    serial_metric_accum_ms += WM_CYCLE_MS;
    if (serial_metric_accum_ms < GOSSIP_INTERVAL_MS) {
        return;
    }
    serial_metric_accum_ms = 0;
    emit_serial_metric(element_id, wm, scr, election_count);
}

#endif /* CONFIG_NETWORKING */
