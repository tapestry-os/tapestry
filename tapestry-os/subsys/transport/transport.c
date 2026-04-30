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
 *   CONFIG_TAPESTRY_TRANSCEIVER_ACOUSTIC → transceiver_acoustic.c (stub)
 *   CONFIG_TAPESTRY_TRANSCEIVER_OPTICAL  → transceiver_optical.c  (stub)
 *   CONFIG_TAPESTRY_TRANSCEIVER_CHEMICAL → transceiver_chemical.c (stub)
 *
 * Telemetry routing:
 *   CONFIG_NETWORKING → UDP unicast to collector (every cycle)
 *   BLE-only          → serial CSV to stdout (throttled to GOSSIP_INTERVAL_MS)
 */

#include <tapestry/transport.h>
#include <tapestry/csm.h>
#include "gossip.h"

#ifdef CONFIG_NETWORKING
#include "transceiver_udp.h"
#include "net_init.h"
#endif

#ifdef CONFIG_BT
#include "transceiver_ble.h"
#endif

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_ACOUSTIC
#include "transceiver_acoustic.h"
#endif
#ifdef CONFIG_TAPESTRY_TRANSCEIVER_OPTICAL
#include "transceiver_optical.h"
#endif
#ifdef CONFIG_TAPESTRY_TRANSCEIVER_CHEMICAL
#include "transceiver_chemical.h"
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(transport, LOG_LEVEL_INF);

/* ── Transceiver registry ────────────────────────────────────────────────── */

#define MAX_TRANSCEIVERS 8

static const tapestry_transceiver_t *active[MAX_TRANSCEIVERS];
static int n_active;

/* ── Private state ───────────────────────────────────────────────────────── */

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

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_ACOUSTIC
    active[n_active++] = &transceiver_acoustic;
#endif
#ifdef CONFIG_TAPESTRY_TRANSCEIVER_OPTICAL
    active[n_active++] = &transceiver_optical;
#endif
#ifdef CONFIG_TAPESTRY_TRANSCEIVER_CHEMICAL
    active[n_active++] = &transceiver_chemical;
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

void transport_send(const element_state_t *own_state)
{
    gossip_send(own_state);
}

/* ── transport_drain ─────────────────────────────────────────────────────── */

int transport_drain(world_model_t *wm, element_id_t own_id)
{
    return gossip_drain(wm, own_id);
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

#include "world_model.h"

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
