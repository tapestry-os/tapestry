/*
 * tapestry-os/subsys/transport/transport.c
 * Tapestry L3 transport multiplexer
 *
 * Implements <tapestry/transport.h> by fanning out to whichever gossip
 * backends are compiled in for this board.  All Kconfig guards live here;
 * application code is unconditionally clean.
 *
 *   CONFIG_BT              → ble_gossip.c  (BLE advertising / scan)
 *   CONFIG_NETWORKING      → udp_gossip.c + net_init.c  (UDP broadcast)
 *   Both                   → both backends; transport_drain() merges frames
 *
 * Telemetry routing:
 *   CONFIG_NETWORKING      → UDP unicast to collector (every cycle)
 *   BLE-only               → serial CSV to stdout (throttled to GOSSIP_INTERVAL_MS)
 */

#include <tapestry/transport.h>
#include <tapestry/csm.h>

#ifdef CONFIG_BT
#include "ble/ble_gossip.h"
#endif

#ifdef CONFIG_NETWORKING
#include "udp/udp_gossip.h"
#include "udp/net_init.h"
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(transport, LOG_LEVEL_INF);

/* ── Private state ───────────────────────────────────────────────────────── */

#ifdef CONFIG_NETWORKING
static udp_gossip_ctx_t udp_ctx;
#endif

#ifndef CONFIG_NETWORKING
/* Throttle serial metric output to avoid flooding the UART. */
static uint32_t serial_metric_accum_ms;
#endif

/* ── transport_init ──────────────────────────────────────────────────────── */

int transport_init(void)
{
#ifdef CONFIG_NETWORKING
    if (net_connect() != 0) {
        LOG_ERR("network bring-up failed");
        return -1;
    }
    if (udp_gossip_init(&udp_ctx) != 0) {
        LOG_ERR("udp_gossip_init failed");
        return -1;
    }
#endif

#ifdef CONFIG_BT
    if (ble_gossip_init() != 0) {
        LOG_WRN("BLE gossip init failed — BLE peers will not be heard");
    }
#endif

#ifndef CONFIG_NETWORKING
    /* Print CSV header once so collect_serial.py can identify columns. */
    printk("HEADER,uptime_ms,element_id,fresh_ratio,quorum_state,role,"
           "leader_id,election_count,mean_age_ms\n");
    serial_metric_accum_ms = GOSSIP_INTERVAL_MS;   /* emit immediately on first call */
#endif

    return 0;
}

/* ── transport_send ──────────────────────────────────────────────────────── */

void transport_send(const element_state_t *own_state)
{
#ifdef CONFIG_NETWORKING
    udp_gossip_send(&udp_ctx, own_state);
#endif
#ifdef CONFIG_BT
    ble_gossip_send(own_state);
#endif
}

/* ── transport_drain ─────────────────────────────────────────────────────── */

int transport_drain(world_model_t *wm, element_id_t own_id)
{
    int total = 0;
#ifdef CONFIG_NETWORKING
    total += udp_gossip_drain(&udp_ctx, wm, own_id);
#endif
#ifdef CONFIG_BT
    total += ble_gossip_drain(wm, own_id);
#endif
    return total;
}

/* ── Auto-ID (BLE discovery window) ─────────────────────────────────────── */

void transport_advertise_nonce(uint32_t nonce)
{
#ifdef CONFIG_BT
    ble_gossip_advertise_nonce(nonce);
#else
    ARG_UNUSED(nonce);
#endif
}

int transport_drain_nonces(uint32_t *out, int max)
{
#ifdef CONFIG_BT
    return ble_gossip_drain_nonces(out, max);
#else
    ARG_UNUSED(out);
    ARG_UNUSED(max);
    return 0;
#endif
}

int transport_drain_claimed(bool *claimed_out, int max_id)
{
#ifdef CONFIG_BT
    return ble_gossip_drain_claimed(claimed_out, max_id);
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
    udp_gossip_send_metric(&udp_ctx, wm, element_id);
    if (scr != NULL) {
        udp_gossip_send_scr_metric(&udp_ctx, scr, election_count);
    }
}

#else /* BLE-only: emit serial CSV */

#include <zephyr/kernel.h>   /* k_uptime_get_32 */
#include "../../subsys/csm/world_model.h"

static void emit_serial_metric(element_id_t element_id,
                                const world_model_t *wm,
                                const scr_state_t   *scr,
                                uint32_t             election_count)
{
    const wm_consistency_metric_t *m = wm_get_metric(wm);

    float    age_sum = 0.0f;
    uint8_t  age_cnt = 0;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->state.id == ELEMENT_ID_INVALID || e->is_self || !e->is_active) {
            continue;
        }
        age_sum += (float)e->age_ms;
        age_cnt++;
    }
    float mean_age = age_cnt > 0 ? age_sum / (float)age_cnt : 0.0f;

    uint8_t  quorum_state  = 0;
    uint8_t  role          = 0;
    uint8_t  leader_id     = 0xFFu;
    uint32_t elections     = election_count;

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
           (unsigned)elections,
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
