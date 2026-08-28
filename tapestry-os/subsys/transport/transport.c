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

#ifdef CONFIG_HWINFO
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

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK
#include "transceiver_syslink_p2p.h"
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

/* Outcome of the most recent auto-ID window, retained so it can be
 * re-logged long after the window (console-less boots lose the live log). */
static struct {
    uint32_t beacons_tx;
    uint32_t nonces_heard;
    uint32_t ids_running;
} s_neg;

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

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK
    active[n_active++] = &transceiver_syslink_p2p;
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

bool transport_poll_directive(element_id_t own_id,
                              tapestry_directive_frame_t *out)
{
    return gossip_poll_directive(out, own_id);
}

/* ── transport_negotiate_id ──────────────────────────────────────────────── */

static uint32_t hw_nonce(void)
{
#ifdef CONFIG_HWINFO
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
    /* Discovery beacons are re-sent far faster than normal gossip: on
     * one-shot media each send is a single unacknowledged RF packet, and
     * measured syslink P2P delivery under load is ~35% (2026-07-19 runs) —
     * at 2 Hz a 6 s window carries only 12 tries and can plausibly miss
     * entirely; at ~7 Hz it carries ~40 (miss probability ~0.65^40 ≈ 0).
     * BLE ignores the extra sends (they just refresh the adv payload the
     * controller is already re-broadcasting ~every 100 ms).  Boot-window
     * only, so the airtime cost is irrelevant. */
#define AUTO_ID_BEACON_MS 150u

    uint32_t own_nonce = hw_nonce();
    uint32_t peer_nonces[MAX_ELEMENTS];
    bool     claimed[MAX_ELEMENTS];
    int      n_peers = 0;
    uint32_t beacon_accum_ms = AUTO_ID_BEACON_MS;    /* send on first cycle */

    memset(peer_nonces, 0, sizeof(peer_nonces));
    memset(claimed,     0, sizeof(claimed));

    LOG_INF("auto_id: nonce=0x%08x  window=%u ms",
            own_nonce, CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS);

    uint32_t beacons_sent = 0;
    uint32_t diag_accum_ms = 0;

    for (uint32_t elapsed = 0;
         elapsed < CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS;
         elapsed += WM_CYCLE_MS) {

        /* Re-send the beacon each AUTO_ID_BEACON_MS (see the rationale at
         * the top of this function): one-shot media transmit a single
         * packet per send, so redundancy must come from repetition here. */
        beacon_accum_ms += WM_CYCLE_MS;
        if (beacon_accum_ms >= AUTO_ID_BEACON_MS) {
            gossip_send_discovery(own_nonce);
            beacons_sent++;
            beacon_accum_ms = 0;
        }

        /* Once-per-second progress line: a healthy two-element negotiation
         * shows nonces=1 within a second or two of window overlap.  Watching
         * this stay at 0 on both consoles while beacons_tx climbs is the
         * radio-blackout signature (see also the syslink p2p counter log). */
        diag_accum_ms += WM_CYCLE_MS;
        if (diag_accum_ms >= 1000u) {
            diag_accum_ms = 0;
            int n_running = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                if (claimed[i]) { n_running++; }
            }
            LOG_INF("auto_id: t=%u ms  beacons_tx=%u  nonces_heard=%d  "
                    "ids_running=%d",
                    elapsed, beacons_sent, n_peers, n_running);
        }

        uint32_t batch[8];
        int got = gossip_drain_discovery(batch, ARRAY_SIZE(batch),
                                         claimed, MAX_ELEMENTS);
#ifdef CONFIG_BT
        /* The BLE scan callback pre-sorts frames into its own queues; the
         * generic drain above only sees what reaches the BLE rx queue
         * (claimed frames), so merge the BLE-side nonce queue too. */
        got += ble_transceiver_drain_nonces(batch + got,
                                            (int)ARRAY_SIZE(batch) - got);
        transport_drain_claimed(claimed, MAX_ELEMENTS);
#endif
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

    /* Retain the window's outcome: a console attached AFTER a console-less
     * boot can still learn what the (long gone) discovery window saw —
     * the application re-logs this via transport_get_negotiation_stats(). */
    s_neg.beacons_tx   = beacons_sent;
    s_neg.nonces_heard = (uint32_t)n_peers;
    s_neg.ids_running  = (uint32_t)n_running;
    return id;
}

void transport_get_negotiation_stats(uint32_t *beacons_tx,
                                     uint32_t *nonces_heard,
                                     uint32_t *ids_running)
{
    *beacons_tx   = s_neg.beacons_tx;
    *nonces_heard = s_neg.nonces_heard;
    *ids_running  = s_neg.ids_running;
}

/* ── Auto-ID lower-level primitives ─────────────────────────────────────── */

void transport_advertise_nonce(uint32_t nonce)
{
    /* Medium-agnostic: the discovery beacon is an ordinary gossip frame
     * (id=ELEMENT_ID_INVALID, nonce in update_seq), so tx via every
     * registered transceiver.  On BLE this lands in the advertising
     * payload, replicating the old ble_transceiver_advertise_nonce path. */
    gossip_send_discovery(nonce);
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
