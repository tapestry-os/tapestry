/*
 * tapestry-os/subsys/transport/transceiver_udp.c
 * Tapestry L1 UDP transceiver
 *
 * Implements tapestry_transceiver_t for UDP broadcast gossip.
 *
 * tx: wraps the raw tapestry_gossip_frame_t payload in a tapestry_msg_header_t
 *     and broadcasts it to 255.255.255.255:TAPESTRY_GOSSIP_PORT.
 * rx: reads datagrams until it can return one gossip payload (header
 *     stripped) — directive datagrams found along the way are stashed for
 *     rx_directive, other types skipped.  Returns 0 when none are pending.
 * tx_directive/rx_directive: same pattern for tapestry_directive_frame_t
 *     (TAPESTRY_MSG_DIRECTIVE, wire.h v5), sharing the gossip socket.
 *
 * Telemetry (L4/L5 metrics) is sent unicast to CONFIG_TAPESTRY_ORCH_IP via
 * a separate socket opened during init.  These are exposed as UDP-specific
 * extensions rather than through the generic transceiver vtable because no
 * other medium has an equivalent concept.
 */

#include "transceiver_udp.h"

#ifdef CONFIG_NETWORKING

#include <errno.h>
#include <string.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <tapestry/wire.h>
#include <tapestry/csm.h>

LOG_MODULE_REGISTER(transceiver_udp, LOG_LEVEL_INF);

/* Private state — sockets opened once by udp_init(). */
static int gossip_sock = -1;
static int metric_sock = -1;

/* Reused buffers — single-threaded main loop, no locking needed.
 * Gossip buffers are sized for the full wire frame (frame + optional auth);
 * tx_buf is shared with directive tx, which is strictly smaller (wire.h's
 * TAPESTRY_MAX_BODY_SIZE comment). */
static uint8_t tx_buf[TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_GOSSIP_WIRE_SIZE];
static uint8_t rx_buf[TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_GOSSIP_WIRE_SIZE];
static uint8_t metric_tx_buf[TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_METRIC_FRAME_SIZE];

/* Directive datagrams (TAPESTRY_MSG_DIRECTIVE, v5) arrive on the gossip
 * socket interleaved with gossip datagrams.  udp_rx() must not end
 * gossip_drain()'s until-0 loop just because the next datagram happens to
 * be a directive, so it stashes directives here for udp_rx_directive() and
 * keeps reading.  Ring buffer; overflow drops the OLDEST entry — the
 * consumer (gossip.c) applies newest-wins anyway, so the newest frame is
 * the one worth keeping. */
#define UDP_DIR_QUEUE_DEPTH 4
static uint8_t  dir_q[UDP_DIR_QUEUE_DEPTH][TAPESTRY_DIRECTIVE_WIRE_SIZE];
static uint16_t dir_q_len[UDP_DIR_QUEUE_DEPTH];
static uint8_t  dir_q_head;
static uint8_t  dir_q_count;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void fill_addr(struct sockaddr_in *a, const char *ip, uint16_t port)
{
    memset(a, 0, sizeof(*a));
    a->sin_family = AF_INET;
    a->sin_port   = htons(port);
    zsock_inet_pton(AF_INET, ip, &a->sin_addr);
}

/* ── Vtable ops ──────────────────────────────────────────────────────────── */

static int udp_init(void)
{
    gossip_sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (gossip_sock < 0) {
        LOG_ERR("gossip socket: %d", errno);
        return -1;
    }

    int enable = 1;
    zsock_setsockopt(gossip_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));
    zsock_setsockopt(gossip_sock, SOL_SOCKET, SO_REUSEADDR,  &enable, sizeof(enable));

    struct sockaddr_in bind_addr;
    fill_addr(&bind_addr, "0.0.0.0", CONFIG_TAPESTRY_GOSSIP_PORT);
    if (zsock_bind(gossip_sock, (struct sockaddr *)&bind_addr,
                   sizeof(bind_addr)) < 0) {
        LOG_ERR("gossip bind on port %d: %d", CONFIG_TAPESTRY_GOSSIP_PORT, errno);
        zsock_close(gossip_sock);
        gossip_sock = -1;
        return -1;
    }

    metric_sock = -1;
    if (strlen(CONFIG_TAPESTRY_ORCH_IP) > 0) {
        metric_sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (metric_sock < 0) {
            LOG_WRN("metric socket failed (%d) — telemetry disabled", errno);
            metric_sock = -1;
        }
    }

    LOG_INF("UDP transceiver ready — gossip broadcast :%d, metrics → %s:%d",
            CONFIG_TAPESTRY_GOSSIP_PORT,
            strlen(CONFIG_TAPESTRY_ORCH_IP) > 0
                ? CONFIG_TAPESTRY_ORCH_IP : "(disabled)",
            CONFIG_TAPESTRY_METRIC_PORT);
    return 0;
}

static int udp_tx(const uint8_t *data, uint16_t len)
{
    if (len > TAPESTRY_GOSSIP_WIRE_SIZE || gossip_sock < 0) {
        return -EINVAL;
    }

    /* src_id is always the first byte of the gossip frame regardless of auth */
    const tapestry_gossip_frame_t *frame = (const tapestry_gossip_frame_t *)data;

    tapestry_msg_header_t *hdr = (tapestry_msg_header_t *)tx_buf;
    hdr->version     = TAPESTRY_WIRE_VERSION;
    hdr->type        = TAPESTRY_MSG_GOSSIP;
    hdr->src_id      = frame->id;
    hdr->payload_len = len;
    memcpy(tx_buf + TAPESTRY_MSG_HEADER_SIZE, data, len);

    struct sockaddr_in dst;
    fill_addr(&dst, CONFIG_TAPESTRY_GOSSIP_BCAST, CONFIG_TAPESTRY_GOSSIP_PORT);
    zsock_sendto(gossip_sock, tx_buf,
                 (size_t)(TAPESTRY_MSG_HEADER_SIZE + len), 0,
                 (struct sockaddr *)&dst, sizeof(dst));
    return 0;
}

/* Stash one directive payload for udp_rx_directive(). */
static void dir_q_push(const uint8_t *payload, uint16_t len)
{
    if (len < TAPESTRY_DIRECTIVE_FRAME_SIZE ||
        len > TAPESTRY_DIRECTIVE_WIRE_SIZE) {
        return;
    }
    if (dir_q_count == UDP_DIR_QUEUE_DEPTH) {
        dir_q_head = (uint8_t)((dir_q_head + 1u) % UDP_DIR_QUEUE_DEPTH);
        dir_q_count--;   /* overflow: drop the oldest (see the queue comment) */
    }
    uint8_t slot = (uint8_t)((dir_q_head + dir_q_count) % UDP_DIR_QUEUE_DEPTH);
    memcpy(dir_q[slot], payload, len);
    dir_q_len[slot] = len;
    dir_q_count++;
}

static int udp_rx(uint8_t *buf, uint16_t max_len)
{
    if (max_len < TAPESTRY_GOSSIP_WIRE_SIZE || gossip_sock < 0) {
        return -EINVAL;
    }

    /* Loop rather than one-datagram-per-call: a directive (or any other
     * non-gossip datagram) interleaved in the socket queue must not end
     * gossip_drain()'s until-0 loop while gossip datagrams sit behind it. */
    for (;;) {
        ssize_t len = zsock_recv(gossip_sock, rx_buf, sizeof(rx_buf),
                                 ZSOCK_MSG_DONTWAIT);
        if (len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                LOG_WRN("gossip recv error: %d", errno);
            }
            return 0;
        }

        if (len < (ssize_t)TAPESTRY_MSG_HEADER_SIZE) {
            continue;
        }

        const tapestry_msg_header_t *hdr = (const tapestry_msg_header_t *)rx_buf;
        if (hdr->version != TAPESTRY_WIRE_VERSION) {
            /* A stale/mismatched peer — reject rather than risk
             * misinterpreting bytes laid out under a different schema.
             * Rate-limited: a peer running the wrong version stays wrong
             * every cycle, not just once. */
            static uint32_t mismatch_count;
            if ((++mismatch_count % 20u) == 1u) {
                LOG_WRN("gossip version mismatch: peer=%u wire=%u (%u frames)",
                        hdr->src_id, hdr->version, mismatch_count);
            }
            continue;
        }

        ssize_t payload_len = len - (ssize_t)TAPESTRY_MSG_HEADER_SIZE;

        if (hdr->type == TAPESTRY_MSG_DIRECTIVE) {
            dir_q_push(rx_buf + TAPESTRY_MSG_HEADER_SIZE,
                       (uint16_t)payload_len);
            continue;
        }
        if (hdr->type != TAPESTRY_MSG_GOSSIP ||
            payload_len < (ssize_t)TAPESTRY_GOSSIP_FRAME_SIZE) {
            continue;
        }

        /* Return however many payload bytes were actually received so
         * gossip_drain can distinguish authenticated (GOSSIP_WIRE_SIZE)
         * from plain frames. */
        memcpy(buf, rx_buf + TAPESTRY_MSG_HEADER_SIZE, (size_t)payload_len);
        return (int)payload_len;
    }
}

static int udp_rx_directive(uint8_t *buf, uint16_t max_len)
{
    if (max_len < TAPESTRY_DIRECTIVE_WIRE_SIZE || gossip_sock < 0) {
        return -EINVAL;
    }
    if (dir_q_count == 0) {
        return 0;
    }
    uint16_t len = dir_q_len[dir_q_head];
    memcpy(buf, dir_q[dir_q_head], len);
    dir_q_head = (uint8_t)((dir_q_head + 1u) % UDP_DIR_QUEUE_DEPTH);
    dir_q_count--;
    return (int)len;
}

static int udp_tx_directive(const uint8_t *data, uint16_t len)
{
    if (len > TAPESTRY_DIRECTIVE_WIRE_SIZE || gossip_sock < 0) {
        return -EINVAL;
    }

    const tapestry_directive_frame_t *frame =
        (const tapestry_directive_frame_t *)data;

    tapestry_msg_header_t *hdr = (tapestry_msg_header_t *)tx_buf;
    hdr->version     = TAPESTRY_WIRE_VERSION;
    hdr->type        = TAPESTRY_MSG_DIRECTIVE;
    hdr->src_id      = frame->src_id;
    hdr->payload_len = len;
    memcpy(tx_buf + TAPESTRY_MSG_HEADER_SIZE, data, len);

    struct sockaddr_in dst;
    fill_addr(&dst, CONFIG_TAPESTRY_GOSSIP_BCAST, CONFIG_TAPESTRY_GOSSIP_PORT);
    zsock_sendto(gossip_sock, tx_buf,
                 (size_t)(TAPESTRY_MSG_HEADER_SIZE + len), 0,
                 (struct sockaddr *)&dst, sizeof(dst));
    return 0;
}

static void udp_set_power(float level)
{
    ARG_UNUSED(level);
}

const tapestry_transceiver_t transceiver_udp = {
    .type         = TRANSCEIVER_TYPE_UDP,
    .init         = udp_init,
    .tx           = udp_tx,
    .rx           = udp_rx,
    .set_power    = udp_set_power,
    .tx_directive = udp_tx_directive,
    .rx_directive = udp_rx_directive,
};

/* ── UDP-specific extensions (telemetry) ────────────────────────────────── */

void udp_transceiver_send_metric(const world_model_t *wm,
                                 element_id_t element_id)
{
    if (metric_sock < 0) {
        return;
    }

    const wm_consistency_metric_t *m = wm_get_metric(wm);

    float   age_sum  = 0.0f;
    uint8_t age_cnt  = 0;
    float   min_sep  = 200.0f;
    bool    has_peer = false;

    const wm_entry_t *self_entry = wm_get_entry(wm, element_id);

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->state.id == ELEMENT_ID_INVALID || e->is_self || !e->is_active) {
            continue;
        }
        age_sum += (float)e->age_ms;
        age_cnt++;
        has_peer = true;

        if (self_entry) {
            float d = position_distance(&self_entry->state.position,
                                        &e->state.position);
            if (d < min_sep) {
                min_sep = d;
            }
        }
    }

    float    mean_age    = age_cnt > 0 ? age_sum / (float)age_cnt : 0.0f;
    uint16_t min_sep_enc = has_peer ? (uint16_t)(min_sep * 100.0f) : 0xFFFFu;

    tapestry_msg_header_t *hdr = (tapestry_msg_header_t *)metric_tx_buf;
    hdr->version     = TAPESTRY_WIRE_VERSION;
    hdr->type        = TAPESTRY_MSG_METRIC;
    hdr->src_id      = element_id;
    hdr->payload_len = TAPESTRY_METRIC_FRAME_SIZE;

    tapestry_metric_frame_t *p =
        (tapestry_metric_frame_t *)(metric_tx_buf + TAPESTRY_MSG_HEADER_SIZE);
    p->element_id          = element_id;
    p->active_total        = m->active_total;
    p->active_fresh        = m->active_fresh;
    p->active_stale        = m->active_stale;
    p->inactive_total      = m->inactive_total;
    p->collision_count     = m->collision_count;
    p->fresh_ratio         = m->fresh_ratio;
    p->quorum_held         = m->quorum_held ? 1u : 0u;
    p->degraded            = m->degraded    ? 1u : 0u;
    p->confidence          = m->confidence;
    p->cycle_count         = wm->cycle_count;
    p->mean_age_ms         = mean_age;
    p->mean_position_error = 0.0f;
    p->min_separation_x100 = min_sep_enc;

    struct sockaddr_in dst;
    fill_addr(&dst, CONFIG_TAPESTRY_ORCH_IP, CONFIG_TAPESTRY_METRIC_PORT);
    zsock_sendto(metric_sock, metric_tx_buf,
                 TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_METRIC_FRAME_SIZE, 0,
                 (struct sockaddr *)&dst, sizeof(dst));
}

void udp_transceiver_send_scr_metric(const scr_state_t *scr,
                                     uint32_t election_count)
{
    if (metric_sock < 0) {
        return;
    }

    static uint8_t scr_tx_buf[TAPESTRY_MSG_HEADER_SIZE +
                               TAPESTRY_SCR_METRIC_FRAME_SIZE];

    tapestry_msg_header_t *hdr = (tapestry_msg_header_t *)scr_tx_buf;
    hdr->version     = TAPESTRY_WIRE_VERSION;
    hdr->type        = (uint8_t)TAPESTRY_MSG_SCR_METRIC;
    hdr->src_id      = scr->own_id;
    hdr->payload_len = (uint16_t)TAPESTRY_SCR_METRIC_FRAME_SIZE;

    tapestry_scr_metric_frame_t *p =
        (tapestry_scr_metric_frame_t *)(scr_tx_buf + TAPESTRY_MSG_HEADER_SIZE);
    p->element_id     = scr->own_id;
    p->role           = (uint8_t)scr->role;
    p->leader_id      = scr->leader_valid ? scr->leader_id
                                          : ELEMENT_ID_INVALID;
    p->quorum_state   = (uint8_t)scr->quorum_state;
    p->fresh_count    = scr->fresh_count;
    p->task_slot      = scr->task_slot;
    p->election_count = election_count;

    struct sockaddr_in dst;
    fill_addr(&dst, CONFIG_TAPESTRY_ORCH_IP, CONFIG_TAPESTRY_METRIC_PORT);
    zsock_sendto(metric_sock, scr_tx_buf, sizeof(scr_tx_buf), 0,
                 (struct sockaddr *)&dst, sizeof(dst));
}

#endif /* CONFIG_NETWORKING */
