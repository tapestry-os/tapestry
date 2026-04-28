/*
 * comms.c — UDP communication between element and orchestrator
 */

#include <errno.h>
#include <string.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include "comms.h"
#include "sim_protocol.h"
#include <tapestry/wire.h>

LOG_MODULE_REGISTER(comms, LOG_LEVEL_INF);

/* Scratch buffers — single-threaded main loop, no concurrency concern */
static uint8_t tx_buf[TAPESTRY_MAX_MSG_SIZE];
static uint8_t rx_buf[TAPESTRY_MAX_MSG_SIZE];

/* ── Internal helpers ────────────────────────────────────────────────────── */

static void make_addr(struct sockaddr_in *addr, uint16_t port)
{
    memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port   = htons(port);
    zsock_inet_pton(AF_INET, SIM_LOOPBACK_ADDR, &addr->sin_addr);
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int comms_init(comms_t *c, element_id_t element_id, uint16_t orch_port)
{
    c->own_port  = (uint16_t)(SIM_ELEMENT_BASE_PORT + element_id);
    c->orch_port = orch_port;

    c->sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (c->sock < 0) {
        LOG_ERR("zsock_socket() failed: %d", errno);
        return -1;
    }

    struct sockaddr_in bind_addr;
    make_addr(&bind_addr, c->own_port);

    if (zsock_bind(c->sock, (struct sockaddr *)&bind_addr,
                   sizeof(bind_addr)) < 0) {
        LOG_ERR("zsock_bind() on port %u failed: %d", c->own_port, errno);
        zsock_close(c->sock);
        return -1;
    }

    LOG_INF("comms ready — own port %u, orch port %u",
            c->own_port, c->orch_port);
    return 0;
}

void comms_send_gossip(const comms_t *c, const element_state_t *own_state)
{
    tapestry_msg_header_t *hdr = (tapestry_msg_header_t *)tx_buf;
    hdr->type        = TAPESTRY_MSG_GOSSIP;
    hdr->src_id      = own_state->id;
    hdr->payload_len = TAPESTRY_GOSSIP_FRAME_SIZE;

    tapestry_gossip_frame_t *p =
        (tapestry_gossip_frame_t *)(tx_buf + TAPESTRY_MSG_HEADER_SIZE);
    p->id               = own_state->id;
    p->x                = own_state->position.x;
    p->y                = own_state->position.y;
    p->logical_clock    = own_state->logical_clock;
    p->power_state      = (uint8_t)own_state->power_state;
    p->partition_island = own_state->partition_island;
    p->update_seq       = own_state->update_seq;

    struct sockaddr_in orch_addr;
    make_addr(&orch_addr, c->orch_port);

    zsock_sendto(c->sock, tx_buf,
                 TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_GOSSIP_FRAME_SIZE, 0,
                 (struct sockaddr *)&orch_addr, sizeof(orch_addr));
}

void comms_send_metric(const comms_t *c, const world_model_t *wm,
                       element_id_t element_id)
{
    const wm_consistency_metric_t *m = wm_get_metric(wm);

    /* ── Compute mean_age_ms and min_separation from world model ── */
    float    age_sum         = 0.0f;
    uint8_t  age_count       = 0;
    float    min_sep         = WORLD_SIZE * 2.0f;   /* sentinel > any real dist */
    bool     any_active_peer = false;

    const wm_entry_t *self_entry = wm_get_entry(wm, element_id);

    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->state.id == ELEMENT_ID_INVALID || e->is_self || !e->is_active) {
            continue;
        }
        age_sum += (float)e->age_ms;
        age_count++;
        any_active_peer = true;

        if (self_entry != NULL) {
            float d = position_distance(&self_entry->state.position,
                                        &e->state.position);
            if (d < min_sep) {
                min_sep = d;
            }
        }
    }

    float mean_age = (age_count > 0) ? (age_sum / (float)age_count) : 0.0f;
    /* Encode min_separation as uint16 at 0.01 unit resolution.
     * If no active peers exist keep sentinel value 0xFFFF so the
     * visualiser can distinguish "no peers" from "peers at distance 0". */
    uint16_t min_sep_enc = any_active_peer
                           ? (uint16_t)(min_sep * 100.0f)
                           : 0xFFFFu;

    /* ── Pack and send ── */
    tapestry_msg_header_t *hdr = (tapestry_msg_header_t *)tx_buf;
    hdr->type        = TAPESTRY_MSG_METRIC;
    hdr->src_id      = element_id;
    hdr->payload_len = TAPESTRY_METRIC_FRAME_SIZE;

    tapestry_metric_frame_t *p =
        (tapestry_metric_frame_t *)(tx_buf + TAPESTRY_MSG_HEADER_SIZE);
    p->element_id           = element_id;
    p->active_total         = m->active_total;
    p->active_fresh         = m->active_fresh;
    p->active_stale         = m->active_stale;
    p->inactive_total       = m->inactive_total;
    p->collision_count      = m->collision_count;
    p->fresh_ratio          = m->fresh_ratio;
    p->quorum_held          = m->quorum_held ? 1u : 0u;
    p->degraded             = m->degraded    ? 1u : 0u;
    p->confidence           = m->confidence;
    p->cycle_count          = wm->cycle_count;
    p->mean_age_ms          = mean_age;
    p->mean_position_error  = 0.0f;   /* filled by orchestrator */
    p->min_separation_x100  = min_sep_enc;

    struct sockaddr_in orch_addr;
    make_addr(&orch_addr, c->orch_port);

    zsock_sendto(c->sock, tx_buf,
                 TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_METRIC_FRAME_SIZE, 0,
                 (struct sockaddr *)&orch_addr, sizeof(orch_addr));
}

int comms_drain_inbox(const comms_t *c, world_model_t *wm,
                      element_state_t *own_state)
{
    int processed = 0;

    for (;;) {
        ssize_t len = zsock_recv(c->sock, rx_buf, sizeof(rx_buf),
                                  ZSOCK_MSG_DONTWAIT);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;   /* inbox empty */
            }
            LOG_WRN("recv error: %d", errno);
            break;
        }

        if (len < (ssize_t)TAPESTRY_MSG_HEADER_SIZE) {
            continue;   /* too short to be a valid message */
        }

        const tapestry_msg_header_t *hdr = (const tapestry_msg_header_t *)rx_buf;
        const uint8_t *payload           = rx_buf + TAPESTRY_MSG_HEADER_SIZE;

        switch ((sim_msg_type_t)hdr->type) {

        case SIM_MSG_GOSSIP: {
            if (len < (ssize_t)(TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_GOSSIP_FRAME_SIZE)) {
                break;
            }
            const tapestry_gossip_frame_t *g =
                (const tapestry_gossip_frame_t *)payload;

            element_state_t received = {0};
            received.id               = g->id;
            received.position.x       = g->x;
            received.position.y       = g->y;
            received.logical_clock    = g->logical_clock;
            received.power_state      = (power_state_t)g->power_state;
            received.partition_island = g->partition_island;
            received.update_seq       = g->update_seq;

            wm_receive_gossip(wm, &received);
            break;
        }

        case SIM_MSG_CONTROL: {
            if (len < (ssize_t)(TAPESTRY_MSG_HEADER_SIZE + SIM_CTRL_SIZE)) {
                break;
            }
            const sim_ctrl_payload_t *ctrl =
                (const sim_ctrl_payload_t *)payload;

            switch ((sim_ctrl_type_t)ctrl->ctrl_type) {
            case SIM_CTRL_SET_PARTITION:
                own_state->partition_island = ctrl->value;
                LOG_INF("partition island → %u", ctrl->value);
                break;
            case SIM_CTRL_SET_POWER:
                own_state->power_state = (power_state_t)ctrl->value;
                LOG_INF("power state → %u", ctrl->value);
                break;
            case SIM_CTRL_SHUTDOWN:
                LOG_INF("shutdown requested by orchestrator");
                /* main loop checks own_state->power_state == POWER_SLEEP
                 * as the exit signal; set it here as a proxy */
                own_state->power_state = POWER_SLEEP;
                break;
            }
            break;
        }

        default:
            break;   /* ignore unknown types */
        }

        processed++;
    }

    return processed;
}
