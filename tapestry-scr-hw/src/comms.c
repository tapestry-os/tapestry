/*
 * comms.c — UDP transport for Tapestry hardware elements
 */

#include "comms.h"

#include <errno.h>
#include <string.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(comms, LOG_LEVEL_INF);

#define BROADCAST_ADDR  CONFIG_TAPESTRY_GOSSIP_BCAST

/* Single-threaded main loop — no locking needed. */
static uint8_t tx_buf[SIM_HEADER_SIZE + SIM_METRIC_SIZE];
static uint8_t rx_buf[SIM_HEADER_SIZE + SIM_GOSSIP_SIZE];

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void fill_addr(struct sockaddr_in *a, const char *ip, uint16_t port)
{
    memset(a, 0, sizeof(*a));
    a->sin_family = AF_INET;
    a->sin_port   = htons(port);
    zsock_inet_pton(AF_INET, ip, &a->sin_addr);
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int comms_init(hw_comms_t *c)
{
    /* ── Gossip socket: bound, broadcast-enabled ── */
    c->gossip_sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (c->gossip_sock < 0) {
        LOG_ERR("gossip socket: %d", errno);
        return -1;
    }

    int enable = 1;
    zsock_setsockopt(c->gossip_sock, SOL_SOCKET, SO_BROADCAST,
                     &enable, sizeof(enable));
    zsock_setsockopt(c->gossip_sock, SOL_SOCKET, SO_REUSEADDR,
                     &enable, sizeof(enable));

    struct sockaddr_in bind_addr;
    fill_addr(&bind_addr, "0.0.0.0", CONFIG_TAPESTRY_GOSSIP_PORT);
    if (zsock_bind(c->gossip_sock, (struct sockaddr *)&bind_addr,
                   sizeof(bind_addr)) < 0) {
        LOG_ERR("gossip bind on port %d: %d",
                CONFIG_TAPESTRY_GOSSIP_PORT, errno);
        zsock_close(c->gossip_sock);
        return -1;
    }

    /* ── Metric socket: unbound, unicast only ── */
    c->metric_sock = -1;
    if (strlen(CONFIG_TAPESTRY_ORCH_IP) > 0) {
        c->metric_sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (c->metric_sock < 0) {
            LOG_WRN("metric socket failed (%d) — telemetry disabled", errno);
            c->metric_sock = -1;
        }
    }

    LOG_INF("comms ready — gossip broadcast :%d, metrics → %s:%d",
            CONFIG_TAPESTRY_GOSSIP_PORT,
            strlen(CONFIG_TAPESTRY_ORCH_IP) > 0
                ? CONFIG_TAPESTRY_ORCH_IP : "(disabled)",
            CONFIG_TAPESTRY_METRIC_PORT);
    return 0;
}

void comms_send_gossip(const hw_comms_t *c, const element_state_t *own_state)
{
    sim_msg_header_t *hdr = (sim_msg_header_t *)tx_buf;
    hdr->type        = SIM_MSG_GOSSIP;
    hdr->src_id      = own_state->id;
    hdr->payload_len = SIM_GOSSIP_SIZE;

    sim_gossip_payload_t *p =
        (sim_gossip_payload_t *)(tx_buf + SIM_HEADER_SIZE);
    p->id               = own_state->id;
    p->x                = own_state->position.x;
    p->y                = own_state->position.y;
    p->logical_clock    = own_state->logical_clock;
    p->power_state      = (uint8_t)own_state->power_state;
    p->partition_island = 0;   /* not used on hardware */
    p->update_seq       = own_state->update_seq;

    struct sockaddr_in dst;
    fill_addr(&dst, BROADCAST_ADDR, CONFIG_TAPESTRY_GOSSIP_PORT);
    zsock_sendto(c->gossip_sock, tx_buf,
                 SIM_HEADER_SIZE + SIM_GOSSIP_SIZE, 0,
                 (struct sockaddr *)&dst, sizeof(dst));
}

int comms_drain_inbox(const hw_comms_t *c, world_model_t *wm,
                      element_id_t own_id)
{
    int processed = 0;

    for (;;) {
        ssize_t len = zsock_recv(c->gossip_sock, rx_buf, sizeof(rx_buf),
                                 ZSOCK_MSG_DONTWAIT);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            LOG_WRN("gossip recv error: %d", errno);
            break;
        }

        if (len < (ssize_t)SIM_HEADER_SIZE) {
            continue;
        }

        const sim_msg_header_t *hdr = (const sim_msg_header_t *)rx_buf;

        /* Discard own broadcasts reflected back by the network stack. */
        if (hdr->src_id == own_id) {
            continue;
        }

        if (hdr->type != SIM_MSG_GOSSIP) {
            continue;
        }

        if (len < (ssize_t)(SIM_HEADER_SIZE + SIM_GOSSIP_SIZE)) {
            continue;
        }

        const sim_gossip_payload_t *g =
            (const sim_gossip_payload_t *)(rx_buf + SIM_HEADER_SIZE);

        element_state_t received = {0};
        received.id            = g->id;
        received.position.x    = g->x;
        received.position.y    = g->y;
        received.logical_clock = g->logical_clock;
        received.power_state   = (power_state_t)g->power_state;
        received.update_seq    = g->update_seq;
        /* partition_island not used on hardware — leave zero */

        wm_receive_gossip(wm, &received);
        processed++;
    }

    return processed;
}

void comms_send_metric(const hw_comms_t *c, const world_model_t *wm,
                       element_id_t element_id)
{
    if (c->metric_sock < 0) {
        return;
    }

    const wm_consistency_metric_t *m = wm_get_metric(wm);

    /* Compute mean age and min separation from world model entries. */
    float   age_sum  = 0.0f;
    uint8_t age_cnt  = 0;
    float   min_sep  = 200.0f;   /* > any realistic separation */
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

    float    mean_age   = age_cnt > 0 ? age_sum / (float)age_cnt : 0.0f;
    uint16_t min_sep_enc = has_peer ? (uint16_t)(min_sep * 100.0f) : 0xFFFFu;

    sim_msg_header_t *hdr = (sim_msg_header_t *)tx_buf;
    hdr->type        = SIM_MSG_METRIC;
    hdr->src_id      = element_id;
    hdr->payload_len = SIM_METRIC_SIZE;

    sim_metric_payload_t *p =
        (sim_metric_payload_t *)(tx_buf + SIM_HEADER_SIZE);
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
    p->mean_position_error = 0.0f;   /* no ground truth on hardware */
    p->min_separation_x100 = min_sep_enc;

    struct sockaddr_in dst;
    fill_addr(&dst, CONFIG_TAPESTRY_ORCH_IP, CONFIG_TAPESTRY_METRIC_PORT);

    zsock_sendto(c->metric_sock, tx_buf,
                 SIM_HEADER_SIZE + SIM_METRIC_SIZE, 0,
                 (struct sockaddr *)&dst, sizeof(dst));
}

void comms_send_scr_metric(const hw_comms_t *c, const scr_state_t *scr,
                           uint32_t election_count)
{
    if (c->metric_sock < 0) {
        return;
    }

    static uint8_t scr_tx_buf[SIM_HEADER_SIZE + SIM_SCR_METRIC_SIZE];

    sim_msg_header_t *hdr = (sim_msg_header_t *)scr_tx_buf;
    hdr->type        = (uint8_t)SIM_MSG_SCR_METRIC;
    hdr->src_id      = scr->own_id;
    hdr->payload_len = (uint16_t)SIM_SCR_METRIC_SIZE;

    sim_scr_metric_payload_t *p =
        (sim_scr_metric_payload_t *)(scr_tx_buf + SIM_HEADER_SIZE);
    p->element_id     = scr->own_id;
    p->role           = (uint8_t)scr->role;
    p->leader_id      = scr->leader_valid ? scr->leader_id
                                          : ELEMENT_ID_INVALID;
    p->quorum_state   = (uint8_t)scr->quorum_state;
    p->fresh_count    = scr->fresh_count;
    p->_reserved      = 0;
    p->election_count = election_count;

    struct sockaddr_in dst;
    fill_addr(&dst, CONFIG_TAPESTRY_ORCH_IP, CONFIG_TAPESTRY_METRIC_PORT);

    zsock_sendto(c->metric_sock, scr_tx_buf,
                 sizeof(scr_tx_buf), 0,
                 (struct sockaddr *)&dst, sizeof(dst));
}
