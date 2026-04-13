/*
 * comms_scr.c — SCR metric transmission (L5 extension to L4 comms)
 */

#include "comms_scr.h"

#include <string.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(element, LOG_LEVEL_INF);

/* Static TX buffer — single-threaded; no concurrency. */
static uint8_t scr_tx_buf[SIM_HEADER_SIZE + SIM_SCR_METRIC_SIZE];

void comms_send_scr_metric(const comms_t *c,
                           const scr_state_t *scr,
                           uint32_t election_count)
{
    /* ── Header ──────────────────────────────────────────────────────────── */
    sim_msg_header_t *hdr = (sim_msg_header_t *)scr_tx_buf;
    hdr->type        = (uint8_t)SIM_MSG_SCR_METRIC;
    hdr->src_id      = scr->own_id;
    hdr->payload_len = (uint16_t)SIM_SCR_METRIC_SIZE;

    /* ── Payload ─────────────────────────────────────────────────────────── */
    sim_scr_metric_payload_t *p =
        (sim_scr_metric_payload_t *)(scr_tx_buf + SIM_HEADER_SIZE);

    p->element_id     = scr->own_id;
    p->role           = (uint8_t)scr->role;
    p->leader_id      = scr->leader_valid ? scr->leader_id : ELEMENT_ID_INVALID;
    p->quorum_state   = (uint8_t)scr->quorum_state;
    p->fresh_count    = scr->fresh_count;
    p->_reserved      = 0;
    p->election_count = election_count;

    /* ── Send ────────────────────────────────────────────────────────────── */
    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port   = htons(c->orch_port),
    };
    zsock_inet_pton(AF_INET, SIM_LOOPBACK_ADDR, &dst.sin_addr);

    int sent = zsock_sendto(c->sock, scr_tx_buf,
                            sizeof(scr_tx_buf), 0,
                            (struct sockaddr *)&dst, sizeof(dst));
    if (sent < 0) {
        LOG_ERR("comms_send_scr_metric failed: %d", sent);
    }
}
