/*
 * tapestry-os/subsys/transport/udp/udp_gossip.h
 * Tapestry L3 UDP broadcast gossip transport
 *
 * Gossip uses UDP broadcast (255.255.255.255:TAPESTRY_GOSSIP_PORT) so every
 * element on the subnet receives every other element's state without needing
 * to know peer IP addresses — the same semantics as a real RF mesh broadcast.
 *
 * Telemetry metrics are sent unicast to CONFIG_TAPESTRY_ORCH_IP so a laptop
 * collector can log and plot them.  Metrics are silently dropped if
 * CONFIG_TAPESTRY_ORCH_IP is empty (length zero).
 *
 * Wire format: <tapestry/wire.h> — identical to the BLE transport and
 * compatible with the Python telemetry collector (telemetry/collect.py).
 */

#ifndef TAPESTRY_UDP_GOSSIP_H
#define TAPESTRY_UDP_GOSSIP_H

#include <stdint.h>
#include <tapestry/scr.h>
#include <tapestry/wire.h>

typedef struct {
    int gossip_sock;   /* bound to 0.0.0.0:GOSSIP_PORT; SO_BROADCAST set  */
    int metric_sock;   /* unbound; unicast to telemetry collector          */
} udp_gossip_ctx_t;

/* Open and configure both sockets.
 * Returns 0 on success, -1 on failure (error is logged). */
int udp_gossip_init(udp_gossip_ctx_t *c);

/* Broadcast own_state to all elements on the subnet. */
void udp_gossip_send(const udp_gossip_ctx_t *c, const element_state_t *own_state);

/* Non-blocking receive loop.  Processes all pending gossip datagrams,
 * ignoring packets whose src_id matches own_id (own broadcast echo).
 * Returns the number of foreign gossip messages processed. */
int udp_gossip_drain(const udp_gossip_ctx_t *c, world_model_t *wm,
                     element_id_t own_id);

/* Send L4 CSM metric to the telemetry collector. */
void udp_gossip_send_metric(const udp_gossip_ctx_t *c, const world_model_t *wm,
                            element_id_t element_id);

/* Send L5 SCR metric to the telemetry collector. */
void udp_gossip_send_scr_metric(const udp_gossip_ctx_t *c, const scr_state_t *scr,
                                uint32_t election_count);

#endif /* TAPESTRY_UDP_GOSSIP_H */
