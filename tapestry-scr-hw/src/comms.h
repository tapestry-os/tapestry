/*
 * comms.h — UDP transport for Tapestry hardware elements
 *
 * Gossip uses UDP broadcast (255.255.255.255:TAPESTRY_GOSSIP_PORT) so
 * every element on the subnet receives every other element's state
 * without needing to know peer IP addresses — the same semantics as a
 * real RF mesh broadcast.
 *
 * Telemetry metrics are sent unicast to CONFIG_TAPESTRY_ORCH_IP on
 * TAPESTRY_METRIC_PORT so the laptop collector can log and plot them.
 * Metrics are silently dropped if TAPESTRY_ORCH_IP is empty.
 *
 * Wire format is identical to sim_protocol.h + scr_protocol.h so the
 * existing Python telemetry/collect.py parses packets without changes.
 */

#ifndef TAPESTRY_HW_COMMS_H
#define TAPESTRY_HW_COMMS_H

#include <stdint.h>
#include <tapestry/scr.h>
#include "sim_protocol.h"
#include "scr_protocol.h"

typedef struct {
    int gossip_sock;   /* bound to 0.0.0.0:GOSSIP_PORT; SO_BROADCAST set  */
    int metric_sock;   /* unbound; unicast to telemetry collector          */
} hw_comms_t;

/*
 * comms_init — open and configure both sockets.
 * Returns 0 on success, -1 on failure (error is logged).
 */
int comms_init(hw_comms_t *c);

/*
 * comms_send_gossip — broadcast own_state to all elements on the subnet.
 */
void comms_send_gossip(const hw_comms_t *c, const element_state_t *own_state);

/*
 * comms_drain_inbox — non-blocking receive loop.
 * Processes all pending gossip datagrams, ignoring packets whose src_id
 * matches own_id (own broadcast echoed back by the network stack).
 * Returns the number of foreign gossip messages processed.
 */
int comms_drain_inbox(const hw_comms_t *c, world_model_t *wm,
                      element_id_t own_id);

/*
 * comms_send_metric — send L4 CSM metric to the telemetry collector.
 */
void comms_send_metric(const hw_comms_t *c, const world_model_t *wm,
                       element_id_t element_id);

/*
 * comms_send_scr_metric — send L5 SCR metric to the telemetry collector.
 */
void comms_send_scr_metric(const hw_comms_t *c, const scr_state_t *scr,
                           uint32_t election_count);

#endif /* TAPESTRY_HW_COMMS_H */
