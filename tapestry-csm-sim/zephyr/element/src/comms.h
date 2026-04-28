/*
 * comms.h — UDP communication between element and orchestrator
 *
 * Each element owns one UDP socket:
 *   bind:  127.0.0.1 : (SIM_ELEMENT_BASE_PORT + element_id)
 *   send:  127.0.0.1 : SIM_ORCH_PORT   (gossip + metrics → orchestrator)
 *   recv:  own port                     (gossip routed back + control msgs)
 *
 * Uses Zephyr's BSD socket API with CONFIG_NET_NATIVE_OFFLOADED_SOCKETS,
 * which maps directly to host OS socket calls on native_sim — no TAP
 * interface or elevated privileges required.
 */

#ifndef TAPESTRY_COMMS_H
#define TAPESTRY_COMMS_H

#include <stdint.h>
#include <tapestry/csm.h>

typedef struct {
    int      sock;        /* UDP socket fd                          */
    uint16_t own_port;    /* SIM_ELEMENT_BASE_PORT + element_id     */
    uint16_t orch_port;   /* orchestrator receive port              */
} comms_t;

/*
 * comms_init — Create and bind the UDP socket.
 * Returns 0 on success, -1 on failure (logs the error).
 */
int comms_init(comms_t *c, element_id_t element_id, uint16_t orch_port);

/*
 * comms_send_gossip — Serialise own_state as SIM_MSG_GOSSIP and send to orch.
 */
void comms_send_gossip(const comms_t *c, const element_state_t *own_state);

/*
 * comms_send_metric — Serialise the world model's current metric as
 * SIM_MSG_METRIC and send to orch.
 */
void comms_send_metric(const comms_t *c, const world_model_t *wm,
                       element_id_t element_id);

/*
 * comms_drain_inbox — Non-blocking receive loop.
 * Processes all pending inbound datagrams:
 *   SIM_MSG_GOSSIP  → wm_receive_gossip(wm, …)
 *   SIM_MSG_CONTROL → update own_state (partition island / power state)
 * Returns the number of messages processed.
 */
int comms_drain_inbox(const comms_t *c, world_model_t *wm,
                      element_state_t *own_state);

#endif /* TAPESTRY_COMMS_H */
