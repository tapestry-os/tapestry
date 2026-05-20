/*
 * tapestry-os/subsys/transport/gossip.h
 * Gossip framing layer — private to the transport subsystem
 *
 * Sits between transport.c (public API) and the transceiver vtable (physical
 * medium).  Handles packing element_state_t into tapestry_gossip_frame_t and
 * unpacking received frames back into element_state_t for the world model.
 */

#ifndef TAPESTRY_GOSSIP_H
#define TAPESTRY_GOSSIP_H

#include <tapestry/transceiver.h>
#include <tapestry/csm.h>

/* Register the active transceiver set.  Must be called before gossip_send or
 * gossip_drain.  The array must remain valid for the lifetime of the program
 * (typically a static array in transport.c). */
void gossip_register_transceivers(const tapestry_transceiver_t * const *t,
                                  int n);

/* Pack own_state into a gossip frame and transmit via all registered
 * transceivers.  qos_tier is TAPESTRY_QOS_* from wire.h and is embedded in
 * the frame header so peers can prioritise accordingly. */
void gossip_send(const element_state_t *own_state, uint8_t qos_tier);

/* Drain all pending received frames from all registered transceivers into wm.
 * Skips frames whose id matches own_id.
 * Returns total frames processed. */
int gossip_drain(world_model_t *wm, element_id_t own_id);

#endif /* TAPESTRY_GOSSIP_H */
