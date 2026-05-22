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
 * transceivers.  qos_tier is TAPESTRY_QOS_* from wire.h; reserved for
 * future transport-layer prioritization (not carried in the wire frame). */
void gossip_send(const element_state_t *own_state, uint8_t qos_tier);

/* Drain all pending received frames from all registered transceivers into wm.
 * Skips frames whose id matches own_id.
 * When CONFIG_TAPESTRY_MESH_RELAY is enabled, eligible received frames are
 * queued for relay re-transmission; call gossip_relay_flush() afterwards.
 * Returns total frames processed. */
int gossip_drain(world_model_t *wm, element_id_t own_id);

/* Transmit any relay frames queued by gossip_drain.
 * When CONFIG_TAPESTRY_MESH_RELAY is disabled this is a no-op.
 * Introduces a randomised 0-50 ms jitter (set at first-enqueue time) before
 * re-advertising to reduce collision probability on dense networks.
 * Call once per cycle, immediately after gossip_drain completes. */
void gossip_relay_flush(void);

#endif /* TAPESTRY_GOSSIP_H */
