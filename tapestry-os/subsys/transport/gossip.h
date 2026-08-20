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
 * transceivers.  qos_tier is TAPESTRY_QOS_* from wire.h, packed into the
 * frame's relay_qos byte alongside hop_count — see wire.h. Read by the
 * relay ring buffer (gossip_drain/relay_enqueue) to decide which queued
 * frame to evict under pressure. */
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

/* ── Auto-ID discovery (boot window only) ────────────────────────────────── */
/* Medium-agnostic primitives behind transport_negotiate_id().  A discovery
 * beacon is an ordinary gossip frame with id = ELEMENT_ID_INVALID and the
 * hardware nonce in update_seq, so it travels over ANY transceiver.  On
 * one-shot media (syslink P2P, UDP broadcast) the beacon must be re-sent
 * periodically during the window; BLE additionally keeps it in the
 * advertising payload between sends. */

/* Transmit one discovery beacon carrying nonce via all transceivers. */
void gossip_send_discovery(uint32_t nonce);

/* Number of received frames that carried our OWN element id.  On non-BLE
 * media a node cannot hear its own transmission, so a nonzero count means
 * another element shares our identity (auto-ID collision) — its gossip is
 * being silently dropped by the self-echo filter.  Monotonic since boot. */
uint32_t gossip_own_id_frames(void);

/* Drain all transceiver rx queues without a world model, splitting frames:
 * discovery beacons (id == ELEMENT_ID_INVALID) append their nonce to
 * nonces_out[0..max_nonces-1]; normal gossip frames mark claimed_out[id]
 * true for id < max_id (already-running elements).  claimed_out must be a
 * caller-zeroed bool array.  Returns the number of nonces written.
 * Note: frames are consumed — do not interleave with gossip_drain() during
 * the discovery window. */
int gossip_drain_discovery(uint32_t *nonces_out, int max_nonces,
                           bool *claimed_out, int max_id);

#endif /* TAPESTRY_GOSSIP_H */
