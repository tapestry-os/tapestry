/*
 * transceiver_udp_posix.h — Tapestry L1/L3 UDP transceiver, plain POSIX
 *
 * Implements the same tapestry_transceiver_t vtable
 * (tapestry-os/include/tapestry/transceiver.h) as
 * tapestry-os/subsys/transport/transceiver_udp.c, but over plain BSD
 * sockets instead of Zephyr's zsock_* API — this Webots controller is a
 * plain host C build (Webots' own Makefile.include, no Zephyr/west), so
 * the Zephyr networking stack isn't available. This file lives in
 * controllers/common/ — shared by every substrate in this example, not
 * cf21bl-specific. See ../../README.md for why it exists as a separate,
 * additive backend rather than a change to the Zephyr one (hardware and
 * the existing native_sim harnesses keep using transceiver_udp.c unchanged).
 *
 * Wire format (tapestry_gossip_frame_t, tapestry/wire.h) is untouched, so
 * this remains protocol-compatible with every other Tapestry transceiver.
 *
 * Simplification for this sim (documented in README.md "known
 * limitations"): real UDP broadcast to 255.255.255.255 is unreliable
 * between sibling processes on macOS loopback, so this sends unicast to a
 * small fixed set of local peer ports instead of broadcasting.
 *
 * Directive frames (wire.h v5, tx_directive/rx_directive): unlike
 * transceiver_udp.c, this transport carries no tapestry_msg_header_t at
 * all — one gossip frame's bytes ARE the datagram, with peers
 * disambiguated purely by which fixed port sent them. A directive frame
 * therefore cannot share the gossip socket (nothing distinguishes a
 * 30-byte directive from a stray short gossip read); it gets its own
 * socket on DIRECTIVE_PORT_OFFSET above each element's gossip port
 * instead — the sim-scale equivalent of the Zephyr transceiver's
 * TAPESTRY_MSG_DIRECTIVE type tag.
 */

#ifndef TAPESTRY_TRANSCEIVER_UDP_POSIX_H
#define TAPESTRY_TRANSCEIVER_UDP_POSIX_H

#include <stdbool.h>
#include <stdint.h>
#include <tapestry/transceiver.h>

/* Vtable instance — register with gossip_register_transceivers() after
 * calling udp_posix_configure(). */
extern const tapestry_transceiver_t transceiver_udp_posix;

/*
 * udp_posix_configure — set this element's identity and the swarm size
 * before transceiver_udp_posix.init() runs.
 *
 * Each element's receive port is base_port + element_id; tx() fans out to
 * base_port + [0, n_elements) for every id other than our own. All
 * controllers in one Webots world must be started with the same
 * base_port and n_elements for this to form a complete mesh.
 */
void udp_posix_configure(uint8_t element_id, uint8_t n_elements, uint16_t base_port);

/*
 * udp_posix_set_rx_filter — install a predicate consulted for every received
 * gossip datagram.  Returning false drops the frame as if it had never
 * arrived; the transceiver then reads the next queued datagram rather than
 * reporting an empty socket.  NULL (the default) accepts everything, so an
 * example that never calls this is completely unaffected —
 * examples/webots-formation does not call it.
 *
 * This exists to model physical RF obstruction.  It is a RECEIVE-side hook
 * and not a transmit mask, which is a deliberate and slightly
 * counter-intuitive choice: obstruction depends on where BOTH endpoints
 * actually are, and only the receiver has both — its own live position, and
 * the sender's position carried inside the frame.  A transmitter can only
 * consult what it last HEARD about a peer, and that belief necessarily
 * freezes at the moment the peer became unreachable, i.e. while the peer was
 * still on the near side.  A transmit-side model therefore cannot produce a
 * symmetric partition: the side that stayed put keeps believing the link is
 * clear and keeps being heard, yielding a stable one-way link.  That bug is
 * why this is shaped this way.
 *
 * It is NOT a general link-quality or packet-loss knob: the predicate is
 * binary and applies to the gossip socket only.  Its only user today is
 * examples/webots-warehouse/controllers/rover/rf_occlusion.c.
 */
typedef bool (*udp_posix_rx_filter_fn)(const uint8_t *data, uint16_t len);

void udp_posix_set_rx_filter(udp_posix_rx_filter_fn fn);

/* Added to base_port for the directive socket's port range — see the
 * header comment above.  Chosen well clear of any plausible n_elements
 * (gossip ports occupy base_port .. base_port + n_elements - 1). */
#define TAPESTRY_UDP_POSIX_DIRECTIVE_PORT_OFFSET 1000u

#endif /* TAPESTRY_TRANSCEIVER_UDP_POSIX_H */
