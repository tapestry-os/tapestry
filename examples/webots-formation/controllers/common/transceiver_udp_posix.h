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
 */

#ifndef TAPESTRY_TRANSCEIVER_UDP_POSIX_H
#define TAPESTRY_TRANSCEIVER_UDP_POSIX_H

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

#endif /* TAPESTRY_TRANSCEIVER_UDP_POSIX_H */
