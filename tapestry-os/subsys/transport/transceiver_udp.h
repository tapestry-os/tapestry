/*
 * tapestry-os/subsys/transport/transceiver_udp.h
 * UDP transceiver — private interface (transport subsystem only)
 *
 * Exposes the tapestry_transceiver_t vtable instance plus UDP-specific
 * telemetry functions.  Nothing outside the transport subsystem should
 * include this header.
 */

#ifndef TAPESTRY_TRANSCEIVER_UDP_H
#define TAPESTRY_TRANSCEIVER_UDP_H

#ifdef CONFIG_NETWORKING

#include <tapestry/transceiver.h>
#include <tapestry/scr.h>
#include <tapestry/csm.h>

/* Vtable instance registered by transport.c on networking-capable boards. */
extern const tapestry_transceiver_t transceiver_udp;

/* ── UDP-specific extensions (telemetry to the orchestrator) ─────────────── */

/* Send L4 CSM metric frame unicast to the configured collector.  No-op when
 * CONFIG_TAPESTRY_ORCH_IP is empty. */
void udp_transceiver_send_metric(const world_model_t *wm,
                                 element_id_t element_id);

/* Send L5 SCR metric frame unicast to the configured collector. */
void udp_transceiver_send_scr_metric(const scr_state_t *scr,
                                     uint32_t election_count);

#endif /* CONFIG_NETWORKING */
#endif /* TAPESTRY_TRANSCEIVER_UDP_H */
