/*
 * tapestry/transport.h — Tapestry L3 Transport API
 *
 * Stable public interface for gossip transport.  Application code includes
 * only this header; the underlying BLE, UDP, or multi-transport implementation
 * is selected by the build system and compiled into transport.c.
 *
 * Supported configurations (selected by Kconfig / board overlay):
 *   CONFIG_BT only          — BLE advertising gossip (e.g. BBC micro:bit V2)
 *   CONFIG_NETWORKING only  — UDP broadcast gossip   (e.g. EK-RA8D1)
 *   CONFIG_BT + CONFIG_NETWORKING — both transports  (e.g. ESP-WROVER-KIT)
 *
 * Auto-ID protocol:
 *   Call transport_advertise_nonce() during the discovery boot window before
 *   element_id is known.  Drain nonces with transport_drain_nonces() and
 *   claimed IDs with transport_drain_claimed().  Call transport_send() to
 *   switch back to normal gossip advertising.
 *
 * Telemetry:
 *   transport_send_telemetry() routes metric frames to whatever channel is
 *   available: UDP unicast on networking boards, serial CSV on BLE-only boards.
 *   It is a no-op when called from apps that do not use L5 SCR (pass NULL
 *   for scr and 0 for election_count).
 */

#ifndef TAPESTRY_TRANSPORT_H
#define TAPESTRY_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include <tapestry/wire.h>
#include <tapestry/scr.h>   /* includes csm.h: element_state_t, world_model_t,
                               element_id_t, scr_state_t                       */

/* ── Core gossip ─────────────────────────────────────────────────────────── */

/* Bring up all configured transports.
 * On networking boards: acquires an IP address and opens UDP sockets.
 * On BLE boards: enables the BT stack, starts passive scanning and advertising.
 * Returns 0 on success, negative errno on fatal failure. */
int transport_init(void);

/* Update the gossip advertisement / send a UDP broadcast with own_state.
 * qos_tier is one of TAPESTRY_QOS_BEST_EFFORT / SOFT_RT / HARD_RT (wire.h).
 * It is embedded in the gossip frame so peers can prioritize accordingly. */
void transport_send(const element_state_t *own_state, uint8_t qos_tier);

/* Drain all pending received gossip frames into wm.  Skips own_id frames.
 * Returns the total number of frames processed across all transports. */
int transport_drain(world_model_t *wm, element_id_t own_id);

/* ── Auto-ID discovery (boot window only) ────────────────────────────────── */

/*
 * Negotiate a unique element ID autonomously.  Call once after transport_init()
 * before using any other transport function.
 *
 * Runs the full auto-ID protocol:
 *   1. Reads the board's hardware nonce (FICR on nRF; uptime-salted fallback).
 *   2. Broadcasts a discovery beacon (id=ELEMENT_ID_INVALID + nonce) for
 *      CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS while collecting peer nonces and
 *      already-running element IDs from normal gossip.
 *   3. Assigns the nonce-rank-th unclaimed ID as this element's identity.
 *
 * Sets *n_total_out to the estimated total swarm size (co-booting + running).
 * Returns the assigned element_id_t.
 */
element_id_t transport_negotiate_id(int *n_total_out);

/* Lower-level auto-ID primitives — use transport_negotiate_id() in preference.
 * Retained for testing and alternative boot sequences. */

/* Advertise a discovery beacon (gossip frame with id=ELEMENT_ID_INVALID,
 * hardware nonce in update_seq).  Call during the boot window before
 * element_id is known.  transport_send() switches back to normal gossip. */
void transport_advertise_nonce(uint32_t nonce);

/* Drain hardware nonces collected from co-booting peers into out[0..max-1].
 * Returns the count written. */
int transport_drain_nonces(uint32_t *out, int max);

/* Drain the gossip queue without a world model, marking which element IDs
 * have been seen advertising.  claimed_out must be a zeroed bool array of
 * at least max_id elements.  Returns frames drained. */
int transport_drain_claimed(bool *claimed_out, int max_id);

/* ── Telemetry ───────────────────────────────────────────────────────────── */

/* Send one cycle's worth of telemetry.
 * Networking boards: UDP metric frames to the collector (every cycle).
 * BLE-only boards:   serial CSV line to stdout (throttled to GOSSIP_INTERVAL_MS).
 * Pass NULL for scr and 0 for election_count from apps that do not use L5. */
void transport_send_telemetry(const world_model_t *wm, element_id_t element_id,
                              const scr_state_t *scr, uint32_t election_count);

#endif /* TAPESTRY_TRANSPORT_H */
