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
#include <tapestry/scr.h>   /* includes csm.h: element_state_t, world_model_t,
                               element_id_t, scr_state_t                       */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Core gossip ─────────────────────────────────────────────────────────── */

/* Bring up all configured transports.
 * On networking boards: acquires an IP address and opens UDP sockets.
 * On BLE boards: enables the BT stack, starts passive scanning and advertising.
 * Returns 0 on success, negative errno on fatal failure. */
int transport_init(void);

/* Update the gossip advertisement / send a UDP broadcast with own_state.
 * qos_tier is one of TAPESTRY_QOS_BEST_EFFORT / SOFT_RT / HARD_RT (wire.h).
 * It is embedded in the gossip frame so peers can prioritise accordingly. */
void transport_send(const element_state_t *own_state, uint8_t qos_tier);

/* Drain all pending received gossip frames into wm.  Skips own_id frames.
 * Returns the total number of frames processed across all transports. */
int transport_drain(world_model_t *wm, element_id_t own_id);

/* ── Auto-ID discovery (boot window only) ────────────────────────────────── */

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

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_TRANSPORT_H */
