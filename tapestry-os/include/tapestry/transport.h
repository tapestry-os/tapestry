/*
 * tapestry/transport.h — Tapestry L3 Transport API
 *
 * Stable public interface for gossip transport.  Application code includes
 * only this header; the underlying BLE, UDP, or multi-transport implementation
 * is selected by the build system and compiled into transport.c.
 *
 * Supported configurations (selected by Kconfig / board overlay):
 *   CONFIG_BT only          — BLE advertising gossip (e.g. BBC micro:bit V2;
 *                             requires CONFIG_BT_EXT_ADV — see
 *                             transceiver_ble.c's header comment)
 *   CONFIG_NETWORKING only  — UDP broadcast gossip   (e.g. EK-RA8D1,
 *                             ESP-WROVER-KIT — its BLE leg was dropped,
 *                             see esp_wrover_kit_esp32_procpu.conf)
 *   CONFIG_BT + CONFIG_NETWORKING — both transports  (no current board;
 *                             would need a BLE 5.0+ AND networking target)
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
 * qos_tier is one of TAPESTRY_QOS_BEST_EFFORT / SOFT_RT / HARD_RT (wire.h),
 * carried on the wire and read by the relay ring buffer's eviction policy —
 * see the QoS section of <tapestry/wire.h>, which is authoritative. Normal
 * periodic gossip uses TAPESTRY_QOS_SOFT_RT; runtime.c additionally calls
 * this out-of-cycle with TAPESTRY_QOS_HARD_RT on SCR_ABORT_TRIGGERED
 * (quorum just dropped below DEGRADED) instead of waiting for the next
 * scheduled cycle. */
void transport_send(const element_state_t *own_state, uint8_t qos_tier);

/* Drain all pending received gossip frames into wm.  Skips own_id frames.
 * Returns the total number of frames processed across all transports. */
int transport_drain(world_model_t *wm, element_id_t own_id);

/* ── Remote directives (wire.h v5) ───────────────────────────────────────── */

/* Drain pending directive frames from every transport and write the newest
 * one that survives the full receive filter chain (HMAC when enabled,
 * version, addressee = own_id or broadcast, type validity, per-src replay
 * — see gossip.c) to *out.  Returns true when a new directive was accepted
 * this call; *out is untouched otherwise.  Call once per runtime cycle,
 * after transport_drain().  Media without directive support (BLE, syslink
 * today) simply contribute nothing. */
bool transport_poll_directive(element_id_t own_id,
                              tapestry_directive_frame_t *out);

/* ── Auto-ID discovery (boot window only) ────────────────────────────────── */

/*
 * Negotiate a unique element ID autonomously.  Call once after transport_init()
 * before using any other transport function.  Medium-agnostic: works over
 * BLE advertising, syslink P2P, and UDP broadcast alike (discovery beacons
 * are ordinary gossip frames with id=ELEMENT_ID_INVALID; on one-shot media
 * they are re-sent every GOSSIP_INTERVAL_MS during the window).
 *
 * Runs the full auto-ID protocol:
 *   1. Reads the board's hardware nonce (CONFIG_HWINFO device ID — nRF FICR,
 *      STM32 unique ID; uptime-salted fallback without it).
 *   2. Broadcasts a discovery beacon (id=ELEMENT_ID_INVALID + nonce) for
 *      CONFIG_TAPESTRY_AUTO_ID_WINDOW_MS while collecting peer nonces and
 *      already-running element IDs from normal gossip.
 *   3. Assigns the nonce-rank-th unclaimed ID as this element's identity.
 *
 * Sets *n_total_out to the estimated total swarm size (co-booting + running).
 * Returns the assigned element_id_t.
 */
element_id_t transport_negotiate_id(int *n_total_out);

/* Outcome of the most recent transport_negotiate_id() window: beacons
 * transmitted, peer nonces heard, already-running IDs seen.  Retained so an
 * application can re-log it any time — a console attached after a
 * console-less boot still learns what the discovery window saw. */
void transport_get_negotiation_stats(uint32_t *beacons_tx,
                                     uint32_t *nonces_heard,
                                     uint32_t *ids_running);

/*
 * transport_negotiate_id_retry — self-healing recovery when transport_
 * negotiate_id() came back isolated (*n_total < 2 — heard no peers in its
 * boot window).
 *
 * transport.c has no opinion on whether flying/operating solo is
 * acceptable — that is application policy (e.g. a "-DCONFIG_DEMO_ALLOW_
 * SOLO=y" escape hatch). Call this only once the caller has decided
 * solo is NOT acceptable and isolation must instead be recovered from
 * without a power cycle. Blocks (k_msleep-paced, WM_CYCLE_MS per
 * iteration) until recovery, by whichever comes first:
 *
 *   (a) a peer's gossip becomes directly visible (e.g. a peer that
 *       negotiated fine the whole time, or one recovering from this same
 *       loop under its own renegotiated id); or
 *   (b) re-running transport_negotiate_id() at a jittered 15-45 s
 *       interval resolves to n_total >= 2 (e.g. a peer isolated at the
 *       same moment has since renegotiated and is now gossiping under a
 *       real id — auto-ID's rank-by-nonce assignment can differ between
 *       runs, which is exactly what lets two mutually-isolated units
 *       resolve each other this way).
 *
 * Gossips isolated_id's own state at GOSSIP_INTERVAL_MS throughout, so a
 * peer stuck in the same recovery loop can see this element too, and
 * logs a diagnostic line every 2 s (duplicate-ID evidence via
 * gossip_own_id_frames(), or the retained negotiation-window stats) so a
 * grounded unit's radio health is visible without a debugger attached.
 * Reported position stays at the zero-init default throughout — this is
 * diagnostic gossip only, position plays no part in the recovery
 * decision, and transport.c has no platform-specific way to sample a
 * live one. Prints no entry-state log line itself (phrasing like
 * "grounded"/"not arming" is platform-specific); the caller logs that
 * before calling.
 *
 * isolated_id: the id transport_negotiate_id() returned that this call
 * is recovering from.
 *
 * Returns the resolved element id (renegotiated, or unchanged if a peer
 * was seen directly under our original id) and writes the recovered
 * collective size to *n_total_out (always >= 2 on return — this
 * function does not return while still isolated).
 */
element_id_t transport_negotiate_id_retry(element_id_t isolated_id,
                                          int *n_total_out);

/* Lower-level auto-ID primitives — use transport_negotiate_id() in preference.
 * Retained for testing and alternative boot sequences. */

/* Advertise a discovery beacon (gossip frame with id=ELEMENT_ID_INVALID,
 * hardware nonce in update_seq) via every registered transceiver.  Call
 * during the boot window before element_id is known; re-send periodically
 * on one-shot media.  transport_send() switches back to normal gossip. */
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
