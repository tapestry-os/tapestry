/*
 * tapestry-os/subsys/transport/transceiver_ble.h
 * BLE transceiver — private interface (transport subsystem only)
 *
 * Exposes the tapestry_transceiver_t vtable instance plus BLE-specific
 * extensions for the auto-ID boot protocol.  Nothing outside the transport
 * subsystem should include this header.
 */

#ifndef TAPESTRY_TRANSCEIVER_BLE_H
#define TAPESTRY_TRANSCEIVER_BLE_H

#ifdef CONFIG_BT

#include <stdbool.h>
#include <stdint.h>
#include <tapestry/transceiver.h>

/* Vtable instance registered by transport.c on BLE-capable boards. */
extern const tapestry_transceiver_t transceiver_ble;

/* ── BLE-specific extensions (auto-ID boot protocol) ────────────────────── */

/* Advertise a discovery beacon (gossip frame with id=ELEMENT_ID_INVALID,
 * hardware nonce in update_seq) during the boot window. */
void ble_transceiver_advertise_nonce(uint32_t nonce);

/* Drain hardware nonces from co-booting peers into out[0..max-1].
 * Returns count written. */
int ble_transceiver_drain_nonces(uint32_t *out, int max);

/* Drain the BLE RX queue without a world model, marking which element IDs
 * have been seen advertising.  claimed_out must be zeroed, length >= max_id.
 * Returns frames drained. */
int ble_transceiver_drain_claimed(bool *claimed_out, int max_id);

/*
 * Weak hook, called from the BT RX thread for every Tapestry-tagged
 * manufacturer AD (company TAPESTRY_BLE_COMPANY_ID) that is NOT a gossip
 * frame — e.g. an application's own diagnostic advertising set.  `data`
 * starts at the company id.  The default does nothing; an application
 * overrides it by defining a function with this signature.  Keep it
 * short: it runs inside the scan callback.
 */
void ble_transceiver_other_tapestry_ad(const uint8_t *data, uint8_t len,
                                       int8_t rssi);

#endif /* CONFIG_BT */
#endif /* TAPESTRY_TRANSCEIVER_BLE_H */
