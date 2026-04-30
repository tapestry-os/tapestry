/*
 * tapestry/transceiver.h — Tapestry L1 Communication Transceiver Interface
 *
 * Abstract PHY layer: the boundary between the communication protocol (L3)
 * and the physical medium (RF, acoustic, optical, chemical).
 *
 * Status: interface contract only.
 *
 *   The current L3 backends (ble_gossip.c, udp_gossip.c) drive their hardware
 *   directly and have not yet been refactored to call into this API.
 *   That separation is tracked as part of the L3 implementation pass.
 *
 * Intended wire-up when the L3 refactor lands:
 *   L3 transport.c  ──calls──►  transceiver_tx / transceiver_rx
 *   L1 impls        ──drive──►  BLE stack / UDP socket / acoustic DAC / ...
 *
 * Add a new transceiver backend:
 *   1. Create tapestry-os/boards/<board>/transceiver_<medium>.c
 *   2. Implement every function declared below.
 *   3. Wire it into your app's CMakeLists.txt, gated on the appropriate Kconfig.
 */

#ifndef TAPESTRY_TRANSCEIVER_H
#define TAPESTRY_TRANSCEIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Medium type ─────────────────────────────────────────────────────────── */

typedef enum {
    TRANSCEIVER_TYPE_BLE      = 0,
    TRANSCEIVER_TYPE_UDP      = 1,
    TRANSCEIVER_TYPE_ACOUSTIC = 2,
    TRANSCEIVER_TYPE_OPTICAL  = 3,
    TRANSCEIVER_TYPE_CHEMICAL = 4,
} transceiver_type_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * transceiver_type — Report which physical medium this implementation drives.
 */
transceiver_type_t transceiver_type(void);

/*
 * transceiver_init — Bring up the transceiver hardware.
 * Returns 0 on success, negative errno on failure.
 */
int transceiver_init(void);

/*
 * transceiver_tx — Transmit len bytes from data.
 * Non-blocking where the hardware supports it.
 * Returns 0 on success, negative errno if the medium is unavailable.
 */
int transceiver_tx(const uint8_t *data, uint16_t len);

/*
 * transceiver_rx — Copy up to max_len bytes of the next received frame into buf.
 * Non-blocking: returns 0 immediately if nothing is available.
 * Returns number of bytes written to buf, or negative errno on hardware error.
 */
int transceiver_rx(uint8_t *buf, uint16_t max_len);

/*
 * transceiver_set_power — Set transmit power, normalized [0.0, 1.0].
 * 0.0 = minimum (or off), 1.0 = maximum.
 * No-op if power control is unsupported by the medium.
 */
void transceiver_set_power(float level);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_TRANSCEIVER_H */
