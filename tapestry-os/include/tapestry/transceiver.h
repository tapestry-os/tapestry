/*
 * tapestry/transceiver.h — Tapestry L1 Communication Transceiver Interface
 *
 * Defines the vtable that every transceiver backend must implement.
 *
 * All implementations live in tapestry-os/subsys/transport/transceiver_<medium>.c,
 * gated on the appropriate Kconfig symbol.  Board-specific peripheral
 * configuration (pin assignments, DMA channels, clocks) belongs in the
 * board's Kconfig overlay, not in a separate file hierarchy.
 *
 * To add a new transceiver backend:
 *   1. Create tapestry-os/subsys/transport/transceiver_<medium>.c.
 *   2. Define a const tapestry_transceiver_t instance with all ops populated.
 *   3. Declare it in transceiver_<medium>.h (private to the transport subsystem).
 *   4. Register it in transport.c under the appropriate Kconfig guard.
 *
 * The tx/rx contract for gossip frames:
 *   tx — caller passes raw tapestry_gossip_frame_t bytes; the backend wraps
 *        them in whatever medium-specific framing is needed (BLE AD record,
 *        UDP header, etc.) before transmitting.
 *   rx — backend strips medium-specific framing and returns raw
 *        tapestry_gossip_frame_t bytes.  Returns 0 when nothing is pending
 *        (non-blocking); gossip.c calls in a tight loop until 0.
 */

#ifndef TAPESTRY_TRANSCEIVER_H
#define TAPESTRY_TRANSCEIVER_H

#include <stdint.h>

/* ── Medium type ─────────────────────────────────────────────────────────── */

typedef enum {
    TRANSCEIVER_TYPE_BLE      = 0,
    TRANSCEIVER_TYPE_UDP      = 1,
    TRANSCEIVER_TYPE_ACOUSTIC = 2,
    TRANSCEIVER_TYPE_OPTICAL  = 3,
    TRANSCEIVER_TYPE_CHEMICAL = 4,
    TRANSCEIVER_TYPE_SYSLINK  = 5,   /* nRF51 ESB P2P broadcast (CF21BL) */
} transceiver_type_t;

/* ── Vtable ───────────────────────────────────────────────────────────────── */

typedef struct {
    transceiver_type_t type;

    /* Bring up hardware.  Called once by transport_init().
     * Returns 0 on success, negative errno on failure. */
    int (*init)(void);

    /* Transmit len bytes of gossip-frame payload.  Non-blocking where hardware
     * supports it.  Returns 0 on success, negative errno on failure. */
    int (*tx)(const uint8_t *data, uint16_t len);

    /* Copy one received gossip-frame payload (medium framing stripped) into buf.
     * Non-blocking: returns 0 immediately when nothing is available.
     * Returns bytes written on success, negative errno on hardware error. */
    int (*rx)(uint8_t *buf, uint16_t max_len);

    /* Set normalized [0.0, 1.0] transmit power.  No-op if unsupported. */
    void (*set_power)(float level);

    /* Directive frames (wire.h v5, TAPESTRY_MSG_DIRECTIVE) — same tx/rx
     * contract as above but for raw tapestry_directive_frame_t bytes.
     * OPTIONAL: NULL means this medium does not carry directives (BLE and
     * syslink today — the remote-BSE path is UDP/sim-only until the
     * directive path is validated there; see gossip.c).  gossip.c
     * NULL-checks before every call, so existing backends compile
     * unchanged with these members zero-initialized. */
    int (*tx_directive)(const uint8_t *data, uint16_t len);
    int (*rx_directive)(uint8_t *buf, uint16_t max_len);
} tapestry_transceiver_t;

#endif /* TAPESTRY_TRANSCEIVER_H */
