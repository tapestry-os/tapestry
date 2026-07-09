/*
 * cf21bl_syslink_tx.h — shared USART6 TX serialization for the CF2.1 brushless
 *
 * USART6 carries syslink frames to the nRF51822 co-processor and is shared by
 * up to three independent writers depending on build configuration:
 *   - cf21bl_crtp_log.c        (CRTP console log backend)
 *   - cf21bl_pm.c               (standalone battery-autoupdate kick, only
 *                                 when CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK=n)
 *   - transceiver_syslink_p2p.c (gossip TX + battery-autoupdate kick, only
 *                                 when CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK=y)
 *
 * If two writers interleave bytes of a frame, the nRF51 parser eats one
 * frame as the other's payload and both are lost to the checksum — every
 * writer that sends a complete frame with more than one uart_poll_out() call
 * must hold this mutex for the duration (thread context only; skip in ISR
 * or panic paths, matching cf21bl_crtp_log.c's convention).
 *
 * Defined in its own always-compiled translation unit (rather than inside
 * whichever writer happens to be linked) so any subset of the three writers
 * above can be combined without one silently owning a mutex the others
 * cannot find at link time.
 */

#ifndef TAPESTRY_CF21BL_SYSLINK_TX_H
#define TAPESTRY_CF21BL_SYSLINK_TX_H

#include <zephyr/kernel.h>

extern struct k_mutex cf21bl_syslink_tx_mutex;

#endif /* TAPESTRY_CF21BL_SYSLINK_TX_H */
