/*
 * tapestry-os/subsys/transport/transceiver_syslink_p2p.h
 * Syslink P2P transceiver — private to the transport subsystem
 *
 * Implements drone-to-drone gossip via the nRF51822 co-processor's existing
 * SYSLINK_RADIO_P2P_BROADCAST (0x0A) channel.  No nRF51 reflash required —
 * the stock Bitcraze firmware (PLATFORM=cf21bl) ships this capability.
 *
 * Physical path:
 *   STM32 USART6 (1 Mbps) → nRF51 → ESB 2.4 GHz broadcast → peer nRF51
 *   → peer USART6 → peer STM32
 *
 * WARNING: do NOT compile cf21bl_crtp_log.c alongside this transceiver.
 * Both TX on USART6 without mutual exclusion; syslink frames would be
 * corrupted.  Use USART3 (CONFIG_UART_CONSOLE on usart3) for debug output
 * in builds that include this transceiver.
 */

#ifndef TAPESTRY_TRANSCEIVER_SYSLINK_P2P_H
#define TAPESTRY_TRANSCEIVER_SYSLINK_P2P_H

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK

#include <tapestry/transceiver.h>

extern const tapestry_transceiver_t transceiver_syslink_p2p;

/* Link health counters — useful for monitoring gossip flow. */
void syslink_p2p_stats(uint32_t *rx_bytes, uint32_t *rx_frames, uint32_t *tx_frames);

#endif /* CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK */
#endif /* TAPESTRY_TRANSCEIVER_SYSLINK_P2P_H */
