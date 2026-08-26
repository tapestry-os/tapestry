/*
 * tapestry-os/subsys/transport/transceiver_uart.h
 * Wired UART gossip transceiver — private to the transport subsystem
 *
 * A board-agnostic broadcast transceiver for elements that have no radio.
 * Every element's UART TX and RX are tied to one shared multi-drop bus, so a
 * frame written by any element is heard by all the others (and by itself —
 * see the echo-suppression note in the .c file).
 *
 * The UART to use is selected by DTS, not by Kconfig:
 *
 *     chosen { tapestry,gossip-uart = &uart1; };
 *
 * Nothing outside the transport subsystem should include this header.
 */

#ifndef TAPESTRY_TRANSCEIVER_UART_H
#define TAPESTRY_TRANSCEIVER_UART_H

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_UART

#include <stdint.h>
#include <tapestry/transceiver.h>

/* Vtable instance registered by transport.c when the wired bus is built in. */
extern const tapestry_transceiver_t transceiver_uart;

/* Link health counters — the wired analogue of syslink_p2p_stats().
 *   rx_bytes  raw bytes clocked in off the bus
 *   rx_frames valid gossip frames enqueued (echo-suppressed frames excluded)
 *   tx_frames gossip frames written to the bus
 * Any of the three pointers may be NULL. */
void uart_transceiver_stats(uint32_t *rx_bytes, uint32_t *rx_frames,
                            uint32_t *tx_frames);

#endif /* CONFIG_TAPESTRY_TRANSCEIVER_UART */
#endif /* TAPESTRY_TRANSCEIVER_UART_H */
