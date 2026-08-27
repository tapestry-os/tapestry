/*
 * cf21bl_crtp_log.c — Zephyr log backend → syslink CRTP console → nRF51 radio
 *
 * Encodes Zephyr LOG_* output as syslink-framed CRTP console packets and
 * sends them over USART6 (PC6/PC7, 1 Mbps) to the nRF51822 co-processor.
 * The nRF51 factory firmware relays them over 2.4 GHz radio to crazyradio2;
 * crazyflie_console.py on the host receives and prints them.
 *
 * Syslink frame format (per Bitcraze syslink protocol):
 *
 *   Offset  Bytes  Field
 *   ──────  ─────  ─────────────────────────────────────────────────────────
 *   0       2      Start bytes: 0xBC 0xCF
 *   2       1      Type:        0x00 (CRTP packet)
 *   3       1      Length:      payload length = 1 + text_len
 *   4       1      CRTP header: port=0 channel=0 link=0  →  0x00
 *   5       N      ASCII text, N ≤ 30 bytes (nRF51 console port limit)
 *   5+N     2      Fletcher-8 checksum over bytes [type][length][payload...]
 *
 * USART6 is on APB2 (84 MHz on CF2.1 brushless), so 1 Mbps is correct and
 * not affected by the APB1/PPRE1 RCC issue that breaks the console UART.
 *
 * This backend is compiled when USART6 is enabled in the DTS and CONFIG_LOG=y.
 * It auto-starts and runs alongside any other configured backends.
 *
 * Usage:
 *   Build:  west build -p always -b crazyflie21bl tapestry/tapestry-scr-hw
 *   Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu
 *   Read:   python3 tapestry/tapestry-os/tools/crazyflie_console.py
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>

#include "cf21bl_syslink_tx.h"

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart6), okay)

#define CRTP_CONSOLE_MAX_TEXT  30   /* nRF51 console port payload limit */

static const struct device *uart6;

/*
 * send_crtp — transmit one syslink-framed CRTP console packet.
 *
 * Blocking: uart_poll_out() spins until the USART TXE bit clears.
 * At 1 Mbps each byte takes ~10 µs; a max-length frame is ~350 µs.
 */
static void send_crtp(const uint8_t *text, uint8_t len)
{
    /* Log processing runs in thread context (deferred logging); the panic
     * path may arrive from an exception, where blocking is not allowed —
     * send unserialized there (output is best-effort during panic anyway). */
    bool locked = false;
    if (!k_is_in_isr()) {
        k_mutex_lock(&cf21bl_syslink_tx_mutex, K_FOREVER);
        locked = true;
    }

    uint8_t payload_len = 1U + len;   /* CRTP header byte + text */

    /* Fletcher-8 checksum over [type][payload_len][CRTP_hdr][text...] */
    uint8_t ca = 0, cb = 0;
    ca += 0x00U;        cb += ca;   /* type  = 0x00 */
    ca += payload_len;  cb += ca;   /* length */
    ca += 0x00U;        cb += ca;   /* CRTP header byte */
    for (int i = 0; i < len; i++) {
        ca += text[i];
        cb += ca;
    }

    uart_poll_out(uart6, 0xBC);
    uart_poll_out(uart6, 0xCF);
    uart_poll_out(uart6, 0x00);         /* type = CRTP */
    uart_poll_out(uart6, payload_len);
    uart_poll_out(uart6, 0x00);         /* CRTP header: port=0 ch=0 link=0 */
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart6, text[i]);
    }
    uart_poll_out(uart6, ca);
    uart_poll_out(uart6, cb);

    if (locked) {
        k_mutex_unlock(&cf21bl_syslink_tx_mutex);
    }
}

/*
 * crtp_output_write — log_output_func_t callback.
 *
 * Called by Zephyr's log_output layer with formatted ASCII text (may be
 * called multiple times per log message as the 64-byte format buffer fills).
 * Chunks the input into ≤ 30-byte CRTP packets.
 */
static int crtp_output_write(uint8_t *buf, size_t size, void *ctx)
{
    ARG_UNUSED(ctx);

    if (!uart6) {
        return (int)size;   /* backend not ready; discard silently */
    }

    for (size_t off = 0; off < size; off += CRTP_CONSOLE_MAX_TEXT) {
        uint8_t chunk = (uint8_t)MIN(size - off, (size_t)CRTP_CONSOLE_MAX_TEXT);
        send_crtp(buf + off, chunk);
    }
    return (int)size;
}

/*
 * 64-byte formatting buffer.  Zephyr flushes it on each newline or when full,
 * calling crtp_output_write with the accumulated formatted text.
 */
static uint8_t log_buf[64];
LOG_OUTPUT_DEFINE(log_output_crtp, crtp_output_write, log_buf, sizeof(log_buf));

/* ── Zephyr log backend API ──────────────────────────────────────────────── */

static void backend_init(const struct log_backend *const backend)
{
    ARG_UNUSED(backend);
    uart6 = DEVICE_DT_GET(DT_NODELABEL(usart6));
    if (!device_is_ready(uart6)) {
        uart6 = NULL;
    }
}

static void backend_process(const struct log_backend *const backend,
                            union log_msg_generic *msg)
{
    ARG_UNUSED(backend);

    uint32_t flags = LOG_OUTPUT_FLAG_TIMESTAMP |
                     LOG_OUTPUT_FLAG_LEVEL     |
                     LOG_OUTPUT_FLAG_CRLF_LFONLY;

    log_output_msg_process(&log_output_crtp, &msg->log, flags);
}

static void backend_panic(const struct log_backend *const backend)
{
    ARG_UNUSED(backend);
    log_output_flush(&log_output_crtp);
}

static const struct log_backend_api crtp_backend_api = {
    .process = backend_process,
    .panic   = backend_panic,
    .init    = backend_init,
};

/* autostart=true: backend activates with the log subsystem at boot */
LOG_BACKEND_DEFINE(crtp_log_backend, crtp_backend_api, true);

#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(usart6), okay) */
