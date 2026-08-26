/*
 * tapestry-os/subsys/transport/transceiver_uart.c
 * Wired UART broadcast transceiver
 *
 * Gives radio-less elements a gossip medium: one shared multi-drop serial bus.
 * Board-agnostic — any Zephyr target with a spare interrupt-driven UART can
 * use it.  Written for the Ambiq Apollo510 EVB (no radio on the part) but
 * nothing here is Apollo-specific.
 *
 * Wiring (N elements on one bus):
 *
 *     element 0 TX ─┬─────┬─────┬──  bus line A  ──┐
 *     element 1 TX ─┘     │     │                  │
 *     element 2 TX ───────┘     │                  │
 *                               └── … ─────────────┤
 *     every element RX ─────────────────────────────┘
 *     every element GND ── common ground (required)
 *
 * TX pins are push-pull on most SoCs, so tying them directly together is a
 * bus contention hazard the moment two elements transmit at once.  For more
 * than two elements put a series resistor (~1 kΩ) in each TX leg, or use a
 * proper open-drain / RS-485 transceiver per node.  Two elements can be wired
 * straight across (A.TX→B.RX, B.TX→A.RX) with no bus at all — that is the
 * recommended bring-up topology.
 *
 * Wire format (little sibling of the syslink framing in
 * transceiver_syslink_p2p.c):
 *
 *     [0x54 'T'][0x50 'P'][len][payload … len bytes][ck_a][ck_b]
 *
 *     len      payload length in bytes (TAPESTRY_GOSSIP_WIRE_SIZE for gossip)
 *     ck_a/b   Fletcher-8 over [len][payload …]
 *
 * The payload is the raw gossip frame; no additional wrapping.  A corrupted
 * or collided frame fails the checksum and is dropped — gossip is lossy-
 * tolerant by design, and every element re-sends its state every
 * GOSSIP_INTERVAL_MS.
 *
 * Echo suppression:
 *   On a shared bus an element hears its own transmission, which the rest of
 *   the stack does not expect ("on non-BLE media a node cannot hear its own
 *   transmission" — gossip.h).  Nothing breaks functionally (gossip_drain
 *   filters own_id and transport_negotiate_id filters own_nonce), but the
 *   gossip_own_id_frames() auto-ID collision detector would read as a
 *   permanent false positive.  So every transmitted frame's checksum is
 *   parked in a small ring and a frame arriving with a matching checksum
 *   inside ECHO_WINDOW_MS is dropped before it reaches the queue.  A genuine
 *   peer frame never collides with ours: it carries a different element id
 *   (or, during discovery, a different nonce).
 *
 * Kconfig / DTS:
 *   CONFIG_TAPESTRY_TRANSCEIVER_UART=y  — compile and register this backend
 *   CONFIG_UART_INTERRUPT_DRIVEN=y      — required for the RX ISR
 *   chosen { tapestry,gossip-uart = &uartN; }  — selects the bus UART
 *
 * Power management:
 *   A UART peripheral that is not clocked cannot receive.  When CONFIG_PM is
 *   enabled this transceiver takes a permanent policy lock on
 *   PM_STATE_SUSPEND_TO_RAM so the idle thread cannot drop the SoC into a
 *   state that stops the UART clock between gossip cycles.  Shallower idle
 *   states are left available.
 */

#include "transceiver_uart.h"

#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_PM
#include <zephyr/pm/policy.h>
#endif

#include <tapestry/wire.h>

LOG_MODULE_REGISTER(uart_bus, LOG_LEVEL_INF);

#if !DT_HAS_CHOSEN(tapestry_gossip_uart)
#error "CONFIG_TAPESTRY_TRANSCEIVER_UART=y requires a DTS chosen node: \
chosen { tapestry,gossip-uart = &uartN; };  Add it in a board overlay."
#endif

/* ── Framing constants ───────────────────────────────────────────────────── */

#define UART_BUS_MAGIC_0  0x54u   /* 'T' */
#define UART_BUS_MAGIC_1  0x50u   /* 'P' */
#define UART_BUS_MTU      64u

BUILD_ASSERT(TAPESTRY_GOSSIP_WIRE_SIZE <= UART_BUS_MTU,
             "gossip wire frame exceeds the UART bus MTU");

/* ── Bus device ──────────────────────────────────────────────────────────── */

static const struct device *bus_uart =
    DEVICE_DT_GET(DT_CHOSEN(tapestry_gossip_uart));

/* Serializes TX against concurrent gossip / relay sends.  Thread context
 * only — every caller reaches tx() from the element's main loop or a work
 * handler, never from an ISR. */
static K_MUTEX_DEFINE(tx_mutex);

/* ── RX queue ────────────────────────────────────────────────────────────── */

#define RX_QUEUE_DEPTH 8

K_MSGQ_DEFINE(uart_rx_msgq, TAPESTRY_GOSSIP_WIRE_SIZE, RX_QUEUE_DEPTH, 4);

/* ── Diagnostic counters ─────────────────────────────────────────────────── */

static uint32_t g_rx_bytes;    /* raw bytes clocked in off the bus          */
static uint32_t g_rx_frames;   /* valid peer frames enqueued                */
static uint32_t g_tx_frames;   /* frames written to the bus                 */
static uint32_t g_ck_fail;     /* checksum failures (corruption, collision) */
static uint32_t g_len_bad;     /* frames whose len is not a gossip frame    */
static uint32_t g_q_drop;      /* valid frames lost to a full queue         */
static uint32_t g_echo_drop;   /* our own transmissions heard back          */
static uint32_t g_resync;      /* frames abandoned mid-parse on an idle gap  */

void uart_transceiver_stats(uint32_t *rx_bytes, uint32_t *rx_frames,
                            uint32_t *tx_frames)
{
    if (rx_bytes)  { *rx_bytes  = g_rx_bytes;  }
    if (rx_frames) { *rx_frames = g_rx_frames; }
    if (tx_frames) { *tx_frames = g_tx_frames; }
}

/* ── Echo suppression ring ───────────────────────────────────────────────── */
/*
 * Written by tx() (thread context), read by the RX ISR.  A torn read can
 * only cost one suppressed echo — the checksum would simply fail to match
 * and a self-frame would be counted once against gossip_own_id_frames().
 * Not worth an ISR lock on the TX path.
 */
#define ECHO_RING_DEPTH 4
#define ECHO_WINDOW_MS  250

static struct {
    uint16_t cksum;      /* ck_a << 8 | ck_b of a frame we transmitted */
    uint32_t sent_ms;    /* k_uptime_get_32() at transmit time         */
} g_echo[ECHO_RING_DEPTH];

static uint8_t g_echo_head;

static void echo_record(uint16_t cksum)
{
    g_echo[g_echo_head].cksum   = cksum;
    g_echo[g_echo_head].sent_ms = k_uptime_get_32();
    g_echo_head = (uint8_t)((g_echo_head + 1u) % ECHO_RING_DEPTH);
}

static bool echo_match(uint16_t cksum)
{
    const uint32_t now = k_uptime_get_32();

    for (int i = 0; i < ECHO_RING_DEPTH; i++) {
        if (g_echo[i].cksum == cksum &&
            (now - g_echo[i].sent_ms) <= ECHO_WINDOW_MS) {
            return true;
        }
    }
    return false;
}

/* ── TX ──────────────────────────────────────────────────────────────────── */

/*
 * bus_send — frame and transmit one payload.
 * uart_poll_out() is safe alongside interrupt-driven RX: the two paths touch
 * different halves of the peripheral and no Zephyr UART driver mixes poll TX
 * with interrupt TX.
 */
static void bus_send(const uint8_t *data, uint8_t len)
{
    uint8_t ca = 0, cb = 0;

    ca += len;  cb += ca;
    for (int i = 0; i < (int)len; i++) {
        ca += data[i];
        cb += ca;
    }

    k_mutex_lock(&tx_mutex, K_FOREVER);

    /* Record the echo signature BEFORE the first byte leaves: on a fast bus
     * the frame can be fully clocked back in before uart_poll_out() returns
     * for the last byte. */
    echo_record((uint16_t)(((uint16_t)ca << 8) | cb));

    uart_poll_out(bus_uart, UART_BUS_MAGIC_0);
    uart_poll_out(bus_uart, UART_BUS_MAGIC_1);
    uart_poll_out(bus_uart, len);
    for (int i = 0; i < (int)len; i++) {
        uart_poll_out(bus_uart, data[i]);
    }
    uart_poll_out(bus_uart, ca);
    uart_poll_out(bus_uart, cb);

    k_mutex_unlock(&tx_mutex);
}

/* ── RX parser (ISR context) ─────────────────────────────────────────────── */

typedef enum {
    PARSE_MAGIC1,
    PARSE_MAGIC2,
    PARSE_LEN,
    PARSE_DATA,
    PARSE_CKA,
    PARSE_CKB,
} parse_state_t;

/*
 * Inter-byte resync.  A length byte corrupted upward makes the parser wait for
 * bytes that will never come, and the frames that follow get consumed as that
 * phantom frame's payload — one glitch silently eats several good frames.  A
 * gap longer than a whole frame can only mean the sender stopped mid-frame, so
 * treat it as a truncation and hunt for a fresh preamble.
 *
 * A 27-byte frame at 115200 takes 2.3 ms and gossip is sent every 500 ms, so
 * 20 ms sits comfortably between "still transmitting" and "next frame".
 * Scanning for a preamble mid-payload was rejected as the alternative: payload
 * bytes can legitimately contain 0x54 0x50, so it would resync on good frames.
 */
#define FRAME_GAP_MS 20u

static uint32_t g_last_byte_ms;

static parse_state_t g_state;
static uint8_t  g_len;
static uint8_t  g_pos;
static uint8_t  g_buf[UART_BUS_MTU];
static uint8_t  g_ca, g_cb;
static uint8_t  g_cka;          /* received ck_a, compared in the CKB state */

static void bus_rx_byte(uint8_t c)
{
    const uint32_t now = k_uptime_get_32();

    if (g_state != PARSE_MAGIC1 && (now - g_last_byte_ms) > FRAME_GAP_MS) {
        g_state = PARSE_MAGIC1;
        g_resync++;
    }
    g_last_byte_ms = now;

    switch (g_state) {
    case PARSE_MAGIC1:
        if (c == UART_BUS_MAGIC_0) {
            g_state = PARSE_MAGIC2;
        }
        break;

    case PARSE_MAGIC2:
        /* A 'T' here is the start of a fresh preamble, not a failed one:
         * stay armed rather than dropping back and losing the byte. */
        if (c == UART_BUS_MAGIC_1) {
            g_state = PARSE_LEN;
        } else if (c != UART_BUS_MAGIC_0) {
            g_state = PARSE_MAGIC1;
        }
        break;

    case PARSE_LEN:
        g_len = c;
        g_pos = 0;
        g_ca = 0; g_cb = 0;
        g_ca += c; g_cb += g_ca;
        if (c == 0u || c > UART_BUS_MTU) {
            /* Framing garbage — a length we can never satisfy.  Resync. */
            g_len_bad++;
            g_state = PARSE_MAGIC1;
        } else {
            g_state = PARSE_DATA;
        }
        break;

    case PARSE_DATA:
        g_buf[g_pos++] = c;
        g_ca += c; g_cb += g_ca;
        if (g_pos >= g_len) {
            g_state = PARSE_CKA;
        }
        break;

    case PARSE_CKA:
        g_cka = c;
        g_state = PARSE_CKB;
        break;

    case PARSE_CKB:
        if (g_cka == g_ca && c == g_cb) {
            if (echo_match((uint16_t)(((uint16_t)g_ca << 8) | g_cb))) {
                g_echo_drop++;
            } else if (g_len != TAPESTRY_GOSSIP_WIRE_SIZE) {
                /* Intact frame, but not a gossip frame — a peer built with a
                 * different CONFIG_TAPESTRY_WIRE_AUTH_ENABLED setting, say. */
                g_len_bad++;
            } else if (k_msgq_put(&uart_rx_msgq, g_buf, K_NO_WAIT) == 0) {
                g_rx_frames++;
            } else {
                g_q_drop++;
            }
        } else {
            g_ck_fail++;
        }
        g_state = PARSE_MAGIC1;
        break;
    }
}

static void uart_rx_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    /*
     * uart_irq_update() above is what actually clears latched RX errors on
     * these parts — the Renesas RA8 SCI-B driver writes CFCLR (PERC/FERC/
     * ORERC) from inside its irq_update, and uart_err_check() there is
     * read-only.  Do not reorder: reaching the FIFO read without an
     * irq_update leaves an overrun latched, and on SCI-B a latched ORER
     * halts reception entirely.
     *
     * err_check is still called because other backends do clear in it, and
     * because an unhandled latched condition is what starved the CF2.1's
     * USART6 (see transceiver_syslink_p2p.c).
     */
    (void)uart_err_check(dev);

    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
        bus_rx_byte(c);
        g_rx_bytes++;
    }
}

/* ── Periodic counter log ────────────────────────────────────────────────── */
/*
 * Same contract as the syslink counter log: one line every 2 s, only when
 * something changed.  On a wired bus this is the first place a miswired
 * ground or a swapped TX/RX shows up — tx climbing with rx_frames stuck at
 * zero on both elements means the bus is not connected; ck_fail climbing
 * means it is connected but at the wrong baud or colliding.
 */
#define BUS_STATS_LOG_MS 2000

static void stats_log_fn(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    static uint32_t last_tx, last_rxf, last_ck, last_len, last_drop, last_echo,
                    last_resync;

    if (g_tx_frames != last_tx   || g_rx_frames != last_rxf ||
        g_ck_fail   != last_ck   || g_len_bad   != last_len ||
        g_q_drop    != last_drop || g_echo_drop != last_echo ||
        g_resync    != last_resync) {
        LOG_INF("uart_bus: tx=%u rx_frames=%u rx_bytes=%u ck_fail=%u "
                "len_bad=%u q_drop=%u echo=%u resync=%u",
                g_tx_frames, g_rx_frames, g_rx_bytes,
                g_ck_fail, g_len_bad, g_q_drop, g_echo_drop, g_resync);
        last_tx     = g_tx_frames;
        last_rxf    = g_rx_frames;
        last_ck     = g_ck_fail;
        last_len    = g_len_bad;
        last_drop   = g_q_drop;
        last_echo   = g_echo_drop;
        last_resync = g_resync;
    }
    k_work_schedule(dwork, K_MSEC(BUS_STATS_LOG_MS));
}

static K_WORK_DELAYABLE_DEFINE(stats_log_work, stats_log_fn);

/* ── Transceiver vtable ──────────────────────────────────────────────────── */

static int uart_bus_init(void)
{
    if (!device_is_ready(bus_uart)) {
        LOG_ERR("gossip UART %s not ready", bus_uart->name);
        return -ENODEV;
    }

#ifdef CONFIG_PM
    /* Keep the UART clocked between gossip cycles — see the power-management
     * note at the top of this file.  Never released: the bus is the
     * element's only link to the collective for as long as it is running. */
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
#endif

    uart_irq_callback_user_data_set(bus_uart, uart_rx_cb, NULL);
    uart_irq_rx_enable(bus_uart);

    /*
     * Enable the error interrupt too, not just RX.
     *
     * This is not belt-and-braces — without it a single overrun can kill the
     * receiver permanently.  On Renesas SCI-B an overrun raises the separate
     * ERI vector, not RXI, and it halts reception until ORER is cleared.  The
     * clear happens in the driver's irq_update(), which only runs when our
     * callback fires.  With ERI masked, an overrun means: reception stops,
     * RXI never fires again, irq_update() is never called, ORER is never
     * cleared — the bus goes silent for good with no error reported anywhere.
     *
     * Overruns are far more likely on a noisy line than a clean one, so this
     * is not a theoretical concern on a shared wired bus.
     */
    uart_irq_err_enable(bus_uart);

    k_work_schedule(&stats_log_work, K_MSEC(BUS_STATS_LOG_MS));

    LOG_INF("uart gossip bus ready on %s  mtu=%u  queue=%u slots",
            bus_uart->name, UART_BUS_MTU, RX_QUEUE_DEPTH);
    return 0;
}

static int uart_bus_tx(const uint8_t *data, uint16_t len)
{
    if (len > UART_BUS_MTU) {
        return -EMSGSIZE;
    }

    bus_send(data, (uint8_t)len);
    g_tx_frames++;
    return 0;
}

static int uart_bus_rx(uint8_t *buf, uint16_t max_len)
{
    if (max_len < TAPESTRY_GOSSIP_WIRE_SIZE) {
        return -ENOMEM;
    }

    if (k_msgq_get(&uart_rx_msgq, buf, K_NO_WAIT) == 0) {
        return TAPESTRY_GOSSIP_WIRE_SIZE;
    }
    return 0;
}

static void uart_bus_set_power(float level)
{
    /* A wired bus has no transmit power to set. */
    ARG_UNUSED(level);
}

const tapestry_transceiver_t transceiver_uart = {
    .type      = TRANSCEIVER_TYPE_UART,
    .init      = uart_bus_init,
    .tx        = uart_bus_tx,
    .rx        = uart_bus_rx,
    .set_power = uart_bus_set_power,
};
