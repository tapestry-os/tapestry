/*
 * tapestry-os/subsys/transport/transceiver_syslink_p2p.c
 * Syslink P2P broadcast transceiver for the Crazyflie 2.1 brushless
 *
 * Uses the nRF51822 co-processor's SYSLINK_RADIO_P2P_BROADCAST channel for
 * drone-to-drone gossip over 2.4 GHz ESB radio without reflashing the nRF51.
 *
 * Wire protocol (Bitcraze syslink, USART6 at 1 Mbps):
 *
 *   TX to nRF51:
 *     [0xBC][0xCF][0x0A][len][port=0x00][gossip_frame...][ck_a][ck_b]
 *     len = 1 (port) + gossip_frame_size (20 bytes) = 21
 *     checksum: Fletcher-8 over [type][len][port][gossip_frame...]
 *
 *   RX from nRF51 (P2P broadcast received from peer drone):
 *     same frame format; payload[0]=port is stripped, payload[1..len-1]
 *     is the raw gossip frame, deposited into rx_msgq for gossip.c to drain.
 *
 * nRF51 activation:
 *   The nRF51 will not send syslink packets until it first receives one.
 *   init() sends SYSLINK_RADIO_CHANNEL to both activate the link and lock
 *   all drones to CONFIG_TAPESTRY_P2P_CHANNEL (default 80 = 2480 MHz).
 *
 * Kconfig guards:
 *   CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK=y  — compile and register this backend
 *   CONFIG_TAPESTRY_P2P_CHANNEL            — shared ESB channel (default 80)
 *   CONFIG_UART_INTERRUPT_DRIVEN=y         — required for USART6 RX ISR
 *
 * CONFIG_CF21BL_PM coexistence:
 *   This transceiver owns USART6 whenever it is built in, so cf21bl_pm.c
 *   does no UART I/O of its own in that configuration (see cf21bl_pm.h) —
 *   its RX parsing is done here (SYSLINK_PM_BATTERY_STATE forwarded to
 *   cf21bl_pm_syslink_input()) and its TX activation (the
 *   SYSLINK_PM_BATTERY_AUTOUPDATE kick, otherwise the nRF51 never sends
 *   battery packets) is done here too.
 *
 * cf21bl_crtp_log.c coexistence:
 *   All TX on USART6 (this file, cf21bl_pm.c's standalone mode,
 *   cf21bl_crtp_log.c's console backend) is serialized through the shared
 *   cf21bl_syslink_tx_mutex (cf21bl_syslink_tx.c) — a build may freely
 *   combine this transceiver with the CRTP console backend.
 */

#include "transceiver_syslink_p2p.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <tapestry/wire.h>

LOG_MODULE_REGISTER(syslink_p2p, LOG_LEVEL_INF);

/* ── Syslink constants ───────────────────────────────────────────────────── */

#define SYSLINK_MAGIC_0             0xBCu
#define SYSLINK_MAGIC_1             0xCFu
#define SYSLINK_RADIO_CHANNEL       0x01u
#define SYSLINK_RADIO_P2P_BROADCAST 0x0Au
#define SYSLINK_PM_BATTERY_STATE    0x13u
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14u

#ifdef CONFIG_CF21BL_PM
/* Board-level battery monitor (cf21bl_pm.c) consumes PM battery packets
 * arriving on the same syslink stream.  Declared here rather than via the
 * board header so the transport subsystem needs no board include path. */
extern void cf21bl_pm_syslink_input(const uint8_t *payload, uint8_t len);
extern float cf21bl_pm_vbat(void);
#endif

/* Shared USART6 TX serialization (cf21bl_syslink_tx.c, always compiled on
 * this board) — required whenever cf21bl_crtp_log.c's console backend is
 * linked into the same build as this transceiver.  Declared here rather
 * than via the board header, same rationale as the PM externs above. */
extern struct k_mutex cf21bl_syslink_tx_mutex;

#define SYSLINK_MTU                 64u
#define TAPESTRY_P2P_PORT           0x00u   /* custom Tapestry gossip port */

#ifndef CONFIG_TAPESTRY_P2P_CHANNEL
#define CONFIG_TAPESTRY_P2P_CHANNEL 80
#endif

/* ── UART device ─────────────────────────────────────────────────────────── */

static const struct device *uart6;

/* ── RX ring buffer ──────────────────────────────────────────────────────── */

#define RX_QUEUE_DEPTH 8

K_MSGQ_DEFINE(rx_msgq, TAPESTRY_GOSSIP_WIRE_SIZE, RX_QUEUE_DEPTH, 4);

/* Diagnostic counters — read via syslink_p2p_stats() */
static uint32_t g_rx_bytes;    /* raw bytes received from nRF51 on USART6 */
static uint32_t g_rx_frames;   /* valid P2P frames enqueued */
static uint32_t g_tx_frames;   /* gossip frames sent to nRF51 */

void syslink_p2p_stats(uint32_t *rx_bytes, uint32_t *rx_frames, uint32_t *tx_frames)
{
    *rx_bytes  = g_rx_bytes;
    *rx_frames = g_rx_frames;
    *tx_frames = g_tx_frames;
}

/* ── Syslink TX ──────────────────────────────────────────────────────────── */

/*
 * syslink_send — frame and transmit one syslink packet.
 * Checksum is Fletcher-8 over [type][length][data...].
 * uart_poll_out() is safe from any thread context when
 * CONFIG_UART_INTERRUPT_DRIVEN=y; the STM32 UART driver does not mix
 * poll TX with interrupt TX.
 *
 * Thread context only (holds cf21bl_syslink_tx_mutex) — every caller in
 * this file runs from init() or a k_work handler, never an ISR.
 */
static void syslink_send(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint8_t ca = 0, cb = 0;
    ca += type;  cb += ca;
    ca += len;   cb += ca;
    for (int i = 0; i < (int)len; i++) {
        ca += data[i];
        cb += ca;
    }

    k_mutex_lock(&cf21bl_syslink_tx_mutex, K_FOREVER);
    uart_poll_out(uart6, SYSLINK_MAGIC_0);
    uart_poll_out(uart6, SYSLINK_MAGIC_1);
    uart_poll_out(uart6, type);
    uart_poll_out(uart6, len);
    for (int i = 0; i < (int)len; i++) {
        uart_poll_out(uart6, data[i]);
    }
    uart_poll_out(uart6, ca);
    uart_poll_out(uart6, cb);
    k_mutex_unlock(&cf21bl_syslink_tx_mutex);
}

#ifdef CONFIG_CF21BL_PM
/*
 * PM battery-telemetry activation — the nRF51 stays silent on
 * SYSLINK_PM_BATTERY_STATE until it receives one SYSLINK_PM_BATTERY_AUTOUPDATE
 * request (crazyflie2-nrf-firmware syslinkEnableBatteryMessages()).  In
 * standalone mode (no transport) cf21bl_pm.c sends this itself; here the
 * transport owns USART6, so we must send it — cf21bl_pm_init() only logs and
 * does no UART I/O in this build (see cf21bl_pm.h).  Retries every 2 s (via
 * the same mutex-serialized syslink_send() gossip uses) until the first
 * battery packet is observed, matching cf21bl_pm.c's own standalone cadence.
 */
#define PM_AUTOUPDATE_RETRY_MS 2000

static void pm_kick_fn(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);

    if (cf21bl_pm_vbat() > 0.0f) {
        return;   /* telemetry flowing — stop retrying */
    }

    syslink_send(SYSLINK_PM_BATTERY_AUTOUPDATE, NULL, 0u);
    k_work_schedule(dwork, K_MSEC(PM_AUTOUPDATE_RETRY_MS));
}

static K_WORK_DELAYABLE_DEFINE(pm_kick_work, pm_kick_fn);
#endif /* CONFIG_CF21BL_PM */

/* ── Syslink RX parser (ISR context) ────────────────────────────────────── */

/*
 * Byte-by-byte syslink frame state machine, called from the UART RX ISR.
 * All state is file-static; the ISR is the only writer.
 * k_msgq_put() is interrupt-safe in Zephyr.
 */
typedef enum {
    PARSE_MAGIC1,
    PARSE_MAGIC2,
    PARSE_TYPE,
    PARSE_LEN,
    PARSE_DATA,
    PARSE_CKA,
    PARSE_CKB,
} parse_state_t;

static parse_state_t g_state;
static uint8_t  g_type;
static uint8_t  g_len;
static uint8_t  g_pos;
static uint8_t  g_buf[SYSLINK_MTU];
static uint8_t  g_ca, g_cb;
static uint8_t  g_cka;          /* received ck_a, compared in CKB state */

static void syslink_rx_byte(uint8_t c)
{
    switch (g_state) {
    case PARSE_MAGIC1:
        if (c == SYSLINK_MAGIC_0) {
            g_state = PARSE_MAGIC2;
        }
        break;

    case PARSE_MAGIC2:
        g_state = (c == SYSLINK_MAGIC_1) ? PARSE_TYPE : PARSE_MAGIC1;
        break;

    case PARSE_TYPE:
        g_type = c;
        g_ca = 0; g_cb = 0;
        g_ca += c; g_cb += g_ca;
        g_state = PARSE_LEN;
        break;

    case PARSE_LEN:
        g_len = c;
        g_pos = 0;
        g_ca += c; g_cb += g_ca;
        g_state = (c == 0u) ? PARSE_CKA : PARSE_DATA;
        break;

    case PARSE_DATA:
        if (g_pos < SYSLINK_MTU) {
            g_buf[g_pos] = c;
        }
        g_pos++;
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
            /* Valid frame.  If it is a P2P broadcast with a payload large
             * enough to contain a gossip frame, queue it. */
            if (g_type == SYSLINK_RADIO_P2P_BROADCAST &&
                g_len >= 2u + TAPESTRY_GOSSIP_WIRE_SIZE) {
                /* nRF51 firmware (main.c:286) format:
                 *   g_buf[0] = P2P port  (skip)
                 *   g_buf[1] = RSSI      (skip — nRF51 inserts inter-drone RSSI)
                 *   g_buf[2..] = raw gossip frame */
                if (k_msgq_put(&rx_msgq, g_buf + 2u, K_NO_WAIT) == 0) {
                    g_rx_frames++;
                }
            }
#ifdef CONFIG_CF21BL_PM
            else if (g_type == SYSLINK_PM_BATTERY_STATE &&
                     g_len <= SYSLINK_MTU) {
                cf21bl_pm_syslink_input(g_buf, g_len);
            }
#endif
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

    /* Read-and-clear UART error flags — a latched error condition (e.g.
     * overrun, or line glitches coupling into USART6) re-asserts the
     * interrupt forever and starves the system until the IWDG resets it.
     * See pm_uart_cb in cf21bl_pm.c for the observed failure mode. */
    (void)uart_err_check(dev);

    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
        syslink_rx_byte(c);
        g_rx_bytes++;
    }
}

/* ── Transceiver vtable ──────────────────────────────────────────────────── */

static int syslink_init(void)
{
    uart6 = DEVICE_DT_GET(DT_NODELABEL(usart6));
    if (!device_is_ready(uart6)) {
        LOG_ERR("USART6 not ready");
        return -ENODEV;
    }

    /* Enable interrupt-driven RX so the ISR can accumulate syslink bytes. */
    uart_irq_callback_user_data_set(uart6, uart_rx_cb, NULL);
    uart_irq_rx_enable(uart6);

    /* Activate the nRF51 syslink TX path and set the shared P2P channel.
     * The nRF51 will not send syslink packets until it first receives one;
     * SYSLINK_RADIO_CHANNEL (0x01) serves as both the activation trigger
     * and the channel configuration for all three drones. */
    uint8_t ch = (uint8_t)CONFIG_TAPESTRY_P2P_CHANNEL;
    syslink_send(SYSLINK_RADIO_CHANNEL, &ch, 1u);

#ifdef CONFIG_CF21BL_PM
    k_work_schedule(&pm_kick_work, K_NO_WAIT);
#endif

    LOG_INF("syslink P2P ready  channel=%u  queue=%u slots",
            CONFIG_TAPESTRY_P2P_CHANNEL, RX_QUEUE_DEPTH);
    return 0;
}

static int syslink_tx(const uint8_t *data, uint16_t len)
{
    /* Gossip frame (20 bytes) + 1 port byte must fit in SYSLINK_MTU. */
    if ((uint32_t)len + 1u > SYSLINK_MTU) {
        return -EMSGSIZE;
    }

    /* Build payload: [port][gossip_frame...] */
    uint8_t payload[SYSLINK_MTU];
    payload[0] = TAPESTRY_P2P_PORT;
    memcpy(payload + 1u, data, len);

    syslink_send(SYSLINK_RADIO_P2P_BROADCAST, payload, (uint8_t)(len + 1u));
    g_tx_frames++;
    return 0;
}

static int syslink_rx(uint8_t *buf, uint16_t max_len)
{
    if (max_len < TAPESTRY_GOSSIP_WIRE_SIZE) {
        return -ENOMEM;
    }

    uint8_t frame[TAPESTRY_GOSSIP_WIRE_SIZE];
    if (k_msgq_get(&rx_msgq, frame, K_NO_WAIT) == 0) {
        memcpy(buf, frame, TAPESTRY_GOSSIP_WIRE_SIZE);
        return TAPESTRY_GOSSIP_WIRE_SIZE;
    }
    return 0;
}

static void syslink_set_power(float level)
{
    /* SYSLINK_RADIO_POWER (0x07): nRF51 accepts 0..7 mapped to TX power.
     * Not needed for a short-range indoor formation demo; no-op for now. */
    ARG_UNUSED(level);
}

const tapestry_transceiver_t transceiver_syslink_p2p = {
    .type      = TRANSCEIVER_TYPE_SYSLINK,
    .init      = syslink_init,
    .tx        = syslink_tx,
    .rx        = syslink_rx,
    .set_power = syslink_set_power,
};
