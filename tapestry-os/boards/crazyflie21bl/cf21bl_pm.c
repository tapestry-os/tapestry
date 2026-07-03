/*
 * cf21bl_pm.c — battery monitoring via nRF51 syslink for the CF2.1 brushless
 *
 * See cf21bl_pm.h for the wire format and operating modes.
 */

#include "cf21bl_pm.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cf21bl_pm, LOG_LEVEL_INF);

/* Battery thresholds — match stock platform_defaults_cf21bl.h */
#define PM_BAT_LOW_V           3.35f
#define PM_BAT_CRITICAL_V      3.0f
#define PM_LOW_SUSTAIN_MS      5000
#define PM_CRITICAL_SUSTAIN_MS 2000

/* Reference voltage for thrust compensation: the pack voltage at which
 * CF21BL_HOVER_T (cf21bl_stabilizer.c) was measured.  Kconfig, millivolts. */
#define PM_VREF_V              ((float)CONFIG_CF21BL_PM_VREF_MV / 1000.0f)

/* Ignore readings below this — damaged pack or garbled packet (stock
 * motorsCompensateBatteryVoltage() applies the same 2 V sanity floor). */
#define PM_SANITY_MIN_V        2.0f

/* IIR smoothing per packet (packets arrive at ~10 Hz from the nRF51) */
#define PM_FILTER_ALPHA        0.2f

/* ── State (written from ISR context; 32-bit aligned loads are atomic) ────── */

static volatile float   g_vbat;           /* filtered volts; 0 = no data yet */
static volatile bool    g_low;
static volatile bool    g_critical;       /* latched */
static volatile int64_t g_last_pkt_ms;

static int64_t g_below_low_since;         /* 0 = currently above threshold */
static int64_t g_below_crit_since;

/* ── Public API ─────────────────────────────────────────────────────────────── */

float cf21bl_pm_vbat(void)
{
    return g_vbat;
}

float cf21bl_pm_thrust_scale(void)
{
    float v = g_vbat;

    if (v < PM_SANITY_MIN_V) {
        return 1.0f;
    }
    float scale = PM_VREF_V / v;
    if (scale < 0.8f) { scale = 0.8f; }
    if (scale > 1.3f) { scale = 1.3f; }
    return scale;
}

bool cf21bl_pm_battery_low(void)
{
    return g_low;
}

bool cf21bl_pm_battery_critical(void)
{
    return g_critical;
}

void cf21bl_pm_syslink_input(const uint8_t *payload, uint8_t len)
{
    /* [flags u8][vBat f32 LE][chargeCurrent f32 LE] — need flags + vBat */
    if (len < 5u) {
        return;
    }

    float v;
    memcpy(&v, &payload[1], sizeof(v));   /* STM32 is little-endian, as is the wire */

    if (!(v >= 0.0f && v < 5.0f)) {       /* NaN / garbage guard */
        return;
    }

    int64_t now = k_uptime_get();
    g_last_pkt_ms = now;

    float filt = g_vbat;
    if (filt <= 0.0f) {
        filt = v;                          /* first sample seeds the filter */
        LOG_INF("battery telemetry active: %.2f V", (double)v);
    } else {
        filt += PM_FILTER_ALPHA * (v - filt);
    }
    g_vbat = filt;

    if (v < PM_SANITY_MIN_V) {
        /* Implausible reading: don't drive threshold logic from it. */
        return;
    }

    /* Low-battery warning: sustained PM_LOW_SUSTAIN_MS below threshold. */
    if (filt < PM_BAT_LOW_V) {
        if (g_below_low_since == 0) {
            g_below_low_since = now;
        } else if (now - g_below_low_since > PM_LOW_SUSTAIN_MS) {
            g_low = true;
        }
    } else {
        g_below_low_since = 0;
        g_low = false;
    }

    /* Critical: sustained PM_CRITICAL_SUSTAIN_MS below threshold, latched. */
    if (!g_critical) {
        if (filt < PM_BAT_CRITICAL_V) {
            if (g_below_crit_since == 0) {
                g_below_crit_since = now;
            } else if (now - g_below_crit_since > PM_CRITICAL_SUSTAIN_MS) {
                g_critical = true;
            }
        } else {
            g_below_crit_since = 0;
        }
    }
}

/* ── Standalone USART6 mode ─────────────────────────────────────────────────── */
/* Only when the transport subsystem's syslink transceiver is not built:
 * that transceiver owns USART6 and forwards battery packets to
 * cf21bl_pm_syslink_input() from its own RX parser. */

#ifndef CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define SYSLINK_MAGIC_0               0xBCu
#define SYSLINK_MAGIC_1               0xCFu
#define SYSLINK_PM_BATTERY_STATE      0x13u
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14u
#define SYSLINK_MTU                   32u

static const struct device *pm_uart;

/* Shared USART6 TX serialization — defined in cf21bl_crtp_log.c (always
 * compiled on this board).  Interleaving two writers' frames makes the
 * nRF51 parser eat one frame as the other's payload; both are then lost. */
extern struct k_mutex cf21bl_syslink_tx_mutex;

/* RX diagnostics: distinguishes "UART RX dead" (bytes stay 0) from "syslink
 * flows but no battery packets" (frames > 0, pm_frames == 0 → the nRF51 is
 * not sending SYSLINK_PM_BATTERY_STATE, i.e. our autoupdate request is not
 * getting through or its firmware predates it). */
static volatile uint32_t g_rx_bytes;
static volatile uint32_t g_rx_frames;     /* checksum-valid frames, any type */
static volatile uint32_t g_rx_pm_frames;  /* type 0x13 frames               */

/* Minimal syslink frame parser (same wire format as the transport
 * transceiver: [0xBC][0xCF][type][len][data…][ck_a][ck_b], Fletcher-8 over
 * [type][len][data…]).  Runs in the UART RX ISR; this module is the only
 * USART6 RX user in this build. */
typedef enum {
    PM_MAGIC1, PM_MAGIC2, PM_TYPE, PM_LEN, PM_DATA, PM_CKA, PM_CKB,
} pm_parse_state_t;

static pm_parse_state_t pm_state;
static uint8_t pm_type, pm_len, pm_pos, pm_cka;
static uint8_t pm_buf[SYSLINK_MTU];
static uint8_t pm_ca, pm_cb;

static void pm_rx_byte(uint8_t c)
{
    switch (pm_state) {
    case PM_MAGIC1:
        if (c == SYSLINK_MAGIC_0) { pm_state = PM_MAGIC2; }
        break;
    case PM_MAGIC2:
        pm_state = (c == SYSLINK_MAGIC_1) ? PM_TYPE : PM_MAGIC1;
        break;
    case PM_TYPE:
        pm_type = c;
        pm_ca = c; pm_cb = c;
        pm_state = PM_LEN;
        break;
    case PM_LEN:
        pm_len = c;
        pm_pos = 0;
        pm_ca += c; pm_cb += pm_ca;
        pm_state = (c == 0u) ? PM_CKA : PM_DATA;
        break;
    case PM_DATA:
        if (pm_pos < SYSLINK_MTU) { pm_buf[pm_pos] = c; }
        pm_pos++;
        pm_ca += c; pm_cb += pm_ca;
        if (pm_pos >= pm_len) { pm_state = PM_CKA; }
        break;
    case PM_CKA:
        pm_cka = c;
        pm_state = PM_CKB;
        break;
    case PM_CKB:
        if (pm_cka == pm_ca && c == pm_cb) {
            g_rx_frames++;
            if (pm_type == SYSLINK_PM_BATTERY_STATE && pm_len <= SYSLINK_MTU) {
                g_rx_pm_frames++;
                cf21bl_pm_syslink_input(pm_buf, pm_len);
            }
        }
        pm_state = PM_MAGIC1;
        break;
    }
}

static void pm_uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
        return;
    }
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
        g_rx_bytes++;
        pm_rx_byte(c);
    }
}

/* Thread context only (holds the shared TX mutex). */
static void pm_send(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint8_t ca = 0, cb = 0;
    ca += type; cb += ca;
    ca += len;  cb += ca;

    k_mutex_lock(&cf21bl_syslink_tx_mutex, K_FOREVER);

    uart_poll_out(pm_uart, SYSLINK_MAGIC_0);
    uart_poll_out(pm_uart, SYSLINK_MAGIC_1);
    uart_poll_out(pm_uart, type);
    uart_poll_out(pm_uart, len);
    for (uint8_t i = 0; i < len; i++) {
        ca += data[i]; cb += ca;
        uart_poll_out(pm_uart, data[i]);
    }
    uart_poll_out(pm_uart, ca);
    uart_poll_out(pm_uart, cb);

    k_mutex_unlock(&cf21bl_syslink_tx_mutex);
}

/*
 * The nRF51 sends SYSLINK_PM_BATTERY_STATE periodically only after receiving
 * SYSLINK_PM_BATTERY_AUTOUPDATE (crazyflie2-nrf-firmware main.c:
 * syslinkEnableBatteryMessages()).  Re-send the request every 2 s until data
 * flows, from the system workqueue so the TX mutex serializes us against the
 * CRTP console backend.  Logs a diagnostic while unanswered: rx_bytes==0
 * means the nRF51 is sending nothing at all (UART RX path problem);
 * frames>0 with pm==0 means syslink is alive but our request isn't taking
 * effect (frame lost, or nRF51 firmware without AUTOUPDATE support).
 */
static void pm_kick_fn(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);

    if (g_last_pkt_ms != 0) {
        return;                        /* telemetry flowing — stop retrying */
    }

    static int attempts;
    if (attempts > 0) {
        LOG_WRN("no battery telemetry yet (attempt %d: rx_bytes=%u frames=%u pm=%u)",
                attempts, g_rx_bytes, g_rx_frames, g_rx_pm_frames);
    }
    attempts++;

    pm_send(SYSLINK_PM_BATTERY_AUTOUPDATE, NULL, 0);
    k_work_schedule(dwork, K_SECONDS(2));
}

static K_WORK_DELAYABLE_DEFINE(pm_kick_work, pm_kick_fn);

int cf21bl_pm_init(void)
{
    pm_uart = DEVICE_DT_GET(DT_NODELABEL(usart6));
    if (!device_is_ready(pm_uart)) {
        LOG_ERR("USART6 not ready — no battery telemetry");
        return -ENODEV;
    }

    uart_irq_callback_user_data_set(pm_uart, pm_uart_cb, NULL);
    uart_irq_rx_enable(pm_uart);

    k_work_schedule(&pm_kick_work, K_NO_WAIT);

    LOG_INF("battery monitor ready (standalone syslink RX, VREF=%d mV)",
            CONFIG_CF21BL_PM_VREF_MV);
    return 0;
}

#else /* CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK */

int cf21bl_pm_init(void)
{
    /* Transport owns USART6 and forwards battery packets to
     * cf21bl_pm_syslink_input(); its init already wakes the nRF51 TX path. */
    LOG_INF("battery monitor ready (via transport syslink, VREF=%d mV)",
            CONFIG_CF21BL_PM_VREF_MV);
    return 0;
}

#endif /* CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK */
