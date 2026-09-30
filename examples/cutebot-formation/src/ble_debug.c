/*
 * ble_debug.c — see ble_debug.h for the packet format and why it is a
 * separate legacy advertising set.
 */
#include "ble_debug.h"

#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>

#include "transceiver_ble.h"   /* ble_transceiver_other_tapestry_ad() */

LOG_MODULE_REGISTER(ble_debug, LOG_LEVEL_INF);

#define KIND_STATUS   0x10u
#define KIND_TEXT     0x11u
#define PAYLOAD_LEN   27u   /* 31-byte legacy PDU - AD len/type (2) - company (2) */
#define TEXT_LEN      22u   /* PAYLOAD_LEN - kind/id/seq (4) - tseq (1) */

/* A new packet every 120 ms, from the system workqueue — NOT the main
 * loop, which a lone robot never reaches (it sits in transport_negotiate_
 * id_retry() until a peer appears).  Status every 4th packet (~2 Hz), or
 * whenever no text is queued. */
#define PACKET_PERIOD_MS   120
#define STATUS_EVERY_TICKS 4u

/* Advertise every 50 ms: each 120 ms packet goes out two or three times
 * before the next replaces it, which is what gives the receiver a second
 * chance at a lost one. */
#define ADV_INTERVAL_MIN BT_GAP_MS_TO_ADV_INTERVAL(50)
#define ADV_INTERVAL_MAX BT_GAP_MS_TO_ADV_INTERVAL(60)

static uint8_t s_mfr[2 + PAYLOAD_LEN] = { 0xD7, 0x08 };   /* TAPESTRY_BLE_COMPANY_ID */
static const struct bt_data s_ad[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, s_mfr, sizeof(s_mfr)),
};

static struct bt_le_ext_adv *s_adv;
static uint16_t s_seq;
static uint8_t  s_tseq;
static uint32_t s_ticks;

/* Every text chunk goes out in TWO packets (same seq, so the receiver
 * de-duplicates): at the ~16% packet loss measured on hardware, a
 * 5-chunk line sent once arrived intact only ~40% of the time. */
static uint8_t  s_last_text[PAYLOAD_LEN];
static bool     s_text_resend;

/* Shared between the main thread (writers) and the work item (reader). */
static struct k_spinlock s_lock;
static uint8_t  s_id = 0xFFu;
static uint8_t  s_drops;
/* Until the main loop publishes one: "alive, not negotiated yet". */
static ble_debug_status_t s_status = { .step = -1 };

RING_BUF_DECLARE(s_text, 2048);

/*
 * Airtime diagnostics, logged over serial every DIAG_PERIOD_MS: is this
 * robot getting its debug packets out, and does it hear its peers' debug
 * packets?  They tell a congested robot radio apart from a receiver
 * that is missing packets.
 */
#define DIAG_PERIOD_MS 5000u
#define DIAG_MAX_IDS   8u
static uint32_t s_diag_runs;
static uint32_t s_diag_fail;
static int      s_diag_last_err;
static uint32_t s_diag_max_set_ms;
static uint32_t s_diag_start_ms;
static atomic_t s_heard[DIAG_MAX_IDS];   /* peer debug packets heard, by id */
static atomic_t s_heard_other;           /* id >= DIAG_MAX_IDS (e.g. 0xFF)  */

/* Scan-callback hook (BT RX thread): another element's debug packet. */
void ble_transceiver_other_tapestry_ad(const uint8_t *data, uint8_t len,
                                       int8_t rssi)
{
    ARG_UNUSED(rssi);
    if (len != 2u + PAYLOAD_LEN ||
        (data[2] != KIND_STATUS && data[2] != KIND_TEXT)) {
        return;
    }
    uint8_t id = data[3];
    atomic_inc(id < DIAG_MAX_IDS ? &s_heard[id] : &s_heard_other);
}

static void diag_tick(uint32_t set_ms, int ret)
{
    s_diag_runs++;
    s_diag_max_set_ms = MAX(s_diag_max_set_ms, set_ms);
    if (ret != 0) {
        s_diag_fail++;
        s_diag_last_err = ret;
    }

    uint32_t now = k_uptime_get_32();
    if (now - s_diag_start_ms < DIAG_PERIOD_MS) {
        return;
    }
    uint32_t span = now - s_diag_start_ms;
    uint32_t h[DIAG_MAX_IDS];
    for (unsigned i = 0; i < DIAG_MAX_IDS; i++) {
        h[i] = (uint32_t)atomic_set(&s_heard[i], 0);
    }
    uint32_t other = (uint32_t)atomic_set(&s_heard_other, 0);

    LOG_INF("dbg tx: %u pkts in %u ms (%u/s), set_data fail %u (last %d), "
            "max %u ms | heard peers' dbg: id0=%u id1=%u id2=%u id3=%u "
            "other=%u", s_diag_runs, span, s_diag_runs * 1000u / span,
            s_diag_fail, s_diag_last_err, s_diag_max_set_ms,
            h[0], h[1], h[2], h[3], other);
    s_diag_runs = s_diag_fail = s_diag_max_set_ms = 0;
    s_diag_last_err = 0;
    s_diag_start_ms = now;
}

static void packet_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(s_packet_work, packet_work_fn);

int ble_debug_init(void)
{
    /* No options: legacy, non-connectable, non-scannable (ADV_NONCONN_IND). */
    struct bt_le_adv_param param =
        BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_NONE, ADV_INTERVAL_MIN,
                             ADV_INTERVAL_MAX, NULL);

    int ret = bt_le_ext_adv_create(&param, NULL, &s_adv);
    if (ret != 0) {
        LOG_ERR("debug adv set create failed: %d "
                "(built without ble_debug.conf?)", ret);
        s_adv = NULL;
        return ret;
    }
    ret = bt_le_ext_adv_set_data(s_adv, s_ad, ARRAY_SIZE(s_ad), NULL, 0);
    if (ret == 0) {
        ret = bt_le_ext_adv_start(s_adv, BT_LE_EXT_ADV_START_DEFAULT);
    }
    if (ret != 0) {
        LOG_ERR("debug adv set start failed: %d", ret);
        bt_le_ext_adv_delete(s_adv);
        s_adv = NULL;
        return ret;
    }
    k_work_schedule(&s_packet_work, K_NO_WAIT);
    LOG_INF("BLE debug telemetry on (legacy adv, 50 ms)");
    return 0;
}

void ble_debug_set_id(uint8_t id)
{
    k_spinlock_key_t key = k_spin_lock(&s_lock);
    s_id = id;
    k_spin_unlock(&s_lock, key);
}

void ble_debug_printf(const char *fmt, ...)
{
    char line[128];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintk(line, sizeof(line) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)sizeof(line) - 2) {
        n = (int)sizeof(line) - 2;   /* truncated */
    }
    line[n++] = '\n';

    k_spinlock_key_t key = k_spin_lock(&s_lock);
    if (ring_buf_space_get(&s_text) < (uint32_t)n) {
        if (s_drops < UINT8_MAX) {
            s_drops++;
        }
    } else {
        ring_buf_put(&s_text, (const uint8_t *)line, (uint32_t)n);
    }
    k_spin_unlock(&s_lock, key);
}

void ble_debug_set_status(const ble_debug_status_t *st)
{
    k_spinlock_key_t key = k_spin_lock(&s_lock);
    s_status = *st;
    k_spin_unlock(&s_lock, key);
}

/* World units / radians -> int16 hundredths / milliradians. */
static int16_t q16(float v, float scale)
{
    float s = v * scale;
    if (!isfinite(s)) {
        return INT16_MIN;
    }
    if (s > (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (s < (float)(INT16_MIN + 1)) {
        return INT16_MIN + 1;   /* INT16_MIN is reserved for "not valid" */
    }
    return (int16_t)lrintf(s);
}

/*
 * Status body, 23 bytes, all little-endian:
 *   [0-3]  uptime ms (u32)
 *   [4]    step (i8)
 *   [5]    flags: b0-1 quorum, b2 suspended, b3 discovered,
 *                 b4 point_valid, b5 target_valid
 *   [6]    fresh peers (low nibble, saturates at 15) | track << 4
 *   [7-8]  x    [9-10] y      (i16, units * 100)
 *   [11-12] heading            (i16, milliradians)
 *   [13-14] tx  [15-16] ty     (i16, units * 100; INT16_MIN = none)
 *   [17-18] px  [19-20] py     (i16, units * 100; INT16_MIN = none)
 *   [21]   gray_max (u8)
 *   [22]   text lines dropped so far (u8, saturating)
 */
static void pack_status(uint8_t *b, const ble_debug_status_t *st,
                        uint8_t drops)
{
    sys_put_le32(k_uptime_get_32(), &b[0]);
    b[4] = (uint8_t)st->step;
    b[5] = (uint8_t)((st->quorum & 0x3u) |
                     (st->suspended    ? BIT(2) : 0u) |
                     (st->discovered   ? BIT(3) : 0u) |
                     (st->point_valid  ? BIT(4) : 0u) |
                     (st->target_valid ? BIT(5) : 0u));
    b[6] = (uint8_t)(MIN(st->fresh_peers, 15u) | ((st->track & 0xFu) << 4));
    sys_put_le16((uint16_t)q16(st->x, 100.0f), &b[7]);
    sys_put_le16((uint16_t)q16(st->y, 100.0f), &b[9]);
    sys_put_le16((uint16_t)q16(st->heading, 1000.0f), &b[11]);
    sys_put_le16((uint16_t)(st->target_valid ? q16(st->tx, 100.0f) : INT16_MIN), &b[13]);
    sys_put_le16((uint16_t)(st->target_valid ? q16(st->ty, 100.0f) : INT16_MIN), &b[15]);
    sys_put_le16((uint16_t)(st->point_valid  ? q16(st->px, 100.0f) : INT16_MIN), &b[17]);
    sys_put_le16((uint16_t)(st->point_valid  ? q16(st->py, 100.0f) : INT16_MIN), &b[19]);
    b[21] = st->gray_max;
    b[22] = drops;
}

static void packet_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);
    s_ticks++;

    uint8_t *p = &s_mfr[2];

    k_spinlock_key_t key = k_spin_lock(&s_lock);
    bool text_waiting = s_text_resend || !ring_buf_is_empty(&s_text);
    if (!text_waiting || s_ticks % STATUS_EVERY_TICKS == 0u) {
        memset(p, 0, PAYLOAD_LEN);
        p[0] = KIND_STATUS;
        p[1] = s_id;
        sys_put_le16(++s_seq, &p[2]);
        pack_status(&p[4], &s_status, s_drops);
    } else if (s_text_resend) {
        memcpy(p, s_last_text, PAYLOAD_LEN);   /* same seq: a repeat */
        s_text_resend = false;
    } else {
        memset(p, 0, PAYLOAD_LEN);
        p[0] = KIND_TEXT;
        p[1] = s_id;
        sys_put_le16(++s_seq, &p[2]);
        p[4] = s_tseq++;
        ring_buf_get(&s_text, &p[5], TEXT_LEN);
        memcpy(s_last_text, p, PAYLOAD_LEN);
        s_text_resend = true;
    }
    k_spin_unlock(&s_lock, key);

    uint32_t t_set = k_uptime_get_32();
    int ret = bt_le_ext_adv_set_data(s_adv, s_ad, ARRAY_SIZE(s_ad), NULL, 0);
    diag_tick(k_uptime_get_32() - t_set, ret);
    if (ret != 0) {
        static bool warned;
        if (!warned) {
            warned = true;
            LOG_WRN("debug adv set_data failed: %d", ret);
        }
    }
    k_work_schedule(&s_packet_work, K_MSEC(PACKET_PERIOD_MS));
}
