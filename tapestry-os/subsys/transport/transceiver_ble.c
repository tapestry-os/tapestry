/*
 * tapestry-os/subsys/transport/transceiver_ble.c
 * Tapestry L1 BLE transceiver
 *
 * Implements tapestry_transceiver_t for BLE Manufacturer-Specific advertising.
 * Elements simultaneously advertise their own gossip frame and passively scan
 * for peer advertisements — no connections, no pairing.
 *
 * tx: writes raw tapestry_gossip_frame_t bytes into the manufacturer AD record
 *     and calls bt_le_adv_update_data to push the update.
 * rx: dequeues one frame from the scan-callback ring buffer per call.
 *
 * Manufacturer-specific AD layout:
 *   [0-1]  TAPESTRY_BLE_COMPANY_ID  (0xD7, 0x08)
 *   [2]    TAPESTRY_MSG_GOSSIP (1)
 *   [3-N]  gossip frame + optional auth tag (TAPESTRY_GOSSIP_WIRE_SIZE bytes)
 *
 * Use opt=0 (not BT_LE_ADV_OPT_USE_IDENTITY) so each board advertises with a
 * session-unique RPA.  When CONFIG_BT_SETTINGS is absent and FICR is
 * unpopulated, using the identity address causes the nRF controller to suppress
 * packets whose source matches its own — blocking all cross-board gossip.
 */

#include "transceiver_ble.h"

#ifdef CONFIG_BT

#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <tapestry/wire.h>
#include <tapestry/csm.h>

LOG_MODULE_REGISTER(transceiver_ble, LOG_LEVEL_INF);

#define MFR_OFF_COMPANY  0
#define MFR_OFF_TYPE     2
#define MFR_OFF_FRAME    3
/* Full manufacturer AD payload: company(2) + type(1) + frame + auth tag    */
#define MFR_DATA_SIZE    (MFR_OFF_FRAME + TAPESTRY_GOSSIP_WIRE_SIZE)

#define RX_QUEUE_DEPTH  8

/* Queue stores raw wire bytes (frame + optional auth tag) so gossip.c can
 * verify authentication before interpreting the frame content. */
K_MSGQ_DEFINE(ble_rx_q,        TAPESTRY_GOSSIP_WIRE_SIZE, RX_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(ble_discovery_q, sizeof(uint32_t),           RX_QUEUE_DEPTH, 4);

static uint8_t mfr_data[MFR_DATA_SIZE];

static struct bt_data adv_data[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfr_data, MFR_DATA_SIZE),
};

/* ── Scan callback ───────────────────────────────────────────────────────── */

struct parse_ctx {
    bool    found;
    uint8_t wire[TAPESTRY_GOSSIP_WIRE_SIZE]; /* raw frame + optional auth tag */
};

static bool parse_ad_element(struct bt_data *data, void *user_data)
{
    struct parse_ctx *ctx = user_data;

    if (data->type != BT_DATA_MANUFACTURER_DATA ||
        data->data_len < MFR_DATA_SIZE) {
        return true;
    }

    const uint8_t *d = data->data;
    if (d[MFR_OFF_COMPANY]     != TAPESTRY_BLE_COMPANY_ID_LO ||
        d[MFR_OFF_COMPANY + 1] != TAPESTRY_BLE_COMPANY_ID_HI ||
        d[MFR_OFF_TYPE]        != TAPESTRY_MSG_GOSSIP) {
        return true;
    }

    memcpy(ctx->wire, d + MFR_OFF_FRAME, TAPESTRY_GOSSIP_WIRE_SIZE);
    ctx->found = true;
    return false;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
                    struct net_buf_simple *buf)
{
    ARG_UNUSED(addr);
    ARG_UNUSED(rssi);
    ARG_UNUSED(adv_type);

    struct parse_ctx ctx = {0};
    bt_data_parse(buf, parse_ad_element, &ctx);
    if (!ctx.found) {
        return;
    }

    /* Cast to read the id field without interpreting auth tag bytes. */
    const tapestry_gossip_frame_t *f =
        (const tapestry_gossip_frame_t *)ctx.wire;

    if (f->id == ELEMENT_ID_INVALID) {
        uint32_t nonce = f->update_seq;
        k_msgq_put(&ble_discovery_q, &nonce, K_NO_WAIT);
    } else {
        if (k_msgq_put(&ble_rx_q, ctx.wire, K_NO_WAIT) != 0) {
            LOG_WRN("BLE RX queue full — frame dropped");
        }
    }
}

/* ── Vtable ops ──────────────────────────────────────────────────────────── */

static int ble_init(void)
{
    mfr_data[MFR_OFF_COMPANY]     = TAPESTRY_BLE_COMPANY_ID_LO;
    mfr_data[MFR_OFF_COMPANY + 1] = TAPESTRY_BLE_COMPANY_ID_HI;
    mfr_data[MFR_OFF_TYPE]        = TAPESTRY_MSG_GOSSIP;
    memset(mfr_data + MFR_OFF_FRAME, 0, TAPESTRY_GOSSIP_WIRE_SIZE);

    int ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("bt_enable: %d", ret);
        return ret;
    }

    static const struct bt_le_scan_param scan_params = {
        .type     = BT_LE_SCAN_TYPE_PASSIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_INTERVAL,
    };

    ret = bt_le_scan_start(&scan_params, scan_cb);
    if (ret) {
        LOG_ERR("bt_le_scan_start: %d", ret);
        return ret;
    }

    ret = bt_le_adv_start(
        BT_LE_ADV_PARAM(0,
                        BT_GAP_ADV_FAST_INT_MIN_2,
                        BT_GAP_ADV_FAST_INT_MAX_2,
                        NULL),
        adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (ret) {
        LOG_ERR("bt_le_adv_start: %d", ret);
        return ret;
    }

    LOG_INF("BLE transceiver ready (scan + advertising)");
    return 0;
}

static int ble_tx(const uint8_t *data, uint16_t len)
{
    if (len > TAPESTRY_GOSSIP_WIRE_SIZE) {
        return -EINVAL;
    }
    memcpy(mfr_data + MFR_OFF_FRAME, data, len);
    int ret = bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (ret) {
        LOG_WRN("bt_le_adv_update_data: %d", ret);
    }
    return ret;
}

static int ble_rx(uint8_t *buf, uint16_t max_len)
{
    uint8_t wire[TAPESTRY_GOSSIP_WIRE_SIZE];
    if (max_len < TAPESTRY_GOSSIP_WIRE_SIZE) {
        return -EINVAL;
    }
    if (k_msgq_get(&ble_rx_q, wire, K_NO_WAIT) != 0) {
        return 0;
    }
    memcpy(buf, wire, TAPESTRY_GOSSIP_WIRE_SIZE);
    return (int)TAPESTRY_GOSSIP_WIRE_SIZE;
}

static void ble_set_power(float level)
{
    ARG_UNUSED(level);
    /* bt_le_set_tx_power() availability varies by target; no-op for now */
}

const tapestry_transceiver_t transceiver_ble = {
    .type      = TRANSCEIVER_TYPE_BLE,
    .init      = ble_init,
    .tx        = ble_tx,
    .rx        = ble_rx,
    .set_power = ble_set_power,
};

/* ── BLE-specific extensions (auto-ID boot protocol) ────────────────────── */

void ble_transceiver_advertise_nonce(uint32_t nonce)
{
    tapestry_gossip_frame_t *p = (tapestry_gossip_frame_t *)(mfr_data + MFR_OFF_FRAME);
    memset(p, 0, sizeof(*p));
    p->id         = ELEMENT_ID_INVALID;
    p->update_seq = nonce;
    bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}

int ble_transceiver_drain_nonces(uint32_t *out, int max)
{
    uint32_t nonce;
    int n = 0;
    while (n < max && k_msgq_get(&ble_discovery_q, &nonce, K_NO_WAIT) == 0) {
        out[n++] = nonce;
    }
    return n;
}

int ble_transceiver_drain_claimed(bool *claimed_out, int max_id)
{
    uint8_t wire[TAPESTRY_GOSSIP_WIRE_SIZE];
    int n = 0;
    while (k_msgq_get(&ble_rx_q, wire, K_NO_WAIT) == 0) {
        const tapestry_gossip_frame_t *f = (const tapestry_gossip_frame_t *)wire;
        if (f->id < (uint8_t)max_id) {
            claimed_out[f->id] = true;
        }
        n++;
    }
    return n;
}

#endif /* CONFIG_BT */
