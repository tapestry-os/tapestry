/*
 * tapestry-os/subsys/transport/transceiver_ble.c
 * Tapestry L1 BLE transceiver
 *
 * Implements tapestry_transceiver_t for BLE Manufacturer-Specific advertising.
 * Elements simultaneously advertise their own gossip frame and passively scan
 * for peer advertisements — no connections, no pairing.
 *
 * tx: writes raw tapestry_gossip_frame_t bytes into the manufacturer AD record
 *     and calls bt_le_ext_adv_set_data to push the update.
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
 *
 * Requires CONFIG_BT_EXT_ADV (LE Extended Advertising, Bluetooth 5.0+):
 * TAPESTRY_GOSSIP_WIRE_SIZE grew past the legacy 31-byte BLE advertising
 * payload once the gossip frame gained z/orientation (wire.h v3, 42-byte
 * frame). TX now uses the bt_le_ext_adv_* API (bt_le_ext_adv_create /
 * _start / _set_data) instead of bt_le_adv_start / _update_data — the RX
 * scan path is UNCHANGED, since Zephyr's legacy bt_le_scan_start callback
 * already receives extended advertising reports once CONFIG_BT_EXT_ADV is
 * enabled (confirmed against zephyr/subsys/bluetooth/controller/
 * Kconfig.ll_sw_split, which unconditionally selects
 * BT_CTLR_ADV_EXT_SUPPORT for every Nordic nRF5x target).
 *
 * The original ESP32 (esp_wrover_kit, ESP32-D0WD-V3) cannot do this: its
 * Bluetooth controller is 4.2-only (Espressif's own HAL declares
 * SOC_BLE_50_SUPPORTED for esp32c2/c3/c5/c6/h2/c61/s3 but not the plain
 * esp32). That board keeps gossiping over WiFi/UDP (its other configured
 * transport — see README's "CONFIG_BT + CONFIG_NETWORKING" note); its BLE
 * leg is dropped rather than kept on legacy advertising with a smaller,
 * board-conditional frame, which would break the "one wire format,
 * substrate-agnostic" invariant the rest of L3 relies on.
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

/*
 * Extended advertising's per-PDU data budget is controller-dependent —
 * BT_GAP_ADV_MAX_EXT_ADV_DATA_LEN (1650 bytes) is the protocol ceiling
 * WITH chaining across multiple AUX_CHAIN_IND PDUs, which this transceiver
 * deliberately does not use (chaining is real added complexity this frame
 * doesn't need). A single, non-chained extended-adv PDU's practical limit
 * on real hardware is smaller and this codebase has no board on hand to
 * verify it against, so this asserts a conservative, clearly-under-any-
 * real-limit threshold rather than a number that looks precise but isn't
 * verified. Current need: 3 (prefix) + 42 (v3 frame) + 4 (auth tag) = 49
 * bytes, comfortably under it.
 *
 * These numbers were hand-maintained and went stale once already: the
 * frame grew 21 → 22 bytes when the `achieved` field was appended, and
 * nothing caught it because no build in this repo compiled the auth path.
 * The BUILD_ASSERT below is the real guard — it fails the build rather
 * than the radio if the frame outgrows the budget.
 */
#define MFR_DATA_MAX_SIZE  200u   /* conservative single-PDU ceiling — see above */
BUILD_ASSERT(MFR_DATA_SIZE <= MFR_DATA_MAX_SIZE,
             "Tapestry gossip AD record exceeds this conservative single-PDU "
             "extended-advertising budget. Verify the real per-PDU limit on "
             "target hardware before raising MFR_DATA_MAX_SIZE, or shrink "
             "tapestry_gossip_frame_t / the auth tag.");

#define RX_QUEUE_DEPTH  8

/* Queue stores raw wire bytes (frame + optional auth tag) so gossip.c can
 * verify authentication before interpreting the frame content. */
K_MSGQ_DEFINE(ble_rx_q,        TAPESTRY_GOSSIP_WIRE_SIZE, RX_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(ble_discovery_q, sizeof(uint32_t),           RX_QUEUE_DEPTH, 4);

static uint8_t mfr_data[MFR_DATA_SIZE];

static struct bt_data adv_data[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfr_data, MFR_DATA_SIZE),
};

/* Extended advertising set handle — created once in ble_init(), then
 * reused by every ble_tx()/ble_transceiver_advertise_nonce() call to
 * update its data.  NULL cb: no advertiser-activity notifications needed. */
static struct bt_le_ext_adv *ext_adv;

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

    /* BT_LE_EXT_ADV_NCONN: non-connectable extended advertising, opt=0 (no
     * BT_LE_ADV_OPT_USE_IDENTITY) — same RPA rationale as the legacy param
     * above, just via the extended-adv parameter set. */
    ret = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, NULL, &ext_adv);
    if (ret) {
        LOG_ERR("bt_le_ext_adv_create: %d", ret);
        return ret;
    }

    ret = bt_le_ext_adv_set_data(ext_adv, adv_data, ARRAY_SIZE(adv_data),
                                 NULL, 0);
    if (ret) {
        LOG_ERR("bt_le_ext_adv_set_data: %d", ret);
        return ret;
    }

    /* BT_LE_EXT_ADV_START_DEFAULT: timeout=0, num_events=0 — advertise
     * continuously, matching the legacy transceiver's behavior. */
    ret = bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (ret) {
        LOG_ERR("bt_le_ext_adv_start: %d", ret);
        return ret;
    }

    LOG_INF("BLE transceiver ready (scan + extended advertising)");
    return 0;
}

static int ble_tx(const uint8_t *data, uint16_t len)
{
    if (len > TAPESTRY_GOSSIP_WIRE_SIZE) {
        return -EINVAL;
    }
    memcpy(mfr_data + MFR_OFF_FRAME, data, len);
    int ret = bt_le_ext_adv_set_data(ext_adv, adv_data, ARRAY_SIZE(adv_data),
                                     NULL, 0);
    if (ret) {
        LOG_WRN("bt_le_ext_adv_set_data: %d", ret);
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
    p->version    = TAPESTRY_WIRE_VERSION;
    bt_le_ext_adv_set_data(ext_adv, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
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
