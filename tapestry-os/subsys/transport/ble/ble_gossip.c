/*
 * tapestry-os/subsys/transport/ble/ble_gossip.c
 * Tapestry L3 BLE advertising/scanning gossip transport
 *
 * Runs alongside the UDP transport on boards that have both (ESP32).
 * On BLE-only boards (micro:bit) this is the sole gossip medium.
 *
 * All boards using this transport interoperate over BLE without any bridge.
 *
 * Manufacturer-specific AD layout:
 *   [0-1] TAPESTRY_BLE_COMPANY_ID  (0xD7, 0x08)
 *   [2]   TAPESTRY_MSG_GOSSIP (1)
 *   [3-21] tapestry_gossip_frame_t (19 bytes, packed)
 *
 * Total manufacturer data = 22 bytes; fits in 31-byte legacy ADV payload.
 */

#include "ble_gossip.h"

#ifdef CONFIG_BT

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_gossip, LOG_LEVEL_INF);

#define MFR_OFF_COMPANY  0
#define MFR_OFF_TYPE     2
#define MFR_OFF_GOSSIP   3
#define MFR_DATA_SIZE    (MFR_OFF_GOSSIP + TAPESTRY_GOSSIP_FRAME_SIZE)   /* 22 */

#define RX_QUEUE_DEPTH  8

K_MSGQ_DEFINE(ble_gossip_rx_q,    sizeof(tapestry_gossip_frame_t), RX_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(ble_discovery_rx_q, sizeof(uint32_t),                RX_QUEUE_DEPTH, 4);

static uint8_t mfr_data[MFR_DATA_SIZE];

static struct bt_data adv_data[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfr_data, MFR_DATA_SIZE),
};


struct parse_ctx {
    bool                    found;
    tapestry_gossip_frame_t gossip;
};

static bool parse_ad_element(struct bt_data *data, void *user_data)
{
    struct parse_ctx *ctx = user_data;

    if (data->type != BT_DATA_MANUFACTURER_DATA) {
        return true;
    }
    if (data->data_len < MFR_DATA_SIZE) {
        return true;
    }

    const uint8_t *d = data->data;
    if (d[MFR_OFF_COMPANY]     != TAPESTRY_BLE_COMPANY_ID_LO ||
        d[MFR_OFF_COMPANY + 1] != TAPESTRY_BLE_COMPANY_ID_HI ||
        d[MFR_OFF_TYPE]        != TAPESTRY_MSG_GOSSIP) {
        return true;
    }

    memcpy(&ctx->gossip, d + MFR_OFF_GOSSIP, sizeof(ctx->gossip));
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

    if (ctx.found) {
        if (ctx.gossip.id == ELEMENT_ID_INVALID) {
            /* Discovery beacon from a co-booting peer: extract nonce. */
            uint32_t nonce = ctx.gossip.update_seq;
            k_msgq_put(&ble_discovery_rx_q, &nonce, K_NO_WAIT);
        } else {
            if (k_msgq_put(&ble_gossip_rx_q, &ctx.gossip, K_NO_WAIT) != 0) {
                LOG_WRN("BLE RX queue full — frame dropped");
            }
        }
    }
}

int ble_gossip_init(void)
{
    mfr_data[MFR_OFF_COMPANY]     = TAPESTRY_BLE_COMPANY_ID_LO;
    mfr_data[MFR_OFF_COMPANY + 1] = TAPESTRY_BLE_COMPANY_ID_HI;
    mfr_data[MFR_OFF_TYPE]        = TAPESTRY_MSG_GOSSIP;
    memset(mfr_data + MFR_OFF_GOSSIP, 0, TAPESTRY_GOSSIP_FRAME_SIZE);

    int ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("bt_enable: %d", ret);
        return ret;
    }

    static const struct bt_le_scan_param scan_params = {
        .type     = BT_LE_SCAN_TYPE_PASSIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_INTERVAL, /* full-window: scan whenever not advertising */
    };

    ret = bt_le_scan_start(&scan_params, scan_cb);
    if (ret) {
        LOG_ERR("bt_le_scan_start: %d", ret);
        return ret;
    }

    /* BT_LE_ADV_PARAM expands to a compound literal — pass inline, not via
     * a static variable (compound literals have automatic storage duration).
     *
     * Use opt=0 (not BT_LE_ADV_OPT_USE_IDENTITY) so each board advertises
     * with a session-unique RPA.  When two boards share the same identity
     * address (possible when CONFIG_BT_SETTINGS is absent and FICR is
     * unpopulated), BT_LE_ADV_OPT_USE_IDENTITY causes the nRF controller to
     * suppress received packets whose source address matches its own, blocking
     * all cross-board gossip. */
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

    LOG_INF("BLE gossip ready (scan + advertising)");
    return 0;
}

void ble_gossip_send(const element_state_t *own_state)
{
    tapestry_gossip_frame_t *p =
        (tapestry_gossip_frame_t *)(mfr_data + MFR_OFF_GOSSIP);

    p->id               = own_state->id;
    p->x                = own_state->position.x;
    p->y                = own_state->position.y;
    p->logical_clock    = own_state->logical_clock;
    p->power_state      = (uint8_t)own_state->power_state;
    p->partition_island = 0;
    p->update_seq       = own_state->update_seq;

    int ret = bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (ret) {
        LOG_WRN("bt_le_adv_update_data: %d", ret);
    }
}

int ble_gossip_drain(world_model_t *wm, element_id_t own_id)
{
    tapestry_gossip_frame_t g;
    int processed = 0;

    while (k_msgq_get(&ble_gossip_rx_q, &g, K_NO_WAIT) == 0) {
        if (g.id == own_id) {
            continue;
        }

        element_state_t received = {0};
        received.id            = g.id;
        received.position.x    = g.x;
        received.position.y    = g.y;
        received.logical_clock = g.logical_clock;
        received.power_state   = (power_state_t)g.power_state;
        received.update_seq    = g.update_seq;

        wm_receive_gossip(wm, &received);
        processed++;
    }

    return processed;
}

void ble_gossip_advertise_nonce(uint32_t nonce)
{
    tapestry_gossip_frame_t *p = (tapestry_gossip_frame_t *)(mfr_data + MFR_OFF_GOSSIP);
    memset(p, 0, sizeof(*p));
    p->id         = ELEMENT_ID_INVALID;
    p->update_seq = nonce;
    bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}

int ble_gossip_drain_nonces(uint32_t *out, int max)
{
    uint32_t nonce;
    int n = 0;
    while (n < max && k_msgq_get(&ble_discovery_rx_q, &nonce, K_NO_WAIT) == 0) {
        out[n++] = nonce;
    }
    return n;
}

int ble_gossip_drain_claimed(bool *claimed_out, int max_id)
{
    tapestry_gossip_frame_t g;
    int n = 0;
    while (k_msgq_get(&ble_gossip_rx_q, &g, K_NO_WAIT) == 0) {
        if (g.id < (uint8_t)max_id) {
            claimed_out[g.id] = true;
        }
        n++;
    }
    return n;
}

#endif /* CONFIG_BT */
