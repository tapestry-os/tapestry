/*
 * net_init.c — network bring-up for Tapestry hardware elements
 *
 * WiFi path  (CONFIG_WIFI=y):
 *   1. Register net_mgmt callbacks for WiFi and IPv4 events.
 *   2. Request association with the AP (TAPESTRY_WIFI_SSID/PSK).
 *   3. Block until DHCP delivers an IPv4 address.
 *
 * Ethernet path (CONFIG_WIFI unset):
 *   1. Register a net_mgmt callback for IPv4 address assignment.
 *   2. Start DHCPv4 on the default interface.
 *   3. Block until an address is assigned.
 */

#include "net_init.h"

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(net_init, LOG_LEVEL_INF);

/* Released by the IPv4 address-add event handler. */
static K_SEM_DEFINE(ip_acquired, 0, 1);

static struct net_mgmt_event_callback ipv4_cb;

static void ipv4_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t event,
                               struct net_if *iface)
{
    ARG_UNUSED(cb);
    ARG_UNUSED(iface);

    if (event == NET_EVENT_IPV4_ADDR_ADD) {
        LOG_INF("IPv4 address acquired");
        k_sem_give(&ip_acquired);
    }
}

/* ── WiFi path ───────────────────────────────────────────────────────────── */

#ifdef CONFIG_WIFI

#include <string.h>
#include <zephyr/net/wifi_mgmt.h>

static struct net_mgmt_event_callback wifi_cb;

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t event,
                               struct net_if *iface)
{
    ARG_UNUSED(iface);

    if (event == NET_EVENT_WIFI_CONNECT_RESULT) {
        const struct wifi_status *s = (const struct wifi_status *)cb->info;
        if (s->status) {
            LOG_ERR("WiFi association failed: %d", s->status);
        } else {
            LOG_INF("WiFi associated — waiting for DHCP");
        }
    } else if (event == NET_EVENT_WIFI_DISCONNECT_RESULT) {
        LOG_WRN("WiFi disconnected");
    }
}

int net_connect(void)
{
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("no network interface found");
        return -1;
    }

    /* Register callbacks before issuing the connect request. */
    net_mgmt_init_event_callback(
        &wifi_cb, wifi_event_handler,
        NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_cb);

    net_mgmt_init_event_callback(
        &ipv4_cb, ipv4_event_handler,
        NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&ipv4_cb);

    struct wifi_connect_req_params params = {
        .ssid        = CONFIG_TAPESTRY_WIFI_SSID,
        .ssid_length = (uint8_t)strlen(CONFIG_TAPESTRY_WIFI_SSID),
        .psk         = CONFIG_TAPESTRY_WIFI_PSK,
        .psk_length  = (uint8_t)strlen(CONFIG_TAPESTRY_WIFI_PSK),
        .security    = WIFI_SECURITY_TYPE_PSK,
        .channel     = WIFI_CHANNEL_ANY,
        .band        = WIFI_FREQ_BAND_2_4_GHZ,
        .mfp         = WIFI_MFP_OPTIONAL,
    };

    LOG_INF("connecting to SSID: %s", CONFIG_TAPESTRY_WIFI_SSID);
    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
                       &params, sizeof(params));
    if (ret) {
        LOG_ERR("NET_REQUEST_WIFI_CONNECT failed: %d", ret);
        return ret;
    }

    k_sem_take(&ip_acquired, K_FOREVER);

    /* Disable WiFi power save so the AP delivers broadcast frames immediately
     * rather than buffering them until the next DTIM beacon interval.
     * Without this, gossip broadcast can be delayed 10-20 s on some APs. */
    struct wifi_ps_params ps = { .enabled = WIFI_PS_DISABLED };
    ret = net_mgmt(NET_REQUEST_WIFI_PS, iface, &ps, sizeof(ps));
    if (ret) {
        LOG_WRN("failed to disable WiFi PS: %d (gossip may be delayed)", ret);
    } else {
        LOG_INF("WiFi power save disabled");
    }

    return 0;
}

/* ── Ethernet path ───────────────────────────────────────────────────────── */

#else

int net_connect(void)
{
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("no network interface found");
        return -1;
    }

    net_mgmt_init_event_callback(
        &ipv4_cb, ipv4_event_handler,
        NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&ipv4_cb);

    net_dhcpv4_start(iface);
    LOG_INF("DHCP started — waiting for IPv4 address");

    k_sem_take(&ip_acquired, K_FOREVER);
    return 0;
}

#endif /* CONFIG_WIFI */
