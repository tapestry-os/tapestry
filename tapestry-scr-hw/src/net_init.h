/*
 * net_init.h — network bring-up for Tapestry hardware elements
 *
 * Provides a single blocking call that acquires an IPv4 address before
 * the main gossip loop starts.  The implementation selects the correct
 * path at compile time:
 *
 *   CONFIG_WIFI set   → join the AP configured by TAPESTRY_WIFI_SSID/PSK,
 *                        then wait for DHCP to assign an address.
 *   CONFIG_WIFI unset → start DHCPv4 on the default interface (Ethernet)
 *                        and wait for an address to be assigned.
 */

#ifndef TAPESTRY_NET_INIT_H
#define TAPESTRY_NET_INIT_H

/*
 * net_connect — bring up the network and acquire an IPv4 address.
 *
 * Blocks until an address is obtained.
 * Returns 0 on success, -1 on unrecoverable error (logged).
 */
int net_connect(void);

#endif /* TAPESTRY_NET_INIT_H */
