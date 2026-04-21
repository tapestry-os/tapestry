/*
 * ble_gossip.h — optional BLE advertising/scanning gossip transport
 *
 * On boards that have CONFIG_BT=y (currently: ESP32-WROVER-KIT), this layer
 * runs alongside the UDP comms layer so the element participates in both the
 * LAN swarm (via UDP) and the BLE swarm (via advertising).
 *
 * The wire format is shared across all board targets so a micro:bit element
 * interoperates directly with the ESP32 element without any bridge process.
 */

#ifndef TAPESTRY_BLE_GOSSIP_H
#define TAPESTRY_BLE_GOSSIP_H

#ifdef CONFIG_BT

#include <tapestry/csm.h>
#include "sim_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enable BT, start passive scanning, begin non-connectable advertising.
 * Returns 0 on success, negative errno on failure. */
int ble_gossip_init(void);

/* Update the BLE advertising payload with own_state. */
void ble_gossip_send(const element_state_t *own_state);

/* Drain received BLE gossip frames into wm.  Skips own_id frames.
 * Returns the number of frames processed. */
int ble_gossip_drain(world_model_t *wm, element_id_t own_id);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BT */
#endif /* TAPESTRY_BLE_GOSSIP_H */
