/*
 * tapestry-os/subsys/transport/gossip.c
 * Tapestry gossip framing layer
 *
 * Medium-agnostic: packs element_state_t ↔ tapestry_gossip_frame_t and
 * delegates raw bytes to whichever transceiver backends are registered.
 * The BLE and UDP transceivers each strip/add their own medium headers
 * internally; this layer only ever sees raw gossip-frame bytes.
 */

#include "gossip.h"

#include <string.h>
#include <tapestry/wire.h>
#include "world_model.h"

static const tapestry_transceiver_t * const *g_transceivers;
static int g_n;

void gossip_register_transceivers(const tapestry_transceiver_t * const *t,
                                  int n)
{
    g_transceivers = t;
    g_n = n;
}

void gossip_send(const element_state_t *own_state)
{
    tapestry_gossip_frame_t f = {
        .id               = own_state->id,
        .x                = own_state->position.x,
        .y                = own_state->position.y,
        .logical_clock    = own_state->logical_clock,
        .power_state      = (uint8_t)own_state->power_state,
        .partition_island = 0,
        .update_seq       = own_state->update_seq,
    };

    for (int i = 0; i < g_n; i++) {
        g_transceivers[i]->tx((const uint8_t *)&f, (uint16_t)sizeof(f));
    }
}

int gossip_drain(world_model_t *wm, element_id_t own_id)
{
    uint8_t buf[TAPESTRY_GOSSIP_FRAME_SIZE];
    int total = 0;

    for (int i = 0; i < g_n; i++) {
        int len;
        while ((len = g_transceivers[i]->rx(buf, sizeof(buf))) > 0) {
            if (len < (int)sizeof(tapestry_gossip_frame_t)) {
                continue;
            }
            const tapestry_gossip_frame_t *g =
                (const tapestry_gossip_frame_t *)buf;

            if (g->id == own_id) {
                continue;
            }

            element_state_t received = {0};
            received.id            = g->id;
            received.position.x    = g->x;
            received.position.y    = g->y;
            received.logical_clock = g->logical_clock;
            received.power_state   = (power_state_t)g->power_state;
            received.update_seq    = g->update_seq;

            wm_receive_gossip(wm, &received);
            total++;
        }
    }

    return total;
}
