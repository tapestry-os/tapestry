/*
 * tapestry-os/subsys/transport/gossip.c
 * Tapestry gossip framing layer
 *
 * Medium-agnostic: packs element_state_t ↔ tapestry_gossip_frame_t and
 * delegates raw bytes to whichever transceiver backends are registered.
 * The BLE and UDP transceivers each strip/add their own medium headers
 * internally; this layer only ever sees raw gossip-frame bytes.
 *
 * When CONFIG_TAPESTRY_WIRE_AUTH_ENABLED is set, each transmitted frame is
 * followed by a TAPESTRY_WIRE_AUTH_TAG_SIZE-byte truncated HMAC-SHA256 tag.
 * Received frames whose tag does not verify are dropped and logged.
 */

#include "gossip.h"

#include <string.h>
#include <tapestry/wire.h>
#include "world_model.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gossip, LOG_LEVEL_WRN);

/* ── Optional HMAC-SHA256 authentication ─────────────────────────────────── */

#ifdef CONFIG_TAPESTRY_WIRE_AUTH_ENABLED
#include <mbedtls/md.h>

static void hmac4_sign(const uint8_t *data, size_t len, uint8_t *out4)
{
    const char *key    = CONFIG_TAPESTRY_WIRE_AUTH_KEY;
    uint8_t     digest[32];

    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                    (const uint8_t *)key, strlen(key),
                    data, len, digest);
    memcpy(out4, digest, TAPESTRY_WIRE_AUTH_TAG_SIZE);
}

/* Constant-time comparison for the 4-byte tag to resist timing attacks. */
static bool hmac4_verify(const uint8_t *data, size_t data_len,
                          const uint8_t *tag)
{
    uint8_t expected[TAPESTRY_WIRE_AUTH_TAG_SIZE];
    hmac4_sign(data, data_len, expected);

    uint8_t diff = 0;
    for (int i = 0; i < (int)TAPESTRY_WIRE_AUTH_TAG_SIZE; i++) {
        diff |= expected[i] ^ tag[i];
    }
    return diff == 0;
}
#endif /* CONFIG_TAPESTRY_WIRE_AUTH_ENABLED */

/* ── Transceiver registry ─────────────────────────────────────────────────── */

static const tapestry_transceiver_t * const *g_transceivers;
static int g_n;

void gossip_register_transceivers(const tapestry_transceiver_t * const *t,
                                  int n)
{
    g_transceivers = t;
    g_n = n;
}

/* ── gossip_send ─────────────────────────────────────────────────────────── */

void gossip_send(const element_state_t *own_state, uint8_t qos_tier)
{
    tapestry_gossip_frame_t f = {
        .id               = own_state->id,
        .x                = own_state->position.x,
        .y                = own_state->position.y,
        .logical_clock    = own_state->logical_clock,
        .partition_island = 0,
        .update_seq       = own_state->update_seq,
        .energy_level     = own_state->energy_level,
        .health_flags     = own_state->health_flags,
        .qos_tier         = qos_tier,
    };

#ifdef CONFIG_TAPESTRY_WIRE_AUTH_ENABLED
    uint8_t wire[TAPESTRY_GOSSIP_WIRE_SIZE];
    memcpy(wire, &f, sizeof(f));
    hmac4_sign(wire, sizeof(f), wire + sizeof(f));

    for (int i = 0; i < g_n; i++) {
        g_transceivers[i]->tx(wire, (uint16_t)TAPESTRY_GOSSIP_WIRE_SIZE);
    }
#else
    for (int i = 0; i < g_n; i++) {
        g_transceivers[i]->tx((const uint8_t *)&f, (uint16_t)sizeof(f));
    }
#endif
}

/* ── gossip_drain ────────────────────────────────────────────────────────── */

int gossip_drain(world_model_t *wm, element_id_t own_id)
{
    uint8_t buf[TAPESTRY_GOSSIP_WIRE_SIZE];
    int total = 0;

    for (int i = 0; i < g_n; i++) {
        int len;
        while ((len = g_transceivers[i]->rx(buf, sizeof(buf))) > 0) {
            if (len < (int)sizeof(tapestry_gossip_frame_t)) {
                continue;
            }

#ifdef CONFIG_TAPESTRY_WIRE_AUTH_ENABLED
            if (len < (int)TAPESTRY_GOSSIP_WIRE_SIZE) {
                LOG_WRN("short frame (len=%d) — auth tag missing, dropped", len);
                continue;
            }
            if (!hmac4_verify(buf, sizeof(tapestry_gossip_frame_t),
                               buf + sizeof(tapestry_gossip_frame_t))) {
                LOG_WRN("HMAC mismatch — frame dropped");
                continue;
            }
#endif

            const tapestry_gossip_frame_t *g =
                (const tapestry_gossip_frame_t *)buf;

            if (g->id == own_id) {
                continue;
            }

            element_state_t received = {0};
            received.id              = g->id;
            received.position.x      = g->x;
            received.position.y      = g->y;
            received.logical_clock   = g->logical_clock;
            received.update_seq      = g->update_seq;
            received.energy_level    = g->energy_level;
            received.health_flags    = g->health_flags;

            wm_receive_gossip(wm, &received);
            total++;
        }
    }

    return total;
}
