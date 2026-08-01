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
 *
 * Layer B — opportunistic relay (CONFIG_TAPESTRY_MESH_RELAY):
 *   gossip_drain queues frames that have hop_count > 0 and carry a newer
 *   logical_clock than we last relayed for that id.  gossip_relay_flush
 *   re-transmits the queued frames (hop_count decremented) after a random
 *   0-50 ms jitter window to reduce collision probability on dense networks.
 *   The relay ring buffer holds at most RELAY_QUEUE_DEPTH frames; excess frames
 *   are dropped silently to bound static RAM usage.
 */

#include "gossip.h"

#include <string.h>
#include <tapestry/wire.h>
#include <tapestry/csm.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gossip, LOG_LEVEL_WRN);

#ifdef CONFIG_TAPESTRY_MESH_RELAY
#include <zephyr/random/random.h>
#endif

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

/* Frames received carrying our OWN element id (non-BLE: duplicate-ID
 * evidence — see the check in gossip_drain). */
static uint32_t g_own_id_frames;

uint32_t gossip_own_id_frames(void)
{
    return g_own_id_frames;
}

void gossip_register_transceivers(const tapestry_transceiver_t * const *t,
                                  int n)
{
    g_transceivers = t;
    g_n = n;
}

/* ── Internal: transmit a single gossip frame via all transceivers ────────── */

static void tx_frame(const tapestry_gossip_frame_t *f)
{
#ifdef CONFIG_TAPESTRY_WIRE_AUTH_ENABLED
    uint8_t wire[TAPESTRY_GOSSIP_WIRE_SIZE];
    memcpy(wire, f, sizeof(*f));
    hmac4_sign(wire, sizeof(*f), wire + sizeof(*f));

    for (int i = 0; i < g_n; i++) {
        g_transceivers[i]->tx(wire, (uint16_t)TAPESTRY_GOSSIP_WIRE_SIZE);
    }
#else
    for (int i = 0; i < g_n; i++) {
        g_transceivers[i]->tx((const uint8_t *)f, (uint16_t)sizeof(*f));
    }
#endif
}

/* ── Layer B: relay ring buffer (compiled out when relay disabled) ─────────── */

#ifdef CONFIG_TAPESTRY_MESH_RELAY

#define RELAY_QUEUE_DEPTH 8

/* Per-origin last-relayed logical clock, used to suppress duplicate relays. */
static uint32_t relay_clock[MAX_ELEMENTS];

/* Ring buffer of frames queued for relay re-transmission. */
static tapestry_gossip_frame_t relay_q[RELAY_QUEUE_DEPTH];
static uint8_t relay_q_count;

/* Absolute uptime (ms) after which gossip_relay_flush may transmit. */
static uint32_t relay_flush_at_ms;

static void relay_enqueue(const tapestry_gossip_frame_t *f)
{
    if (relay_q_count >= RELAY_QUEUE_DEPTH) {
        LOG_DBG("relay queue full — frame for id %u dropped", f->id);
        return;
    }
    if (relay_q_count == 0) {
        /* Randomise flush time on first enqueue to spread re-advertisements. */
        relay_flush_at_ms = k_uptime_get_32() + (sys_rand32_get() % 50u);
    }
    relay_q[relay_q_count] = *f;
    relay_q[relay_q_count].hop_count--;
    relay_q_count++;
}

#endif /* CONFIG_TAPESTRY_MESH_RELAY */

/* ── gossip_send ─────────────────────────────────────────────────────────── */

void gossip_send(const element_state_t *own_state, uint8_t qos_tier)
{
    tapestry_gossip_frame_t f = {
        .id            = own_state->id,
        .x             = own_state->position.x,
        .y             = own_state->position.y,
        .logical_clock = own_state->logical_clock,
        .update_seq    = own_state->update_seq,
        .energy_level  = own_state->energy_level,
        .health_flags  = own_state->health_flags,
        .hop_count     = IS_ENABLED(CONFIG_TAPESTRY_MESH_RELAY) ? 2u : 0u,
        .version       = TAPESTRY_WIRE_VERSION,
    };

    tx_frame(&f);
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

            if (g->version != TAPESTRY_WIRE_VERSION) {
                /* Checked here (medium-agnostic), not just in the UDP
                 * message header, because BLE and syslink P2P advertise
                 * this frame directly with no header wrapper at all — see
                 * the "version in the frame itself" note in wire.h.
                 * Rate-limited like the duplicate-ID log below: a peer on
                 * the wrong version stays wrong every cycle, not once. */
                static uint32_t mismatch_count;
                if ((++mismatch_count % 20u) == 1u) {
                    LOG_WRN("gossip frame version mismatch: id=%u wire=%u "
                            "(%u frames)", g->id, g->version, mismatch_count);
                }
                continue;
            }

            if (g->id == own_id) {
#ifndef CONFIG_BT
                /* On BLE, hearing your own id is a normal self-echo (RPA
                 * does not suppress self-rx).  On every other medium a
                 * node cannot receive its own transmission — an own-id
                 * frame here means ANOTHER element holds our ID (auto-ID
                 * collision: both negotiated the same identity, so they
                 * silently drop each other's gossip and fly blind).
                 * 2026-07-19 flight 4 failed exactly this way; counted
                 * via gossip_own_id_frames() so the application can react
                 * (grounded renegotiation) instead of relying on this log
                 * line surviving console loss. */
                g_own_id_frames++;
                if ((g_own_id_frames % 20u) == 1u) {
                    LOG_ERR("received OWN id %u from the network — duplicate "
                            "element ID (auto-ID collision) (%u frames)",
                            own_id, g_own_id_frames);
                }
#endif
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

#ifdef CONFIG_TAPESTRY_MESH_RELAY
            /* Queue for relay if the frame has hops remaining and carries a
             * strictly newer clock than we last relayed for this id.
             * Bounds-check id before indexing relay_clock[]. */
            if (g->hop_count > 0 &&
                g->id < MAX_ELEMENTS &&
                g->logical_clock > relay_clock[g->id]) {
                relay_clock[g->id] = g->logical_clock;
                relay_enqueue(g);
            }
#endif
        }
    }

    return total;
}

/* ── Auto-ID discovery primitives ────────────────────────────────────────── */

void gossip_send_discovery(uint32_t nonce)
{
    tapestry_gossip_frame_t f = {0};

    f.id         = ELEMENT_ID_INVALID;
    f.update_seq = nonce;
    f.version    = TAPESTRY_WIRE_VERSION;
    tx_frame(&f);
}

int gossip_drain_discovery(uint32_t *nonces_out, int max_nonces,
                           bool *claimed_out, int max_id)
{
    uint8_t buf[TAPESTRY_GOSSIP_WIRE_SIZE];
    int n_nonces = 0;

    for (int i = 0; i < g_n; i++) {
        int len;
        while ((len = g_transceivers[i]->rx(buf, sizeof(buf))) > 0) {
            if (len < (int)sizeof(tapestry_gossip_frame_t)) {
                continue;
            }
            /* Auth tags are not verified during the discovery window: the
             * outcome (an ID ordering) is validated by the collective's
             * subsequent authenticated gossip anyway. */
            const tapestry_gossip_frame_t *g =
                (const tapestry_gossip_frame_t *)buf;

            if (g->id == ELEMENT_ID_INVALID) {
                if (n_nonces < max_nonces) {
                    nonces_out[n_nonces++] = g->update_seq;
                }
            } else if (claimed_out != NULL && g->id < (uint8_t)max_id) {
                claimed_out[g->id] = true;
            }
        }
    }

    return n_nonces;
}

/* ── gossip_relay_flush ──────────────────────────────────────────────────── */

void gossip_relay_flush(void)
{
#ifdef CONFIG_TAPESTRY_MESH_RELAY
    if (relay_q_count == 0) {
        return;
    }

    /* Wait until the jitter window set at enqueue time has elapsed. */
    if (k_uptime_get_32() < relay_flush_at_ms) {
        return;
    }

    for (uint8_t i = 0; i < relay_q_count; i++) {
        tx_frame(&relay_q[i]);
        LOG_DBG("relayed frame: id=%u hop_count=%u clock=%u",
                relay_q[i].id, relay_q[i].hop_count,
                relay_q[i].logical_clock);
    }
    relay_q_count = 0;
#endif
}
