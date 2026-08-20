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
 *   The relay ring buffer holds at most RELAY_QUEUE_DEPTH frames; under
 *   pressure the lowest-QoS-tier queued frame is evicted to admit a
 *   higher-tier incoming one (see relay_enqueue) — only when no queued
 *   frame outranks the incoming one is it dropped, so a HARD_RT frame is
 *   never the one silently lost.
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
#include <psa/crypto.h>

/*
 * The tag is HMAC-SHA256 over the frame, truncated to its first
 * TAPESTRY_WIRE_AUTH_TAG_SIZE bytes — unchanged on the wire.
 *
 * Computed through the PSA Crypto API rather than the legacy mbedtls_md_*
 * one.  That is not a style preference: Mbed TLS 4.x (what Zephyr 4.4
 * ships) moved the whole of <mbedtls/md.h> behind
 * MBEDTLS_DECLARE_PRIVATE_IDENTIFIERS, so mbedtls_md_hmac() is no longer
 * public and this file did not compile at all with authentication turned
 * on.  Nothing noticed because CONFIG_TAPESTRY_WIRE_AUTH_ENABLED defaults
 * to n and no application, test or CI job had ever set it — see
 * tapestry-os/tests/transport, which now builds and runs this path.
 */
#define AUTH_ALG PSA_ALG_HMAC(PSA_ALG_SHA_256)

static mbedtls_svc_key_id_t auth_key;
static bool                 auth_key_ready;

/* Import the pre-shared deployment key on first use.  Lazy rather than an
 * init hook so transport_init() keeps its signature and a build with auth
 * disabled pays nothing at all. */
static bool auth_key_load(void)
{
    if (auth_key_ready) {
        return true;
    }
    if (psa_crypto_init() != PSA_SUCCESS) {
        LOG_ERR("psa_crypto_init failed — gossip cannot be authenticated");
        return false;
    }

    const char          *key  = CONFIG_TAPESTRY_WIRE_AUTH_KEY;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;

    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attr, AUTH_ALG);
    psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);

    if (psa_import_key(&attr, (const uint8_t *)key, strlen(key),
                       &auth_key) != PSA_SUCCESS) {
        LOG_ERR("HMAC key import failed — check "
                "CONFIG_TAPESTRY_WIRE_AUTH_KEY");
        return false;
    }
    auth_key_ready = true;
    return true;
}

/* Returns false if no tag could be computed.  Callers must then drop the
 * frame: transmitting or accepting one unauthenticated would silently turn
 * authentication off for the whole deployment. */
static bool hmac4_sign(const uint8_t *data, size_t len, uint8_t *out4)
{
    uint8_t mac[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
    size_t  mac_len = 0;

    if (!auth_key_load()) {
        return false;
    }
    if (psa_mac_compute(auth_key, AUTH_ALG, data, len,
                        mac, sizeof(mac), &mac_len) != PSA_SUCCESS) {
        return false;
    }
    if (mac_len < TAPESTRY_WIRE_AUTH_TAG_SIZE) {
        return false;
    }
    memcpy(out4, mac, TAPESTRY_WIRE_AUTH_TAG_SIZE);
    return true;
}

/* Constant-time comparison for the 4-byte tag to resist timing attacks. */
static bool hmac4_verify(const uint8_t *data, size_t data_len,
                          const uint8_t *tag)
{
    uint8_t expected[TAPESTRY_WIRE_AUTH_TAG_SIZE];

    if (!hmac4_sign(data, data_len, expected)) {
        return false;   /* cannot verify — must not accept */
    }

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
    if (!hmac4_sign(wire, sizeof(*f), wire + sizeof(*f))) {
        /* Better a missed gossip cycle than an unauthenticated frame: a
         * peer would reject it anyway, and emitting one would mask a
         * broken key as ordinary packet loss. */
        LOG_ERR("frame not sent — HMAC tag could not be computed");
        return;
    }

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

/* Enqueue f for relay with its hop_count decremented, preserving its qos
 * tier.  Pulled out of relay_enqueue so both the normal-append and the
 * evict-and-replace paths pack the byte identically. */
static void relay_q_store(uint8_t slot, const tapestry_gossip_frame_t *f)
{
    uint8_t hop = TAPESTRY_HOP_COUNT(f->relay_qos);
    uint8_t qos = TAPESTRY_QOS_TIER(f->relay_qos);

    relay_q[slot] = *f;
    relay_q[slot].relay_qos = TAPESTRY_PACK_RELAY_QOS(hop - 1u, qos);
}

static void relay_enqueue(const tapestry_gossip_frame_t *f)
{
    if (relay_q_count >= RELAY_QUEUE_DEPTH) {
        /* Full: evict the lowest-qos queued frame if the incoming one
         * outranks it, so a HARD_RT frame is never the one silently
         * dropped while a lower-tier frame holds a slot. Ties keep the
         * queued frame (drop the incoming one) rather than churn the
         * queue for no gain. */
        uint8_t worst = 0;
        for (uint8_t i = 1; i < RELAY_QUEUE_DEPTH; i++) {
            if (TAPESTRY_QOS_TIER(relay_q[i].relay_qos) <
                TAPESTRY_QOS_TIER(relay_q[worst].relay_qos)) {
                worst = i;
            }
        }
        uint8_t incoming_qos = TAPESTRY_QOS_TIER(f->relay_qos);
        uint8_t worst_qos    = TAPESTRY_QOS_TIER(relay_q[worst].relay_qos);

        if (incoming_qos <= worst_qos) {
            LOG_DBG("relay queue full — frame for id %u dropped (qos %u)",
                    f->id, incoming_qos);
            return;
        }
        LOG_DBG("relay queue full — evicting id %u (qos %u) to admit id %u "
                "(qos %u)", relay_q[worst].id, worst_qos, f->id, incoming_qos);
        relay_q_store(worst, f);
        return;
    }
    if (relay_q_count == 0) {
        /* Randomise flush time on first enqueue to spread re-advertisements. */
        relay_flush_at_ms = k_uptime_get_32() + (sys_rand32_get() % 50u);
    }
    relay_q_store(relay_q_count, f);
    relay_q_count++;
}

#endif /* CONFIG_TAPESTRY_MESH_RELAY */

/* ── gossip_send ─────────────────────────────────────────────────────────── */

void gossip_send(const element_state_t *own_state, uint8_t qos_tier)
{
    uint8_t hop = IS_ENABLED(CONFIG_TAPESTRY_MESH_RELAY) ? 2u : 0u;

    tapestry_gossip_frame_t f = {
        .id            = own_state->id,
        .x             = own_state->position.x,
        .y             = own_state->position.y,
        .z             = own_state->position.z,
        .qw            = own_state->orientation.w,
        .qx            = own_state->orientation.x,
        .qy            = own_state->orientation.y,
        .qz            = own_state->orientation.z,
        .logical_clock = own_state->logical_clock,
        .update_seq    = own_state->update_seq,
        .energy_level  = own_state->energy_level,
        .health_flags  = own_state->health_flags,
        .relay_qos     = TAPESTRY_PACK_RELAY_QOS(hop, qos_tier),
        .achieved      = own_state->goal_achieved ? 1u : 0u,
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
            received.position.z      = g->z;
            received.orientation.w   = g->qw;
            received.orientation.x   = g->qx;
            received.orientation.y   = g->qy;
            received.orientation.z   = g->qz;
            received.logical_clock   = g->logical_clock;
            received.update_seq      = g->update_seq;
            received.energy_level    = g->energy_level;
            received.health_flags    = g->health_flags;
            received.goal_achieved   = (g->achieved != 0);

            wm_receive_gossip(wm, &received);
            total++;

#ifdef CONFIG_TAPESTRY_MESH_RELAY
            /* Queue for relay if the frame has hops remaining and carries a
             * strictly newer clock than we last relayed for this id.
             * Bounds-check id before indexing relay_clock[]. */
            if (TAPESTRY_HOP_COUNT(g->relay_qos) > 0 &&
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
        LOG_DBG("relayed frame: id=%u hop_count=%u qos=%u clock=%u",
                relay_q[i].id, TAPESTRY_HOP_COUNT(relay_q[i].relay_qos),
                TAPESTRY_QOS_TIER(relay_q[i].relay_qos),
                relay_q[i].logical_clock);
    }
    relay_q_count = 0;
#endif
}
