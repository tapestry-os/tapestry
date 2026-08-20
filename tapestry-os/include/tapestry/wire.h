/*
 * tapestry/wire.h — Tapestry L3 On-Wire Frame Format
 *
 * Defines the packed structs and constants that describe every byte
 * exchanged between Tapestry elements, regardless of transport (UDP
 * broadcast, BLE advertising, or future RF mesh).
 *
 * Rules:
 *   - No OS or Zephyr types.  Pure C99 + <stdint.h>.
 *   - All wire structs are __attribute__((packed)) — no padding.
 *   - Python's struct module mirrors each layout with little-endian ('<')
 *     format strings documented in each struct's comment block.  Do not
 *     hand-edit a mirror after changing a struct here — regenerate the
 *     three consumers instead:
 *       python3 tapestry-os/tools/gen_wire_protocol.py
 *
 * Whenever a struct below changes layout (not a pure trailing append —
 * see TAPESTRY_WIRE_VERSION below), bump TAPESTRY_WIRE_VERSION so a stale
 * peer's frames are rejected instead of silently misinterpreted.
 *
 * Message type space
 * ──────────────────
 *   1  TAPESTRY_MSG_GOSSIP       — element ↔ element (or via sim broker)
 *   2  TAPESTRY_MSG_METRIC       — element → telemetry collector
 *   3  (reserved — simulation control; never used on hardware)
 *   4  TAPESTRY_MSG_SCR_METRIC   — element → telemetry collector (L5)
 *
 * BLE wire identification
 * ────────────────────────
 *   BLE gossip frames are carried in Bluetooth Manufacturer-Specific AD
 *   records.  TAPESTRY_BLE_COMPANY_ID_LO / _HI identify the record as a
 *   Tapestry frame so scanners can ignore unrelated advertisements.
 */

#ifndef TAPESTRY_WIRE_H
#define TAPESTRY_WIRE_H

#include <stdint.h>

/* ── BLE frame identification ────────────────────────────────────────────── */

#define TAPESTRY_BLE_COMPANY_ID_LO   0xD7u
#define TAPESTRY_BLE_COMPANY_ID_HI   0x08u

/* ── QoS delivery tiers ──────────────────────────────────────────────────── */
/*
 * Carried on the wire in the low bits of the gossip frame's relay_qos byte
 * (see below) and acted on in two places:
 *   - gossip.c's relay ring buffer admits a higher-tier frame by evicting
 *     the lowest-tier queued frame instead of dropping the incoming one,
 *     so a HARD_RT frame is never the one silently lost under pressure.
 *   - runtime.c sends a HARD_RT frame immediately on SCR_ABORT_TRIGGERED
 *     (quorum just dropped below DEGRADED) instead of waiting for the next
 *     scheduled GOSSIP_INTERVAL_MS cycle.
 * TAPESTRY_QOS_BEST_EFFORT has no current sender: telemetry travels its own
 * separate channel (transport_send_telemetry()), not gossip. It is defined
 * and carried on the wire but not used by any call site today.
 */

#define TAPESTRY_QOS_BEST_EFFORT  0u   /* Background telemetry (unused today) */
#define TAPESTRY_QOS_SOFT_RT      1u   /* Coordination gossip                 */
#define TAPESTRY_QOS_HARD_RT      2u   /* Emergency / control frames          */

/* ── Optional frame authentication ──────────────────────────────────────── */
/*
 * When CONFIG_TAPESTRY_WIRE_AUTH_ENABLED is set each gossip frame is
 * followed on the wire by a TAPESTRY_WIRE_AUTH_TAG_SIZE-byte truncated
 * HMAC-SHA256 tag.  When disabled the tag is absent and the wire format
 * is unchanged from the plain-frame layout documented below.
 *
 * The Python format strings in this file describe the frame itself only.
 * Python consumers that need to handle authenticated frames must read an
 * additional TAPESTRY_WIRE_AUTH_TAG_SIZE bytes after each gossip payload.
 */

#ifdef CONFIG_TAPESTRY_WIRE_AUTH_ENABLED
#  define TAPESTRY_WIRE_AUTH_TAG_SIZE   4u
#else
#  define TAPESTRY_WIRE_AUTH_TAG_SIZE   0u
#endif

/* ── Wire schema version ──────────────────────────────────────────────────── */
/*
 * Bump whenever a frame struct's field layout below changes (add/remove/
 * reorder/retype a field — NOT a pure trailing append, which the length
 * checks on receive already tolerate).  Carried in every message header;
 * receivers reject a frame whose version does not match their own rather
 * than risk misinterpreting bytes laid out under a different schema.  This
 * only guards translation errors — it is not a compatibility mechanism:
 * there is no negotiation, and a version bump is a breaking change for any
 * peer still on the old one.
 *
 * v2: tapestry_gossip_frame_t's hop_count byte was repacked into relay_qos
 * (hop_count in bits [1:0], qos tier in bits [3:2] — see "QoS delivery
 * tiers" above) to carry the QoS tier without growing the frame, which
 * matters because the BLE advertising payload has zero spare bytes (see
 * transceiver_ble.c's budget comment).  A v1 sender's hop_count values are
 * always 0-2 with the upper bits always zero, which happens to decode under
 * v2 as an unchanged hop_count and qos=BEST_EFFORT — but the version check
 * still rejects the mismatch rather than rely on that coincidence, since a
 * v2 sender's qos bits would misparse as a v1 hop_count outside its
 * expected 0-2 range.
 */
#define TAPESTRY_WIRE_VERSION   2u

/* ── Message types ───────────────────────────────────────────────────────── */

typedef enum {
    TAPESTRY_MSG_GOSSIP     = 1,
    TAPESTRY_MSG_METRIC     = 2,
    /* 3: sim-only control — never transmitted by hardware elements */
    TAPESTRY_MSG_SCR_METRIC = 4,
} tapestry_msg_type_t;

/* ── Message header ──────────────────────────────────────────────────────── */
/*
 * Python format: struct.Struct('<BBBH')
 * Size: 5 bytes
 */
typedef struct {
    uint8_t  version;       /* TAPESTRY_WIRE_VERSION — see above     */
    uint8_t  type;          /* tapestry_msg_type_t                   */
    uint8_t  src_id;        /* sender element ID                     */
    uint16_t payload_len;   /* bytes following this header           */
} __attribute__((packed)) tapestry_msg_header_t;

#define TAPESTRY_MSG_HEADER_SIZE   ((uint16_t)sizeof(tapestry_msg_header_t))   /* 5 */

/* ── Gossip frame ────────────────────────────────────────────────────────── */
/*
 * Carries one element's authoritative state to all peers.
 * Sent every GOSSIP_INTERVAL_MS; received and fed into wm_receive_gossip().
 *
 * Python format: struct.Struct('<BffIIBBBBB')
 * Size: 22 bytes
 * Fields: id, x, y, logical_clock, update_seq,
 *         energy_level, health_flags, relay_qos, achieved, version
 *
 * relay_qos: hop_count (bits [1:0]) and qos tier (bits [3:2]) packed into
 *   one byte — see TAPESTRY_HOP_COUNT() / TAPESTRY_QOS_TIER() /
 *   TAPESTRY_PACK_RELAY_QOS() below. Bits [7:4] are reserved and must be 0.
 *
 *   hop_count is the relay TTL.  First-party frames start at 2 when
 *   CONFIG_TAPESTRY_MESH_RELAY is enabled (0 otherwise).  Each relay node
 *   decrements by 1 before re-advertising; frames with hop_count == 0 are
 *   never re-advertised, capping relay depth at two hops.
 *
 *   qos tier is one of TAPESTRY_QOS_* above.  Used by the relay ring buffer
 *   to decide which queued frame to evict under pressure — see gossip.c.
 *
 * achieved: this element's L6/L7 own-goal achievement predicate
 *   (choreo_goal_achieved()) as of its last gossip send — 0 or 1.  Lets
 *   peers aggregate a collective ("scope = all") achievement predicate
 *   from gossiped state alone; see choreo_collective_achieved().
 *   Eventually consistent like every other gossiped field — no barrier.
 *
 * version: TAPESTRY_WIRE_VERSION, carried IN the frame itself (not just the
 *   tapestry_msg_header_t wrapper) because BLE and syslink P2P advertise
 *   this frame directly with no header wrapper at all — see wire.h's "Wire
 *   schema version" section.  Stays the LAST field (not first, unlike the
 *   message header) so `id` stays the frame's first byte, which
 *   transceiver_udp.c relies on when extracting src_id before the header
 *   is populated; new fields are inserted before it, never after.
 *
 * When CONFIG_TAPESTRY_WIRE_AUTH_ENABLED is set, TAPESTRY_WIRE_AUTH_TAG_SIZE
 * additional bytes follow the frame on the wire (not counted here).
 */
typedef struct {
    uint8_t  id;
    float    x;
    float    y;
    uint32_t logical_clock;
    uint32_t update_seq;
    uint8_t  energy_level;         /* Battery/power [0=empty, 100=full]       */
    uint8_t  health_flags;         /* ELEMENT_HEALTH_* bitmask (see csm.h)    */
    uint8_t  relay_qos;            /* hop_count[1:0] | qos_tier[3:2] — packed */
    uint8_t  achieved;             /* own-goal achievement predicate, 0/1     */
    uint8_t  version;              /* TAPESTRY_WIRE_VERSION — see above       */
} __attribute__((packed)) tapestry_gossip_frame_t;

#define TAPESTRY_GOSSIP_FRAME_SIZE   ((uint16_t)sizeof(tapestry_gossip_frame_t))   /* 22 */

/* ── relay_qos packing ───────────────────────────────────────────────────── */

#define TAPESTRY_RELAY_QOS_HOP_MASK    0x03u   /* bits [1:0]: hop_count 0-2 */
#define TAPESTRY_RELAY_QOS_QOS_SHIFT   2u
#define TAPESTRY_RELAY_QOS_QOS_MASK    0x0Cu   /* bits [3:2]: qos tier 0-2  */

#define TAPESTRY_HOP_COUNT(relay_qos) \
    ((uint8_t)((relay_qos) & TAPESTRY_RELAY_QOS_HOP_MASK))

#define TAPESTRY_QOS_TIER(relay_qos) \
    ((uint8_t)(((relay_qos) & TAPESTRY_RELAY_QOS_QOS_MASK) >> TAPESTRY_RELAY_QOS_QOS_SHIFT))

#define TAPESTRY_PACK_RELAY_QOS(hop_count, qos_tier) \
    ((uint8_t)(((hop_count) & TAPESTRY_RELAY_QOS_HOP_MASK) | \
               (((qos_tier) << TAPESTRY_RELAY_QOS_QOS_SHIFT) & TAPESTRY_RELAY_QOS_QOS_MASK)))

/* Full on-wire size: frame + optional HMAC auth tag */
#define TAPESTRY_GOSSIP_WIRE_SIZE    \
    ((uint16_t)(TAPESTRY_GOSSIP_FRAME_SIZE + TAPESTRY_WIRE_AUTH_TAG_SIZE))

/* ── L4 metric frame ─────────────────────────────────────────────────────── */
/*
 * Sent by each element to the telemetry collector every cycle.
 * mean_position_error is zero when sent by elements; the orchestrator
 * fills it in by comparing believed positions against ground truth.
 *
 * Python format: struct.Struct('<BBBBBBfBBfIffH')
 * Size: 30 bytes
 */
typedef struct {
    uint8_t  element_id;
    uint8_t  active_total;
    uint8_t  active_fresh;
    uint8_t  active_stale;
    uint8_t  inactive_total;
    uint8_t  collision_count;
    float    fresh_ratio;
    uint8_t  quorum_held;
    uint8_t  degraded;
    float    confidence;
    uint32_t cycle_count;
    float    mean_age_ms;
    float    mean_position_error;   /* filled by orchestrator */
    uint16_t min_separation_x100;  /* min peer separation * 100; 0xFFFF = no peers */
} __attribute__((packed)) tapestry_metric_frame_t;

#define TAPESTRY_METRIC_FRAME_SIZE   ((uint16_t)sizeof(tapestry_metric_frame_t))   /* 30 */

/* ── L5 SCR metric frame ─────────────────────────────────────────────────── */
/*
 * Carries one element's SCR role/quorum snapshot to the telemetry collector.
 *
 * Python format: struct.Struct('<BBBBBBI')
 * Size: 10 bytes
 * Fields: element_id, role, leader_id, quorum_state, fresh_count,
 *         task_slot, election_count
 */
typedef struct {
    uint8_t  element_id;
    uint8_t  role;           /* scr_role_t cast to uint8_t                    */
    uint8_t  leader_id;      /* elected leader ID; ELEMENT_ID_INVALID if LOST */
    uint8_t  quorum_state;   /* scr_quorum_state_t cast to uint8_t            */
    uint8_t  fresh_count;    /* non-self trusted fresh peers this tick        */
    uint8_t  task_slot;      /* ordinal in sorted peer list (0 = leader)      */
    uint32_t election_count; /* cumulative leader changes since element start  */
} __attribute__((packed)) tapestry_scr_metric_frame_t;

#define TAPESTRY_SCR_METRIC_FRAME_SIZE   10   /* sizeof(tapestry_scr_metric_frame_t) */

/* ── Worst-case receive buffer size ──────────────────────────────────────── */
/* Metric frame (30 B) > gossip wire frame (22+4 = 26 B), so metric wins.   */

#define TAPESTRY_MAX_MSG_SIZE   (TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_METRIC_FRAME_SIZE)   /* 35 */

#endif /* TAPESTRY_WIRE_H */
