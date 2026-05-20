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
 *     format strings documented in each struct's comment block.
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
 * Carried in tapestry_gossip_frame_t::qos_tier.  Transport backends may use
 * this to prioritise frames; at minimum the value is logged and forwarded.
 */

#define TAPESTRY_QOS_BEST_EFFORT  0u   /* Background telemetry                */
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

/* ── Message types ───────────────────────────────────────────────────────── */

typedef enum {
    TAPESTRY_MSG_GOSSIP     = 1,
    TAPESTRY_MSG_METRIC     = 2,
    /* 3: sim-only control — never transmitted by hardware elements */
    TAPESTRY_MSG_SCR_METRIC = 4,
} tapestry_msg_type_t;

/* ── Message header ──────────────────────────────────────────────────────── */
/*
 * Python format: struct.Struct('<BBH')
 * Size: 4 bytes
 */
typedef struct {
    uint8_t  type;          /* tapestry_msg_type_t                   */
    uint8_t  src_id;        /* sender element ID                     */
    uint16_t payload_len;   /* bytes following this header           */
} __attribute__((packed)) tapestry_msg_header_t;

#define TAPESTRY_MSG_HEADER_SIZE   ((uint16_t)sizeof(tapestry_msg_header_t))   /* 4 */

/* ── Gossip frame ────────────────────────────────────────────────────────── */
/*
 * Carries one element's authoritative state to all peers.
 * Sent every GOSSIP_INTERVAL_MS; received and fed into wm_receive_gossip().
 *
 * Python format: struct.Struct('<BffIBIBBB')
 * Size: 21 bytes
 * Fields: id, x, y, logical_clock, partition_island, update_seq,
 *         energy_level, health_flags, qos_tier
 *
 * When CONFIG_TAPESTRY_WIRE_AUTH_ENABLED is set, TAPESTRY_WIRE_AUTH_TAG_SIZE
 * additional bytes follow the frame on the wire (not counted here).
 */
typedef struct {
    uint8_t  id;
    float    x;
    float    y;
    uint32_t logical_clock;
    uint8_t  partition_island;     /* 0 on hardware; set by sim broker        */
    uint32_t update_seq;
    uint8_t  energy_level;         /* Battery/power [0=empty, 100=full]       */
    uint8_t  health_flags;         /* ELEMENT_HEALTH_* bitmask (see state.h)  */
    uint8_t  qos_tier;             /* TAPESTRY_QOS_* delivery priority        */
} __attribute__((packed)) tapestry_gossip_frame_t;

#define TAPESTRY_GOSSIP_FRAME_SIZE   ((uint16_t)sizeof(tapestry_gossip_frame_t))   /* 21 */

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
/* Metric frame (30 B) > gossip wire frame (21+4 = 25 B), so metric wins.   */

#define TAPESTRY_MAX_MSG_SIZE   (TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_METRIC_FRAME_SIZE)   /* 34 */

#endif /* TAPESTRY_WIRE_H */
