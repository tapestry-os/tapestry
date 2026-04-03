/*
 * sim_protocol.h — Tapestry CSM Simulation Wire Protocol
 *
 * Shared between the Zephyr element app (C) and the Python orchestrator.
 * All wire structs are packed (__attribute__((packed))) — no padding.
 * Python's struct module mirrors each layout using little-endian ('<')
 * format strings documented in each struct's comment block.
 *
 * Port assignments
 * ─────────────────
 *   SIM_ORCH_PORT           5100   orchestrator listens (all element→orch traffic)
 *   SIM_ELEMENT_BASE_PORT   5000   element N listens on 5000+N (orch→element)
 *
 * Message flow
 * ─────────────
 *   element ──GOSSIP──►  orchestrator ──GOSSIP──► elements in same island
 *   element ──METRIC──►  orchestrator (written to telemetry CSV)
 *   orchestrator ──CONTROL──► element  (partition / power / shutdown)
 */

#ifndef SIM_PROTOCOL_H
#define SIM_PROTOCOL_H

#include <stdint.h>

/* ── Port constants ──────────────────────────────────────────────────────── */

#define SIM_ORCH_PORT           5100
#define SIM_ELEMENT_BASE_PORT   5000
#define SIM_LOOPBACK_ADDR       "127.0.0.1"

/* ── Message types ───────────────────────────────────────────────────────── */

typedef enum {
    SIM_MSG_GOSSIP  = 1,   /* element → orch → element(s)  */
    SIM_MSG_METRIC  = 2,   /* element → orch               */
    SIM_MSG_CONTROL = 3,   /* orch    → element            */
} sim_msg_type_t;

/* ── Message header ──────────────────────────────────────────────────────── */
/*
 * Python format: struct.Struct('<BBH')
 * Size: 4 bytes
 */

typedef struct {
    uint8_t  type;          /* sim_msg_type_t                */
    uint8_t  src_id;        /* sender element ID             */
    uint16_t payload_len;   /* bytes following this header   */
} __attribute__((packed)) sim_msg_header_t;

#define SIM_HEADER_SIZE  ((uint16_t)sizeof(sim_msg_header_t))   /* 4 */

/* ── Gossip payload ──────────────────────────────────────────────────────── */
/*
 * Python format: struct.Struct('<BffIBBI')
 * Size: 19 bytes
 * Fields: id, x, y, logical_clock, power_state, partition_island, update_seq
 */

typedef struct {
    uint8_t  id;
    float    x;
    float    y;
    uint32_t logical_clock;
    uint8_t  power_state;        /* power_state_t cast to uint8  */
    uint8_t  partition_island;
    uint32_t update_seq;
} __attribute__((packed)) sim_gossip_payload_t;

#define SIM_GOSSIP_SIZE  ((uint16_t)sizeof(sim_gossip_payload_t))   /* 19 */

/* ── Metric payload ──────────────────────────────────────────────────────── */
/*
 * Python format: struct.Struct('<BBBBBBfBBfIffH')
 * Size: 30 bytes
 * Fields: element_id, active_total, active_fresh, active_stale,
 *         inactive_total, collision_count, fresh_ratio,
 *         quorum_held, degraded, confidence, cycle_count,
 *         mean_age_ms, mean_position_error, min_separation_x100
 *
 * confidence           — proximity to quorum threshold [0.0, 1.0].
 *                        1.0 = at or above quorum; 0.0 = no fresh peers.
 *                        Always 1.0 when consistency_bias == 0.0 (pure AP).
 * degraded             — 1 when quorum lost (element below its threshold).
 * mean_age_ms          — average age_ms of active non-self entries (ms)
 * mean_position_error  — average |believed pos − actual pos| per active peer
 *                        computed by orchestrator and injected before logging;
 *                        element sends 0.0 and broker fills it in.
 * min_separation_x100  — minimum observed separation * 100 (0.01 unit res).
 *                        uint16 gives range 0..655.35 units, exceeds WORLD_SIZE.
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
    uint16_t min_separation_x100;
} __attribute__((packed)) sim_metric_payload_t;

#define SIM_METRIC_SIZE  ((uint16_t)sizeof(sim_metric_payload_t))   /* 30 */

/* ── Control payload ─────────────────────────────────────────────────────── */
/*
 * Python format: struct.Struct('<BB')
 * Size: 2 bytes
 */

typedef enum {
    SIM_CTRL_SET_PARTITION = 1,   /* value = new partition_island [0..255] */
    SIM_CTRL_SET_POWER     = 2,   /* value = new power_state_t             */
    SIM_CTRL_SHUTDOWN      = 3,   /* value = ignored                       */
} sim_ctrl_type_t;

typedef struct {
    uint8_t ctrl_type;   /* sim_ctrl_type_t */
    uint8_t value;
} __attribute__((packed)) sim_ctrl_payload_t;

#define SIM_CTRL_SIZE  ((uint16_t)sizeof(sim_ctrl_payload_t))   /* 2 */

/* ── Worst-case receive buffer ───────────────────────────────────────────── */

#define SIM_MAX_MSG_SIZE  (SIM_HEADER_SIZE + SIM_METRIC_SIZE)   /* 34 bytes */

#endif /* SIM_PROTOCOL_H */
