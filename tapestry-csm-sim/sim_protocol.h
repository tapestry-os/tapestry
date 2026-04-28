/*
 * sim_protocol.h — Tapestry simulation wire protocol extensions
 *
 * Simulation-specific additions on top of <tapestry/wire.h>: port
 * assignments, the loopback address, and the orchestrator→element
 * control message type.  Canonical gossip/metric frame types live in
 * wire.h and are used directly by element source files.
 *
 * Port assignments
 * ─────────────────
 *   SIM_ORCH_PORT           5100   orchestrator listens (element→orch traffic)
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

#include <tapestry/wire.h>

/* ── Simulation transport ────────────────────────────────────────────────── */

#define SIM_ORCH_PORT           5100
#define SIM_ELEMENT_BASE_PORT   5000
#define SIM_LOOPBACK_ADDR       "127.0.0.1"

/* ── Simulation-only message type: orchestrator → element control ─────────── */

typedef enum {
    SIM_MSG_GOSSIP  = TAPESTRY_MSG_GOSSIP,   /* 1: element ↔ element (via broker)    */
    SIM_MSG_METRIC  = TAPESTRY_MSG_METRIC,   /* 2: element → orchestrator            */
    SIM_MSG_CONTROL = 3,                      /* 3: orchestrator → element (sim-only) */
} sim_msg_type_t;

typedef enum {
    SIM_CTRL_SET_PARTITION = 1,   /* value = new partition_island [0..255] */
    SIM_CTRL_SET_POWER     = 2,   /* value = new power_state_t             */
    SIM_CTRL_SHUTDOWN      = 3,   /* value = ignored                       */
} sim_ctrl_type_t;

typedef struct {
    uint8_t ctrl_type;   /* sim_ctrl_type_t */
    uint8_t value;
} __attribute__((packed)) sim_ctrl_payload_t;

#define SIM_CTRL_SIZE   ((uint16_t)sizeof(sim_ctrl_payload_t))   /* 2 */

#endif /* SIM_PROTOCOL_H */
