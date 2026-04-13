/*
 * scr_protocol.h — Wire protocol additions for the L5 SCR simulation.
 *
 * Extends tapestry-csm-sim/sim_protocol.h (included here) with one new
 * message type and payload struct for SCR state snapshots.
 *
 * Message flow (per cycle, element → orchestrator):
 *   SIM_MSG_METRIC      (type 2) — L4 CSM metric    [sim_protocol.h]
 *   SIM_MSG_SCR_METRIC  (type 4) — L5 SCR metric    [this file]
 *
 * The orchestrator buffers both for a given element_id and emits one
 * combined telemetry row when both arrive.
 *
 *   sim_scr_metric_payload_t layout
 *   ────────────────────────────────
 *   offset  field           type     description
 *      0    element_id      uint8    sender element ID
 *      1    role            uint8    scr_role_t  (0=NONE, 1=FOLLOWER, 2=LEADER)
 *      2    leader_id       uint8    elected leader; 0xFF = ELEMENT_ID_INVALID
 *      3    quorum_state    uint8    0=LOST, 1=DEGRADED, 2=HEALTHY
 *      4    fresh_count     uint8    non-self fresh peers this tick
 *      5    _reserved       uint8    padding (zero)
 *      6    election_count  uint32   cumulative leader changes since startup
 *   Total: 10 bytes.  Python format: struct.Struct('<BBBBBBI')
 *          Note: 'I' is unsigned 32-bit int in Python's struct module.
 */

#ifndef TAPESTRY_SCR_PROTOCOL_H
#define TAPESTRY_SCR_PROTOCOL_H

#include "../tapestry-csm-sim/sim_protocol.h"

/* ── New message type ────────────────────────────────────────────────────── */
/*
 * SIM_MSG_SCR_METRIC extends the sim_msg_type_t enum from sim_protocol.h.
 * Defined as a macro to avoid redefining the enum.
 */
#define SIM_MSG_SCR_METRIC  4

/* ── SCR metric payload ──────────────────────────────────────────────────── */

typedef struct {
    uint8_t  element_id;
    uint8_t  role;           /* scr_role_t cast to uint8_t                   */
    uint8_t  leader_id;      /* elected leader ID; ELEMENT_ID_INVALID if LOST*/
    uint8_t  quorum_state;   /* scr_quorum_state_t cast to uint8_t           */
    uint8_t  fresh_count;    /* non-self fresh peers this tick               */
    uint8_t  _reserved;      /* pad to even boundary; must be zero           */
    uint32_t election_count; /* cumulative leader changes since element start */
} __attribute__((packed)) sim_scr_metric_payload_t;

#define SIM_SCR_METRIC_SIZE  10   /* sizeof(sim_scr_metric_payload_t) */

#endif /* TAPESTRY_SCR_PROTOCOL_H */
