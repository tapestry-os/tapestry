/*
 * status_tx.h — per-element status feed for the warehouse supervisor
 *
 * WHY: without this, a Webots window full of AMRs shows you robots driving
 * around and nothing else. Whether a robot currently holds quorum, which
 * zone it believes it owns, how many peers it can hear, and which script
 * step it is on are all invisible — and those are the only things this
 * example is actually demonstrating. The supervisor renders them as
 * on-screen labels; this is the channel it reads.
 *
 * DELIBERATELY NOT PART OF TAPESTRY. This is presentation scaffolding, not
 * a layer. It is a one-way, best-effort, fire-and-forget datagram to a
 * single local listener; nothing in L3-L7 reads it, no element reads
 * another element's status, and no behavior anywhere depends on it. If the
 * supervisor is not running, every send is silently discarded and the
 * demonstration is unaffected. It is emphatically NOT a back channel that
 * quietly gives the collective a shared view it would not otherwise have —
 * the whole claim of these scenes is that coordination comes from gossip
 * alone, and that claim would be worthless if this existed inside it.
 *
 * NOT A WIRE PROTOCOL. tapestry/wire.h is the wire format, versioned and
 * endian-defined. This struct is passed between sibling processes on one
 * machine with one compiler, so it is sent as raw bytes with no packing,
 * versioning, or byte-order handling. Do not copy this pattern onto
 * anything that leaves the host.
 */

#ifndef TAPESTRY_WAREHOUSE_STATUS_TX_H
#define TAPESTRY_WAREHOUSE_STATUS_TX_H

#include <stdbool.h>
#include <stdint.h>

/* Added to the gossip base port for the supervisor's listening port.
 * Chosen clear of both the gossip range (base + element_id) and the
 * directive range (base + 1000 + element_id). */
#define WH_STATUS_PORT_OFFSET  2000u

#define WH_STATUS_MAGIC  0x7A

/*
 * wh_status_t::fleet — which coordination scheme sent this report. Added for
 * scenes 2/3 (Tapestry vs. a centralized "cloud" coordinator, run
 * side-by-side on the same floor under the same fault — see
 * controllers/cloud_bot/README in that directory's header comments).
 *
 * WH_FLEET_TAPESTRY is zero deliberately: every existing wh_status_t
 * construction in rover/main.c zero-initializes with `= {0}` and never sets
 * `.fleet` explicitly, so this extension requires no change to that file at
 * all — old code is correct by construction under the new schema.
 */
#define WH_FLEET_TAPESTRY  0u
#define WH_FLEET_CLOUD     1u

/*
 * Reserved element_id for the cloud coordinator's own heartbeat report — not
 * a fleet member, so it must never collide with a real robot id and must
 * never be used to index a per-robot tracking table. Tapestry robots use
 * ids 0-7; cloud robots use ids 16-23 (see worlds/scene2_partition_vs_cloud
 * .wbt); 255 is clear of both with room to grow either fleet.
 */
#define WH_COORDINATOR_ID  255u

/* Bit flags in wh_status_t::flags. */
#define WH_STATUS_F_SUSPENDED   0x01u  /* choreo lifecycle == SUSPENDED     */
#define WH_STATUS_F_CP_FROZEN   0x02u  /* CP-mode consistency freeze active */
#define WH_STATUS_F_FAILED      0x04u  /* injected failure has fired        */
#define WH_STATUS_F_RF_BLOCKED  0x08u  /* at least one link is obstructed   */
/*
 * WH_STATUS_F_NO_COORD — a cloud robot has gone longer than its heartbeat
 * timeout without a fresh, unobstructed directive from its coordinator, and
 * has frozen (zero commanded velocity) as a result. Deliberately a
 * SEPARATE bit from WH_STATUS_F_SUSPENDED rather than reusing it: a
 * Tapestry SUSPENDED goal is preserved and resumes automatically once
 * quorum recovers (choreo.h's CHOREO_STATE_SUSPENDED contract); a cloud
 * robot's freeze has no such recovery path — nothing on the robot itself
 * can ever un-freeze it, only a resumed coordinator broadcast can, and in
 * scene 3 the coordinator never resumes. Conflating the two bits would
 * misrepresent a permanent stall as a temporary pause.
 */
#define WH_STATUS_F_NO_COORD    0x10u

typedef struct {
    uint8_t  magic;         /* WH_STATUS_MAGIC — rejects stray datagrams    */
    uint8_t  element_id;    /* WH_COORDINATOR_ID for a coordinator heartbeat */
    uint8_t  scene;
    uint8_t  fleet;         /* WH_FLEET_* */
    uint8_t  quorum;        /* scr_quorum_state_t — TAPESTRY fleet only;     */
                            /* 0xFF ("n/a") on a CLOUD report: a hub-and-    */
                            /* spoke robot has no quorum concept to report   */
    uint8_t  role;          /* scr_role_t — TAPESTRY fleet only, else 0xFF   */
    uint8_t  swarm_size;    /* scr_get_swarm_size() — TAPESTRY only, else 0xFF */
    uint8_t  task_slot;     /* scr_get_task_slot() — TAPESTRY only, else 0xFF */
    uint8_t  fresh_peers;   /* non-stale peers — TAPESTRY only, else 0xFF    */
    uint8_t  blocked_peers; /* links this element's RF model is refusing     */
    uint8_t  step;          /* choreo_script_step() — TAPESTRY only, else 0xFF */
    uint8_t  flags;         /* WH_STATUS_F_*                                 */
    uint8_t  _pad;
    float    x, y;          /* world-frame position, meters                  */
} wh_status_t;

/* Open the sending socket. base_port must match the gossip base port.
 * Failure is non-fatal and silent thereafter. */
void status_tx_init(uint16_t base_port);

/* Fire-and-forget. Never blocks, never reports failure. */
void status_tx_send(const wh_status_t *st);

#endif /* TAPESTRY_WAREHOUSE_STATUS_TX_H */
