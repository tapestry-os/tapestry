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
 *   5  TAPESTRY_MSG_DIRECTIVE    — BSE host → element (remote L6, v5)
 *   6  TAPESTRY_MSG_FACT         — element ↔ element shared facts (L4, v7)
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
 * mattered because the (legacy) BLE advertising payload had zero spare
 * bytes.
 *
 * v3: tapestry_gossip_frame_t gained z (altitude) and a unit-quaternion
 * orientation (qw/qx/qy/qz) — full 6DoF pose, not just 2D position.  This
 * grows the frame from 22 to 42 bytes, which no longer fits the legacy
 * BLE advertising payload (29 usable bytes) at all — transceiver_ble.c now
 * requires CONFIG_BT_EXT_ADV (LE Extended Advertising, Bluetooth 5.0+;
 * see that file's header comment for which board(s) this drops BLE
 * support on and why).
 *
 * v4: tapestry_gossip_frame_t gains current_track (Choreo SDK Design doc
 * §7 tracks) — which track (by index into the shared track table every
 * element holds identically) this element is currently active in.  0 for
 * every element on a script with no tracks (today's only case) — a no-op
 * value, not a behavior change.  Grows the frame 42 -> 43 bytes.  A new
 * field, not a repack of relay_qos's spare bits, on purpose: relay_qos is
 * an L3 (transport) concept — hop_count, qos_tier — while current_track
 * is L6/L7 state, same layer as `achieved` next to it; conflating the two
 * in one byte would blur that line for no wire-budget reason (200-byte
 * single-PDU ceiling since BLE5 Extended Advertising, 43 of 200 bytes
 * used — no longer the tight fit v3's bump was solving for).
 *
 * v5: adds tapestry_directive_frame_t (TAPESTRY_MSG_DIRECTIVE) — a remote
 * L6 BSE host commanding an element's per-tick directive over the wire.
 * No existing frame's layout changed, so this bump is not guarding a
 * translation error; it exists so a fleet is uniformly directive-aware or
 * uniformly not.  A v4 element flying under a v5 BSE host would silently
 * never see a directive and run its local BSE forever — behaviorally safe
 * but operationally confusing; rejecting the whole mix at the gossip layer
 * makes the mismatch visible at deploy time instead.
 *
 * v6: tapestry_gossip_frame_t gains `discovered` (the discovery feature's
 * TAPESTRY_BSE_ANCHOR_DISCOVERER / CHOREO_EVENT_DISCOVERY — an element's
 * own-detected "found the target" bit, same shape as `achieved` next to
 * it: 0/1, gossiped so peers can resolve an anchor or a collective
 * discovery event from wm alone), and `health_flags` widens uint8_t ->
 * uint16_t (the original 8 bits were fully committed — 6 individual
 * ELEMENT_HEALTH_* flags plus the 2-bit departure-reason subfield, zero
 * spare — so a bit that reused it the way NO_POSITION/POSITION_STALE/
 * DEPARTED each did before, additively and without a version bump, is no
 * longer available). Also adds four zero-filled `reserved0`..`reserved3`
 * bytes immediately before `version`, so the NEXT small additive field
 * can claim one without moving `version`'s offset — i.e. without needing
 * its own version bump, the same "older receivers just don't know about it"
 * tolerance this field's own past extensions relied on. Frame grows
 * 43 -> 49 bytes, still far under the 200-byte BLE5 Extended Advertising
 * ceiling (v3's comment above) — bumped because `version` must stay the
 * last field (see its own doc below), so any new field is inserted
 * before it and necessarily moves its offset; not guarding a translation
 * error the way v2/v3 were, same as v4/v5.
 *
 * v7: adds tapestry_fact_frame_t (TAPESTRY_MSG_FACT) — L4 shared facts
 * (csm.h), a replicated table of facts about the world re-sent on a
 * Trickle schedule — and removes `discovered` from the gossip frame
 * (49 -> 48 bytes): what a finder found is now a fact every element holds
 * (and a rebooted element relearns from any peer), not a per-element bit
 * whose meaning depended on where its sender happened to be.  The bit
 * stays in element_state_t as local-only state.
 */
#define TAPESTRY_WIRE_VERSION   7u

/* ── Message types ───────────────────────────────────────────────────────── */

typedef enum {
    TAPESTRY_MSG_GOSSIP     = 1,
    TAPESTRY_MSG_METRIC     = 2,
    /* 3: sim-only control — never transmitted by hardware elements */
    TAPESTRY_MSG_SCR_METRIC = 4,
    TAPESTRY_MSG_DIRECTIVE  = 5,
    TAPESTRY_MSG_FACT       = 6,
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
 * Python format: struct.Struct('<BfffffffIIBHBBBBBBBB')
 * Size: 48 bytes
 * Fields: id, x, y, z, qw, qx, qy, qz, logical_clock, update_seq,
 *         energy_level, health_flags, relay_qos, achieved,
 *         current_track, reserved0, reserved1, reserved2, reserved3,
 *         version
 *
 * x, y, z: position, meters (or the abstract [0,100] sim-world unit on
 *   platforms that use that convention — see csm.h's WORLD_SIZE).
 *
 * qw, qx, qy, qz: unit quaternion, w-first (orientation_t in csm.h — see
 *   that type's comment for the required reference-frame convention:
 *   same world frame as x/y/z, ENU if that frame is geographically
 *   anchored, never a local/body/boot-relative frame). Elements with no
 *   attitude sensing gossip identity ({1,0,0,0}), not a zero-initialized
 *   {0,0,0,0} — the latter is not a valid rotation and would corrupt any
 *   consumer that assumes unit norm. gossip_send() always sends
 *   own_state->orientation as given; callers own picking a sensible
 *   default (element_state_t's owner is responsible for setting
 *   orientation to orientation_identity() if it has nothing better).
 *
 * health_flags: ELEMENT_HEALTH_* bitmask (csm.h). uint16_t as of v6 (was
 *   uint8_t through v5) — the 8-bit form was fully committed (6
 *   individual flag bits plus the 2-bit ELEMENT_DEPARTED_REASON_MASK
 *   subfield, zero spare), unlike NO_POSITION/POSITION_STALE/DEPARTED,
 *   which each landed in a still-spare bit of the 8-bit form with no
 *   version bump. The 8 new high bits are unused today — future
 *   ELEMENT_HEALTH_* flags land there the same additive, no-bump way.
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
 * current_track: this element's active track index (choreo_current_
 *   track(), v4).  0 for every element on a script with no [[tracks]] —
 *   the only case before this field existed, so it's a no-op default.
 *   Lets peers filter FORM/EXCHANGE/MOVE participant sets to only those
 *   peers helping the SAME collective activity, without re-deriving a
 *   peer's track membership from its (not fully gossiped) capabilities —
 *   see collect_participants() in bse.c.
 *
 * reserved0..reserved3: v6, always sent as 0 and never read by current
 *   code — pure future-proofing so the NEXT small additive field can
 *   claim one of these (rename it in the struct, regenerate) without
 *   moving `version`'s offset, i.e. without needing its own version
 *   bump. Four individually named bytes rather than a `reserved[4]`
 *   array — tools/gen_wire_protocol.py's field parser only understands
 *   "TYPE name;" lines, not array declarators, so a future rename is a
 *   one-line change here with no parser work, ever.
 *
 * version: TAPESTRY_WIRE_VERSION, carried IN the frame itself (not just the
 *   tapestry_msg_header_t wrapper) because BLE and syslink P2P advertise
 *   this frame directly with no header wrapper at all — see wire.h's "Wire
 *   schema version" section.  Stays the LAST field (not first, unlike the
 *   message header) so `id` stays the frame's first byte, which
 *   transceiver_udp.c relies on when extracting src_id before the header
 *   is populated; new fields (including reservedN's own eventual reuse)
 *   are inserted before it, never after.
 *
 * When CONFIG_TAPESTRY_WIRE_AUTH_ENABLED is set, TAPESTRY_WIRE_AUTH_TAG_SIZE
 * additional bytes follow the frame on the wire (not counted here).
 */
typedef struct {
    uint8_t  id;
    float    x;
    float    y;
    float    z;
    float    qw;
    float    qx;
    float    qy;
    float    qz;
    uint32_t logical_clock;
    uint32_t update_seq;
    uint8_t  energy_level;         /* Battery/power [0=empty, 100=full]       */
    uint16_t health_flags;         /* ELEMENT_HEALTH_* bitmask (v6: u16, was  */
                                    /* u8 through v5 — see csm.h)              */
    uint8_t  relay_qos;            /* hop_count[1:0] | qos_tier[3:2] — packed */
    uint8_t  achieved;             /* own-goal achievement predicate, 0/1     */
    uint8_t  current_track;        /* active track index (v4, choreo.h §7)    */
    /* Always 0, unread by any code today — future fields (v6). Four
     * individually named bytes, not uint8_t reserved[4]: tools/
     * gen_wire_protocol.py's field parser only understands "TYPE name;"
     * lines, not array declarators, so this shape is what lets the next
     * addition just rename one (e.g. reserved1 -> new_field) and
     * regenerate, with no parser changes anywhere. */
    uint8_t  reserved0;
    uint8_t  reserved1;
    uint8_t  reserved2;
    uint8_t  reserved3;
    uint8_t  version;              /* TAPESTRY_WIRE_VERSION — see above       */
} __attribute__((packed)) tapestry_gossip_frame_t;

#define TAPESTRY_GOSSIP_FRAME_SIZE   ((uint16_t)sizeof(tapestry_gossip_frame_t))   /* 48 */

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

/* ── Directive frame (v5) ────────────────────────────────────────────────── */
/*
 * Carries one per-element behavioral directive from a remote L6 BSE host
 * (edge node, or an elected SCR_CAP_BSE_HOST element) to an element.  The
 * element treats it as a REFINEMENT of its locally computed directive, not
 * a replacement for its script: choreo.c only steers by a remote directive
 * while remote directives stay fresh, and falls back to the local
 * open-core BSE the moment they go stale — see choreo.h's remote-directive
 * section for the staleness/re-adoption constants and the degraded-mode
 * ladder.
 *
 * Python format: struct.Struct('<BBBfffffHIB')
 * Size: 30 bytes
 * Fields: src_id, target_id, type, x, y, z, spring_k, spacing, goal_id,
 *         seq, version
 *
 * src_id: the BSE host's element id.  First byte of the frame, matching
 *   the gossip frame's id-first convention.
 *
 * target_id: the element this directive addresses, or
 *   TAPESTRY_DIRECTIVE_TARGET_ALL (0xFF, == ELEMENT_ID_INVALID — an id no
 *   element can hold) to address every element.  Receivers drop frames
 *   addressed to another element before any further processing.
 *
 * type: tapestry_bse_directive_type_t (bse.h) cast to uint8_t.  Receivers
 *   drop frames whose type is not one of the four defined directives —
 *   a directive vocabulary mismatch must fail closed, not steer.
 *
 * x/y/z: MOVE_TO_POINT target (same units as gossip positions).
 * spring_k/spacing: MAINTAIN_SPRING parameters.  Unused fields are 0.
 *
 * goal_id: the L7 goal this directive serves (choreo_goal_t::id), echoed
 *   so an element can attribute a directive to a goal in telemetry.
 *   Opaque to the receiver's steering logic.
 *
 * seq: strictly monotonic per src_id.  Receivers accept a frame only if
 *   its seq is strictly greater than the last accepted seq from that src
 *   (first frame from a src is always accepted), which makes a replayed
 *   authenticated frame inert.  SENDER REQUIREMENT: seq must be monotonic
 *   across host restarts (e.g. derived from epoch milliseconds), because
 *   receivers only reset their per-src tracking on their own reboot.
 *
 * version: TAPESTRY_WIRE_VERSION, last field — same rationale as the
 *   gossip frame (media that advertise frames with no header wrapper).
 *
 * There is deliberately NO timestamp field: elements share no wall clock,
 * so staleness is measured by the receiver from local arrival time
 * (choreo.c's CHOREO_REMOTE_STALE_MS), never from sender-side time.
 *
 * When CONFIG_TAPESTRY_WIRE_AUTH_ENABLED is set, TAPESTRY_WIRE_AUTH_TAG_SIZE
 * additional bytes follow the frame on the wire (not counted here).
 * SAFETY: an unauthenticated directive path must never fly on real
 * hardware — directive injection is a full hijack of the collective's
 * steering.  Builds without wire auth are for simulation only.
 */
typedef struct {
    uint8_t  src_id;        /* BSE host element id                       */
    uint8_t  target_id;     /* addressee, or TAPESTRY_DIRECTIVE_TARGET_ALL */
    uint8_t  type;          /* tapestry_bse_directive_type_t             */
    float    x;             /* MOVE_TO_POINT target                      */
    float    y;
    float    z;
    float    spring_k;      /* MAINTAIN_SPRING stiffness                 */
    float    spacing;       /* MAINTAIN_SPRING target distance           */
    uint16_t goal_id;       /* L7 goal this directive serves (opaque)    */
    uint32_t seq;           /* strictly monotonic per src_id — see above */
    uint8_t  version;       /* TAPESTRY_WIRE_VERSION — last, like gossip */
} __attribute__((packed)) tapestry_directive_frame_t;

#define TAPESTRY_DIRECTIVE_FRAME_SIZE  ((uint16_t)sizeof(tapestry_directive_frame_t))  /* 30 */

#define TAPESTRY_DIRECTIVE_TARGET_ALL  0xFFu   /* == ELEMENT_ID_INVALID */

/* Highest valid `type` value — tracks tapestry_bse_directive_type_t
 * (bse.h: IDLE=0 … MAINTAIN_SPRING=3) without L3 having to include an L6
 * header.  Receivers drop frames whose type exceeds this (fail closed). */
#define TAPESTRY_DIRECTIVE_TYPE_MAX    3u

/* Full on-wire size: frame + optional HMAC auth tag */
#define TAPESTRY_DIRECTIVE_WIRE_SIZE   \
    ((uint16_t)(TAPESTRY_DIRECTIVE_FRAME_SIZE + TAPESTRY_WIRE_AUTH_TAG_SIZE))

/* ── Fact frame (v7) ─────────────────────────────────────────────────────── */
/*
 * Carries up to two L4 shared-fact records (csm.h's fact_t) plus a digest
 * of the sender's whole fact table.  Sent on a Trickle schedule
 * (wm_fact_trickle_tick()), separate from the 500 ms state gossip; on BLE
 * it rides its own advertising set, so the state set's payload never
 * alternates.  A frame with count 0 is a digest-only beacon: a freshly
 * booted element's empty table announces itself so its peers notice the
 * mismatch and re-send at FACT_IMIN_MS.
 *
 * Python format: struct.Struct('<BBHBBHBBIfffHBBHBBIfffHB')
 * Size: 53 bytes, 57 with the auth tag.  The smallest transport budget is
 *   the cutebot BLE build's CONFIG_BT_CTLR_ADV_DATA_LEN_MAX=64: less 2 AD
 *   header and 3 company/type bytes leaves 59, so frame + tag fit with 2
 *   bytes to spare.  Syslink P2P (64-byte MTU, 1 port byte) has 63.
 *
 * sender_id: first byte, as in every frame.
 * count: valid records, 0..2; unused record bytes are zero.
 * digest: wm_fact_digest() of the sender's table AFTER this frame's records
 *   were packed — what receivers compare with their own (wm_fact_heard_
 *   digest()).
 * rN_*: fact_t fields in declaration order — type, origin, epoch, seq,
 *   flags, stamp, x, y, z, attr (24 bytes a record).  Flattened rather
 *   than a nested record struct because tools/gen_wire_protocol.py parses
 *   only "TYPE name;" lines.
 * version: TAPESTRY_WIRE_VERSION, last field — same rationale as the
 *   gossip frame.
 *
 * When CONFIG_TAPESTRY_WIRE_AUTH_ENABLED is set, TAPESTRY_WIRE_AUTH_TAG_SIZE
 * additional bytes follow the frame on the wire (not counted here).
 */
typedef struct {
    uint8_t  sender_id;
    uint8_t  count;
    uint16_t digest;
    uint8_t  r0_type;
    uint8_t  r0_origin;
    uint16_t r0_epoch;
    uint8_t  r0_seq;
    uint8_t  r0_flags;
    uint32_t r0_stamp;
    float    r0_x;
    float    r0_y;
    float    r0_z;
    uint16_t r0_attr;
    uint8_t  r1_type;
    uint8_t  r1_origin;
    uint16_t r1_epoch;
    uint8_t  r1_seq;
    uint8_t  r1_flags;
    uint32_t r1_stamp;
    float    r1_x;
    float    r1_y;
    float    r1_z;
    uint16_t r1_attr;
    uint8_t  version;
} __attribute__((packed)) tapestry_fact_frame_t;

#define TAPESTRY_FACT_FRAME_SIZE  ((uint16_t)sizeof(tapestry_fact_frame_t))  /* 53 */
#define TAPESTRY_FACT_RECORD_SIZE 24u
#define TAPESTRY_FACT_RECORDS_MAX 2u

/* Full on-wire size: frame + optional HMAC auth tag */
#define TAPESTRY_FACT_WIRE_SIZE   \
    ((uint16_t)(TAPESTRY_FACT_FRAME_SIZE + TAPESTRY_WIRE_AUTH_TAG_SIZE))

/* ── Worst-case receive buffer size ──────────────────────────────────────── */
/*
 * Actual max of the bodies that ever ride behind tapestry_msg_header_t
 * (TAPESTRY_SCR_METRIC_FRAME_SIZE and TAPESTRY_DIRECTIVE_WIRE_SIZE are
 * smaller and never dominate; the v7 fact frame is the largest today) — not a hardcoded assumption
 * about which one wins.  It used
 * to be hardcoded ("metric frame > gossip wire frame, so metric wins"),
 * which silently stopped being true once tapestry_gossip_frame_t grew
 * past the metric frame's 30 bytes (v3: z + orientation) and would have
 * left every RX buffer sized off this macro undersized.
 */
#define TAPESTRY_MAX_BODY_SIZE_2 \
    (TAPESTRY_GOSSIP_WIRE_SIZE > TAPESTRY_METRIC_FRAME_SIZE \
     ? TAPESTRY_GOSSIP_WIRE_SIZE : TAPESTRY_METRIC_FRAME_SIZE)
#define TAPESTRY_MAX_BODY_SIZE \
    (TAPESTRY_FACT_WIRE_SIZE > TAPESTRY_MAX_BODY_SIZE_2 \
     ? TAPESTRY_FACT_WIRE_SIZE : TAPESTRY_MAX_BODY_SIZE_2)

#define TAPESTRY_MAX_MSG_SIZE \
    (TAPESTRY_MSG_HEADER_SIZE + TAPESTRY_MAX_BODY_SIZE)   /* 58 (62 with auth) */

#endif /* TAPESTRY_WIRE_H */
