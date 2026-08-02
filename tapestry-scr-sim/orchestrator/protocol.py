"""
protocol.py — Python mirror of wire.h + sim_protocol.h + scr_protocol.h

All struct formats are little-endian ('<') to match the C packed structs.
The wire.h-derived section below is GENERATED — see its own marker
comments.  MSG_CONTROL and CTRL_FMT come from sim_protocol.h (sim-only,
not part of wire.h) and remain hand-maintained; update them here whenever
sim_protocol.h changes.
"""

import logging
import struct

log = logging.getLogger(__name__)

# ── Ports ─────────────────────────────────────────────────────────────────────

ORCH_PORT         = 5100
ELEMENT_BASE_PORT = 5000
LOOPBACK          = "127.0.0.1"

# === BEGIN GENERATED WIRE PROTOCOL (tools/gen_wire_protocol.py — DO NOT EDIT) ===
# Mirrors tapestry-os/include/tapestry/wire.h.
# Regenerate after any wire.h change:
#   python3 tapestry-os/tools/gen_wire_protocol.py
#
# WIRE_VERSION bumps whenever a struct format below changes;
# decode() rejects a header whose version does not match —
# see wire.h's "Wire schema version" section for why.

WIRE_VERSION = 1

MSG_GOSSIP     = 1
MSG_METRIC     = 2
MSG_SCR_METRIC = 4

HEADER_FMT     = struct.Struct('<BBBH')  #  5 bytes: version,type,src_id,payload_len
GOSSIP_FMT     = struct.Struct('<BffIIBBBB')  # 21 bytes: id,x,y,logical_clock,update_seq,energy_level,health_flags,hop_count,version
METRIC_FMT     = struct.Struct('<BBBBBBfBBfIffH')  # 30 bytes: element_id,active_total,active_fresh,active_stale,inactive_total,collision_count,fresh_ratio,quorum_held,degraded,confidence,cycle_count,mean_age_ms,mean_position_error,min_separation_x100
SCR_METRIC_FMT = struct.Struct('<BBBBBBI')  # 10 bytes: element_id,role,leader_id,quorum_state,fresh_count,task_slot,election_count
# === END GENERATED WIRE PROTOCOL ===

# ── Sim-only extensions (sim_protocol.h — not part of wire.h) ─────────────────

MSG_CONTROL = 3

CTRL_SET_PARTITION = 1
CTRL_SET_POWER     = 2
CTRL_SHUTDOWN      = 3

CTRL_FMT = struct.Struct('<BB')   # ctrl_type, value — 2 bytes

ELEMENT_ID_INVALID = 0xFF

# ── Encode ────────────────────────────────────────────────────────────────────

def encode_gossip(state: dict) -> bytes:
    header  = HEADER_FMT.pack(WIRE_VERSION, MSG_GOSSIP, state['id'], GOSSIP_FMT.size)
    payload = GOSSIP_FMT.pack(
        state['id'],
        state['x'],
        state['y'],
        state['logical_clock'],
        state['update_seq'],
        state.get('energy_level', 100),
        state.get('health_flags', 0),
        state.get('hop_count', 0),
        WIRE_VERSION,
    )
    return header + payload


def encode_control(src_id: int, ctrl_type: int, value: int) -> bytes:
    header  = HEADER_FMT.pack(WIRE_VERSION, MSG_CONTROL, src_id, CTRL_FMT.size)
    payload = CTRL_FMT.pack(ctrl_type, value)
    return header + payload

# ── Decode ────────────────────────────────────────────────────────────────────

def decode(data: bytes) -> dict | None:
    """
    Parse a raw UDP datagram into a typed dict.
    Returns None if the datagram is malformed or the type is unknown.

    Result 'type' values: 'gossip', 'metric', 'scr_metric'

    scr_metric keys:
        type, src_id, element_id, role, leader_id, quorum_state,
        fresh_count, election_count
    """
    if len(data) < HEADER_FMT.size:
        return None

    version, msg_type, src_id, payload_len = HEADER_FMT.unpack_from(data)
    payload = data[HEADER_FMT.size:]

    if version != WIRE_VERSION:
        log.warning("wire version mismatch: src=%d wire=%d (expected %d)",
                    src_id, version, WIRE_VERSION)
        return None

    if len(payload) < payload_len:
        return None

    if msg_type == MSG_GOSSIP and len(payload) >= GOSSIP_FMT.size:
        id_, x, y, clock, seq, energy, health, hop, frame_version = \
            GOSSIP_FMT.unpack_from(payload)
        # Checked here too, not just the header above: BLE and syslink P2P
        # carry this frame with no header wrapper at all on real hardware,
        # so a frame-level check is what actually protects those transports
        # (see wire.h's "Wire schema version" section). Redundant with the
        # header check for UDP specifically, which is fine.
        if frame_version != WIRE_VERSION:
            log.warning("gossip frame version mismatch: id=%d wire=%d "
                        "(expected %d)", id_, frame_version, WIRE_VERSION)
            return None
        return {
            'type':          'gossip',
            'src_id':        src_id,
            'id':            id_,
            'x':             x,
            'y':             y,
            'logical_clock': clock,
            'update_seq':    seq,
            'energy_level':  energy,
            'health_flags':  health,
            'hop_count':     hop,
        }

    if msg_type == MSG_METRIC and len(payload) >= METRIC_FMT.size:
        eid, at, af, ast, it, cc, ratio, qh, deg, conf, cycle, \
            mean_age, mean_pos_err, min_sep_x100 = \
            METRIC_FMT.unpack_from(payload)
        min_sep = None if min_sep_x100 == 0xFFFF else min_sep_x100 / 100.0
        return {
            'type':                'metric',
            'src_id':              src_id,
            'element_id':          eid,
            'active_total':        at,
            'active_fresh':        af,
            'active_stale':        ast,
            'inactive_total':      it,
            'collision_count':     cc,
            'fresh_ratio':         ratio,
            'quorum_held':         bool(qh),
            'degraded':            bool(deg),
            'confidence':          conf,
            'cycle_count':         cycle,
            'mean_age_ms':         mean_age,
            'mean_position_error': mean_pos_err,
            'min_separation':      min_sep,
        }

    if msg_type == MSG_SCR_METRIC and len(payload) >= SCR_METRIC_FMT.size:
        eid, role, leader, qstate, fresh, task_slot, elec = \
            SCR_METRIC_FMT.unpack_from(payload)
        return {
            'type':            'scr_metric',
            'src_id':          src_id,
            'element_id':      eid,
            'role':            role,       # 0=NONE,1=FOLLOWER,2=LEADER,3=RELAY,4=SENSOR,5=ACTUATOR
            'leader_id':       leader,     # ELEMENT_ID_INVALID (0xFF) = no leader
            'quorum_state':    qstate,     # 0=LOST, 1=DEGRADED, 2=HEALTHY
            'fresh_count':     fresh,
            'task_slot':       task_slot,  # ordinal in sorted peer list (0 = leader)
            'election_count':  elec,
        }

    return None
