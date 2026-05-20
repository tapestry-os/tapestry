"""
protocol.py — Python mirror of sim_protocol.h + scr_protocol.h

All struct formats are little-endian ('<') to match the C packed structs.

Message types
─────────────
  MSG_GOSSIP      = 1   element → orch → element(s)
  MSG_METRIC      = 2   element → orch  (L4 CSM metric)
  MSG_CONTROL     = 3   orch → element
  MSG_SCR_METRIC  = 4   element → orch  (L5 SCR metric)

Struct sizes
────────────
  HEADER_FMT       '<BBH'             type, src_id, payload_len       —  4 bytes
  GOSSIP_FMT       '<BffIIBBB'        id,x,y,clock,seq,               — 20 bytes
                                       energy_level,health_flags,hop_count
  METRIC_FMT       '<BBBBBBfBBfIffH'  L4 CSM metric fields            — 30 bytes
  SCR_METRIC_FMT   '<BBBBBBI'         L5 SCR metric fields             — 10 bytes
  CTRL_FMT         '<BB'              ctrl_type, value                 —  2 bytes
"""

import struct

# ── Ports ─────────────────────────────────────────────────────────────────────

ORCH_PORT         = 5100
ELEMENT_BASE_PORT = 5000
LOOPBACK          = "127.0.0.1"

# ── Message types ─────────────────────────────────────────────────────────────

MSG_GOSSIP     = 1
MSG_METRIC     = 2
MSG_CONTROL    = 3
MSG_SCR_METRIC = 4

# ── Control subtypes ──────────────────────────────────────────────────────────

CTRL_SET_PARTITION = 1
CTRL_SET_POWER     = 2
CTRL_SHUTDOWN      = 3

# ── Struct formats ────────────────────────────────────────────────────────────

HEADER_FMT     = struct.Struct('<BBH')
GOSSIP_FMT     = struct.Struct('<BffIIBBB')
METRIC_FMT     = struct.Struct('<BBBBBBfBBfIffH')
SCR_METRIC_FMT = struct.Struct('<BBBBBBI')
CTRL_FMT       = struct.Struct('<BB')

ELEMENT_ID_INVALID = 0xFF

# ── Encode ────────────────────────────────────────────────────────────────────

def encode_gossip(state: dict) -> bytes:
    header  = HEADER_FMT.pack(MSG_GOSSIP, state['id'], GOSSIP_FMT.size)
    payload = GOSSIP_FMT.pack(
        state['id'],
        state['x'],
        state['y'],
        state['logical_clock'],
        state['update_seq'],
        state.get('energy_level', 100),
        state.get('health_flags', 0),
        state.get('hop_count', 0),
    )
    return header + payload


def encode_control(src_id: int, ctrl_type: int, value: int) -> bytes:
    header  = HEADER_FMT.pack(MSG_CONTROL, src_id, CTRL_FMT.size)
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

    msg_type, src_id, payload_len = HEADER_FMT.unpack_from(data)
    payload = data[HEADER_FMT.size:]

    if len(payload) < payload_len:
        return None

    if msg_type == MSG_GOSSIP and len(payload) >= GOSSIP_FMT.size:
        id_, x, y, clock, seq, energy, health, hop = \
            GOSSIP_FMT.unpack_from(payload)
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
