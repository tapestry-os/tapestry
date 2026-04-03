"""
protocol.py — Python mirror of sim_protocol.h

All struct formats are little-endian ('<') to match the C packed structs.
Update this file whenever sim_protocol.h changes — sizes in the docstrings
must stay in sync with the C-side #defines.
"""

import struct

# ── Ports ─────────────────────────────────────────────────────────────────────

ORCH_PORT         = 5100
ELEMENT_BASE_PORT = 5000
LOOPBACK          = "127.0.0.1"

# ── Message types ─────────────────────────────────────────────────────────────

MSG_GOSSIP  = 1
MSG_METRIC  = 2
MSG_CONTROL = 3

# ── Control subtypes ──────────────────────────────────────────────────────────

CTRL_SET_PARTITION = 1
CTRL_SET_POWER     = 2
CTRL_SHUTDOWN      = 3

# ── Struct formats ────────────────────────────────────────────────────────────
#
# HEADER_FMT   '<BBH'           type, src_id, payload_len             — 4 bytes
# GOSSIP_FMT   '<BffIBBI'       id, x, y, clock, power, island, seq   — 19 bytes
# METRIC_FMT   '<BBBBBBfBBIffH' eid,at,af,as_,it,cc,ratio,qh,cf,      — 28 bytes
#                               cycle,mean_age,mean_pos_err,min_sep_x100
# CTRL_FMT     '<BB'            ctrl_type, value                       — 2 bytes

HEADER_FMT = struct.Struct('<BBH')
GOSSIP_FMT = struct.Struct('<BffIBBI')
METRIC_FMT = struct.Struct('<BBBBBBfBBIffH')
CTRL_FMT   = struct.Struct('<BB')

# ── Encode ────────────────────────────────────────────────────────────────────

def encode_gossip(state: dict) -> bytes:
    """Pack a gossip message from an element_state dict."""
    header  = HEADER_FMT.pack(MSG_GOSSIP, state['id'], GOSSIP_FMT.size)
    payload = GOSSIP_FMT.pack(
        state['id'],
        state['x'],
        state['y'],
        state['logical_clock'],
        state['power_state'],
        state['partition_island'],
        state['update_seq'],
    )
    return header + payload


def encode_control(src_id: int, ctrl_type: int, value: int) -> bytes:
    """Pack a control message from the orchestrator to an element."""
    header  = HEADER_FMT.pack(MSG_CONTROL, src_id, CTRL_FMT.size)
    payload = CTRL_FMT.pack(ctrl_type, value)
    return header + payload

# ── Decode ────────────────────────────────────────────────────────────────────

def decode(data: bytes) -> dict | None:
    """
    Parse a raw UDP datagram into a typed dict.
    Returns None if the datagram is too short or the type is unknown.

    Gossip result keys:
        type, src_id, id, x, y, logical_clock, power_state,
        partition_island, update_seq

    Metric result keys:
        type, src_id, element_id, active_total, active_fresh, active_stale,
        inactive_total, collision_count, fresh_ratio, quorum_held,
        cp_frozen, cycle_count
    """
    if len(data) < HEADER_FMT.size:
        return None

    msg_type, src_id, payload_len = HEADER_FMT.unpack_from(data)
    payload = data[HEADER_FMT.size:]

    if len(payload) < payload_len:
        return None

    if msg_type == MSG_GOSSIP and len(payload) >= GOSSIP_FMT.size:
        id_, x, y, clock, power, island, seq = GOSSIP_FMT.unpack_from(payload)
        return {
            'type':              'gossip',
            'src_id':            src_id,
            'id':                id_,
            'x':                 x,
            'y':                 y,
            'logical_clock':     clock,
            'power_state':       power,
            'partition_island':  island,
            'update_seq':        seq,
        }

    if msg_type == MSG_METRIC and len(payload) >= METRIC_FMT.size:
        eid, at, af, ast, it, cc, ratio, qh, cf, cycle, \
            mean_age, mean_pos_err, min_sep_x100 = \
            METRIC_FMT.unpack_from(payload)
        # Decode min_separation: 0xFFFF sentinel means no active peers
        min_sep = None if min_sep_x100 == 0xFFFF else min_sep_x100 / 100.0
        return {
            'type':                 'metric',
            'src_id':               src_id,
            'element_id':           eid,
            'active_total':         at,
            'active_fresh':         af,
            'active_stale':         ast,
            'inactive_total':       it,
            'collision_count':      cc,
            'fresh_ratio':          ratio,
            'quorum_held':          bool(qh),
            'cp_frozen':            bool(cf),
            'cycle_count':          cycle,
            'mean_age_ms':          mean_age,
            'mean_position_error':  mean_pos_err,   # broker will fill this
            'min_separation':       min_sep,
        }

    return None
