"""
protocol.py — Python mirror of wire.h + sim_protocol.h + scr_protocol.h

Wire format is identical to the simulation so the same struct layouts
decode packets from physical elements without modification.  The wire.h-
derived section below is GENERATED — see its own marker comments.
"""

import logging
import struct

log = logging.getLogger(__name__)

# ── Collector port ────────────────────────────────────────────────────────────

COLLECTOR_PORT = 5100

# === BEGIN GENERATED WIRE PROTOCOL (tools/gen_wire_protocol.py — DO NOT EDIT) ===
# Mirrors tapestry-os/include/tapestry/wire.h.
# Regenerate after any wire.h change:
#   python3 tapestry-os/tools/gen_wire_protocol.py
#
# WIRE_VERSION bumps whenever a struct format below changes;
# decode() rejects a header whose version does not match —
# see wire.h's "Wire schema version" section for why.

WIRE_VERSION = 4

MSG_GOSSIP     = 1
MSG_METRIC     = 2
MSG_SCR_METRIC = 4

HEADER_FMT     = struct.Struct('<BBBH')  #  5 bytes: version,type,src_id,payload_len
GOSSIP_FMT     = struct.Struct('<BfffffffIIBBBBBB')  # 43 bytes: id,x,y,z,qw,qx,qy,qz,logical_clock,update_seq,energy_level,health_flags,relay_qos,achieved,current_track,version
METRIC_FMT     = struct.Struct('<BBBBBBfBBfIffH')  # 30 bytes: element_id,active_total,active_fresh,active_stale,inactive_total,collision_count,fresh_ratio,quorum_held,degraded,confidence,cycle_count,mean_age_ms,mean_position_error,min_separation_x100
SCR_METRIC_FMT = struct.Struct('<BBBBBBI')  # 10 bytes: element_id,role,leader_id,quorum_state,fresh_count,task_slot,election_count
# === END GENERATED WIRE PROTOCOL ===

ELEMENT_ID_INVALID = 0xFF

# ── Decode ────────────────────────────────────────────────────────────────────

def decode(data: bytes) -> dict | None:
    """
    Parse a raw UDP datagram.  Returns a typed dict or None if malformed.

    metric keys:
        type, src_id, element_id, active_total, active_fresh, active_stale,
        inactive_total, collision_count, fresh_ratio, quorum_held, degraded,
        confidence, cycle_count, mean_age_ms, mean_position_error,
        min_separation

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
        eid, role, leader, qstate, fresh, _reserved, elec = \
            SCR_METRIC_FMT.unpack_from(payload)
        return {
            'type':           'scr_metric',
            'src_id':         src_id,
            'element_id':     eid,
            'role':           role,     # 0=NONE 1=FOLLOWER 2=LEADER
            'leader_id':      leader,   # ELEMENT_ID_INVALID (0xFF) = no leader
            'quorum_state':   qstate,   # 0=LOST 1=DEGRADED 2=HEALTHY
            'fresh_count':    fresh,
            'election_count': elec,
        }

    return None
