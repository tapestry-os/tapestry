#!/usr/bin/env python3
"""
directive_probe.py — broadcast wire-v5 directive frames at a live element.

A stand-in for the (future, licensed) remote BSE host: streams
TAPESTRY_MSG_DIRECTIVE datagrams over UDP so the OSS element-side path —
gossip_poll_directive()'s filter chain, choreo.c's debounced adoption,
staleness fallback, and re-adoption — can be exercised end to end against
tapestry-scr-hw on a networking board, with nothing but a laptop on the
same subnet.  Ctrl-C mid-run IS the failover test: the element must log
"remote BSE stale ... local BSE fallback" within ~1.5 s and keep flying
its local script.

Usage (5 Hz stream of MOVE_TO_POINT to (60, 40) for every element):
    python3 sdk/tools/directive_probe.py --x 60 --y 40

Replay-protection note (wire.h): seq is derived from epoch milliseconds,
so restarting the probe never sends a seq the element has already seen —
the sender-side monotonic-across-restarts requirement, satisfied the way
the header suggests.

With CONFIG_TAPESTRY_WIRE_AUTH_ENABLED firmware, pass --key with the
board's CONFIG_TAPESTRY_WIRE_AUTH_KEY; frames are then followed by the
4-byte truncated HMAC-SHA256 tag.  Without --key against an auth build,
every frame must be dropped with "auth tag missing" — itself a useful
negative test.
"""

import argparse
import hmac
import hashlib
import socket
import struct
import time

# === BEGIN GENERATED WIRE PROTOCOL (tools/gen_wire_protocol.py — DO NOT EDIT) ===
# Mirrors tapestry-os/include/tapestry/wire.h.
# Regenerate after any wire.h change:
#   python3 tapestry-os/tools/gen_wire_protocol.py
#
# WIRE_VERSION bumps whenever a struct format below changes;
# decode() rejects a header whose version does not match —
# see wire.h's "Wire schema version" section for why.

WIRE_VERSION = 5

MSG_GOSSIP     = 1
MSG_METRIC     = 2
MSG_SCR_METRIC = 4
MSG_DIRECTIVE  = 5

HEADER_FMT     = struct.Struct('<BBBH')  #  5 bytes: version,type,src_id,payload_len
GOSSIP_FMT     = struct.Struct('<BfffffffIIBBBBBB')  # 43 bytes: id,x,y,z,qw,qx,qy,qz,logical_clock,update_seq,energy_level,health_flags,relay_qos,achieved,current_track,version
METRIC_FMT     = struct.Struct('<BBBBBBfBBfIffH')  # 30 bytes: element_id,active_total,active_fresh,active_stale,inactive_total,collision_count,fresh_ratio,quorum_held,degraded,confidence,cycle_count,mean_age_ms,mean_position_error,min_separation_x100
SCR_METRIC_FMT = struct.Struct('<BBBBBBI')  # 10 bytes: element_id,role,leader_id,quorum_state,fresh_count,task_slot,election_count
DIRECTIVE_FMT  = struct.Struct('<BBBfffffHIB')  # 30 bytes: src_id,target_id,type,x,y,z,spring_k,spacing,goal_id,seq,version
# === END GENERATED WIRE PROTOCOL ===

AUTH_TAG_SIZE = 4

DIRECTIVE_TYPES = {"idle": 0, "hold": 1, "move": 2, "spring": 3}


def build_frame(args: argparse.Namespace, seq: int) -> bytes:
    frame = DIRECTIVE_FMT.pack(  # noqa: F821 — defined in the generated block
        args.src,
        args.target,
        DIRECTIVE_TYPES[args.type],
        args.x, args.y, args.z,
        args.spring_k, args.spacing,
        args.goal_id,
        seq,
        WIRE_VERSION,  # noqa: F821 — defined in the generated block
    )
    if args.key:
        frame += hmac.new(args.key.encode(), frame,
                          hashlib.sha256).digest()[:AUTH_TAG_SIZE]
    header = HEADER_FMT.pack(  # noqa: F821 — defined in the generated block
        WIRE_VERSION, MSG_DIRECTIVE, args.src, len(frame))  # noqa: F821
    return header + frame


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--type", choices=sorted(DIRECTIVE_TYPES), default="move",
                    help="directive type (default: move = MOVE_TO_POINT)")
    ap.add_argument("--x", type=float, default=50.0)
    ap.add_argument("--y", type=float, default=50.0)
    ap.add_argument("--z", type=float, default=0.0)
    ap.add_argument("--spring-k", type=float, default=0.0)
    ap.add_argument("--spacing", type=float, default=0.0)
    ap.add_argument("--target", type=int, default=0xFF,
                    help="element id, or 255 = every element (default)")
    ap.add_argument("--src", type=int, default=30,
                    help="BSE host element id; must be < MAX_ELEMENTS=32 "
                         "or receivers fail closed (default: 30)")
    ap.add_argument("--goal-id", type=int, default=0)
    ap.add_argument("--rate", type=float, default=5.0,
                    help="frames per second (default 5 — comfortably inside "
                         "the element's 1500 ms staleness window)")
    ap.add_argument("--count", type=int, default=0,
                    help="stop after N frames (default: run until Ctrl-C)")
    ap.add_argument("--bcast", default="255.255.255.255",
                    help="broadcast address; use the subnet-directed form "
                         "(e.g. 192.168.86.255) across WiFi/Ethernet")
    ap.add_argument("--port", type=int, default=5000,
                    help="CONFIG_TAPESTRY_GOSSIP_PORT (default 5000)")
    ap.add_argument("--key", default=None,
                    help="CONFIG_TAPESTRY_WIRE_AUTH_KEY for auth builds; "
                         "omit for plain builds")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    sent = 0
    try:
        while args.count == 0 or sent < args.count:
            seq = int(time.time() * 1000) & 0xFFFFFFFF
            sock.sendto(build_frame(args, seq), (args.bcast, args.port))
            sent += 1
            if sent == 1 or sent % 25 == 0:
                print(f"sent {sent} frames  (seq={seq}  type={args.type}  "
                      f"target={args.target}  {'signed' if args.key else 'PLAIN'})")
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print(f"\nstopped after {sent} frames — element should log local-BSE "
              "fallback within ~1.5 s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
