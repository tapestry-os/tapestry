#!/usr/bin/env python3
"""
observer_log.py — record a run from the Tapestry BLE observer
(examples/ble-observer on an nRF5340 DK).  Standard library only.

Live, from the DK's serial port:
    python3 observer_log.py /dev/ttyACM0
Re-decode a capture (a raw.log written by a live run, or plain DK output):
    python3 observer_log.py --replay logs/ble-20261001-101500/raw.log

Writes into a run directory (default logs/ble-<local time>/):
    raw.log     every serial line, prefixed with host time — replayable
    gossip.csv  one row per new gossip frame, de-duplicated by
                (id, update_seq), so mesh relays of a frame collapse too
    status.csv  one row per new debug status packet
    stats.csv   the DK's once-a-second scan counters
    events.log  one time-ordered log: robot text lines, ID-negotiation
                beacons, reboots, id changes, lost text, DK messages
    summary.txt per-robot packet counts, estimated loss and RSSI

Debug packets are de-duplicated by (id, seq) — by (address, seq) while a
robot is still negotiating (id 255).  A repeat only counts within
DEDUP_WINDOW_S, so a robot that reboots and restarts its counters is
never mistaken for a duplicate (a reboot takes longer than the window).  Text chunks are reassembled per
robot: a lost chunk is marked in place, and a partial line is flushed,
marked, after TEXT_FLUSH_S rather than held.

Packet formats: tapestry-os/include/tapestry/wire.h (gossip) and
examples/cutebot-formation/src/ble_debug.h / ble_debug.c (debug).
"""

import argparse
import csv
import math
import os
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

# === BEGIN GENERATED WIRE PROTOCOL (tools/gen_wire_protocol.py — DO NOT EDIT) ===
# Mirrors tapestry-os/include/tapestry/wire.h.
# Regenerate after any wire.h change:
#   python3 tapestry-os/tools/gen_wire_protocol.py
#
# WIRE_VERSION bumps whenever a struct format below changes;
# decode() rejects a header whose version does not match —
# see wire.h's "Wire schema version" section for why.

WIRE_VERSION = 6

MSG_GOSSIP     = 1
MSG_METRIC     = 2
MSG_SCR_METRIC = 4
MSG_DIRECTIVE  = 5

HEADER_FMT     = struct.Struct('<BBBH')  #  5 bytes: version,type,src_id,payload_len
GOSSIP_FMT     = struct.Struct('<BfffffffIIBHBBBBBBBBB')  # 49 bytes: id,x,y,z,qw,qx,qy,qz,logical_clock,update_seq,energy_level,health_flags,relay_qos,achieved,discovered,current_track,reserved0,reserved1,reserved2,reserved3,version
METRIC_FMT     = struct.Struct('<BBBBBBfBBfIffH')  # 30 bytes: element_id,active_total,active_fresh,active_stale,inactive_total,collision_count,fresh_ratio,quorum_held,degraded,confidence,cycle_count,mean_age_ms,mean_position_error,min_separation_x100
SCR_METRIC_FMT = struct.Struct('<BBBBBBI')  # 10 bytes: element_id,role,leader_id,quorum_state,fresh_count,task_slot,election_count
DIRECTIVE_FMT  = struct.Struct('<BBBfffffHIB')  # 30 bytes: src_id,target_id,type,x,y,z,spring_k,spacing,goal_id,seq,version
# === END GENERATED WIRE PROTOCOL ===

# Must name GOSSIP_FMT's fields in order.  A wire.h change regenerates the
# format above; if this list is then stale, the length check at import
# fails loudly instead of mislabelling columns.
GOSSIP_FIELDS = (
    "id", "x", "y", "z", "qw", "qx", "qy", "qz", "logical_clock",
    "update_seq", "energy_level", "health_flags", "relay_qos", "achieved",
    "discovered", "current_track", "reserved0", "reserved1", "reserved2",
    "reserved3", "version",
)
assert len(GOSSIP_FIELDS) == len(GOSSIP_FMT.unpack(bytes(GOSSIP_FMT.size))), \
    "GOSSIP_FIELDS is out of step with the regenerated GOSSIP_FMT"

AUTH_TAG_SIZE = 4             # CONFIG_TAPESTRY_WIRE_AUTH_ENABLED builds
ID_NEGOTIATING = 0xFF         # ELEMENT_ID_INVALID
# cutebot-dars-demo firmware: reserved0..3 carry int16 disc_x/disc_y
# (hundredths), valid when this health_flags bit is set.
HEALTH_DISC_POINT_VALID = 0x0100

# ble_debug.c pack_status(): 23 bytes after kind/id/seq.
STATUS_FMT = struct.Struct("<IbBBhhhhhhhBB")
DEBUG_PAYLOAD_LEN = 27
I16_NONE = -32768

LINE_FORMAT_VERSION = 1       # examples/ble-observer/src/main.c
BAUD = 1_000_000

TEXT_FLUSH_S = 3.0            # a partial text line older than this is flushed
DEDUP_WINDOW_S = 5.0          # a repeat later than this is a new packet
REBOOT_SEQ_DROP = 1000        # a debug seq this far backwards means a reboot

GOSSIP_COLUMNS = [
    "dk_s", "host_time", "addr", "rssi", "adv", "id", "update_seq",
    "x", "y", "z", "yaw_deg", "logical_clock", "energy", "health_flags",
    "hop", "qos", "achieved", "discovered", "track", "reserved_hex",
    "disc_x", "disc_y", "version", "auth_tag",
]
STATUS_COLUMNS = [
    "dk_s", "host_time", "addr", "rssi", "id", "seq", "uptime_s", "step",
    "quorum", "suspended", "discovered", "point_valid", "target_valid",
    "fresh_peers", "track", "x", "y", "heading_deg", "tx", "ty", "px", "py",
    "gray_max", "text_drops",
]
STATS_COLUMNS = ["dk_s", "host_time", "reports", "tapestry", "incomplete",
                 "dropped_total"]


@dataclass
class Report:
    """One DK line: t_us,rssi,addr,adv,kind,id,seq,hex."""
    dk_s: float
    rssi: int
    addr: str
    adv: str
    kind: str
    id: int
    seq: int
    payload: bytes


def parse_report(line):
    parts = line.split(",", 7)
    if len(parts) != 8:
        raise ValueError("expected 8 fields")
    t_us, rssi, addr, adv, kind, id_, seq, hexdata = parts
    return Report(int(t_us) / 1e6, int(rssi), addr, adv, kind, int(id_),
                  int(seq), bytes.fromhex(hexdata))


def yaw_deg(qw, qx, qy, qz):
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                   1.0 - 2.0 * (qy * qy + qz * qz)))


def decode_gossip(payload):
    """payload = type byte + frame (+ tag).  Returns a dict or None."""
    body = payload[1:]
    if len(body) < GOSSIP_FMT.size:
        return None
    g = dict(zip(GOSSIP_FIELDS, GOSSIP_FMT.unpack_from(body), strict=True))
    extra = body[GOSSIP_FMT.size:]
    g["auth_tag"] = extra.hex() if extra else ""
    g["extra_len"] = len(extra)
    raw = bytes(g[f"reserved{i}"] for i in range(4))
    g["reserved_hex"] = raw.hex()
    if g["health_flags"] & HEALTH_DISC_POINT_VALID:
        dx, dy = struct.unpack("<hh", raw)
        g["disc_x"], g["disc_y"] = dx / 100.0, dy / 100.0
    else:
        g["disc_x"] = g["disc_y"] = ""
    return g


def decode_status(payload):
    b = STATUS_FMT.unpack_from(payload, 4)
    (uptime_ms, step, flags, peers_track, x, y, heading, tx, ty, px, py,
     gray, drops) = b

    def units(v):
        return "" if v == I16_NONE else v / 100.0

    return {
        "uptime_s": uptime_ms / 1000.0, "step": step,
        "quorum": flags & 0x3, "suspended": (flags >> 2) & 1,
        "discovered": (flags >> 3) & 1, "point_valid": (flags >> 4) & 1,
        "target_valid": (flags >> 5) & 1,
        "fresh_peers": peers_track & 0xF, "track": peers_track >> 4,
        "x": units(x), "y": units(y),
        "heading_deg": "" if heading == I16_NONE
        else round(math.degrees(heading / 1000.0), 1),
        "tx": units(tx), "ty": units(ty), "px": units(px), "py": units(py),
        "gray_max": gray, "text_drops": drops,
    }


class RecentKeys:
    """Keys seen in the last DEDUP_WINDOW_S of DK time."""

    def __init__(self):
        self._t = {}              # key -> first-seen dk_s, insertion-ordered

    def seen(self, key, now):
        while self._t:
            k, t = next(iter(self._t.items()))
            if now - t <= DEDUP_WINDOW_S:
                break
            del self._t[k]
        if key in self._t:
            return True
        self._t[key] = now
        return False

    def clear(self):
        self._t.clear()


@dataclass
class Source:
    """Per-robot debug state (keyed by id, or by address while id 255)."""
    name: str
    recent: RecentKeys = field(default_factory=RecentKeys)
    last_seq: int = -1
    last_uptime: float = -1.0
    expected_tseq: int = -1
    partial: bytes = b""          # raw bytes: a UTF-8 character can span chunks
    partial_t: float = 0.0
    resync: bool = False          # next text starts mid-line
    unique: int = 0
    seq_lo: int = -1
    seq_hi: int = -1
    banked_span: int = 0          # seq span of earlier boots (summary)
    rssi_sum: int = 0

    def reset(self):
        self.recent.clear()
        self.last_seq = -1
        self.last_uptime = -1.0
        self.expected_tseq = -1
        if self.seq_lo >= 0:
            self.banked_span += self.seq_hi - self.seq_lo + 1
        self.seq_lo = self.seq_hi = -1


class Recorder:
    def __init__(self, out_dir, echo=True):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.echo = echo
        self._files = []
        self.raw = self._open("raw.log")
        self.events = self._open("events.log")
        self.gossip = self._csv("gossip.csv", GOSSIP_COLUMNS)
        self.status = self._csv("status.csv", STATUS_COLUMNS)
        self.stats = self._csv("stats.csv", STATS_COLUMNS)
        self.sources = {}         # name -> Source
        self.addr_alias = {}      # debug-set address -> id name
        self.gossip_seen = RecentKeys()   # (id, update_seq)
        # id -> [unique, lo, hi, rssi_sum, banked span, last dk_s]
        self.gossip_stats = {}
        self.beacons = set()      # (addr, nonce)
        self.wire_warned = set()
        self.dk_s = 0.0
        self.host_time = ""
        self.bad_lines = 0
        self.dk_resets = 0

    # ── output helpers ──────────────────────────────────────────────────
    def _open(self, name):
        # Line-buffered, so the files can be followed during a run.
        f = open(self.out / name, "w", encoding="utf-8", newline="",
                 buffering=1)
        self._files.append(f)
        return f

    def _csv(self, name, columns):
        w = csv.DictWriter(self._open(name), fieldnames=columns,
                           extrasaction="ignore")
        w.writeheader()
        return w

    def event(self, src, kind, msg, dk_s=None):
        t = self.dk_s if dk_s is None else dk_s
        line = f"{t:10.3f} {self.host_time[11:23]:12} {src:>17} {kind:7} {msg}"
        self.events.write(line + "\n")
        if self.echo:
            print(line, flush=True)

    def close(self):
        for s in self.sources.values():
            self._flush_partial(s, "end of log")
        self.write_summary()
        for f in self._files:
            f.close()

    # ── input ───────────────────────────────────────────────────────────
    def feed(self, line, host_epoch):
        """One serial line (without newline) received at host_epoch."""
        self.host_time = datetime.fromtimestamp(host_epoch).isoformat(
            timespec="milliseconds")
        self.raw.write(f"{host_epoch:.3f}\t{line}\n")
        if not line:
            return
        if line.startswith("#"):
            self._dk_meta(line)
            return
        try:
            r = parse_report(line)
        except ValueError:
            self.bad_lines += 1
            if self.bad_lines <= 20:
                self.event("dk", "garbled", repr(line[:80]))
            return
        if r.dk_s + 1.0 < self.dk_s:
            self.dk_resets += 1
            self.event("dk", "reset", f"DK clock went back to {r.dk_s:.3f}")
        self.dk_s = r.dk_s
        self._flush_stale()
        if r.kind in ("G", "B"):
            self._gossip(r)
        elif r.kind in ("S", "T") and len(r.payload) == DEBUG_PAYLOAD_LEN:
            self._debug(r)

    def _dk_meta(self, line):
        body = line[1:].strip()
        if body.startswith("stats "):
            kv = dict(p.split("=", 1) for p in body.split()[1:] if "=" in p)
            self.stats.writerow({
                "dk_s": int(kv.get("t_us", 0)) / 1e6,
                "host_time": self.host_time,
                "reports": kv.get("reports"), "tapestry": kv.get("tapestry"),
                "incomplete": kv.get("incomplete"),
                "dropped_total": kv.get("dropped"),
            })
            return
        if body.startswith("tapestry-ble-observer"):
            kv = dict(p.split("=", 1) for p in body.split()[1:] if "=" in p)
            if kv.get("fmt") != str(LINE_FORMAT_VERSION):
                self.event("dk", "WARN", f"DK line format {kv.get('fmt')}, "
                           f"this script reads {LINE_FORMAT_VERSION}")
        self.event("dk", "dk", body)

    # ── gossip ──────────────────────────────────────────────────────────
    def _gossip(self, r):
        g = decode_gossip(r.payload)
        if g is None:
            self.event(r.addr, "WARN", f"short gossip frame ({len(r.payload)} B)")
            return
        if g["version"] != WIRE_VERSION and r.addr not in self.wire_warned:
            self.wire_warned.add(r.addr)
            self.event(r.addr, "WARN", f"wire v{g['version']} frame, this "
                       f"script decodes v{WIRE_VERSION} (fields may be wrong)")
        if r.kind == "B":
            if (r.addr, r.seq) not in self.beacons:
                self.beacons.add((r.addr, r.seq))
                self.event(r.addr, "beacon",
                           f"negotiating, nonce 0x{r.seq:08x} rssi {r.rssi}")
            return

        if self.gossip_seen.seen((r.id, g["update_seq"]), r.dk_s):
            return
        u = g["update_seq"]
        st = self.gossip_stats.setdefault(r.id, [0, u, u, 0, 0, r.dk_s])
        if u < st[2] and (st[2] - u > 10 or r.dk_s - st[5] > 3.0):
            # update_seq restarted (a reboot: renegotiating keeps an id
            # silent for seconds).  Bank the old boot's span.
            st[4] += st[2] - st[1] + 1
            st[1] = st[2] = u
        st[5] = r.dk_s
        st[0] += 1
        st[1] = min(st[1], u)
        st[2] = max(st[2], u)
        st[3] += r.rssi

        self.gossip.writerow({
            **g, "dk_s": f"{r.dk_s:.6f}", "host_time": self.host_time,
            "addr": r.addr, "rssi": r.rssi, "adv": r.adv,
            "x": round(g["x"], 3), "y": round(g["y"], 3), "z": round(g["z"], 3),
            "yaw_deg": round(yaw_deg(g["qw"], g["qx"], g["qy"], g["qz"]), 1),
            "energy": g["energy_level"],
            "health_flags": f"0x{g['health_flags']:04x}",
            "hop": g["relay_qos"] & 0x3, "qos": (g["relay_qos"] >> 2) & 0x3,
            "track": g["current_track"],
        })

    # ── debug telemetry ─────────────────────────────────────────────────
    def _source(self, r):
        if r.id == ID_NEGOTIATING:
            name = self.addr_alias.get(r.addr, r.addr)
        else:
            name = str(r.id)
            old = self.addr_alias.get(r.addr)
            if old != name:
                self.addr_alias[r.addr] = name
                if r.addr in self.sources:
                    if name in self.sources:
                        # This id's earlier boot: keep it, under another
                        # name, for the summary.
                        k = 1
                        while f"{name}/boot{k}" in self.sources:
                            k += 1
                        prev = self.sources.pop(name)
                        self._flush_partial(prev, "rebooted")
                        prev.name = f"{name}/boot{k}"
                        self.sources[prev.name] = prev
                        self.event(name, "reboot",
                                   f"earlier boot kept as {prev.name}")
                    s = self.sources.pop(r.addr)
                    s.name = name
                    self.sources[name] = s
                    self.event(name, "id", f"{r.addr} negotiated id {name}")
        s = self.sources.get(name)
        if s is None:
            s = self.sources[name] = Source(name)
            self.event(name, "boot" if r.id == ID_NEGOTIATING else "first",
                       f"debug telemetry from {r.addr}")
        return s

    def _debug(self, r):
        s = self._source(r)
        p = r.payload

        if s.last_seq >= 0 and r.seq + REBOOT_SEQ_DROP < s.last_seq:
            self._reboot(s, f"seq {s.last_seq} -> {r.seq}")
        if r.kind == "S":
            up = STATUS_FMT.unpack_from(p, 4)[0] / 1000.0
            if 0 <= up + 1.0 < s.last_uptime:
                self._reboot(s, f"uptime {s.last_uptime:.1f}s -> {up:.1f}s")

        if s.recent.seen(r.seq, r.dk_s):
            return
        s.unique += 1
        s.rssi_sum += r.rssi
        s.last_seq = r.seq
        s.seq_lo = r.seq if s.seq_lo < 0 else min(s.seq_lo, r.seq)
        s.seq_hi = max(s.seq_hi, r.seq)

        if r.kind == "S":
            st = decode_status(p)
            s.last_uptime = st["uptime_s"]
            self.status.writerow({
                **st, "dk_s": f"{r.dk_s:.6f}", "host_time": self.host_time,
                "addr": r.addr, "rssi": r.rssi, "id": r.id, "seq": r.seq,
            })
        else:
            self._text(s, p[4], p[5:])

    def _reboot(self, s, why):
        self._flush_partial(s, "reboot")
        self.event(s.name, "reboot", why)
        s.reset()

    def _text(self, s, tseq, chunk):
        if s.expected_tseq >= 0 and tseq != s.expected_tseq:
            lost = (tseq - s.expected_tseq) & 0xFF
            self._flush_partial(s, "text lost")
            self.event(s.name, "gap", f"{lost} text chunk(s) lost")
            s.resync = True
        s.expected_tseq = (tseq + 1) & 0xFF

        data = chunk.rstrip(b"\0")
        if not data:
            return
        if not s.partial:
            s.partial_t = self.dk_s
        s.partial += data
        *lines, s.partial = s.partial.split(b"\n")
        for ln in lines:
            self.event(s.name, "text", ("[...]" if s.resync else "") +
                       ln.decode("utf-8", errors="replace"), dk_s=s.partial_t)
            s.resync = False
            s.partial_t = self.dk_s

    def _flush_partial(self, s, why):
        if s.partial:
            self.event(s.name, "text", ("[...]" if s.resync else "") +
                       s.partial.decode("utf-8", errors="replace") +
                       f" [{why}]", dk_s=s.partial_t)
            s.partial = b""
            s.resync = True

    def _flush_stale(self):
        for s in self.sources.values():
            if s.partial and self.dk_s - s.partial_t > TEXT_FLUSH_S:
                self._flush_partial(s, "partial")

    # ── summary ─────────────────────────────────────────────────────────
    def write_summary(self):
        out = []
        out.append("debug telemetry (unique packets / seq span, loss, mean RSSI)")
        for name, s in sorted(self.sources.items()):
            if s.unique == 0:
                continue
            span = s.banked_span + s.seq_hi - s.seq_lo + 1
            out.append(f"  {name:>17}: {s.unique}/{span} packets, "
                       f"{100.0 * (1 - s.unique / span):.1f}% lost, "
                       f"{s.rssi_sum / s.unique:.0f} dBm")
        out.append("gossip (unique update_seq / span, mean RSSI of first copy)")
        for gid, (n, lo, hi, rs, banked, _) in sorted(self.gossip_stats.items()):
            span = banked + hi - lo + 1
            out.append(f"  {gid:>17}: {n}/{span} frames, "
                       f"{100.0 * (1 - n / span):.1f}% lost, {rs / n:.0f} dBm")
        out.append(f"garbled lines: {self.bad_lines}, DK resets: {self.dk_resets}")
        text = "\n".join(out) + "\n"
        (self.out / "summary.txt").write_text(text, encoding="utf-8")
        if self.echo:
            print(text, end="")


def open_serial(path, baud):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}")
    attrs[4] = attrs[5] = speed
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CRTSCTS
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def run_live(rec, port, baud):
    fd = open_serial(port, baud)
    buf = b""
    try:
        while True:
            data = os.read(fd, 4096)
            if not data:
                time.sleep(0.01)
                continue
            now = time.time()
            buf += data
            *lines, buf = buf.split(b"\n")
            for ln in lines:
                rec.feed(ln.decode("ascii", errors="replace").rstrip("\r"), now)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


def run_replay(rec, path):
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            raw = raw.rstrip("\r\n")
            host, sep, line = raw.partition("\t")
            try:
                t = float(host) if sep else time.time()
            except ValueError:
                t, line = time.time(), raw
            rec.feed(line if sep else raw, t)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port", nargs="?", help="DK serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--replay", metavar="FILE", help="decode a capture instead")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--out", help="run directory (default logs/ble-<time>)")
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="don't echo events to the terminal")
    args = ap.parse_args()
    if not args.port and not args.replay:
        ap.error("give a serial port or --replay FILE")

    out = args.out or f"logs/ble-{datetime.now():%Y%m%d-%H%M%S}"
    rec = Recorder(out, echo=not args.quiet)
    print(f"writing {out}/", file=sys.stderr)
    try:
        if args.replay:
            run_replay(rec, args.replay)
        else:
            run_live(rec, args.port, args.baud)
    finally:
        rec.close()


if __name__ == "__main__":
    main()
