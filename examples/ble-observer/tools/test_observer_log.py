"""Tests for observer_log.py:  pytest examples/ble-observer/tools"""

import csv
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import observer_log as ol  # noqa: E402

ADDR_G = "C0:00:00:00:00:01"
ADDR_D = "C0:00:00:00:00:02"


def gossip_line(t_s, eid, useq, x=1.5, y=2.5, health=0, reserved=b"\0\0\0\0",
                tag=b"", addr=ADDR_G, hop=0):
    frame = ol.GOSSIP_FMT.pack(eid, x, y, 0.0, 1.0, 0.0, 0.0, 0.0, 7, useq,
                               90, health, hop, 0, 0, *reserved,
                               ol.WIRE_VERSION)
    payload = bytes([ol.MSG_GOSSIP]) + frame + tag
    kind = "B" if eid == 0xFF else "G"
    return f"{int(t_s * 1e6)},-60,{addr},E,{kind},{eid},{useq},{payload.hex()}"


def debug_line(t_s, kind, eid, seq, body, addr=ADDR_D):
    payload = bytes([0x10 if kind == "S" else 0x11, eid]) + \
        struct.pack("<H", seq) + body
    assert len(payload) == ol.DEBUG_PAYLOAD_LEN
    return f"{int(t_s * 1e6)},-55,{addr},L,{kind},{eid},{seq},{payload.hex()}"


def text_line(t_s, eid, seq, tseq, text, addr=ADDR_D):
    raw = text.encode().ljust(22, b"\0")
    return debug_line(t_s, "T", eid, seq, bytes([tseq]) + raw, addr)


def status_line(t_s, eid, seq, uptime_ms=1000, x=1.0, addr=ADDR_D):
    body = ol.STATUS_FMT.pack(uptime_ms, 3, 0x02 | 0x08, 0x13, int(x * 100),
                              200, 1571, ol.I16_NONE, ol.I16_NONE, 500, 600,
                              40, 0)
    return debug_line(t_s, "S", eid, seq, body, addr)


def run(tmp_path, lines):
    rec = ol.Recorder(tmp_path, echo=False)
    for ln in lines:
        rec.feed(ln, 1_790_000_000.0)
    rec.close()
    events = (tmp_path / "events.log").read_text().splitlines()
    return rec, events


def rows(tmp_path, name):
    with open(tmp_path / name, newline="") as f:
        return list(csv.DictReader(f))


def texts(events):
    return [e.split(" text    ", 1)[1] for e in events if " text    " in e]


def test_gossip_dedup_and_relays(tmp_path):
    run(tmp_path, [
        gossip_line(1.0, 2, 10),
        gossip_line(1.1, 2, 10),                    # repeat
        gossip_line(1.2, 2, 10, addr=ADDR_D, hop=1),  # relay by a peer
        gossip_line(1.3, 2, 11),
    ])
    g = rows(tmp_path, "gossip.csv")
    assert [r["update_seq"] for r in g] == ["10", "11"]
    assert g[0]["x"] == "1.5"


def test_auth_tag_and_reserved_bytes(tmp_path):
    run(tmp_path, [gossip_line(1.0, 1, 5, health=0x0100,
                               reserved=b"\x01\x02\x03\x04",
                               tag=b"\xaa\xbb\xcc\xdd")])
    g = rows(tmp_path, "gossip.csv")[0]
    assert g["reserved_hex"] == "01020304"
    assert g["auth_tag"] == "aabbccdd"
    assert g["health_flags"] == "0x0100"


def fact_line(t_s, sender, digest, recs, kind="F", addr=ADDR_G):
    flat = []
    for r in recs + [None] * (2 - len(recs)):
        flat += list(r) if r else [0] * 10
    frame = ol.FACT_FMT.pack(sender, len(recs), digest, *flat, ol.WIRE_VERSION)
    payload = bytes([ol.MSG_FACT]) + frame
    return f"{int(t_s * 1e6)},-58,{addr},E,{kind},{sender},{digest},{payload.hex()}"


def test_fact_frames_are_deduplicated_and_logged(tmp_path):
    leak = (1, 0, 0x0A0A, 0, 0x0E, 1, 69.0, 68.4, 0.0, 0)
    _, ev = run(tmp_path, [
        fact_line(1.00, 0, 0xBC77, [leak]),
        fact_line(1.03, 0, 0xBC77, [leak]),       # same burst
        fact_line(9.00, 2, 0xBC77, [leak]),       # a peer re-sends it
        fact_line(9.50, 3, 0x0000, []),            # rebooted robot's beacon
        fact_line(9.60, 2, 0x1234, [leak[:4] + (0x0F, 2) + leak[6:]],
                  kind="U"),                       # retracted; old DK firmware
    ])
    f = rows(tmp_path, "facts.csv")
    assert [r["sender"] for r in f] == ["0", "2", "3", "2"]
    assert (f[0]["type"], f[0]["x"], f[0]["y"], f[0]["epoch"]) == \
        ("1", "69.0", "68.4", "0x0a0a")
    assert f[2]["count"] == "0" and f[2]["rec"] == ""
    assert f[3]["retracted"] == "1"
    facts = [e for e in ev if " fact " in e]
    assert len(facts) == 2
    assert "fact target 0/0a0a/0 at (69.0,68.4) stamp 1" in facts[0]
    assert "retracted target 0/0a0a/0" in facts[1]


def test_beacon_logged_once(tmp_path):
    _, ev = run(tmp_path, [gossip_line(1.0, 0xFF, 0xDEADBEEF)] * 3)
    assert sum("beacon" in e for e in ev) == 1
    assert rows(tmp_path, "gossip.csv") == []


def test_status_decode_and_dedup(tmp_path):
    run(tmp_path, [status_line(1.0, 3, 7), status_line(1.05, 3, 7)])
    s = rows(tmp_path, "status.csv")
    assert len(s) == 1
    r = s[0]
    assert (r["step"], r["quorum"], r["discovered"], r["fresh_peers"],
            r["track"]) == ("3", "2", "1", "3", "1")
    assert r["tx"] == "" and r["px"] == "5.0" and r["heading_deg"] == "90.0"


def test_text_reassembly_with_repeats(tmp_path):
    _, ev = run(tmp_path, [
        text_line(1.0, 1, 1, 0, "hello wor"),
        text_line(1.05, 1, 1, 0, "hello wor"),      # the deliberate resend
        text_line(1.1, 1, 2, 1, "ld\nsecond line\n"),
    ])
    assert texts(ev) == ["hello world", "second line"]


def test_text_gap_marked(tmp_path):
    _, ev = run(tmp_path, [
        text_line(1.0, 1, 1, 0, "start of a "),
        text_line(1.2, 1, 3, 2, "tail\nnext\n"),    # tseq 1 lost
    ])
    assert texts(ev) == ["start of a  [text lost]", "[...]tail", "next"]
    assert any("1 text chunk(s) lost" in e for e in ev)


def test_partial_flushed_not_held(tmp_path):
    _, ev = run(tmp_path, [
        text_line(1.0, 1, 1, 0, "no newline yet"),
        status_line(1.0 + ol.TEXT_FLUSH_S + 0.5, 1, 2),
        text_line(5.0, 1, 3, 1, " rest\n"),
    ])
    assert texts(ev) == ["no newline yet [partial]", "[...] rest"]


def test_negotiating_robot_keeps_text_state_across_id(tmp_path):
    _, ev = run(tmp_path, [
        text_line(1.0, 0xFF, 1, 0, "boot: wait"),
        text_line(1.1, 4, 2, 1, "ing for peers\n"),
    ])
    assert texts(ev) == ["boot: waiting for peers"]
    assert any("negotiated id 4" in e for e in ev)


def test_reboot_resets_dedup(tmp_path):
    _, ev = run(tmp_path, [
        status_line(1.0, 2, 5, uptime_ms=60_000),
        status_line(2.0, 2, 5, uptime_ms=500),      # same seq, new boot
    ])
    assert len(rows(tmp_path, "status.csv")) == 2
    assert any("reboot" in e for e in ev)


def test_garbled_and_dk_lines(tmp_path):
    rec, ev = run(tmp_path, [
        "# tapestry-ble-observer fmt=1 wire=6",
        "# stats t_us=1000000 reports=10 tapestry=4 incomplete=0 dropped=0",
        "123,-60,AA:BB",
    ])
    assert rec.bad_lines == 1
    assert rows(tmp_path, "stats.csv")[0]["tapestry"] == "4"
    assert not any("WARN" in e for e in ev)


def test_replay_round_trip(tmp_path):
    live = tmp_path / "live"
    run(live, [text_line(1.0, 1, 1, 0, "abc\n"), gossip_line(1.1, 1, 9)])
    again = tmp_path / "again"
    rec = ol.Recorder(again, echo=False)
    ol.run_replay(rec, live / "raw.log")
    rec.close()
    assert texts((again / "events.log").read_text().splitlines()) == ["abc"]
    assert len(rows(again, "gossip.csv")) == 1


def test_gossip_after_reboot_not_deduped(tmp_path):
    rec, _ = run(tmp_path, [
        gossip_line(1.0, 0, 1), gossip_line(1.5, 0, 2), gossip_line(2.0, 0, 3),
        # rebooted and renegotiated: same id, update_seq starts over
        gossip_line(12.0, 0, 1), gossip_line(12.5, 0, 2),
    ])
    assert [r["update_seq"] for r in rows(tmp_path, "gossip.csv")] == \
        ["1", "2", "3", "1", "2"]
    assert "0: 5/5 frames, 0.0% lost" in (tmp_path / "summary.txt").read_text()


def test_outputs_readable_during_run(tmp_path):
    rec = ol.Recorder(tmp_path, echo=False)
    rec.feed(gossip_line(1.0, 1, 1), 1_790_000_000.0)
    assert len(rows(tmp_path, "gossip.csv")) == 1   # before close()
    rec.close()


def test_rebooted_robot_renegotiates_same_id(tmp_path):
    new = "C0:00:00:00:00:09"
    run(tmp_path, [
        status_line(1.0, 3, 500, uptime_ms=60_000),
        status_line(1.1, 3, 501, uptime_ms=60_120),
        status_line(20.0, 0xFF, 1, uptime_ms=500, addr=new),   # new boot
        status_line(30.0, 3, 80, uptime_ms=10_500, addr=new),  # same id again
        status_line(30.1, 3, 81, uptime_ms=10_620, addr=new),
    ])
    summary = (tmp_path / "summary.txt").read_text()
    assert "3/boot1: 2/2 packets, 0.0% lost" in summary
    assert "3: 3/81 packets" in summary     # seq 1 + 80..81 in one boot


def test_reboot_within_source_banks_span(tmp_path):
    run(tmp_path, [
        status_line(1.0, 2, 700, uptime_ms=90_000),
        status_line(1.1, 2, 701, uptime_ms=90_120),
        status_line(30.0, 2, 3, uptime_ms=12_000),     # rebooted, same addr
        status_line(30.1, 2, 4, uptime_ms=12_120),
    ])
    assert "2: 4/4 packets, 0.0% lost" in (tmp_path / "summary.txt").read_text()


def test_multibyte_character_split_across_chunks(tmp_path):
    line = "heard NO peers — retrying\n".encode()
    cut = line.index("—".encode()) + 1          # inside the 3-byte dash
    a, b = line[:cut], line[cut:]
    _, ev = run(tmp_path, [
        debug_line(1.0, "T", 1, 1, bytes([0]) + a.ljust(22, b"\0")),
        debug_line(1.1, "T", 1, 2, bytes([1]) + b.ljust(22, b"\0")),
    ])
    assert texts(ev) == ["heard NO peers — retrying"]
