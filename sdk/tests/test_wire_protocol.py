"""
test_wire_protocol.py — the Python mirrors of tapestry/wire.h.

Three consumers (two sim orchestrators and the hardware telemetry
collector) each carry a block generated from wire.h by
tapestry-os/tools/gen_wire_protocol.py, plus hand-written encode/decode
code around it that the generator deliberately does not touch.

That split is where drift hides, and did: the v4 gossip frame added
`current_track`, and because the generator only rewrites the
struct.Struct(...) line between its markers, regenerating on its own
would have left encode_gossip() packing 14 values into a 15-field format
and decode() unpacking 15 into 14 names — a green --check over two broken
orchestrators.  gen_wire_protocol.py --check cannot see call sites; these
tests can, and they run in the fast Python job rather than only in the
Zephyr one.
"""

import importlib.util
import re
import struct
import sys

import pytest
from helpers import REPO_ROOT

WIRE_H = REPO_ROOT / "tapestry-os/include/tapestry/wire.h"


def load_module(rel_path: str, name: str):
    """Import a file by path under an explicit module name.

    Not a sys.path insert: all three consumers are named protocol.py, so
    they would shadow each other in sys.modules and only the first one
    imported would ever be tested.
    """
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / rel_path)
    mod  = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


gen = load_module("tapestry-os/tools/gen_wire_protocol.py", "gen_wire_protocol")

# Every consumer of the generated block.  scr-hw decodes metrics only —
# it has no gossip call sites — so it appears in the mirror tests below
# but not the round-trip ones.
CONSUMERS = {
    "csm-sim": "tapestry-csm-sim/orchestrator/protocol.py",
    "scr-sim": "tapestry-scr-sim/orchestrator/protocol.py",
    "scr-hw":  "tapestry-scr-hw/telemetry/protocol.py",
}
SIMS = ["csm-sim", "scr-sim"]

MODULES = {name: load_module(path, f"wire_mirror_{name.replace('-', '_')}")
           for name, path in CONSUMERS.items()}

WIRE_TEXT = WIRE_H.read_text()


# ── The generated block agrees with wire.h ───────────────────────────────────

@pytest.mark.parametrize("consumer", list(CONSUMERS))
@pytest.mark.parametrize("c_struct,py_name", gen.STRUCTS)
def test_every_consumer_mirrors_the_wire_h_struct_layout(consumer, c_struct,
                                                         py_name):
    fields   = gen.parse_struct_fields(WIRE_TEXT, c_struct)
    expected = struct.Struct(gen.struct_format(fields))
    actual   = getattr(MODULES[consumer], py_name)
    assert actual.format == expected.format
    assert actual.size   == expected.size


@pytest.mark.parametrize("consumer", list(CONSUMERS))
def test_every_consumer_mirrors_the_wire_schema_version(consumer):
    assert MODULES[consumer].WIRE_VERSION == gen.parse_version(WIRE_TEXT)


@pytest.mark.parametrize("consumer", list(CONSUMERS))
def test_the_generated_block_is_not_stale(consumer):
    """The pytest-speed equivalent of `gen_wire_protocol.py --check`.

    CI runs --check too, but only inside the Zephyr job — minutes behind a
    toolchain and a west workspace.  A wire.h edit that nobody regenerated
    for should fail in the seconds-long job instead."""
    target = REPO_ROOT / CONSUMERS[consumer]
    _, changed = gen.apply_block(target, gen.render_block(WIRE_TEXT))
    assert not changed, (f"{CONSUMERS[consumer]} is stale — run "
                         f"python3 tapestry-os/tools/gen_wire_protocol.py")


def test_wire_h_prose_matches_its_own_structs():
    """Each frame's comment restates its Python format and byte size, and
    the generator never touches prose — wire.h described the 42-byte v3
    gossip layout for as long as it took to notice by hand."""
    for c_struct, _ in gen.STRUCTS:
        end = re.search(r"\}\s*__attribute__\(\(packed\)\)\s*"
                        + re.escape(c_struct) + r"\s*;", WIRE_TEXT)
        prose = list(re.finditer(
            r"Python format: struct\.Struct\('([^']+)'\)\s*\n"
            r"\s*\*\s*Size: (\d+) bytes", WIRE_TEXT[:end.start()]))
        assert prose, f"{c_struct}: no 'Python format:' comment precedes it"
        fmt, size = prose[-1].group(1), int(prose[-1].group(2))

        expected = struct.Struct(gen.struct_format(
            gen.parse_struct_fields(WIRE_TEXT, c_struct)))
        assert fmt  == expected.format, f"{c_struct} comment format is stale"
        assert size == expected.size,   f"{c_struct} comment size is stale"


# ── The hand-written call sites agree with the generated block ───────────────

# Every field encode_gossip() reads, with a distinct value each so a
# transposition cannot pass.  Floats are exact in binary32 — a mismatch
# here means a wrong field, never a rounding artifact.
GOSSIP = {
    'id':            7,
    'x':             1.5,
    'y':             2.25,
    'z':             3.125,
    'qw':            0.5,
    'qx':            0.25,
    'qy':            0.125,
    'qz':            0.0625,
    'logical_clock': 1234,
    'update_seq':    5678,
    'energy_level':  42,
    'health_flags':  0x03,
    'hop_count':     2,
    'achieved':      True,
    'current_track': 3,
}


@pytest.mark.parametrize("consumer", SIMS)
def test_a_gossip_frame_round_trips_every_field(consumer):
    """encode_gossip -> decode, the arity check the generator cannot make:
    a field added to wire.h that nobody threaded through the hand-written
    pack/unpack raises struct.error here rather than at sim runtime."""
    p   = MODULES[consumer]
    got = p.decode(p.encode_gossip({**GOSSIP, 'qos': p.QOS_HARD_RT}))

    assert got is not None and got['type'] == 'gossip'
    for field, sent in GOSSIP.items():
        assert got[field] == sent, f"{field} did not survive the round trip"
    assert got['qos'] == p.QOS_HARD_RT


@pytest.mark.parametrize("consumer", SIMS)
def test_a_relayed_gossip_frame_preserves_every_field(consumer):
    """The broker path: _route_gossip() re-encodes the dict decode() just
    produced, so a field decode() drops is silently zeroed for every peer
    downstream of the relay (see broker.py's encode_gossip(msg))."""
    p       = MODULES[consumer]
    once    = p.decode(p.encode_gossip({**GOSSIP, 'qos': p.QOS_HARD_RT}))
    relayed = p.decode(p.encode_gossip(once))

    assert relayed is not None
    for field in GOSSIP:
        assert relayed[field] == once[field], f"{field} lost in the relay hop"


@pytest.mark.parametrize("consumer", SIMS)
def test_decode_rejects_a_gossip_frame_from_another_schema_version(consumer):
    """The frame carries its own version because BLE and syslink P2P
    deliver it with no header wrapper at all — that trailing byte is the
    only version check those transports get."""
    p     = MODULES[consumer]
    frame = bytearray(p.encode_gossip({**GOSSIP, 'qos': p.QOS_HARD_RT}))
    frame[-1] = p.WIRE_VERSION + 1          # last payload byte == version

    assert p.decode(bytes(frame)) is None
