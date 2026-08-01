#!/usr/bin/env python3
"""
gen_wire_protocol.py — generate the Python wire-format mirrors of
tapestry/wire.h from wire.h itself.

wire.h is the single source of truth for the L3 on-wire frame layouts.
Historically the Python side (three independent orchestrator/telemetry
scripts) each hand-typed their own struct.Struct(...) format strings —
three unsynchronized copies of the same information, with nothing to
catch drift if wire.h changed and a copy was missed.  This script reads
wire.h directly and regenerates the mirrored section of each consumer,
so there is exactly one place a wire format is ever actually written.

What gets generated (and MUST NOT be hand-edited): TAPESTRY_WIRE_VERSION,
the tapestry_msg_type_t constants, and the four struct.Struct(...) format
objects (HEADER_FMT, GOSSIP_FMT, METRIC_FMT, SCR_METRIC_FMT).  Everything
else in each consumer file — ports, sim-only extensions (MSG_CONTROL,
CTRL_FMT — defined in sim_protocol.h, not wire.h), and all encode/decode
logic — is ordinary hand-maintained code and is left untouched.

Usage:
    python3 tapestry-os/tools/gen_wire_protocol.py [--check]

--check exits 1 if any target file's generated block would change,
without writing anything (for CI: catches a wire.h edit that nobody
regenerated for).

Each target file must already contain the marker pair:
    # === BEGIN GENERATED WIRE PROTOCOL (tools/gen_wire_protocol.py — DO NOT EDIT) ===
    # === END GENERATED WIRE PROTOCOL ===
This script only replaces text between an existing marker pair — it does
not decide where to insert one in a file that doesn't have it yet.
"""

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
WIRE_H    = REPO_ROOT / "tapestry-os" / "include" / "tapestry" / "wire.h"

TARGETS = [
    REPO_ROOT / "tapestry-csm-sim" / "orchestrator" / "protocol.py",
    REPO_ROOT / "tapestry-scr-sim" / "orchestrator" / "protocol.py",
    REPO_ROOT / "tapestry-scr-hw" / "telemetry" / "protocol.py",
]

BEGIN_MARKER = "# === BEGIN GENERATED WIRE PROTOCOL (tools/gen_wire_protocol.py — DO NOT EDIT) ==="
END_MARKER   = "# === END GENERATED WIRE PROTOCOL ==="

# C type -> Python struct format char (fixed-width types only; wire.h rule
# is "no OS types, pure C99 + <stdint.h>", so this is the complete set
# wire.h itself permits).
C_TYPE_TO_FMT = {
    "uint8_t":  "B",
    "int8_t":   "b",
    "uint16_t": "H",
    "int16_t":  "h",
    "uint32_t": "I",
    "int32_t":  "i",
    "float":    "f",
}

# (C struct name, Python Struct variable name) — the four wire.h frames
# with a Python mirror.  Order here is the order emitted, not significant
# to parsing.
STRUCTS = [
    ("tapestry_msg_header_t",       "HEADER_FMT"),
    ("tapestry_gossip_frame_t",     "GOSSIP_FMT"),
    ("tapestry_metric_frame_t",     "METRIC_FMT"),
    ("tapestry_scr_metric_frame_t", "SCR_METRIC_FMT"),
]


class WireParseError(ValueError):
    pass


def _strip_comments(text: str) -> str:
    return re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)


def parse_version(text: str) -> int:
    m = re.search(r"#define\s+TAPESTRY_WIRE_VERSION\s+(\d+)u?", text)
    if not m:
        raise WireParseError("TAPESTRY_WIRE_VERSION not found in wire.h")
    return int(m.group(1))


def parse_msg_types(text: str) -> list[tuple[str, int]]:
    m = re.search(r"typedef\s+enum\s*\{(.*?)\}\s*tapestry_msg_type_t\s*;",
                  text, re.DOTALL)
    if not m:
        raise WireParseError("tapestry_msg_type_t enum not found in wire.h")
    body = _strip_comments(m.group(1))
    types = []
    for name, value in re.findall(r"TAPESTRY_MSG_(\w+)\s*=\s*(\d+)", body):
        types.append((f"MSG_{name}", int(value)))
    if not types:
        raise WireParseError("no TAPESTRY_MSG_* constants found")
    return types


def parse_struct_fields(text: str, struct_name: str) -> list[tuple[str, str]]:
    """Returns [(c_type, field_name), ...] in declaration order.

    Anchored from the END (the closing "} __attribute__((packed)) NAME;",
    unique per struct) back to the nearest preceding "typedef struct {" —
    not matched forward with a non-greedy '.*?', which would happily
    absorb an earlier struct's body too when two typedefs share the
    generic "typedef struct {" opener (as every struct in wire.h does).
    """
    end_re = re.compile(r"\}\s*__attribute__\(\(packed\)\)\s*"
                        + re.escape(struct_name) + r"\s*;")
    end_m = end_re.search(text)
    if not end_m:
        raise WireParseError(f"struct {struct_name} not found in wire.h "
                             f"(expected a __attribute__((packed)) typedef)")
    start_re = re.compile(r"typedef\s+struct\s*\{")
    starts = list(start_re.finditer(text, 0, end_m.start()))
    if not starts:
        raise WireParseError(f"struct {struct_name}: no matching "
                             f"'typedef struct {{' found before it")
    body = _strip_comments(text[starts[-1].end():end_m.start()])
    fields = []
    for line in body.splitlines():
        line = line.strip().rstrip(";").strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 2:
            raise WireParseError(f"{struct_name}: cannot parse field line "
                                 f"{line!r} (expected 'TYPE name;')")
        ctype, field = parts
        if ctype not in C_TYPE_TO_FMT:
            raise WireParseError(f"{struct_name}.{field}: unknown wire type "
                                 f"{ctype!r} (known: "
                                 f"{sorted(C_TYPE_TO_FMT)})")
        fields.append((ctype, field))
    if not fields:
        raise WireParseError(f"struct {struct_name}: no fields parsed")
    return fields


def struct_format(fields: list[tuple[str, str]]) -> str:
    return "<" + "".join(C_TYPE_TO_FMT[ctype] for ctype, _ in fields)


def render_block(wire_text: str) -> str:
    version   = parse_version(wire_text)
    msg_types = parse_msg_types(wire_text)

    lines = [BEGIN_MARKER]
    lines.append("# Mirrors tapestry-os/include/tapestry/wire.h.")
    lines.append("# Regenerate after any wire.h change:")
    lines.append("#   python3 tapestry-os/tools/gen_wire_protocol.py")
    lines.append("#")
    lines.append("# WIRE_VERSION bumps whenever a struct format below changes;")
    lines.append("# decode() rejects a header whose version does not match —")
    lines.append("# see wire.h's \"Wire schema version\" section for why.")
    lines.append("")
    lines.append(f"WIRE_VERSION = {version}")
    lines.append("")
    name_width = max(len(name) for name, _ in msg_types)
    for name, value in msg_types:
        lines.append(f"{name:<{name_width}} = {value}")
    lines.append("")

    rendered = []
    for struct_name, py_name in STRUCTS:
        fields = parse_struct_fields(wire_text, struct_name)
        fmt = struct_format(fields)
        size = sum({"B": 1, "b": 1, "H": 2, "h": 2, "I": 4, "i": 4, "f": 4}[c]
                   for c in fmt[1:])
        field_list = ",".join(f for _, f in fields)
        rendered.append((py_name, fmt, size, field_list))

    py_width = max(len(py_name) for py_name, _, _, _ in rendered)
    for py_name, fmt, size, field_list in rendered:
        lines.append(f"{py_name:<{py_width}} = struct.Struct({fmt!r})"
                     f"  # {size:2d} bytes: {field_list}")

    lines.append(END_MARKER)
    return "\n".join(lines)


def apply_block(target: Path, block: str) -> tuple[str, bool]:
    """Returns (new_text, changed)."""
    text = target.read_text()
    pattern = re.compile(
        re.escape(BEGIN_MARKER) + r".*?" + re.escape(END_MARKER), re.DOTALL)
    if not pattern.search(text):
        raise WireParseError(
            f"{target}: no BEGIN/END GENERATED WIRE PROTOCOL markers found — "
            f"this script only replaces an existing marked block, it does "
            f"not decide where to insert one in a hand-written file")
    new_text = pattern.sub(lambda _m: block, text, count=1)
    return new_text, new_text != text


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if regenerating would change any file; "
                         "write nothing")
    args = ap.parse_args()

    try:
        wire_text = WIRE_H.read_text()
        block     = render_block(wire_text)
    except (OSError, WireParseError) as e:
        print(f"gen_wire_protocol: {e}", file=sys.stderr)
        return 1

    any_changed = False
    for target in TARGETS:
        try:
            new_text, changed = apply_block(target, block)
        except (OSError, WireParseError) as e:
            print(f"gen_wire_protocol: {e}", file=sys.stderr)
            return 1
        rel = target.relative_to(REPO_ROOT)
        if not changed:
            print(f"gen_wire_protocol: {rel} — already up to date")
            continue
        any_changed = True
        if args.check:
            print(f"gen_wire_protocol: {rel} — STALE (regeneration needed)",
                  file=sys.stderr)
        else:
            target.write_text(new_text)
            print(f"gen_wire_protocol: {rel} — updated")

    if args.check and any_changed:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
