#!/usr/bin/env python3
"""
crazyflie_console.py — Crazyflie CRTP console over a Crazyradio USB dongle.

Reads ONLY the firmware log stream (CRTP port 0, channel 0) — not gossip,
telemetry or P2P traffic.  This is a log viewer, not a packet sniffer.

Uses cflib.drivers.crazyradio.Crazyradio directly instead of get_link_driver /
Crazyflie.open_link.  send_packet() is synchronous and returns the radio ACK
immediately; no background thread required.

The nRF51 piggybacks any buffered CRTP data (console port 0 ch 0) from the
STM32 USART6 into the ACK payload.  We poll at ~100 Hz with a null packet.

ONE ADDRESS AT A TIME.  Every Crazyflie ships on the same default address
(0xE7E7E7E7E7), and polling an address that two drones share makes BOTH nRF51s
ACK every poll simultaneously.  The colliding ACKs collapse the link (measured:
~85% ack rate with one drone, 10-70% erratic with two), and the overloaded
nRF51 then corrupts the syslink UART to the STM32 — ck_fail went 3 -> 377 the
moment a second drone powered on (2026-08-24 bench).  Corrupted syslink frames
are dropped before they can be parsed as P2P, so the act of watching the swarm
was destroying the gossip it ran on: flights 15-18 measured 13-17% "P2P
delivery" that was really this, and their console logs were spliced because two
drones' output arrived interleaved on one address with no source ID.

So: give each drone a distinct address, then pass one -a per drone.

Build the second drone with CONFIG_CF21BL_RADIO_ADDR_OVERRIDE=y and
CONFIG_CF21BL_RADIO_ADDR_LSB=232 (decimal; 232 = 0xE8, giving E7E7E7E7E8).
Do NOT reach for cfclient's EEPROM config block: that path needs Bitcraze
firmware, so against a Tapestry build it connects at the link layer and then
hangs forever waiting for a param/log TOC that never comes.  The Kconfig route
sets the address at boot over syslink instead, and deliberately does not
persist — a power-cycle into the bootloader always comes back on the factory
address, so cfloader can never be locked out.

ONE Crazyradio dongle is enough — a
second instance of this script cannot open the same dongle ("Resource busy"),
so instead this polls the addresses round-robin, --switch-ms at a time.  Only
one address is ever being polled, which is the property that matters; each
address gets its own line buffer and its own output file, so the two drones'
console streams can no longer splice into each other either.

Both drones' LEDs go solid (peers visible) the moment nothing is polling a
shared address — that is the confirmation this is fixed.
"""
import argparse
import sys
import time

import cflib.crtp
from cflib.drivers.crazyradio import Crazyradio

DEFAULT_CHANNEL = 80
DEFAULT_ADDRESS = "E7E7E7E7E7"


def parse_address(text):
    """Accept E7E7E7E7E7, 0xE7E7E7E7E7, or E7:E7:E7:E7:E7 — 5 bytes either way."""
    cleaned = text.lower().replace("0x", "").replace(":", "").replace("-", "")
    if len(cleaned) != 10:
        raise argparse.ArgumentTypeError(
            f"address must be 5 bytes (10 hex digits), got {text!r}")
    try:
        return tuple(int(cleaned[i:i + 2], 16) for i in range(0, 10, 2))
    except ValueError as err:
        raise argparse.ArgumentTypeError(
            f"address is not hex: {text!r}") from err


def main():
    ap = argparse.ArgumentParser(
        description="Read one or more Crazyflies' CRTP console over a Crazyradio.",
        epilog="One dongle polls several drones round-robin: -a ADDR1 -a ADDR2.")
    ap.add_argument("-a", "--address", type=parse_address, action="append",
                    metavar="ADDR",
                    help=f"5-byte radio address; repeat for several drones "
                         f"(default {DEFAULT_ADDRESS})")
    ap.add_argument("-c", "--channel", type=int, default=DEFAULT_CHANNEL,
                    help=f"radio channel 0-125 (default {DEFAULT_CHANNEL})")
    ap.add_argument("-o", "--out", metavar="FILE",
                    help="also write the console to FILE. With several -a, "
                         "this is a PREFIX: FILE-<ADDR>.log per drone")
    ap.add_argument("--switch-ms", type=int, default=250, metavar="MS",
                    help="round-robin dwell per address (default 250)")
    ap.add_argument("--stats-every", type=int, default=500, metavar="N",
                    help="print radio ack stats every N polls (0 disables)")
    args = ap.parse_args()

    if not 0 <= args.channel <= 125:
        ap.error(f"channel must be 0-125, got {args.channel}")
    if args.switch_ms < 10:
        ap.error("--switch-ms must be at least 10")

    addresses = args.address or [parse_address(DEFAULT_ADDRESS)]
    multi = len(addresses) > 1

    # Per-address state.  The separate line buffers are the point: one shared
    # buffer is what spliced two drones' console output together in flights
    # 15-18, because CRTP carries no source id.
    class Drone:
        def __init__(self, addr):
            self.addr     = addr
            self.name     = "".join(f"{b:02X}" for b in addr)
            self.tag      = f"[{self.name[-2:]}] " if multi else ""
            self.line_buf = ""
            self.n_total  = 0
            self.n_ack    = 0
            self.fh       = None

    drones = [Drone(a) for a in addresses]

    if args.out:
        for d in drones:
            path = f"{args.out}-{d.name}.log" if multi else args.out
            d.fh = open(path, "a", buffering=1)
            d.path = path

    def emit(d, line):
        print(d.tag + line, flush=True)
        if d.fh:
            d.fh.write(line + "\n")

    cflib.crtp.init_drivers()

    cr = Crazyradio()
    cr.set_channel(args.channel)
    cr.set_data_rate(Crazyradio.DR_2MPS)

    for d in drones:
        emit(d, f"Crazyradio open, polling ch{args.channel}/2M addr {d.name}"
                + (f" -> {d.path}" if d.fh else "") + " (Ctrl-C to quit) ...")
    if multi:
        print(f"round-robin across {len(drones)} addresses, "
              f"{args.switch_ms} ms each", flush=True)

    dwell_polls = max(1, args.switch_ms // 10)   # loop runs at ~100 Hz
    idx = 0

    try:
        while True:
            d = drones[idx % len(drones)]
            idx += 1
            cr.set_address(d.addr)

            for _ in range(dwell_polls):
                # Null uplink packet (port=0xF ch=3 header=0xFF, no data).
                # The nRF51 auto-ACKs and piggybacks buffered console data.
                ack = cr.send_packet([0xFF])
                d.n_total += 1

                if ack and ack.ack:
                    d.n_ack += 1
                    if ack.data:
                        header  = ack.data[0]     # CRTP header byte
                        port    = (header >> 4) & 0xF
                        channel = header & 0x3
                        if port == 0 and channel == 0 and len(ack.data) > 1:
                            # One firmware log line arrives as many small CRTP
                            # packets; buffer until '\n' so nothing prints
                            # mid-line.
                            d.line_buf += bytes(ack.data[1:]).decode(
                                'utf-8', errors='replace')
                            while '\n' in d.line_buf:
                                line, d.line_buf = d.line_buf.split('\n', 1)
                                emit(d, line)

                # Radio ack stats.  Well below ~85% on an address only one
                # drone answers means something else is on it -- see the
                # header.
                if args.stats_every and d.n_total % args.stats_every == 0:
                    emit(d, f'[radio: {d.n_ack}/{d.n_total} acks]')

                time.sleep(0.01)   # 100 Hz poll

    except KeyboardInterrupt:
        pass
    finally:
        cr.close()
        for d in drones:
            if d.line_buf:
                emit(d, d.line_buf)
            emit(d, "Done.")
            if d.fh:
                d.fh.close()


if __name__ == '__main__':
    sys.exit(main())
