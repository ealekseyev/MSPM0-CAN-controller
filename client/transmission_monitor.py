#!/usr/bin/env python3
"""Transmission CAN monitor.

Reads the same serial/log frame format as car_dash.py and prints raw CAN frames
that look relevant to transmission state. This is meant for confirming the real
CAN ID/byte for Park/Reverse/Neutral/Drive when the dashboard mapping is wrong.

Usage:
    python client/transmission_monitor.py COM3
    python client/transmission_monitor.py COM3 -b 115200 --scan-all
    python client/transmission_monitor.py --replay log.txt --scan-all
"""

import argparse
import os
import sys
import time

try:
    import serial.tools.list_ports
except ImportError:
    serial = None

sys.path.insert(0, os.path.dirname(__file__))

from car_dash import FileSource, SerialSource, parse_line


GEAR_CODES = {
    0xE3: "Park",
    0xC2: "Reverse",
    0xD1: "Neutral",
    0xC5: "Drive gear 1",
    0xC6: "Drive gear 2",
    0xC7: "Drive gear 3",
    0xC8: "Drive gear 4",
    0xC9: "Drive gear 5",
    0xCA: "Drive gear 6",
}

DEFAULT_WATCH_IDS = (0x304,)


def parse_can_id(value):
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid CAN ID: {value}") from exc


def fmt_data(data):
    return " ".join(f"{byte:02X}" for byte in data)


def decode_first_byte(data):
    if not data:
        return "empty"
    return GEAR_CODES.get(data[0], f"unknown 0x{data[0]:02X}")


def matching_gear_bytes(data):
    return [
        (idx, byte, GEAR_CODES[byte])
        for idx, byte in enumerate(data)
        if byte in GEAR_CODES
    ]


def print_frame(elapsed, can_id, data, reason):
    print(
        f"{elapsed:9.3f}s  {reason:<24}  "
        f"id=0x{can_id:03X}  dlc={len(data)}  data={fmt_data(data)}",
        flush=True,
    )


def run_monitor(source, source_name, watch_ids, scan_all, changes_only):
    print(f"Transmission CAN monitor [{source_name}]")
    print("Known byte codes: " + ", ".join(f"{name}=0x{code:02X}" for code, name in GEAR_CODES.items()))
    print("Watching IDs: " + ", ".join(f"0x{can_id:03X}" for can_id in sorted(watch_ids)))
    if scan_all:
        print("Scan mode: also printing any frame containing one of those byte values")
    print("Ctrl+C to stop\n")

    start = time.monotonic()
    last_seen = {}

    try:
        while True:
            printed = False
            for line in source.read_lines():
                can_id, data = parse_line(line)
                if can_id is None:
                    continue

                elapsed = time.monotonic() - start

                if can_id in watch_ids:
                    gear = decode_first_byte(data)
                    previous = last_seen.get(can_id)
                    current = tuple(data)
                    last_seen[can_id] = current
                    if not changes_only or previous != current:
                        print_frame(elapsed, can_id, data, f"watch {gear}")
                        printed = True
                    continue

                if scan_all:
                    matches = matching_gear_bytes(data)
                    if matches:
                        labels = ", ".join(
                            f"b{idx}=0x{byte:02X}/{gear}"
                            for idx, byte, gear in matches
                        )
                        print_frame(elapsed, can_id, data, f"candidate {labels}")
                        printed = True

            if not printed:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        source.close()


def main():
    parser = argparse.ArgumentParser(
        description="Print raw CAN frames relevant to transmission gear state."
    )
    parser.add_argument("port", nargs="?", help="Serial port, e.g. COM3")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="Baud rate")
    parser.add_argument("-l", "--list", action="store_true", help="List serial ports")
    parser.add_argument("--replay", metavar="FILE", help="Replay from a log file")
    parser.add_argument(
        "--watch-id",
        action="append",
        type=parse_can_id,
        dest="watch_ids",
        help="CAN ID to decode as gear byte 0. Can be repeated. Default: 0x304",
    )
    parser.add_argument(
        "--scan-all",
        action="store_true",
        help="Print any frame containing known gear byte values anywhere in data",
    )
    parser.add_argument(
        "--changes-only",
        action="store_true",
        help="For watched IDs, print only when data changes",
    )
    args = parser.parse_args()

    if args.list:
        if serial is None:
            print("pyserial is not installed")
            return 1
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device} - {port.description}")
        return 0

    if args.replay:
        source = FileSource(args.replay)
        source_name = f"Replay: {args.replay}"
    elif args.port:
        source = SerialSource(args.port, args.baudrate)
        source_name = f"{args.port} @ {args.baudrate}"
    else:
        parser.error("specify a serial port or --replay FILE")

    watch_ids = set(args.watch_ids or DEFAULT_WATCH_IDS)
    run_monitor(source, source_name, watch_ids, args.scan_all, args.changes_only)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
