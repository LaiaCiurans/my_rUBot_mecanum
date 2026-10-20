#!/usr/bin/env python3
"""Wait until required serial and non-serial Linux devices are usable."""

import argparse
import os
import sys
import time

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--serial", action="append", default=[], metavar="PATH",
        help="serial device to open; may be specified more than once",
    )
    parser.add_argument(
        "--device", action="append", default=[], metavar="PATH",
        help="non-serial device that must exist; may be specified more than once",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def wait_for_path(path, timeout, serial_device):
    serial_module = None
    if serial_device:
        try:
            import serial as serial_module
        except Exception as exc:  # pragma: no cover
            print(f"pyserial not available: {exc}", file=sys.stderr)
            return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not os.path.exists(path):
            time.sleep(0.2)
            continue

        if serial_device:
            try:
                port = serial_module.Serial(path, timeout=0.5)
                port.close()
                print(f"{path}: serial port opened successfully")
                return True
            except Exception:
                time.sleep(0.2)
                continue
        print(f"{path}: device file exists")
        return True
    return False


def main():
    args = parse_args()
    if not args.serial and not args.device:
        print("ERROR: no devices were specified", file=sys.stderr)
        return 2

    for path in args.serial:
        if not wait_for_path(path, args.timeout, serial_device=True):
            print(f"ERROR: serial device not ready in time: {path}", file=sys.stderr)
            return 1

    for path in args.device:
        if not wait_for_path(path, args.timeout, serial_device=False):
            print(f"ERROR: device not ready in time: {path}", file=sys.stderr)
            return 1

    print("All requested hardware devices are ready")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
