#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["eclipse-zenoh==1.10.1"]
# ///
"""Download the current occupancy map from the fleet router and save it.

TerraPhone publishes its map on `<prefix>/<rover>/map/occupancy` while it is
connected to the fleet router. By default this connects to the same GCP router
the apps use, as user `terra`; set TERRA_ROUTER_PASSWORD (or --password) to the
fleet password. A full map is sent when it changes and at least every two
seconds, so this waits for the next one.

Saves two files next to each other:
  <name>.json  the payload exactly as published (width, height, resolution,
               origin_x, origin_y, run_id, sequence, occupancy)
  <name>.png   a picture of the grid: white free, black occupied, grey unknown

Examples:
  TERRA_ROUTER_PASSWORD=... uv run scripts/dashboard/save-occupancy-map.py
  TERRA_ROUTER_PASSWORD=... uv run scripts/dashboard/save-occupancy-map.py --rover 1 --out maps/run1
  uv run scripts/dashboard/save-occupancy-map.py --endpoint tcp/127.0.0.1:7448 --user ""   # local Mac router
"""
import argparse
import json
import os
import queue
import struct
import sys
import time
import zlib
from pathlib import Path

import zenoh

# Same defaults as TerraFleetDefaults (iOS) and TerraSession (Android).
GCP_ROUTER = "tls/argos.34.135.204.210.sslip.io:7448"


def parse_args():
    parser = argparse.ArgumentParser(description="Download the current occupancy map from the fleet router and save it as JSON and PNG.")
    parser.add_argument("--endpoint", default=GCP_ROUTER, help="Zenoh router (default: the GCP router, %(default)s)")
    parser.add_argument("--prefix", default="terra/phone", help="topic prefix (default: %(default)s)")
    parser.add_argument("--rover", default="0", help="rover ID, or * for the first rover that publishes (default: %(default)s)")
    parser.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for a map (default: %(default)s)")
    parser.add_argument("--out", help="output path without extension (default: occupancy-<rover>-<time>)")
    parser.add_argument("--user", default=os.environ.get("TERRA_ROUTER_USER", "terra"), help="router user, empty for none (default: %(default)s, or TERRA_ROUTER_USER)")
    parser.add_argument("--password", default=os.environ.get("TERRA_ROUTER_PASSWORD"), help="router password (or TERRA_ROUTER_PASSWORD)")
    return parser.parse_args()


def open_session(args):
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")
    if args.user or args.password:
        config.insert_json5("transport/auth/usrpwd/user", json.dumps(args.user or ""))
        config.insert_json5("transport/auth/usrpwd/password", json.dumps(args.password or ""))
    try:
        return zenoh.open(config)
    except zenoh.ZError as error:
        hint = "Check the address, and that TERRA_ROUTER_PASSWORD (or --password) is the fleet password." if args.user else "Is the router running?"
        sys.exit(f"Could not connect to {args.endpoint}: {error}\n{hint}")


def wait_for_map(session, key, timeout):
    maps = queue.Queue()

    def receive(sample):
        try:
            maps.put(json.loads(sample.payload.to_bytes()))
        except ValueError:
            pass  # not JSON; ignore

    subscriber = session.declare_subscriber(key, receive)
    try:
        return maps.get(timeout=timeout)
    except queue.Empty:
        return None
    finally:
        subscriber.undeclare()


def check(grid):
    for field in ("width", "height", "resolution", "origin_x", "origin_y", "occupancy"):
        if field not in grid:
            sys.exit(f"map is missing '{field}'")
    if grid["width"] * grid["height"] != len(grid["occupancy"]):
        sys.exit(f"map size {grid['width']}x{grid['height']} does not match {len(grid['occupancy'])} cells")


def png_bytes(grid):
    """Greyscale PNG. Row 0 of the grid is the bottom of the image (+Y up)."""
    width, height, cells = grid["width"], grid["height"], grid["occupancy"]
    rows = []
    for y in reversed(range(height)):
        row = bytearray([0])  # PNG filter type: none
        for value in cells[y * width:(y + 1) * width]:
            row.append(128 if value < 0 else 255 - round(min(value, 100) * 255 / 100))
        rows.append(bytes(row))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)  # 8-bit greyscale
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(b"".join(rows))) + chunk(b"IEND", b"")


def main():
    args = parse_args()
    if args.user and not args.password:
        sys.exit(f"Set TERRA_ROUTER_PASSWORD (or --password) to the fleet password for user '{args.user}'.\n"
                 "For a router without a login, pass --user \"\".")
    key = f"{args.prefix}/{args.rover}/map/occupancy"
    print(f"Waiting up to {args.timeout:g}s for {key} on {args.endpoint} ...")
    with open_session(args) as session:
        grid = wait_for_map(session, key, args.timeout)
    if grid is None:
        sys.exit("No map received. Is TerraPhone connected to this router and mapping (phone sensors running)?")
    check(grid)

    rover = grid.get("rover_id", args.rover)
    out = Path(args.out or f"occupancy-{rover}-{time.strftime('%Y%m%d-%H%M%S')}")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(grid))
    out.with_suffix(".png").write_bytes(png_bytes(grid))

    known = sum(1 for v in grid["occupancy"] if v >= 0)
    print(f"Saved {out.with_suffix('.json')} and {out.with_suffix('.png')}")
    print(f"  rover {rover}, run {grid.get('run_id', '?')}, sequence {grid.get('sequence', '?')}")
    print(f"  {grid['width']}x{grid['height']} cells at {grid['resolution']} m, origin ({grid['origin_x']}, {grid['origin_y']}) m, {known} cells observed")


if __name__ == "__main__":
    main()
