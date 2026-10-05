#!/usr/bin/env python3
"""Receive Terra camera frames or send a repeated differential-drive twist."""
import argparse
import json
import math
from pathlib import Path
import threading
import time


def decode_frame(payload):
    """Return wire metadata and a contiguous numpy array; NaN depth stays NaN."""
    import numpy as np
    header_bytes, pixels = payload.split(b"\n", 1)
    header = json.loads(header_bytes)
    if header.get("version") != 1:
        raise ValueError("Unsupported Terra frame version")
    width, height = header["width"], header["height"]
    if not (isinstance(width, int) and isinstance(height, int)
            and 0 < width <= 2048 and 0 < height <= 2048):
        raise ValueError("Invalid frame dimensions")
    encoding = header["encoding"]
    if encoding == "32FC1_LE":
        shape, dtype = (height, width), np.dtype("<f4")
    elif encoding == "RGBA8_SRGB":
        shape, dtype = (height, width, 4), np.dtype("u1")
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    if len(pixels) != width * height * 4:
        raise ValueError("Incorrect frame payload length")
    return header, np.frombuffer(pixels, dtype=dtype).reshape(shape).copy()


def main():
    import zenoh
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    parser.add_argument("--prefix", default="terra/rover")
    parser.add_argument("--rover", type=int, default=0, help="Rover ID from fleet/state")
    actions = parser.add_subparsers(dest="action", required=True)
    frames = actions.add_parser("frames", help="Receive RGB and depth frames")
    frames.add_argument("--all", action="store_true", help="Receive every rover camera")
    frames.add_argument("--output", type=Path, help="Overwrite latest .npy frames and JSON metadata here")
    drive = actions.add_parser("drive", help="Repeat a velocity command at 20 Hz")
    drive.add_argument("--linear", type=float, default=0.0, help="Forward speed in m/s")
    drive.add_argument("--angular", type=float, default=0.0, help="Left turn in rad/s")
    drive.add_argument("--seconds", type=float, default=5.0)
    fleet = actions.add_parser("fleet", help="List IDs or change the fleet size")
    fleet.add_argument("--count", type=int, help="Desired count, 0 through 32")
    args = parser.parse_args()
    if args.rover < 0:
        parser.error("Rover ID must be nonnegative")
    if args.action == "fleet" and args.count is not None and not 0 <= args.count <= 32:
        parser.error("Fleet count must be between 0 and 32")
    if args.action == "drive" and (not all(math.isfinite(x) for x in
            (args.linear, args.angular, args.seconds)) or args.seconds <= 0):
        parser.error("Drive values must be finite and duration must be positive")
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("client"))
    config.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")
    with zenoh.open(config) as session:
        if args.action == "drive":
            key = f"{args.prefix}/{args.rover}/cmd_vel"
            payload = json.dumps({"linear": args.linear, "angular": args.angular})
            deadline = time.monotonic() + args.seconds
            try:
                while time.monotonic() < deadline:
                    session.put(key, payload)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                pass
            finally:
                session.put(key, json.dumps({"linear": 0.0, "angular": 0.0}))
        elif args.action == "fleet":
            event, state = threading.Event(), {}
            def receive_state(sample):
                nonlocal state
                state = json.loads(bytes(sample.payload))
                event.set()
            subscriber = session.declare_subscriber(f"{args.prefix}/fleet/state", receive_state)
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if args.count is not None:
                        session.put(f"{args.prefix}/fleet/size", json.dumps({"count": args.count}))
                    event.wait(0.1)
                    if event.is_set() and (args.count is None or state["count"] == args.count):
                        print(json.dumps(state, indent=2))
                        break
                    event.clear()
                else:
                    raise SystemExit("No fleet acknowledgement within 5 seconds")
            finally:
                subscriber.undeclare()
        else:
            import numpy as np
            lock, latest = threading.Lock(), {}
            if args.output:
                args.output.mkdir(parents=True, exist_ok=True)

            def receive(sample):
                kind = str(sample.key_expr).rsplit("/", 1)[-1]
                if kind not in ("rgb", "depth"):
                    return
                try:
                    decoded = decode_frame(bytes(sample.payload))
                    with lock:
                        latest[(decoded[0]["rover_id"], kind)] = decoded
                except (ValueError, KeyError, TypeError) as error:
                    print(f"Invalid {kind} frame: {error}")

            sensor_id = "*" if args.all else str(args.rover)
            subscriber = session.declare_subscriber(f"{args.prefix}/{sensor_id}/camera/*", receive)
            print("Receiving frames; Ctrl-C to stop. RGB and depth captures are independent.")
            try:
                while True:
                    time.sleep(0.1)
                    with lock:
                        pending, latest = latest, {}
                    for (rover_id, kind), (header, image) in pending.items():
                        pose = ""
                        camera = header.get("camera")
                        body = header.get("body")
                        if isinstance(camera, dict) and "x" in camera and "z" in camera:
                            pose += f" camera=({float(camera['x']):.2f},{float(camera['y']):.2f},{float(camera['z']):.2f})"
                        if isinstance(body, dict) and "yaw" in body:
                            pose += f" body=({float(body['x']):.2f},{float(body['y']):.2f},yaw={float(body['yaw']):.2f})"
                        print(f"rover={rover_id} {kind}: sequence={header['sequence']} shape={image.shape} "
                              f"received_at={header['received_at']:.3f}s{pose}")
                        if args.output:
                            destination = args.output / str(rover_id)
                            destination.mkdir(parents=True, exist_ok=True)
                            np.save(destination / f"{kind}.npy", image)
                            (destination / f"{kind}.json").write_text(json.dumps(header, indent=2))
                            if kind == "rgb":
                                rgb = image[:, :, :3].tobytes()
                                (destination / "rgb.ppm").write_bytes(
                                    f"P6\n{header['width']} {header['height']}\n255\n".encode() + rgb)
            except KeyboardInterrupt:
                pass
            finally:
                subscriber.undeclare()


if __name__ == "__main__":
    main()
