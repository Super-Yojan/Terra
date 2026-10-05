#!/usr/bin/env python3
"""Run the actual Swift UniFFI client against a local Python Zenoh peer."""
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time
import zenoh

root = Path(__file__).resolve().parent.parent
with socket.socket() as reservation:
    reservation.bind(("127.0.0.1", 0))
    port = reservation.getsockname()[1]
endpoint = f"tcp/127.0.0.1:{port}"
config = zenoh.Config()
config.insert_json5("mode", '"peer"')
config.insert_json5("listen/endpoints", json.dumps([endpoint]))
config.insert_json5("scouting/multicast/enabled", "false")
samples = []
lock = threading.Lock()
def receive(sample):
    with lock:
        samples.append((str(sample.key_expr), json.loads(sample.payload.to_bytes())))
def depth_packet(sequence):
    header = {
        "rover_id": 9,
        "version": 1,
        "width": 1,
        "height": 1,
        "sequence": sequence,
        "received_at": 1.0,
        "encoding": "32FC1_LE",
        "vertical_fov": math.radians(60),
        "near": 0.05,
        "far": 30.0,
        "exposure_time": 0.5,
        "camera": {"x": 0.0, "y": 0.0, "z": 0.5, "qx": -0.5, "qy": 0.5, "qz": -0.5, "qw": 0.5},
        "body": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    }
    return json.dumps(header).encode() + b"\n" + struct.pack("<f", 2.0)
with zenoh.open(config) as session:
    subscriber = session.declare_subscriber("terra/rover/*/cmd_vel", receive)
    stop = threading.Event()
    def publish_depth():
        sequence = 1
        while not stop.is_set():
            session.put("terra/rover/9/camera/depth", depth_packet(sequence))
            sequence += 1
            time.sleep(0.05)
    publisher = threading.Thread(target=publish_depth, daemon=True)
    publisher.start()
    try:
        subprocess.run([str(root / "scripts/check-swift.sh")], cwd=root,
                       env={**os.environ, "TERRA_ZENOH_SMOKE_ENDPOINT": endpoint},
                       check=True, timeout=90)
    finally:
        stop.set()
        publisher.join(timeout=2)
    subscriber.undeclare()
with lock:
    captured = list(samples)
assert captured and all(key == "terra/rover/9/cmd_vel" for key, _ in captured), captured
moving = next(i for i, (_, cmd) in enumerate(captured) if cmd == {"linear": 0.5, "angular": 0.2})
assert any(cmd == {"linear": 0.0, "angular": 0.0} for _, cmd in captured[moving + 1:]), captured
assert captured[-1][1] == {"linear": 0.0, "angular": 0.0}, captured
print(f"Validated {len(captured)} Swift-origin commands, selected rover, lease expiry and final zero")
