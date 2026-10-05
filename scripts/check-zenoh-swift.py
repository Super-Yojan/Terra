#!/usr/bin/env python3
"""Run the actual Swift UniFFI client against a local Python Zenoh peer."""
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
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
with zenoh.open(config) as session:
    subscriber = session.declare_subscriber("terra/rover/*/cmd_vel", receive)
    subprocess.run([str(root / "scripts/check-swift.sh")], cwd=root,
                   env={**os.environ, "TERRA_ZENOH_SMOKE_ENDPOINT": endpoint},
                   check=True, timeout=60)
    subscriber.undeclare()
with lock:
    captured = list(samples)
assert captured and all(key == "terra/rover/9/cmd_vel" for key, _ in captured), captured
moving = next(i for i, (_, cmd) in enumerate(captured) if cmd == {"linear": 0.5, "angular": 0.2})
assert any(cmd == {"linear": 0.0, "angular": 0.0} for _, cmd in captured[moving + 1:]), captured
assert captured[-1][1] == {"linear": 0.0, "angular": 0.0}, captured
print(f"Validated {len(captured)} Swift-origin commands, selected rover, lease expiry and final zero")
