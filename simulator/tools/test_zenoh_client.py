import json
import os
import struct
import threading
import time
import unittest

import numpy as np
from zenoh_client import decode_frame, encode_goal


class FrameTests(unittest.TestCase):
    def test_depth_metres_nan_and_row_order(self):
        metadata = {"version": 1, "width": 2, "height": 1, "encoding": "32FC1_LE", "sequence": 3}
        header, image = decode_frame(json.dumps(metadata).encode() + b"\n" + struct.pack("<ff", 2.5, float("nan")))
        self.assertEqual(header["sequence"], 3)
        self.assertEqual(image.shape, (1, 2))
        self.assertEqual(image[0, 0], 2.5)
        self.assertTrue(np.isnan(image[0, 1]))

    def test_rgba_preserves_channels(self):
        metadata = {"version": 1, "width": 1, "height": 1, "encoding": "RGBA8_SRGB"}
        _, image = decode_frame(json.dumps(metadata).encode() + b"\n" + bytes([10, 20, 30, 255]))
        self.assertEqual(image.tolist(), [[[10, 20, 30, 255]]])

    def test_goal_payload_matches_the_terra_contract(self):
        class Args:
            cancel = False
            x, y, yaw = 10.0, -2.0, None
            lat = lon = token = None
        self.assertEqual(json.loads(encode_goal(Args())),
                         {"frame": "local", "x": 10.0, "y": -2.0})
        Args.lat, Args.lon, Args.x, Args.y = 38.8299, -77.3075, None, None
        Args.token = "goal-1"
        self.assertEqual(json.loads(encode_goal(Args())), {
            "frame": "wgs84", "latitude": 38.8299, "longitude": -77.3075, "token": "goal-1"})
        Args.cancel = True
        self.assertEqual(json.loads(encode_goal(Args())), {"cancel": True})

    def test_rejects_truncated_payload(self):
        metadata = {"version": 1, "width": 2, "height": 1, "encoding": "32FC1_LE"}
        with self.assertRaises(ValueError):
            decode_frame(json.dumps(metadata).encode() + b"\n" + bytes(4))

    @unittest.skipUnless(os.environ.get("TERRA_TEST_ZENOH_ENDPOINT"), "requires Rust round-trip test server")
    def test_rust_interoperability(self):
        import zenoh
        config = zenoh.Config()
        config.insert_json5("mode", json.dumps("client"))
        config.insert_json5("connect/endpoints", json.dumps([os.environ["TERRA_TEST_ZENOH_ENDPOINT"]]))
        config.insert_json5("scouting/multicast/enabled", "false")
        event, seen, errors = threading.Event(), set(), []
        with zenoh.open(config) as session:
            def receive(sample):
                try:
                    header, image = decode_frame(bytes(sample.payload))
                    if header["encoding"] == "32FC1_LE":
                        self.assertEqual(image[0, 0], 2.0)
                        self.assertTrue(np.isnan(image[0, 1]))
                        seen.add("depth")
                    else:
                        self.assertEqual(image.tolist(), [[[10,20,30,255], [40,50,60,255]]])
                        seen.add("rgb")
                    if len(seen) == 2:
                        event.set()
                except Exception as error:
                    errors.append(error)
                    event.set()
            subscriber = session.declare_subscriber("terra/rover/0/camera/*", receive)
            self.assertTrue(event.wait(5), "RGB and depth did not arrive")
            self.assertFalse(errors, errors)
            for _ in range(10):
                session.put("terra/rover/0/cmd_vel", json.dumps({"linear": 1.0, "angular": 0.25}))
                time.sleep(0.05)
            session.put("terra/rover/0/cmd_vel", json.dumps({"linear": 0.0, "angular": 0.0}))
            subscriber.undeclare()


if __name__ == "__main__":
    unittest.main()
