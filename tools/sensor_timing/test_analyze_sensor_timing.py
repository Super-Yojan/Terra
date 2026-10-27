"""Synthetic recordings with known causes; run: python3 -m unittest test_analyze_sensor_timing"""
import heapq
import tempfile
import unittest
from pathlib import Path

import analyze_sensor_timing as ast


def simulate(seconds=5.0, queue_stall=0.0, ios_batch=0.0, sensor_gap=0.0, bare=True):
    """A serial terra.control queue fed by 100 Hz IMU, a coalescing 10 ms timer and
    30 Hz AR frames. Once a second one of three faults is injected."""
    rows, events, seq = [], [], 0
    def push(t, kind, payload=None):
        nonlocal seq
        heapq.heappush(events, (t, seq, kind, payload)); seq += 1
    for i in range(int(seconds * 100)):
        ts = i * 0.01
        if sensor_gap and 0.5 <= ts % 1.0 < 0.5 + sensor_gap:
            continue  # iOS took no sample
        delay = 0.003
        if ios_batch and 0.5 <= ts % 1.0 < 0.5 + ios_batch:
            delay = 0.5 + ios_batch - ts % 1.0 + 0.003  # held, then delivered in a burst
        push(ts + delay, "imu", ts)
        if bare:
            rows.append(("bare_imu", ts, ts + delay))
    for i in range(int(seconds * 30)):
        ts = i / 30
        push(ts + 0.004, "ar", ts)
    for i in range(int(seconds * 100)):
        push(i * 0.01 + 0.0005, "timer", None)
    busy_until, last_imu, last_vio, timer_pending = 0.0, float("nan"), float("nan"), False
    queue, out = [], []
    while events:
        t, _, kind, payload = heapq.heappop(events)
        if kind == "timer":
            if timer_pending:
                continue
            timer_pending = True
        queue.append((t, kind, payload))
        # run everything whose turn has come
        while queue and (not events or max(busy_until, queue[0][0]) <= events[0][0]):
            enq, k, p = queue.pop(0)
            start = max(busy_until, enq)
            if k == "imu":
                dur = 0.0002
                last_imu = p
                out.append(f"imu,0,{p},{enq},{start},{start + dur},,")
            elif k == "timer":
                timer_pending = False
                dur = 0.001
                out.append(f"tick,0,{last_imu},,{start},{start + dur},{last_vio},1")
            else:
                dur = 0.005
                if queue_stall and abs(p % 1.0 - 0.5) < 0.016:
                    dur = queue_stall
                last_vio = p
                out.append(f"ar,0,{p},{enq},{start},{start + 0.001},{start + dur},0")
            busy_until = start + dur
    lines = ["# condition=synthetic", "# build=release", "stream,seq,t_sensor,t_os,t_queue,t_done,a,b"]
    seqs = {}
    for line in out + [f"{s},0,{a},{b},,,," for s, a, b in rows]:
        stream, _, rest = line.split(",", 2)
        seqs[stream] = seqs.get(stream, -1) + 1
        lines.append(f"{stream},{seqs[stream]},{rest}")
    f = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False)
    f.write("\n".join(lines) + "\n"); f.close()
    return ast.summarise(ast.load(Path(f.name)))


class Attribution(unittest.TestCase):
    def test_clean_run_has_no_stale_ticks(self):
        s = simulate()
        self.assertEqual(s["stale_imu"]["stale_ticks"], 0)
        self.assertAlmostEqual(s["streams"]["imu"]["rate_hz"], 100, delta=1)

    def test_long_ar_frame_blames_terra_queue(self):
        s = simulate(queue_stall=0.16)
        c = s["stale_imu"]["causes"]
        self.assertGreater(c.get("terra_queue", 0), 0)
        self.assertEqual(c.get("ios_delivery", 0) + c.get("ios_sensor_gap", 0), 0)
        self.assertEqual(set(s["stale_imu"]["terra_blocker"]), {"ar_frame"})
        self.assertGreater(s["tick"]["late_over_turning_max_step"], 0)

    def test_ios_batching_blames_delivery(self):
        s = simulate(ios_batch=0.15)
        c = s["stale_imu"]["causes"]
        self.assertGreater(c.get("ios_delivery", 0), 0)
        self.assertEqual(c.get("terra_queue", 0), 0)
        self.assertEqual(s["stale_imu"]["bare_logger_fresh_at_stale_tick"], 0)
        self.assertGreater(s["streams"]["imu"]["batched_deliveries"], 0)

    def test_missing_samples_blame_sensor_gap(self):
        s = simulate(sensor_gap=0.15)
        c = s["stale_imu"]["causes"]
        self.assertGreater(c.get("ios_sensor_gap", 0), 0)
        self.assertEqual(c.get("terra_queue", 0), 0)
        self.assertGreater(s["streams"]["imu"]["sensor_gaps_over_2x"], 0)

    def test_report_renders(self):
        self.assertIn("synthetic", ast.report([simulate(queue_stall=0.16)]))


if __name__ == "__main__":
    unittest.main()
