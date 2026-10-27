#!/usr/bin/env python3
"""Summarise TerraPhone sensor timing CSVs (Debug tools > Sensor timing).

Answers, per recording: how fast each sensor arrives, how old it is at each hop
(iOS delivery, Terra's control queue, the Rust push), and for every control tick
that would trip the 0.1 s stale_imu gate, which hop made the newest sample late.

    python3 analyze_sensor_timing.py C0-*.csv C2-*.csv [--json out.json] [--plot outdir]

Standard library only; --plot needs matplotlib.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

IMU_FRESHNESS = 0.1  # terra-types timing::IMU_FRESHNESS_SECONDS
VIO_FRESHNESS = 0.35  # timing::VIO_FRESHNESS_SECONDS
TURNING_MAX_STEP = 0.1  # timing::TURNING_MAX_STEP_SECONDS
NAN = float("nan")


@dataclass
class Recording:
    path: Path
    meta: dict[str, str] = field(default_factory=dict)
    rows: dict[str, list[tuple[float, ...]]] = field(default_factory=dict)

    @property
    def condition(self) -> str:
        return self.meta.get("condition", self.path.stem)


def load(path: Path) -> Recording:
    rec = Recording(path)
    with path.open() as f:
        lines = []
        for line in f:
            if line.startswith("#"):
                key, _, value = line[1:].strip().partition("=")
                if value:
                    rec.meta[key.strip()] = value.strip()
            else:
                lines.append(line)
    for row in csv.DictReader(lines):
        values = tuple(
            float(row[k]) if row[k] not in ("", None) else NAN
            for k in ("seq", "t_sensor", "t_os", "t_queue", "t_done", "a", "b")
        )
        rec.rows.setdefault(row["stream"], []).append(values)
    for stream in rec.rows.values():
        stream.sort(key=lambda r: r[1])  # by sensor stamp
    return rec


SEQ, SENSOR, OS, QUEUE, DONE, A, B = range(7)


def pct(values: list[float], p: float) -> float:
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return NAN
    k = (len(values) - 1) * p / 100
    lo, hi = math.floor(k), math.ceil(k)
    return values[lo] + (values[hi] - values[lo]) * (k - lo)


def dist(values: list[float]) -> dict[str, float]:
    finite = [v for v in values if math.isfinite(v)]
    return {
        "n": len(finite),
        "p50": pct(finite, 50),
        "p95": pct(finite, 95),
        "p99": pct(finite, 99),
        "max": max(finite) if finite else NAN,
    }


def stream_stats(rows: list[tuple[float, ...]]) -> dict:
    """Rate, sensor-side gaps and per-hop latency for one sample stream."""
    if len(rows) < 2:
        return {"n": len(rows)}
    stamps = [r[SENSOR] for r in rows]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    nominal = statistics.median(gaps)
    span = stamps[-1] - stamps[0]
    # Seq is per-stream in record order; a jump means rows lost to a full buffer.
    seqs = sorted(r[SEQ] for r in rows)
    os_sorted = sorted(r[OS] for r in rows if math.isfinite(r[OS]))
    # Batched delivery: callbacks <1 ms apart for samples taken >=0.5 period apart.
    batched = sum(
        1
        for prev, cur in zip(rows, rows[1:])
        if math.isfinite(prev[OS]) and math.isfinite(cur[OS])
        and cur[OS] - prev[OS] < 0.001 and cur[SENSOR] - prev[SENSOR] >= 0.5 * nominal
    )
    return {
        "n": len(rows),
        "rate_hz": (len(rows) - 1) / span if span > 0 else NAN,
        "nominal_period_s": nominal,
        "sensor_gaps_over_2x": sum(g > 2 * nominal for g in gaps),
        "sensor_gap": dist(gaps),
        "lost_rows": int(seqs[-1] - seqs[0] + 1 - len(seqs)),
        "batched_deliveries": batched,
        "delivery_s": dist([r[OS] - r[SENSOR] for r in rows]),
        "queue_wait_s": dist([r[QUEUE] - r[OS] for r in rows]),
        "push_s": dist([r[DONE] - r[QUEUE] for r in rows]),
        "end_to_end_s": dist([r[DONE] - r[SENSOR] for r in rows]),
        "delivery_interval_s": dist([b - a for a, b in zip(os_sorted, os_sorted[1:])]),
    }


def attribute_stale_ticks(rec: Recording) -> dict:
    """For each tick whose newest IMU sample is >= 0.1 s old, find the hop that held
    back the next sample: none taken (iOS sensor gap), taken but not yet handed to the
    app (iOS delivery), or handed over but still waiting on terra.control (Terra)."""
    ticks = rec.rows.get("tick", [])
    imu = rec.rows.get("imu", [])
    bare = rec.rows.get("bare_imu", [])
    blockers = [("ar_frame", r[QUEUE], r[A]) for r in rec.rows.get("ar", [])]
    blockers += [("floor_apply", r[QUEUE], r[DONE]) for r in rec.rows.get("block", [])]
    blockers += [("detector", r[QUEUE], r[DONE]) for r in rec.rows.get("detector", [])]
    blockers = [b for b in blockers if math.isfinite(b[1]) and math.isfinite(b[2])]
    imu_stamps = [r[SENSOR] for r in imu]
    bare_stamps = [r[SENSOR] for r in bare]
    causes: dict[str, int] = {}
    blamed: dict[str, int] = {}
    bare_fresh = 0
    examples = []
    stale = 0
    for t in ticks:
        now, newest = t[QUEUE], t[SENSOR]
        if not (math.isfinite(now) and math.isfinite(newest)) or now - newest < IMU_FRESHNESS:
            continue
        stale += 1
        i = bisect.bisect_right(imu_stamps, newest)
        nxt = imu[i] if i < len(imu) and imu[i][SENSOR] <= now else None
        if nxt is None:
            cause = "ios_sensor_gap"
        elif not math.isfinite(nxt[OS]) or nxt[OS] > now:
            cause = "ios_delivery"
        else:
            cause = "terra_queue"
        causes[cause] = causes.get(cause, 0) + 1
        # Longest control-queue block overlapping the window the sample was late in.
        window = [b for b in blockers if b[1] < now and b[2] > newest]
        if cause == "terra_queue":
            name = max(window, key=lambda b: b[2] - b[1])[0] if window else "unlogged_block"
            blamed[name] = blamed.get(name, 0) + 1
        # Did the bare logger, on its own queue, hold a fresh sample at the same moment?
        j = bisect.bisect_right(bare_stamps, now)
        fresh = [r for r in bare[max(0, j - 30):j] if r[SENSOR] > now - IMU_FRESHNESS and r[OS] <= now]
        bare_fresh += bool(fresh)
        if len(examples) < 10:
            examples.append({"tick": now, "imu_age": now - newest, "cause": cause,
                             "next_sample_os": nxt[OS] if nxt else None,
                             "next_sample_queue": nxt[QUEUE] if nxt else None})
    return {
        "ticks": len(ticks),
        "stale_ticks": stale,
        "stale_fraction": stale / len(ticks) if ticks else NAN,
        "causes": causes,
        "terra_blocker": blamed,
        "bare_logger_fresh_at_stale_tick": bare_fresh if bare else None,
        "examples": examples,
    }


def tick_stats(rec: Recording) -> dict:
    ticks = rec.rows.get("tick", [])
    if len(ticks) < 2:
        return {"n": len(ticks)}
    starts = sorted(t[QUEUE] for t in ticks)
    intervals = [b - a for a, b in zip(starts, starts[1:])]
    return {
        "n": len(ticks),
        "interval_s": dist(intervals),
        "late_over_turning_max_step": sum(i > TURNING_MAX_STEP for i in intervals),
        "imu_age_s": dist([t[QUEUE] - t[SENSOR] for t in ticks]),
        "vio_age_s": dist([t[QUEUE] - t[A] for t in ticks]),
        "stale_vio_ticks": sum(t[QUEUE] - t[A] >= VIO_FRESHNESS for t in ticks if math.isfinite(t[A])),
        "step_s": dist([t[DONE] - t[QUEUE] for t in ticks]),
        "safety_inactive_fraction": sum(t[B] == 0 for t in ticks) / len(ticks),
    }


def summarise(rec: Recording) -> dict:
    out = {"file": rec.path.name, "condition": rec.condition, "build": rec.meta.get("build"),
           "model": rec.meta.get("model"), "os": rec.meta.get("os"), "streams": {}}
    for name, rows in rec.rows.items():
        if name in ("tick", "meta", "battery"):
            continue
        out["streams"][name] = stream_stats(rows)
        if name == "ar":
            out["streams"][name]["process_s"] = dist([r[A] - r[QUEUE] for r in rows])
            dropped = [r[B] for r in rows if math.isfinite(r[B])]
            out["streams"][name]["dropped_frames"] = (max(dropped) - min(dropped)) if dropped else 0
        if name == "block":
            out["streams"][name]["duration_s"] = dist([r[DONE] - r[QUEUE] for r in rows])
    out["tick"] = tick_stats(rec)
    out["stale_imu"] = attribute_stale_ticks(rec)
    meta = rec.rows.get("meta", [])
    out["thermal_max"] = max((r[A] for r in meta), default=NAN)
    out["low_power"] = any(r[B] == 1 for r in meta)
    battery = [r for r in rec.rows.get("battery", []) if r[A] >= 0]
    out["battery_start"] = battery[0][A] if battery else NAN
    out["battery_end"] = battery[-1][A] if battery else NAN
    return out


def ms(v: float) -> str:
    return "–" if not math.isfinite(v) else f"{v * 1000:.1f}"


def report(summaries: list[dict]) -> str:
    out = []
    out.append("| condition | build | IMU Hz | iOS delivery p50/p99 ms | Terra queue p50/p99 ms | tick age p99 ms | stale_imu ticks | iOS gap / iOS delivery / Terra queue | bare fresh | late ticks |")
    out.append("|---|---|---|---|---|---|---|---|---|---|")
    for s in summaries:
        imu = s["streams"].get("imu") or s["streams"].get("bare_imu") or {}
        st = s["stale_imu"]
        c = st["causes"]
        tick = s["tick"]
        out.append("| {} | {} | {} | {}/{} | {}/{} | {} | {} ({}) | {}/{}/{} | {} | {} |".format(
            s["condition"], s["build"] or "?",
            f"{imu.get('rate_hz', NAN):.1f}" if imu else "–",
            ms(imu.get("delivery_s", {}).get("p50", NAN)), ms(imu.get("delivery_s", {}).get("p99", NAN)),
            ms(imu.get("queue_wait_s", {}).get("p50", NAN)), ms(imu.get("queue_wait_s", {}).get("p99", NAN)),
            ms(tick.get("imu_age_s", {}).get("p99", NAN)),
            st["stale_ticks"], f"{st['stale_fraction']:.1%}" if math.isfinite(st["stale_fraction"]) else "–",
            c.get("ios_sensor_gap", 0), c.get("ios_delivery", 0), c.get("terra_queue", 0),
            st["bare_logger_fresh_at_stale_tick"] if st["bare_logger_fresh_at_stale_tick"] is not None else "–",
            tick.get("late_over_turning_max_step", "–"),
        ))
    for s in summaries:
        out.append(f"\n## {s['condition']} ({s['file']})")
        out.append("| stream | n | Hz | gaps>2× | max gap ms | batched | delivery p50/p99 | queue p50/p99 | push p99 | end-to-end p50/p99 |")
        out.append("|---|---|---|---|---|---|---|---|---|---|")
        for name, st in sorted(s["streams"].items()):
            if st.get("n", 0) < 2 or "rate_hz" not in st:
                continue
            out.append("| {} | {} | {:.1f} | {} | {} | {} | {}/{} | {}/{} | {} | {}/{} |".format(
                name, st["n"], st["rate_hz"], st["sensor_gaps_over_2x"], ms(st["sensor_gap"]["max"]),
                st["batched_deliveries"],
                ms(st["delivery_s"]["p50"]), ms(st["delivery_s"]["p99"]),
                ms(st["queue_wait_s"]["p50"]), ms(st["queue_wait_s"]["p99"]),
                ms(st["push_s"]["p99"]),
                ms(st["end_to_end_s"]["p50"]), ms(st["end_to_end_s"]["p99"])))
        if s["stale_imu"]["terra_blocker"]:
            out.append(f"\nTerra-queue stale ticks by blocker: {s['stale_imu']['terra_blocker']}")
        if "process_s" in s["streams"].get("ar", {}):
            p = s["streams"]["ar"]["process_s"]
            out.append(f"AR frame processing on terra.control: p50 {ms(p['p50'])} ms, p99 {ms(p['p99'])} ms, max {ms(p['max'])} ms")
        if "duration_s" in s["streams"].get("block", {}):
            p = s["streams"]["block"]["duration_s"]
            out.append(f"Floor+barrier apply on terra.control: p50 {ms(p['p50'])} ms, max {ms(p['max'])} ms")
        out.append(f"Thermal state max {s['thermal_max']}, low power {s['low_power']}, battery {s['battery_start']:.0%} → {s['battery_end']:.0%}"
                   if math.isfinite(s["battery_start"]) else f"Thermal state max {s['thermal_max']}, low power {s['low_power']}")
    return "\n".join(out)


def plot(recs: list[Recording], outdir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    outdir.mkdir(parents=True, exist_ok=True)
    for rec in recs:
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        t0 = min(r[QUEUE] for r in rec.rows.get("tick", [(0, 0, 0, 0)]))
        for name, color in (("imu", "C0"), ("bare_imu", "C2")):
            rows = rec.rows.get(name, [])
            axes[0].plot([r[OS] - t0 for r in rows], [1000 * (r[OS] - r[SENSOR]) for r in rows], ".", ms=2, color=color, label=f"{name} iOS delivery")
        rows = rec.rows.get("imu", [])
        axes[0].plot([r[QUEUE] - t0 for r in rows], [1000 * (r[QUEUE] - r[OS]) for r in rows], ".", ms=2, color="C3", label="imu Terra queue wait")
        axes[0].set_ylabel("ms"); axes[0].legend(loc="upper right", fontsize=8)
        ticks = rec.rows.get("tick", [])
        axes[1].plot([t[QUEUE] - t0 for t in ticks], [1000 * (t[QUEUE] - t[SENSOR]) for t in ticks], lw=0.7, label="IMU age at tick")
        axes[1].axhline(1000 * IMU_FRESHNESS, color="red", ls="--", lw=0.8, label="stale_imu gate")
        axes[1].set_ylabel("ms"); axes[1].legend(loc="upper right", fontsize=8)
        for name, color in (("ar", "C1"), ("block", "C4"), ("detector", "C5")):
            rows = rec.rows.get(name, [])
            end = A if name == "ar" else DONE
            axes[2].plot([r[QUEUE] - t0 for r in rows], [1000 * (r[end] - r[QUEUE]) for r in rows], ".", ms=3, color=color, label=f"{name} time on terra.control")
        axes[2].set_ylabel("ms"); axes[2].set_xlabel("s"); axes[2].legend(loc="upper right", fontsize=8)
        fig.suptitle(rec.condition)
        fig.tight_layout()
        fig.savefig(outdir / f"{rec.path.stem}.png", dpi=120)
        plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--json", type=Path, help="write the full summary here")
    parser.add_argument("--plot", type=Path, help="write one timeline PNG per file into this folder")
    args = parser.parse_args(argv)
    recs = [load(p) for p in args.csv]
    summaries = [summarise(r) for r in recs]
    print(report(summaries))
    if args.json:
        args.json.write_text(json.dumps(summaries, indent=2, default=str))
    if args.plot:
        plot(recs, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
