#!/usr/bin/env python3
"""Measure Cartographer localization quality on the fly-car — before tuning control.

The fly-car has NO IMU: yaw and position come entirely from the map->laser_link
TF that Cartographer publishes. If that TF jitters or jumps while the car is
STANDING STILL, no controller gain can drive a straight line — the map is the
root cause. This script quantifies that directly.

Test protocol (STATIONARY, isolates carto from the controller):
  1. Start Cartographer (e.g. `ros2 launch my_launch l_path_tune.launch.py`).
  2. Do NOT send any route. Leave the car motionless on the ground.
  3. Run this script. It records map->laser_link for --duration-s seconds and
     prints a PASS/FAIL verdict on stationary jitter, discontinuous jumps
     (pose-graph optimization teleports), and TF freshness/rate.

A moving-reference variant (hand-push along a straight edge) is out of scope
here on purpose: this test must be control-independent to be conclusive.

As with l_path_tuning.py, the analysis helpers have no ROS imports so they can
be unit-tested on the Ubuntu 24.04 dev machine; ROS 2 Humble is imported only
inside run_ros on the Ubuntu 22.04 fly-car board.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Sequence


# ---- Verdict thresholds (heuristic, override on the CLI) ---------------------
# Good 2D Cartographer, car standing still, feature-adequate surroundings.
DEFAULTS = {
    "duration_s": 40.0,
    "rate_hz": 50.0,            # how often we sample the TF
    # Stationary noise: a still car's pose must barely move.
    "yaw_std_deg": 0.30,        # std of yaw over the whole capture
    "yaw_pp_deg": 1.00,         # peak-to-peak yaw
    "xy_std_cm": 0.50,          # std of x and of y
    "xy_pp_cm": 2.00,           # peak-to-peak x and y
    # Discontinuous jumps between consecutive samples = optimization teleports.
    "yaw_step_deg": 0.50,       # max single-step |dyaw|
    "xy_step_cm": 1.00,         # max single-step |dx| or |dy|
    # TF liveness.
    "min_rate_hz": 8.0,         # effective distinct-stamp update rate
    "max_age_s": 0.30,          # worst observed stamp age
}


@dataclass(frozen=True)
class PoseSample:
    t_s: float          # monotonic time since capture start
    x_cm: float
    y_cm: float
    yaw_deg: float
    tf_age_s: float     # now - transform.header.stamp, seconds
    stamp_ns: int       # transform stamp, for distinct-stamp rate


def normalize_deg(angle: float) -> float:
    r = math.radians(angle)
    return math.degrees(math.atan2(math.sin(r), math.cos(r)))


def angle_diff_deg(a: float, b: float) -> float:
    return normalize_deg(a - b)


def _mean(v: Sequence[float]) -> float:
    return sum(v) / len(v) if v else 0.0


def _std(v: Sequence[float]) -> float:
    if len(v) < 2:
        return 0.0
    m = _mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def _pp(v: Sequence[float]) -> float:
    return (max(v) - min(v)) if v else 0.0


def _yaw_std_deg(yaws: Sequence[float]) -> float:
    """Std of angles taken around their circular mean (handles wraparound)."""
    if len(yaws) < 2:
        return 0.0
    s = _mean([math.sin(math.radians(y)) for y in yaws])
    c = _mean([math.cos(math.radians(y)) for y in yaws])
    mean_deg = math.degrees(math.atan2(s, c))
    devs = [angle_diff_deg(y, mean_deg) for y in yaws]
    return math.sqrt(sum(d * d for d in devs) / (len(devs) - 1))


def _yaw_pp_deg(yaws: Sequence[float]) -> float:
    if len(yaws) < 2:
        return 0.0
    s = _mean([math.sin(math.radians(y)) for y in yaws])
    c = _mean([math.cos(math.radians(y)) for y in yaws])
    mean_deg = math.degrees(math.atan2(s, c))
    devs = [angle_diff_deg(y, mean_deg) for y in yaws]
    return max(devs) - min(devs)


def analyze(samples: Sequence[PoseSample]) -> dict[str, object]:
    """Reduce raw samples to the metrics the verdict is built on."""
    if len(samples) < 2:
        return {"sample_count": len(samples), "usable": False}

    xs = [s.x_cm for s in samples]
    ys = [s.y_cm for s in samples]
    yaws = [s.yaw_deg for s in samples]

    # Discontinuous jumps between consecutive samples.
    dx = [abs(b.x_cm - a.x_cm) for a, b in zip(samples, samples[1:])]
    dy = [abs(b.y_cm - a.y_cm) for a, b in zip(samples, samples[1:])]
    dyaw = [abs(angle_diff_deg(b.yaw_deg, a.yaw_deg)) for a, b in zip(samples, samples[1:])]

    def _argmax_step(steps: Sequence[float]) -> tuple[float, float]:
        if not steps:
            return 0.0, 0.0
        i = max(range(len(steps)), key=lambda k: steps[k])
        return steps[i], samples[i + 1].t_s

    max_dx, at_dx = _argmax_step(dx)
    max_dy, at_dy = _argmax_step(dy)
    max_dyaw, at_dyaw = _argmax_step(dyaw)

    # Effective update rate from distinct TF stamps (spin sampling oversamples).
    distinct_stamps = sorted({s.stamp_ns for s in samples})
    span_s = (distinct_stamps[-1] - distinct_stamps[0]) / 1e9 if len(distinct_stamps) > 1 else 0.0
    tf_rate_hz = (len(distinct_stamps) - 1) / span_s if span_s > 0 else 0.0

    duration = samples[-1].t_s - samples[0].t_s
    return {
        "usable": True,
        "sample_count": len(samples),
        "duration_s": duration,
        "sample_rate_hz": (len(samples) - 1) / duration if duration > 0 else 0.0,
        "tf_rate_hz": tf_rate_hz,
        "max_tf_age_s": max(s.tf_age_s for s in samples),
        "mean_tf_age_s": _mean([s.tf_age_s for s in samples]),
        "x_std_cm": _std(xs),
        "y_std_cm": _std(ys),
        "x_pp_cm": _pp(xs),
        "y_pp_cm": _pp(ys),
        "yaw_std_deg": _yaw_std_deg(yaws),
        "yaw_pp_deg": _yaw_pp_deg(yaws),
        "max_step_dx_cm": max_dx, "max_step_dx_at_s": at_dx,
        "max_step_dy_cm": max_dy, "max_step_dy_at_s": at_dy,
        "max_step_dyaw_deg": max_dyaw, "max_step_dyaw_at_s": at_dyaw,
    }


def build_verdict(metrics: dict[str, object], thr: dict[str, float]) -> dict[str, object]:
    """Map metrics to per-check PASS/FAIL. Each row: (value, op, threshold, pass)."""
    if not metrics.get("usable"):
        return {"overall_pass": False, "checks": [], "note": "too few samples"}

    def le(name, value, limit):  # value must be <= limit
        return {"check": name, "value": round(float(value), 4),
                "limit": limit, "op": "<=", "pass": float(value) <= limit}

    def ge(name, value, limit):  # value must be >= limit
        return {"check": name, "value": round(float(value), 4),
                "limit": limit, "op": ">=", "pass": float(value) >= limit}

    checks = [
        le("yaw_std_deg", metrics["yaw_std_deg"], thr["yaw_std_deg"]),
        le("yaw_pp_deg", metrics["yaw_pp_deg"], thr["yaw_pp_deg"]),
        le("x_std_cm", metrics["x_std_cm"], thr["xy_std_cm"]),
        le("y_std_cm", metrics["y_std_cm"], thr["xy_std_cm"]),
        le("x_pp_cm", metrics["x_pp_cm"], thr["xy_pp_cm"]),
        le("y_pp_cm", metrics["y_pp_cm"], thr["xy_pp_cm"]),
        le("max_step_dyaw_deg", metrics["max_step_dyaw_deg"], thr["yaw_step_deg"]),
        le("max_step_dx_cm", metrics["max_step_dx_cm"], thr["xy_step_cm"]),
        le("max_step_dy_cm", metrics["max_step_dy_cm"], thr["xy_step_cm"]),
        ge("tf_rate_hz", metrics["tf_rate_hz"], thr["min_rate_hz"]),
        le("max_tf_age_s", metrics["max_tf_age_s"], thr["max_age_s"]),
    ]
    return {"overall_pass": all(c["pass"] for c in checks), "checks": checks}


def format_report(metrics: dict[str, object], verdict: dict[str, object]) -> str:
    lines = []
    if not metrics.get("usable"):
        return "NO USABLE DATA — Cartographer never produced map->laser_link TF.\n"
    lines.append(
        f"samples={metrics['sample_count']} over {metrics['duration_s']:.1f}s "
        f"(sample {metrics['sample_rate_hz']:.0f}Hz, distinct-TF {metrics['tf_rate_hz']:.1f}Hz)"
    )
    lines.append("")
    lines.append(f"{'check':<20}{'value':>12}{'  ':>2}{'limit':>10}   result")
    lines.append("-" * 56)
    for c in verdict["checks"]:
        mark = "✅" if c["pass"] else "❌"
        lines.append(
            f"{c['check']:<20}{c['value']:>12}{'  '}{c['op']}{c['limit']:>8}   {mark}"
        )
    lines.append("-" * 56)
    lines.append(f"worst yaw jump {metrics['max_step_dyaw_deg']:.2f}° at t={metrics['max_step_dyaw_at_s']:.1f}s"
                 f" | worst xy jump {max(metrics['max_step_dx_cm'], metrics['max_step_dy_cm']):.2f}cm")
    overall = "✅ CARTO OK — localization is not the bottleneck; proceed to control tuning." \
        if verdict["overall_pass"] else \
        "❌ CARTO SUSPECT — a stationary car's pose is noisy/jumping. Fix the map before tuning gains."
    lines.append("")
    lines.append(overall)
    return "\n".join(lines) + "\n"


def run_ros(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.time import Time
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python modules required; run this on the Humble board") from exc

    if os.environ.get("ROS_DOMAIN_ID") != "1":
        raise RuntimeError("ROS_DOMAIN_ID must be 1 (run: export ROS_DOMAIN_ID=1)")

    class CartoProbe(Node):
        def __init__(self) -> None:
            super().__init__("carto_quality_probe")
            self.tf_buffer = Buffer(cache_time=Duration(seconds=3.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            # Motion guards: warn loudly if the car is not actually stationary.
            self.moved = False
            self.ground_enabled = False
            self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
            self.create_subscription(Bool, "/ground_enable", self._ground_cb, 10)

        def _cmd_cb(self, msg: object) -> None:
            if abs(float(msg.linear.x)) > 1e-3 or abs(float(msg.angular.z)) > 1e-3:
                self.moved = True

        def _ground_cb(self, msg: object) -> None:
            self.ground_enabled = bool(msg.data)

        def sample(self) -> PoseSample | None:
            try:
                tf = self.tf_buffer.lookup_transform(args.map_frame, args.child_frame, Time())
            except TransformException:
                return None
            stamp = tf.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            age = max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e9)
            q = tf.transform.rotation
            yaw = math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            return PoseSample(
                t_s=0.0,  # filled by caller
                x_cm=tf.transform.translation.x * 100.0,
                y_cm=tf.transform.translation.y * 100.0,
                yaw_deg=yaw, tf_age_s=age, stamp_ns=stamp_ns)

    thr = {k: getattr(args, k, DEFAULTS[k]) for k in DEFAULTS}
    run_dir = Path(args.output_dir).expanduser() / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"run directory: {run_dir}")
    print(f"HOLD THE CAR STILL for {args.duration_s:.0f}s — do not send a route.", flush=True)

    rclpy.init(args=[])
    samples: list[PoseSample] = []
    node: CartoProbe | None = None
    try:
        node = CartoProbe()
        # Let TF warm up; abort early if carto never publishes.
        warmup_deadline = time.monotonic() + args.warmup_s
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.sample() is not None:
                break
        else:
            raise RuntimeError(
                f"no {args.map_frame}->{args.child_frame} TF within {args.warmup_s:.0f}s — "
                "is Cartographer running and converged?")

        start = time.monotonic()
        next_tick = start
        period = 1.0 / max(args.rate_hz, 1.0)
        while True:
            now = time.monotonic()
            if now - start >= args.duration_s:
                break
            if now < next_tick:
                rclpy.spin_once(node, timeout_sec=next_tick - now)
                continue
            next_tick += period
            rclpy.spin_once(node, timeout_sec=0.0)
            s = node.sample()
            if s is not None:
                samples.append(PoseSample(now - start, s.x_cm, s.y_cm, s.yaw_deg,
                                          s.tf_age_s, s.stamp_ns))

        if node.moved or node.ground_enabled:
            print("\n⚠ MOTION DETECTED (/cmd_vel or /ground_enable) — the car was NOT stationary; "
                  "stationary metrics are invalid. Re-run without sending a route.\n")

        metrics = analyze(samples)
        verdict = build_verdict(metrics, thr)

        with (run_dir / "samples.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=[k for k in asdict(samples[0])] if samples else ["t_s"])
            w.writeheader()
            for s in samples:
                w.writerow(asdict(s))
        (run_dir / "summary.json").write_text(
            json.dumps({"thresholds": thr, "metrics": metrics, "verdict": verdict,
                        "moved": node.moved, "ground_enabled": node.ground_enabled},
                       ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        report = format_report(metrics, verdict)
        (run_dir / "summary.txt").write_text(report, encoding="utf-8")
        print("\n" + report)
        return 0 if verdict.get("overall_pass") else 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration-s", type=float, default=DEFAULTS["duration_s"])
    p.add_argument("--rate-hz", type=float, default=DEFAULTS["rate_hz"])
    p.add_argument("--warmup-s", type=float, default=20.0,
                   help="max seconds to wait for the first TF before aborting")
    p.add_argument("--map-frame", default="map")
    p.add_argument("--child-frame", default="laser_link")
    p.add_argument("--output-dir", default="~/kian_flycar/test_log/carto_quality")
    # Threshold overrides.
    for name in ("yaw_std_deg", "yaw_pp_deg", "xy_std_cm", "xy_pp_cm",
                 "yaw_step_deg", "xy_step_cm", "min_rate_hz", "max_age_s"):
        p.add_argument(f"--{name.replace('_', '-')}", type=float, default=DEFAULTS[name])
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration_s <= 0 or args.rate_hz <= 0:
        raise SystemExit("duration and rate must be positive")
    try:
        return run_ros(args)
    except RuntimeError as exc:
        print(f"error: {exc}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
