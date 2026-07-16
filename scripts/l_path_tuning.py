#!/usr/bin/env python3
"""Run and capture the fly-car's one fixed ground L route.

The analysis helpers intentionally have no ROS imports so they can be tested on
the Ubuntu 24.04 development machine. ROS 2 Humble imports happen only in
``run_ros`` on the Ubuntu 22.04 fly-car board.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, NamedTuple, Sequence


class Waypoint(NamedTuple):
    x_cm: float
    y_cm: float
    z_cm: float
    yaw_deg: float


ROUTE_POINTS = (
    Waypoint(0.0, 0.0, 0.0, 0.0),
    Waypoint(50.0, 0.0, 0.0, 74.1),
    Waypoint(150.0, 350.0, 0.0, 74.1),
)
ROUTE_FLOATS = [value for point in ROUTE_POINTS for value in point]
TARGET_MATCH_TOL_CM = 0.5
POSITION_TOL_CM = 5.0
ENDPOINT_TOL_CM = 12.0
ENDPOINT_YAW_TOL_DEG = 5.0
BASELINE_PARAMS = {
    "kp_v": 1.0,
    "v_max_mps": 0.18,
    "v_min_mps": 0.14,
    "straight_kp_w": 0.22,
    "straight_w_max_rps": 0.16,
    "straight_yaw_deadband_deg": 1.0,
    "yaw_lpf_alpha": 0.25,
    "ki_w": 0.0,
    "kd_w": 0.02,
    "yaw_rate_lpf_alpha": 0.4,
    "w_slew_rps2": 1.2,
    "w_bias_rps": 0.0,
    "kp_w": 0.60,
    "w_max_rps": 0.70,
    "w_min_rps": 0.40,
    "align_gate_deg": 30.0,
    "pos_tol_cm": 5.0,
    "yaw_tol_deg": 3.0,
}


@dataclass(frozen=True)
class Sample:
    t_s: float
    segment: str
    target_x_cm: float
    target_y_cm: float
    target_yaw_deg: float
    pose_x_cm: float
    pose_y_cm: float
    pose_yaw_deg: float
    v_cmd_mps: float
    w_cmd_rps: float
    heading_error_deg: float
    cross_track_cm: float
    ground_enable: bool
    flight_enable: bool


CSV_FIELDS = (
    "t_s", "segment", "target_x_cm", "target_y_cm", "target_yaw_deg",
    "pose_x_cm", "pose_y_cm", "pose_yaw_deg", "v_cmd_mps", "w_cmd_rps",
    "heading_error_deg", "cross_track_cm", "ground_enable", "flight_enable",
)


@dataclass
class EndpointLatch:
    stable_s: float = 0.5
    distance_tol_cm: float = ENDPOINT_TOL_CM
    yaw_tol_deg: float = ENDPOINT_YAW_TOL_DEG
    _entered_s: float | None = None

    def update(self, now_s: float, distance_cm: float, yaw_error_deg: float) -> bool:
        inside = (
            distance_cm <= self.distance_tol_cm
            and abs(yaw_error_deg) <= self.yaw_tol_deg
        )
        if not inside:
            self._entered_s = None
            return False
        if self._entered_s is None:
            self._entered_s = now_s
        return now_s - self._entered_s >= self.stable_s


def create_run_directory(root: Path, now: datetime | None = None) -> Path:
    root = Path(root).expanduser()
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    for suffix in range(1000):
        candidate = root / (stamp if suffix == 0 else f"{stamp}_{suffix:02d}")
        try:
            candidate.mkdir(parents=True, exist_ok=False)
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError(f"too many run-directory collisions below {root}")


def normalize_deg(angle: float) -> float:
    angle_rad = math.radians(angle)
    return math.degrees(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))


def angle_error_deg(target: float, actual: float) -> float:
    return normalize_deg(target - actual)


def segment_heading_deg(segment: str) -> float:
    if segment in ("WAIT_START", "LEG_1"):
        return ROUTE_POINTS[0].yaw_deg
    if segment in ("TURN", "LEG_2", "DONE"):
        return ROUTE_POINTS[1].yaw_deg
    raise ValueError(f"segment has no commanded heading: {segment}")


def cross_track_cm(segment: str, x_cm: float, y_cm: float) -> float:
    if segment == "LEG_1":
        x0, y0, x1, y1 = 0.0, 0.0, 50.0, 0.0
    elif segment == "LEG_2":
        x0, y0, x1, y1 = 50.0, 0.0, 150.0, 350.0
    else:
        raise ValueError(f"unsupported straight segment: {segment}")

    dx = x1 - x0
    dy = y1 - y0
    return (dx * (y_cm - y0) - dy * (x_cm - x0)) / math.hypot(dx, dy)


def parse_overrides(items: Sequence[str]) -> dict[str, float]:
    overrides: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"parameter override must be NAME=VALUE: {item!r}")
        name, raw_value = item.split("=", 1)
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
            raise ValueError(f"invalid parameter name: {name!r}")
        if not raw_value:
            raise ValueError(f"missing value for parameter {name!r}")
        try:
            value = float(raw_value)
        except ValueError as exc:
            raise ValueError(f"parameter {name!r} must be numeric: {raw_value!r}") from exc
        if not math.isfinite(value):
            raise ValueError(f"parameter {name!r} must be finite")
        overrides[name] = value
    return overrides


def _matches_target(target: Sequence[float], waypoint: Waypoint) -> bool:
    return (
        len(target) >= 4
        and abs(float(target[0]) - waypoint.x_cm) <= TARGET_MATCH_TOL_CM
        and abs(float(target[1]) - waypoint.y_cm) <= TARGET_MATCH_TOL_CM
        and abs(float(target[2]) - waypoint.z_cm) <= TARGET_MATCH_TOL_CM
        and abs(angle_error_deg(float(target[3]), waypoint.yaw_deg)) <= 0.2
    )


def classify_target(target: Sequence[float] | None, distance_cm: float) -> str:
    if target is None or _matches_target(target, ROUTE_POINTS[0]):
        return "WAIT_START"
    if _matches_target(target, ROUTE_POINTS[1]):
        return "TURN" if distance_cm <= POSITION_TOL_CM else "LEG_1"
    if _matches_target(target, ROUTE_POINTS[2]):
        return "LEG_2"
    raise ValueError(f"unexpected target: {target!r}")


def count_effective_sign_flips(
    values: Iterable[float], threshold: float = 0.02
) -> int:
    previous_sign = 0
    flips = 0
    for value in values:
        if abs(value) < threshold:
            continue
        sign = 1 if value > 0.0 else -1
        if previous_sign and sign != previous_sign:
            flips += 1
        previous_sign = sign
    return flips


def _rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _duration(samples: Sequence[Sample]) -> float:
    return max(0.0, samples[-1].t_s - samples[0].t_s) if samples else 0.0


def _summarize_straight(samples: Sequence[Sample]) -> dict[str, float | int]:
    if not samples:
        return {"sample_count": 0}
    speeds = [abs(sample.v_cmd_mps) for sample in samples]
    heading = [sample.heading_error_deg for sample in samples]
    cross_track = [sample.cross_track_cm for sample in samples]
    angular = [sample.w_cmd_rps for sample in samples]
    distance_cm = sum(
        math.hypot(b.pose_x_cm - a.pose_x_cm, b.pose_y_cm - a.pose_y_cm)
        for a, b in zip(samples, samples[1:])
    )
    return {
        "sample_count": len(samples),
        "duration_s": _duration(samples),
        "mean_speed_mps": sum(speeds) / len(speeds),
        "max_speed_mps": max(speeds),
        "travel_distance_cm": distance_cm,
        "heading_error_rms_deg": _rms(heading),
        "heading_error_max_abs_deg": max(abs(value) for value in heading),
        "heading_error_peak_to_peak_deg": max(heading) - min(heading),
        "cross_track_rms_cm": _rms(cross_track),
        "cross_track_max_abs_cm": max(abs(value) for value in cross_track),
        "w_sign_flips": count_effective_sign_flips(angular),
        "w_max_abs_rps": max(abs(value) for value in angular),
    }


def _summarize_turn(samples: Sequence[Sample]) -> dict[str, float | int]:
    if not samples:
        return {"sample_count": 0}
    heading = [sample.heading_error_deg for sample in samples]
    return {
        "sample_count": len(samples),
        "duration_s": _duration(samples),
        "max_overshoot_deg": max(0.0, -min(heading)),
        "final_yaw_error_deg": abs(heading[-1]),
        "w_max_abs_rps": max(abs(sample.w_cmd_rps) for sample in samples),
    }


def summarize_samples(samples: Sequence[Sample]) -> dict[str, object]:
    grouped = {
        segment: [sample for sample in samples if sample.segment == segment]
        for segment in ("LEG_1", "TURN", "LEG_2")
    }
    return {
        "LEG_1": _summarize_straight(grouped["LEG_1"]),
        "TURN": _summarize_turn(grouped["TURN"]),
        "LEG_2": _summarize_straight(grouped["LEG_2"]),
    }


def _run_cli(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, text=True, capture_output=True, check=False)


def _parameter_snapshot(
    overrides: dict[str, float], dry_run: bool
) -> tuple[str, list[dict[str, object]]]:
    results: list[dict[str, object]] = []
    requested = dict(BASELINE_PARAMS) if dry_run else {}
    requested.update(overrides)
    for name, value in requested.items():
        command = [
            "ros2", "param", "get" if dry_run else "set",
            "/diff_drive_controller", name,
        ]
        if not dry_run:
            # Keep a decimal point so the ROS 2 CLI sends a DOUBLE, not INTEGER.
            command.append(repr(float(value)))
        result = _run_cli(command)
        record = {
            "command": command,
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
        }
        results.append(record)
        if result.returncode != 0 or (not dry_run and "successful" not in result.stdout.lower()):
            raise RuntimeError(
                f"parameter {'check' if dry_run else 'set'} failed for {name}: "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )
    dump = _run_cli(["ros2", "param", "dump", "/diff_drive_controller"])
    if dump.returncode != 0:
        raise RuntimeError(f"parameter dump failed: {dump.stderr.strip()}")
    return dump.stdout, results


def _write_reports(
    run_dir: Path,
    samples: Sequence[Sample],
    status: str,
    elapsed_s: float,
) -> dict[str, object]:
    segments = summarize_samples(samples)
    report: dict[str, object] = {
        "status": status,
        "elapsed_s": elapsed_s,
        "sample_count": len(samples),
        "segments": segments,
    }
    (run_dir / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    lines = [f"status: {status}", f"elapsed_s: {elapsed_s:.2f}"]
    for name in ("LEG_1", "TURN", "LEG_2"):
        item = segments[name]
        lines.append(f"{name}: " + ", ".join(f"{k}={v}" for k, v in item.items()))
    (run_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def run_ros(args: argparse.Namespace) -> int:
    print("WARNING: REMOVE ALL PROPELLERS BEFORE THIS GROUND TEST.", flush=True)
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.time import Time
        from std_msgs.msg import Bool, Float32MultiArray
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python modules are required; run this on the Humble board") from exc

    class FixedLTuningNode(Node):
        def __init__(self) -> None:
            super().__init__("fixed_l_tuning")
            latched = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.route_pub = self.create_publisher(
                Float32MultiArray, "/wildlife/waypoints", latched
            )
            self.target_sub = self.create_subscription(
                Float32MultiArray, "/target_position", self._target_cb, latched
            )
            self.ground_sub = self.create_subscription(
                Bool, "/ground_enable", self._ground_cb, latched
            )
            self.flight_sub = self.create_subscription(
                Bool, "/flight_enable", self._flight_cb, latched
            )
            self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
            self.tf_buffer = Buffer(cache_time=Duration(seconds=3.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.target: tuple[float, ...] | None = None
            self.ground_enable: bool | None = None
            self.flight_enable: bool | None = None
            self.v_cmd_mps = 0.0
            self.w_cmd_rps = 0.0
            self.last_pose: tuple[float, float, float] | None = None
            self.last_tf_age_s = math.inf

        def _target_cb(self, msg: object) -> None:
            data = tuple(float(value) for value in msg.data)
            self.target = data[:4] if len(data) >= 4 else None

        def _ground_cb(self, msg: object) -> None:
            self.ground_enable = bool(msg.data)

        def _flight_cb(self, msg: object) -> None:
            self.flight_enable = bool(msg.data)

        def _cmd_cb(self, msg: object) -> None:
            self.v_cmd_mps = float(msg.linear.x)
            self.w_cmd_rps = float(msg.angular.z)

        def lookup_pose(self) -> tuple[float, float, float]:
            transform = self.tf_buffer.lookup_transform("map", "laser_link", Time())
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            self.last_tf_age_s = max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e9)
            q = transform.transform.rotation
            yaw = math.degrees(
                math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
            )
            self.last_pose = (
                transform.transform.translation.x * 100.0,
                transform.transform.translation.y * 100.0,
                yaw,
            )
            return self.last_pose

        def publish_route(self, values: Sequence[float]) -> None:
            message = Float32MultiArray()
            message.data = [float(value) for value in values]
            self.route_pub.publish(message)

    run_dir = create_run_directory(Path(args.output_dir))
    print(f"run directory: {run_dir}")
    overrides = parse_overrides(args.set_values)
    samples: list[Sample] = []
    start_mono = time.monotonic()
    status = "ABORTED_BEFORE_START"
    node: FixedLTuningNode | None = None
    route_sent = False
    csv_file = (run_dir / "samples.csv").open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
    writer.writeheader()

    def spin_for(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))

    def hold_current_pose() -> None:
        if node is not None and route_sent and node.last_pose is not None:
            x_cm, y_cm, yaw_deg = node.last_pose
            node.publish_route((x_cm, y_cm, 0.0, yaw_deg))
            spin_for(0.2)

    if os.environ.get("ROS_DOMAIN_ID") != "1":
        csv_file.close()
        _write_reports(run_dir, samples, "ABORTED: ROS_DOMAIN_ID must be 1", 0.0)
        raise RuntimeError("ROS_DOMAIN_ID must be 1 (run: export ROS_DOMAIN_ID=1)")

    rclpy.init(args=[])
    try:
        node = FixedLTuningNode()
        spin_for(1.0)
        required = {"route_test_node", "diff_drive_controller", "chassis_mux"}
        present = set(node.get_node_names())
        missing = sorted(required - present)
        if missing:
            raise RuntimeError("missing required ROS nodes: " + ", ".join(missing))
        if node.flight_enable is True:
            raise RuntimeError("/flight_enable is true; refusing to start ground test")
        if node.ground_enable is None or node.flight_enable is None:
            raise RuntimeError("did not receive latched chassis enable states")
        try:
            node.lookup_pose()
        except TransformException as exc:
            raise RuntimeError(f"map->laser_link TF unavailable: {exc}") from exc
        if node.last_tf_age_s > 0.5:
            raise RuntimeError(f"map->laser_link TF is stale ({node.last_tf_age_s:.3f}s)")
        if node.route_pub.get_subscription_count() < 1:
            raise RuntimeError("/wildlife/waypoints has no subscriber")

        raw_dump, command_results = _parameter_snapshot(overrides, args.dry_run)
        (run_dir / "params.yaml").write_text(raw_dump, encoding="utf-8")
        (run_dir / "params.json").write_text(
            json.dumps(
                {
                    "baseline_recommendations": BASELINE_PARAMS,
                    "requested_overrides": overrides,
                    "dry_run": args.dry_run,
                    "commands": command_results,
                    "raw_dump": raw_dump,
                },
                ensure_ascii=False,
                indent=2,
            ) + "\n",
            encoding="utf-8",
        )
        if args.dry_run:
            status = "DRY_RUN_OK"
            print("dry-run passed; route was not published")
            return 0

        collection_start_mono = time.monotonic()
        for _ in range(3):
            node.publish_route(ROUTE_FLOATS)
            route_sent = True
            spin_for(0.1)

        latch = EndpointLatch(stable_s=args.endpoint_stable_s)
        status = "RUNNING"
        next_tick = time.monotonic()
        tf_fail_since: float | None = None
        while rclpy.ok():
            now_mono = time.monotonic()
            if now_mono - collection_start_mono > args.timeout_s:
                raise RuntimeError(f"test timed out after {args.timeout_s:.1f}s")
            if node.flight_enable is True:
                raise RuntimeError("/flight_enable became true during ground test")
            if now_mono < next_tick:
                rclpy.spin_once(node, timeout_sec=next_tick - now_mono)
                continue
            next_tick += 0.05
            rclpy.spin_once(node, timeout_sec=0.0)
            try:
                x_cm, y_cm, yaw_deg = node.lookup_pose()
                if node.last_tf_age_s > 0.5:
                    raise RuntimeError(f"map->laser_link TF stale ({node.last_tf_age_s:.3f}s)")
                tf_fail_since = None
            except TransformException as exc:
                tf_fail_since = tf_fail_since or now_mono
                if now_mono - tf_fail_since > 0.5:
                    raise RuntimeError(f"map->laser_link TF failed for 0.5s: {exc}") from exc
                continue
            if node.target is None:
                continue
            target = node.target
            distance_cm = math.hypot(target[0] - x_cm, target[1] - y_cm)
            segment = classify_target(target, distance_cm)
            heading_error = angle_error_deg(segment_heading_deg(segment), yaw_deg)
            cross_track = cross_track_cm(segment, x_cm, y_cm) if segment in ("LEG_1", "LEG_2") else 0.0
            sample = Sample(
                t_s=now_mono - collection_start_mono,
                segment=segment,
                target_x_cm=target[0],
                target_y_cm=target[1],
                target_yaw_deg=target[3],
                pose_x_cm=x_cm,
                pose_y_cm=y_cm,
                pose_yaw_deg=yaw_deg,
                v_cmd_mps=node.v_cmd_mps,
                w_cmd_rps=node.w_cmd_rps,
                heading_error_deg=heading_error,
                cross_track_cm=cross_track,
                ground_enable=bool(node.ground_enable),
                flight_enable=bool(node.flight_enable),
            )
            samples.append(sample)
            writer.writerow({name: getattr(sample, name) for name in CSV_FIELDS})
            csv_file.flush()
            if segment == "LEG_2" and latch.update(now_mono, distance_cm, heading_error):
                status = "COMPLETED"
                print("fixed L endpoint stable; collection complete")
                break
        return 0
    except KeyboardInterrupt:
        status = "ABORTED_CTRL_C"
        print("interrupted; publishing current-pose hold route", file=sys.stderr)
        hold_current_pose()
        return 130
    except Exception as exc:
        status = f"ABORTED: {exc}"
        print(status, file=sys.stderr)
        hold_current_pose()
        return 1
    finally:
        csv_file.close()
        elapsed = time.monotonic() - start_mono
        report = _write_reports(run_dir, samples, status, elapsed)
        print(json.dumps(report["segments"], ensure_ascii=False, indent=2))
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run and capture the fly-car's fixed one-shot ground L route. REMOVE PROPELLERS."
    )
    parser.add_argument(
        "--set", dest="set_values", action="append", default=[], metavar="NAME=VALUE",
        help="hot-set one diff_drive_controller parameter before publishing the route",
    )
    parser.add_argument(
        "--output-dir", default="~/kian_flycar/test_log/l_path_runs",
        help="root directory for timestamped run artifacts",
    )
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--endpoint-stable-s", type=float, default=0.5)
    parser.add_argument("--dry-run", action="store_true", help="check only; do not set parameters or move")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.timeout_s <= 0.0 or args.endpoint_stable_s < 0.0:
        raise SystemExit("timeout and endpoint stability duration must be non-negative")
    try:
        parse_overrides(args.set_values)
        return run_ros(args)
    except (RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
