#!/usr/bin/env python3
"""Verify an isolated Subject 1 or Subject 2 fixture graph at runtime."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseArray, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


class FixtureMonitor(Node):
    def __init__(self, mode: str) -> None:
        super().__init__(f"verify_{mode}_fixture_runtime")
        self.mode = mode
        self.subject = "subject2" if mode.startswith("subject2") else "subject1"
        self.state: dict[str, object] = {
            "static_tf": False,
            "map_odom_identity": False,
            "valid": False,
            "obstacles": 0,
            "cloud_messages": 0,
            "obstacle_messages": 0,
            "detected": False,
            "active": False,
            "inactive": False,
            "linear_x": 0.0,
            "angular_z": 0.0,
            "fault_invalid": False,
            "zero_command": False,
            "zero_after_fault": False,
            "saw_positive": False,
            "zero_after_positive": False,
            "last_trusted_position": None,
            "last_trusted_orientation": None,
        }
        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage, "/fixture/tf_static", self._on_static_tf, transient
        )
        self.create_subscription(
            TFMessage, "/fixture/tf", self._on_dynamic_tf, reliable
        )

        if self.subject == "subject2":
            self.create_subscription(
                Bool,
                "/fixture/localization/odom_valid",
                self._on_valid,
                transient,
            )
            self.create_subscription(
                Twist, "/fixture/cmd_vel", self._on_command, reliable
            )
            self.create_subscription(
                Odometry,
                "/fixture/localization/last_trusted_odom",
                self._on_last_trusted,
                transient,
            )
        else:
            self.create_subscription(
                PointCloud2,
                "/fixture/livox/lidar",
                self._on_cloud,
                reliable,
            )
            self.create_subscription(
                PoseArray,
                "/fixture/subject1/obstacles",
                self._on_obstacles,
                reliable,
            )
            self.create_subscription(
                Bool,
                "/fixture/subject1/obstacle_detected",
                self._on_detected,
                reliable,
            )
            self.create_subscription(
                Bool,
                "/fixture/subject1/avoidance_active",
                self._on_active,
                reliable,
            )
            self.create_subscription(
                TwistStamped,
                "/fixture/subject1/avoid_cmd_vel",
                self._on_command,
                reliable,
            )

    def _on_static_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == "base_link"
                and transform.child_frame_id == "livox_frame"
            ):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.state["static_tf"] = (
                    abs(translation.x) < 1.0e-9
                    and abs(translation.y) < 1.0e-9
                    and abs(translation.z) < 1.0e-9
                    and abs(rotation.x) < 1.0e-9
                    and abs(rotation.y) < 1.0e-9
                    and abs(rotation.z) < 1.0e-9
                    and abs(rotation.w - 1.0) < 1.0e-9
                )

    def _on_dynamic_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == "map"
                and transform.child_frame_id == "odom"
            ):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.state["map_odom_identity"] = (
                    abs(translation.x) < 1.0e-9
                    and abs(translation.y) < 1.0e-9
                    and abs(translation.z) < 1.0e-9
                    and abs(rotation.x) < 1.0e-9
                    and abs(rotation.y) < 1.0e-9
                    and abs(rotation.z) < 1.0e-9
                    and abs(rotation.w - 1.0) < 1.0e-9
                )

    def _on_valid(self, message: Bool) -> None:
        if message.data:
            self.state["valid"] = True
        elif bool(self.state["valid"]):
            self.state["fault_invalid"] = True

    def _on_obstacles(self, message: PoseArray) -> None:
        self.state["obstacle_messages"] = int(self.state["obstacle_messages"]) + 1
        self.state["obstacles"] = max(int(self.state["obstacles"]), len(message.poses))

    def _on_cloud(self, _: PointCloud2) -> None:
        self.state["cloud_messages"] = int(self.state["cloud_messages"]) + 1

    def _on_detected(self, message: Bool) -> None:
        self.state["detected"] = bool(self.state["detected"]) or message.data

    def _on_active(self, message: Bool) -> None:
        if message.data:
            self.state["active"] = True
        else:
            self.state["inactive"] = True

    def _on_last_trusted(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.state["last_trusted_position"] = [
            position.x,
            position.y,
            position.z,
        ]
        self.state["last_trusted_orientation"] = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]

    def _on_command(self, message: Twist | TwistStamped) -> None:
        twist = message if isinstance(message, Twist) else message.twist
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        if math.isfinite(linear_x) and math.isfinite(angular_z) and linear_x > 0.0:
            self.state["saw_positive"] = True
            self.state["linear_x"] = linear_x
            self.state["angular_z"] = angular_z
        if (
            math.isfinite(linear_x)
            and math.isfinite(angular_z)
            and abs(linear_x) < 1.0e-12
            and abs(angular_z) < 1.0e-12
        ):
            self.state["zero_command"] = True
            if bool(self.state["saw_positive"]):
                self.state["zero_after_positive"] = True
        if (
            bool(self.state["fault_invalid"])
            and math.isfinite(linear_x)
            and math.isfinite(angular_z)
            and abs(linear_x) < 1.0e-12
            and abs(angular_z) < 1.0e-12
        ):
            self.state["zero_after_fault"] = True

    def complete(self) -> bool:
        if self.mode == "subject2":
            return (
                bool(self.state["static_tf"])
                and bool(self.state["map_odom_identity"])
                and bool(self.state["valid"])
                and float(self.state["linear_x"]) > 0.0
                and float(self.state["angular_z"]) > 0.0
            )
        if self.mode == "subject2_fault":
            return (
                bool(self.state["static_tf"])
                and bool(self.state["map_odom_identity"])
                and bool(self.state["valid"])
                and float(self.state["linear_x"]) > 0.0
                and bool(self.state["fault_invalid"])
                and bool(self.state["zero_after_fault"])
            )
        if self.mode == "subject1_none":
            return (
                bool(self.state["static_tf"])
                and int(self.state["obstacle_messages"]) > 0
                and int(self.state["obstacles"]) == 0
                and not bool(self.state["detected"])
                and bool(self.state["inactive"])
                and bool(self.state["zero_command"])
                and float(self.state["linear_x"]) == 0.0
            )
        if self.mode == "subject1_blocked":
            return (
                bool(self.state["static_tf"])
                and int(self.state["obstacles"]) > 0
                and bool(self.state["detected"])
                and bool(self.state["active"])
                and bool(self.state["zero_command"])
                and float(self.state["linear_x"]) == 0.0
            )
        if self.mode in {"subject1_fault", "subject1_replay"}:
            return (
                bool(self.state["static_tf"])
                and int(self.state["obstacles"]) > 0
                and bool(self.state["detected"])
                and bool(self.state["active"])
                and bool(self.state["saw_positive"])
                and bool(self.state["zero_after_positive"])
            )
        if self.mode == "subject1_invalid":
            return (
                bool(self.state["static_tf"])
                and int(self.state["cloud_messages"]) > 0
                and int(self.state["obstacle_messages"]) == 0
                and bool(self.state["active"])
                and bool(self.state["zero_command"])
            )
        return (
            bool(self.state["static_tf"])
            and int(self.state["obstacles"]) > 0
            and bool(self.state["detected"])
            and bool(self.state["active"])
            and float(self.state["linear_x"]) > 0.0
            and float(self.state["angular_z"]) > 0.0
        )

    def assert_isolated(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        canonical = (
            {
                "/livox_odometry_mapped",
                "/localization/odom",
                "/localization/trusted_odom",
                "/localization/odom_valid",
                "/localization/last_trusted_odom",
                "/localization/map_odom_update",
                "/subject2/path",
                "/subject2/target_point",
                "/control/cmd_vel",
                "/cmd_vel",
                "/tf",
                "/tf_static",
            }
            if self.subject == "subject2"
            else {
                "/livox/lidar",
                "/livox_odometry_mapped",
                "/localization/odom",
                "/localization/odom_valid",
                "/localization/map_odom_update",
                "/subject1/obstacles",
                "/subject1/obstacle_detected",
                "/subject1/next_waypoint_base",
                "/subject1/avoidance_active",
                "/subject1/avoid_cmd_vel",
                "/subject1/selected_trajectory",
                "/tf",
                "/tf_static",
            }
        )
        leaked = sorted(canonical.intersection(graph))
        if leaked:
            raise RuntimeError(f"fixture leaked canonical topics: {leaked}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "subject",
        choices=(
            "subject1",
            "subject1_none",
            "subject1_blocked",
            "subject1_fault",
            "subject1_replay",
            "subject1_invalid",
            "subject2",
            "subject2_fault",
        ),
    )
    parser.add_argument(
        "--expected-fault",
        choices=("stale", "translation_jump"),
        default="stale",
    )
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument(
        "--snapshot",
        type=Path,
        default=Path("/home/sunrise/.ros/ugv_mvp/last_good_subject2_odom.json"),
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be a positive finite value")

    rclpy.init()
    monitor = FixtureMonitor(args.subject)
    initial_snapshot_mtime = (
        args.snapshot.stat().st_mtime_ns if args.snapshot.exists() else None
    )

    def snapshot_is_fresh_and_valid() -> bool:
        if args.subject != "subject2_fault":
            return True
        if not args.snapshot.exists():
            return False
        if (
            initial_snapshot_mtime is not None
            and args.snapshot.stat().st_mtime_ns <= initial_snapshot_mtime
        ):
            return False
        try:
            contents = json.loads(args.snapshot.read_text())
        except (json.JSONDecodeError, OSError):
            return False
        last_position = monitor.state["last_trusted_position"]
        last_orientation = monitor.state["last_trusted_orientation"]
        if last_position is None or last_orientation is None:
            return False

        def close_vector(first, second) -> bool:
            try:
                return (
                    isinstance(first, list)
                    and len(first) == len(second)
                    and all(
                        math.isfinite(float(value))
                        and abs(float(value) - expected) < 1.0e-9
                        for value, expected in zip(first, second)
                    )
                )
            except (TypeError, ValueError, OverflowError):
                return False

        return (
            contents.get("fault") == args.expected_fault
            and contents.get("frame_id") == "odom"
            and contents.get("child_frame_id") == "base_link"
            and close_vector(contents.get("position_m"), last_position)
            and close_vector(contents.get("orientation_xyzw"), last_orientation)
        )

    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not (
            monitor.complete() and snapshot_is_fresh_and_valid()
        ):
            rclpy.spin_once(monitor, timeout_sec=0.1)
        monitor.assert_isolated()
        if not monitor.complete() or not snapshot_is_fresh_and_valid():
            print(
                "FIXTURE_RUNTIME_FAILED " + json.dumps(monitor.state, sort_keys=True),
                file=sys.stderr,
            )
            return 1
        print(
            "FIXTURE_RUNTIME_OK "
            + json.dumps(
                {
                    "subject": args.subject,
                    "snapshot": snapshot_is_fresh_and_valid(),
                    **monitor.state,
                },
                sort_keys=True,
            )
        )
        return 0
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
