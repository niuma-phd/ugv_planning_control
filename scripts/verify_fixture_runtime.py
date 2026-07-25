#!/usr/bin/env python3
"""Verify the isolated Subject 2 fixture graph and fail-closed behavior."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path as FilePath
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


ODOM_FAULT_MODES = {
    "subject2_odom_timeout",
    "subject2_odom_jump",
    "subject2_odom_invalid_stamp",
}
PATH_ALWAYS_INVALID_MODES = {
    "subject2_path_wrong_frame",
    "subject2_path_zero_stamp",
    "subject2_path_negative_stamp",
    "subject2_path_invalid_nanosec",
    "subject2_path_empty",
    "subject2_path_wrong_pose_frame",
    "subject2_path_nonfinite",
}
PATH_FAIL_AFTER_DRIVING_MODES = {
    "subject2_path_timeout",
    "subject2_path_replay",
}


class FixtureMonitor(Node):
    def __init__(self, mode: str) -> None:
        super().__init__(f"verify_{mode}_fixture_runtime")
        self.mode = mode
        self.state: dict[str, object] = {
            "static_tf": False,
            "map_odom_identity": False,
            "valid_seen": False,
            "current_valid": False,
            "fault_invalid": False,
            "path_messages": 0,
            "path_frame": "",
            "last_path_stamp_ns": 0,
            "target_messages": 0,
            "target_frame": "",
            "saw_positive": False,
            "saw_left": False,
            "saw_right": False,
            "saw_straight": False,
            "zero_command": False,
            "zero_after_path_count": 0,
            "zero_after_positive": False,
            "zero_after_fault": False,
            "linear_x": 0.0,
            "angular_z": 0.0,
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
        self.create_subscription(
            Bool,
            "/fixture/localization/odom_valid",
            self._on_valid,
            transient,
        )
        self.create_subscription(
            Odometry,
            "/fixture/localization/last_trusted_odom",
            self._on_last_trusted,
            transient,
        )
        self.create_subscription(
            PathMessage, "/fixture/subject2/path", self._on_path, reliable
        )
        self.create_subscription(
            PointStamped,
            "/fixture/subject2/target_point",
            self._on_target,
            reliable,
        )
        self.create_subscription(Twist, "/fixture/cmd_vel", self._on_command, reliable)

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
            self.state["valid_seen"] = True
        elif bool(self.state["valid_seen"]):
            self.state["fault_invalid"] = True
        self.state["current_valid"] = message.data

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

    def _on_path(self, message: PathMessage) -> None:
        self.state["path_messages"] = int(self.state["path_messages"]) + 1
        self.state["path_frame"] = message.header.frame_id
        self.state["last_path_stamp_ns"] = int(
            message.header.stamp.sec
        ) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _on_target(self, message: PointStamped) -> None:
        self.state["target_messages"] = int(self.state["target_messages"]) + 1
        self.state["target_frame"] = message.header.frame_id

    def _on_command(self, message: Twist) -> None:
        linear_x = message.linear.x
        angular_z = message.angular.z
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            return
        zero = abs(linear_x) < 1.0e-12 and abs(angular_z) < 1.0e-12
        if linear_x > 0.0:
            self.state["saw_positive"] = True
            self.state["linear_x"] = linear_x
            self.state["angular_z"] = angular_z
            if angular_z > 1.0e-6:
                self.state["saw_left"] = True
            elif angular_z < -1.0e-6:
                self.state["saw_right"] = True
            else:
                self.state["saw_straight"] = True
        if zero:
            self.state["zero_command"] = True
            if int(self.state["path_messages"]) > 0:
                self.state["zero_after_path_count"] = (
                    int(self.state["zero_after_path_count"]) + 1
                )
            if bool(self.state["saw_positive"]):
                self.state["zero_after_positive"] = True
            if bool(self.state["fault_invalid"]):
                self.state["zero_after_fault"] = True

    def _common_ready(self) -> bool:
        return (
            bool(self.state["static_tf"])
            and bool(self.state["map_odom_identity"])
            and bool(self.state["valid_seen"])
            and int(self.state["path_messages"]) > 0
        )

    def complete(self) -> bool:
        if not self._common_ready():
            return False
        if self.mode == "subject2":
            return (
                bool(self.state["saw_positive"])
                and bool(self.state["saw_left"])
                and int(self.state["target_messages"]) > 0
                and self.state["target_frame"] == "map"
            )
        if self.mode == "subject2_right":
            return bool(self.state["saw_positive"]) and bool(self.state["saw_right"])
        if self.mode == "subject2_line":
            return bool(self.state["saw_positive"]) and bool(self.state["saw_straight"])
        if self.mode in PATH_ALWAYS_INVALID_MODES:
            return (
                bool(self.state["current_valid"])
                and bool(self.state["zero_command"])
                and int(self.state["zero_after_path_count"]) >= 5
                and not bool(self.state["saw_positive"])
            )
        if self.mode in PATH_FAIL_AFTER_DRIVING_MODES:
            return (
                bool(self.state["current_valid"])
                and bool(self.state["saw_positive"])
                and bool(self.state["zero_after_positive"])
            )
        if self.mode in ODOM_FAULT_MODES:
            return (
                bool(self.state["saw_positive"])
                and bool(self.state["fault_invalid"])
                and bool(self.state["zero_after_fault"])
            )
        return False

    def assert_isolated(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        canonical = {
            "/livox_odometry_mapped",
            "/localization/odom",
            "/localization/trusted_odom",
            "/localization/odom_valid",
            "/localization/last_trusted_odom",
            "/localization/map_odom_update",
            "/subject2/path",
            "/subject2/target_point",
            "/cmd_vel",
            "/tf",
            "/tf_static",
        }
        leaked = sorted(canonical.intersection(graph))
        if leaked:
            raise RuntimeError(f"fixture leaked canonical topics: {leaked}")
        final_types = graph.get("/fixture/cmd_vel")
        if final_types != ["geometry_msgs/msg/Twist"]:
            raise RuntimeError(f"fixture final command type mismatch: {final_types!r}")
        publishers = self.get_publishers_info_by_topic("/fixture/cmd_vel")
        if len(publishers) != 1:
            raise RuntimeError(
                "fixture must have exactly one final command publisher; "
                f"found {len(publishers)}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode",
        choices=(
            "subject2",
            "subject2_right",
            "subject2_line",
            "subject2_path_timeout",
            "subject2_path_replay",
            "subject2_path_wrong_frame",
            "subject2_path_zero_stamp",
            "subject2_path_negative_stamp",
            "subject2_path_invalid_nanosec",
            "subject2_path_empty",
            "subject2_path_wrong_pose_frame",
            "subject2_path_nonfinite",
            "subject2_odom_timeout",
            "subject2_odom_jump",
            "subject2_odom_invalid_stamp",
        ),
    )
    parser.add_argument(
        "--expected-fault",
        choices=("stale", "translation_jump", "invalid_stamp"),
        default="stale",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--snapshot",
        type=FilePath,
        required=True,
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be a positive finite value")

    rclpy.init()
    monitor = FixtureMonitor(args.mode)

    def snapshot_is_fresh_and_valid() -> bool:
        if args.mode not in ODOM_FAULT_MODES:
            return True
        if not args.snapshot.exists():
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
                    "mode": args.mode,
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
