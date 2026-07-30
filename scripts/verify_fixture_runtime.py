#!/usr/bin/env python3
"""Verify the isolated direct-odom Subject 2 control graph."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


MODES = (
    "subject2",
    "subject2_right",
    "subject2_line",
    "subject2_waypoint_file",
)


class FixtureMonitor(Node):
    def __init__(self, mode: str) -> None:
        super().__init__(f"verify_{mode}_fixture_runtime")
        self.mode = mode
        self.state: dict[str, object] = {
            "static_tf": False,
            "map_odom_identity": False,
            "canonical_odom_messages": 0,
            "canonical_odom_frames_valid": True,
            "target_messages": 0,
            "target_frame": "",
            "target_point": None,
            "saw_positive": False,
            "saw_left": False,
            "saw_right": False,
            "saw_straight": False,
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
            TransformStamped,
            "/fixture/localization/map_odom",
            self._on_map_odom,
            transient,
        )
        self.create_subscription(
            Odometry,
            "/fixture/localization/odom",
            self._on_canonical_odom,
            reliable,
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

    def _on_map_odom(self, message: TransformStamped) -> None:
        transform = message.transform
        self.state["map_odom_identity"] = (
            message.header.frame_id == "map"
            and message.child_frame_id == "odom"
            and abs(transform.translation.x) < 1.0e-9
            and abs(transform.translation.y) < 1.0e-9
            and abs(transform.translation.z) < 1.0e-9
            and abs(transform.rotation.x) < 1.0e-9
            and abs(transform.rotation.y) < 1.0e-9
            and abs(transform.rotation.z) < 1.0e-9
            and abs(transform.rotation.w - 1.0) < 1.0e-9
        )

    def _on_canonical_odom(self, message: Odometry) -> None:
        self.state["canonical_odom_messages"] = (
            int(self.state["canonical_odom_messages"]) + 1
        )
        self.state["canonical_odom_frames_valid"] = bool(
            self.state["canonical_odom_frames_valid"]
        ) and bool(
            message.header.frame_id == "odom"
            and message.child_frame_id == "base_link"
            and math.isfinite(message.pose.pose.position.x)
            and math.isfinite(message.pose.pose.position.y)
        )

    def _on_target(self, message: PointStamped) -> None:
        self.state["target_messages"] = int(self.state["target_messages"]) + 1
        self.state["target_frame"] = message.header.frame_id
        self.state["target_point"] = [message.point.x, message.point.y]

    def _on_command(self, message: Twist) -> None:
        linear_x = float(message.linear.x)
        angular_z = float(message.angular.z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            return
        if linear_x <= 0.0:
            return
        self.state["saw_positive"] = True
        if angular_z > 1.0e-6:
            self.state["saw_left"] = True
        elif angular_z < -1.0e-6:
            self.state["saw_right"] = True
        else:
            self.state["saw_straight"] = True

    def complete(self) -> bool:
        target = self.state["target_point"]
        common = (
            bool(self.state["static_tf"])
            and bool(self.state["map_odom_identity"])
            and int(self.state["canonical_odom_messages"]) > 0
            and bool(self.state["canonical_odom_frames_valid"])
            and int(self.state["target_messages"]) > 0
            and self.state["target_frame"] == "map"
            and isinstance(target, list)
            and len(target) == 2
            and all(math.isfinite(float(value)) for value in target)
            and bool(self.state["saw_positive"])
        )
        if not common:
            return False
        assert isinstance(target, list)
        if self.mode in ("subject2", "subject2_waypoint_file"):
            return bool(self.state["saw_left"]) and float(target[1]) > 0.0
        if self.mode == "subject2_right":
            return bool(self.state["saw_right"]) and float(target[1]) < 0.0
        return bool(self.state["saw_straight"]) and abs(float(target[1])) < 1.0e-9

    def assert_isolated(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        canonical = {
            "/livox_odometry_mapped",
            "/localization/odom",
            "/localization/map_odom",
            "/subject2/target_point",
            "/cmd_vel",
            "/tf",
            "/tf_static",
        }
        leaked = sorted(canonical.intersection(graph))
        if leaked:
            raise RuntimeError(f"fixture leaked canonical topics: {leaked}")
        publishers = self.get_publishers_info_by_topic("/fixture/cmd_vel")
        if len(publishers) != 1:
            raise RuntimeError(
                "fixture must have exactly one final command publisher; "
                f"found {len(publishers)}"
            )
        for retired in (
            "/fixture/localization/trusted_odom",
            "/fixture/localization/odom_valid",
            "/fixture/localization/navigation_enabled",
            "/fixture/localization/recovery_state",
        ):
            publishers = self.get_publishers_info_by_topic(retired)
            subscribers = self.get_subscriptions_info_by_topic(retired)
            if publishers or subscribers:
                raise RuntimeError(
                    "retired production topic is still connected: "
                    f"{retired} (publishers={len(publishers)}, "
                    f"subscribers={len(subscribers)})"
                )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=MODES)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be a positive finite value")

    rclpy.init()
    monitor = FixtureMonitor(args.mode)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not monitor.complete():
            rclpy.spin_once(monitor, timeout_sec=0.1)
        monitor.assert_isolated()
        if not monitor.complete():
            print(json.dumps(monitor.state, ensure_ascii=False, indent=2), file=sys.stderr)
            return 1
        print(json.dumps(monitor.state, ensure_ascii=False, indent=2))
        return 0
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
