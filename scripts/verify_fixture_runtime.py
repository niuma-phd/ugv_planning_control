#!/usr/bin/env python3
"""Verify an isolated Subject 1 or Subject 2 fixture graph at runtime."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseArray, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


class FixtureMonitor(Node):
    def __init__(self, subject: str) -> None:
        super().__init__(f"verify_{subject}_fixture_runtime")
        self.subject = subject
        self.state: dict[str, object] = {
            "static_tf": False,
            "valid": False,
            "obstacles": 0,
            "detected": False,
            "active": False,
            "linear_x": 0.0,
            "angular_z": 0.0,
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

        if subject == "subject2":
            self.create_subscription(
                Bool,
                "/fixture/localization/odom_valid",
                self._on_valid,
                transient,
            )
            self.create_subscription(
                TwistStamped, "/fixture/control/cmd_vel", self._on_command, reliable
            )
        else:
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

    def _on_valid(self, message: Bool) -> None:
        self.state["valid"] = bool(self.state["valid"]) or message.data

    def _on_obstacles(self, message: PoseArray) -> None:
        self.state["obstacles"] = max(
            int(self.state["obstacles"]), len(message.poses)
        )

    def _on_detected(self, message: Bool) -> None:
        self.state["detected"] = bool(self.state["detected"]) or message.data

    def _on_active(self, message: Bool) -> None:
        self.state["active"] = bool(self.state["active"]) or message.data

    def _on_command(self, message: TwistStamped) -> None:
        linear_x = message.twist.linear.x
        angular_z = message.twist.angular.z
        if (
            math.isfinite(linear_x)
            and math.isfinite(angular_z)
            and linear_x > 0.0
        ):
            self.state["linear_x"] = linear_x
            self.state["angular_z"] = angular_z

    def complete(self) -> bool:
        if self.subject == "subject2":
            return (
                bool(self.state["static_tf"])
                and bool(self.state["valid"])
                and float(self.state["linear_x"]) > 0.0
                and float(self.state["angular_z"]) > 0.0
            )
        return (
            bool(self.state["static_tf"])
            and int(self.state["obstacles"]) > 0
            and bool(self.state["detected"])
            and bool(self.state["active"])
            and float(self.state["linear_x"]) > 0.0
            and abs(float(self.state["angular_z"])) > 1.0e-6
        )

    def assert_isolated(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        canonical = (
            "/control/cmd_vel"
            if self.subject == "subject2"
            else "/subject1/avoid_cmd_vel"
        )
        if canonical in graph:
            raise RuntimeError(f"fixture leaked canonical command topic {canonical}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("subject", choices=("subject1", "subject2"))
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be a positive finite value")

    rclpy.init()
    monitor = FixtureMonitor(args.subject)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not monitor.complete():
            rclpy.spin_once(monitor, timeout_sec=0.1)
        monitor.assert_isolated()
        if not monitor.complete():
            print(
                "FIXTURE_RUNTIME_FAILED " + json.dumps(monitor.state, sort_keys=True),
                file=sys.stderr,
            )
            return 1
        print(
            "FIXTURE_RUNTIME_OK "
            + json.dumps(
                {"subject": args.subject, **monitor.state}, sort_keys=True
            )
        )
        return 0
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
