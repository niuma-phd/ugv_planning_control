from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .common import path_points


class PathFixtureNode(Node):
    def __init__(self) -> None:
        super().__init__("path_fixture")
        self.declare_parameter("production_mode", True)
        if bool(self.get_parameter("production_mode").value):
            raise RuntimeError("path_fixture refuses to run with production_mode=true")

        self.declare_parameter("topic", "/subject2/path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("shape", "line")
        self.declare_parameter("length_m", 8.0)
        self.declare_parameter("spacing_m", 0.25)
        self.declare_parameter("radius_m", 6.0)
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("stop_after_s", -1.0)
        self.declare_parameter("stamp_mode", "current")
        self.declare_parameter("freeze_stamp_after_s", 3.0)
        self.declare_parameter("empty_path", False)
        self.declare_parameter("pose_frame_override", "")
        self.declare_parameter("inject_nonfinite_x", False)
        rate_hz = float(self.get_parameter("rate_hz").value)
        stamp_mode = str(self.get_parameter("stamp_mode").value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive and finite")
        if stamp_mode not in {
            "current",
            "zero",
            "freeze",
            "negative",
            "invalid_nanosec",
        }:
            raise ValueError(
                "stamp_mode must be current, zero, freeze, negative, "
                "or invalid_nanosec"
            )
        if (
            stamp_mode == "freeze"
            and float(self.get_parameter("freeze_stamp_after_s").value) < 0.0
        ):
            raise ValueError("freeze_stamp_after_s must be non-negative")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            Path, str(self.get_parameter("topic").value), qos
        )
        self.started_ns = self.get_clock().now().nanoseconds
        self.frozen_stamp = None
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_path)
        self.publish_path()

    def publish_path(self) -> None:
        now = self.get_clock().now()
        elapsed_s = (now.nanoseconds - self.started_ns) / 1.0e9
        stop_after_s = float(self.get_parameter("stop_after_s").value)
        if stop_after_s >= 0.0 and elapsed_s >= stop_after_s:
            return

        frame_id = str(self.get_parameter("frame_id").value)
        path = Path()
        stamp_mode = str(self.get_parameter("stamp_mode").value)
        if stamp_mode == "current":
            path.header.stamp = now.to_msg()
        elif stamp_mode == "freeze":
            freeze_after_s = float(self.get_parameter("freeze_stamp_after_s").value)
            if elapsed_s < freeze_after_s:
                path.header.stamp = now.to_msg()
            else:
                if self.frozen_stamp is None:
                    self.frozen_stamp = now.to_msg()
                path.header.stamp = self.frozen_stamp
        elif stamp_mode == "negative":
            path.header.stamp.sec = -1
        elif stamp_mode == "invalid_nanosec":
            path.header.stamp.sec = 1
            path.header.stamp.nanosec = 1_000_000_000
        path.header.frame_id = frame_id
        points = (
            ()
            if bool(self.get_parameter("empty_path").value)
            else path_points(
                str(self.get_parameter("shape").value),
                float(self.get_parameter("length_m").value),
                float(self.get_parameter("spacing_m").value),
                float(self.get_parameter("radius_m").value),
            )
        )
        pose_frame_override = str(
            self.get_parameter("pose_frame_override").value
        )
        inject_nonfinite_x = bool(
            self.get_parameter("inject_nonfinite_x").value
        )
        for index, (x_m, y_m, yaw_rad) in enumerate(points):
            pose = PoseStamped()
            pose.header.stamp.sec = path.header.stamp.sec
            pose.header.stamp.nanosec = path.header.stamp.nanosec
            pose.header.frame_id = pose_frame_override or path.header.frame_id
            pose.pose.position.x = (
                math.nan if inject_nonfinite_x and index == 0 else x_m
            )
            pose.pose.position.y = y_m
            pose.pose.orientation.z = math.sin(0.5 * yaw_rad)
            pose.pose.orientation.w = math.cos(0.5 * yaw_rad)
            path.poses.append(pose)
        try:
            self.publisher.publish(path)
        except Exception:
            if not rclpy.ok():
                return
            raise


def main() -> None:
    rclpy.init()
    node = PathFixtureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
