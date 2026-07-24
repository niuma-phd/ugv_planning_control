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
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            Path, str(self.get_parameter("topic").value), qos
        )
        self.timer = self.create_timer(1.0, self.publish_path)
        self.publish_path()

    def publish_path(self) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        points = path_points(
            str(self.get_parameter("shape").value),
            float(self.get_parameter("length_m").value),
            float(self.get_parameter("spacing_m").value),
            float(self.get_parameter("radius_m").value),
        )
        for x_m, y_m, yaw_rad in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x_m
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
