from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .waypoint_file import load_waypoints


class WaypointFilePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_file_publisher")
        self.declare_parameter("path_file", "")
        self.declare_parameter("topic", "/subject2/path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("rate_hz", 1.0)

        path_file = str(self.get_parameter("path_file").value).strip()
        topic = str(self.get_parameter("topic").value).strip()
        frame_id = str(self.get_parameter("frame_id").value).strip()
        rate_hz = float(self.get_parameter("rate_hz").value)
        if not path_file:
            raise ValueError("path_file is required")
        if not topic:
            raise ValueError("topic must not be empty")
        if not frame_id:
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive and finite")

        # Parse the whole file before creating a publisher. Invalid input can never
        # expose a partial Path to a transient-local subscriber.
        self.waypoints = load_waypoints(path_file)
        self.frame_id = frame_id
        self.last_stamp_ns = 0
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(Path, topic, qos)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_path)
        self.publish_path()
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {path_file}; publishing on {topic}"
        )

    def publish_path(self) -> None:
        stamp_ns = max(1, self.get_clock().now().nanoseconds, self.last_stamp_ns + 1)
        self.last_stamp_ns = stamp_ns
        stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = waypoint.x_m
            pose.pose.position.y = waypoint.y_m
            pose.pose.position.z = waypoint.z_m
            pose.pose.orientation.z = math.sin(0.5 * waypoint.yaw_rad)
            pose.pose.orientation.w = math.cos(0.5 * waypoint.yaw_rad)
            path.poses.append(pose)
        try:
            self.publisher.publish(path)
        except Exception:
            if not rclpy.ok():
                return
            raise


def main() -> None:
    rclpy.init()
    node: WaypointFilePublisherNode | None = None
    try:
        node = WaypointFilePublisherNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
