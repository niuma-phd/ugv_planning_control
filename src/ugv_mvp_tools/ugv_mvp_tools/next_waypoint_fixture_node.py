from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class NextWaypointFixtureNode(Node):
    def __init__(self) -> None:
        super().__init__("next_waypoint_fixture")
        self.declare_parameter("production_mode", True)
        if bool(self.get_parameter("production_mode").value):
            raise RuntimeError(
                "next_waypoint_fixture refuses to run with production_mode=true"
            )
        self.declare_parameter("topic", "/subject1/next_waypoint_base")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("x_m", 4.0)
        self.declare_parameter("y_m", 0.0)
        self.publisher = self.create_publisher(
            PointStamped, str(self.get_parameter("topic").value), 10
        )
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self) -> None:
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.point.x = float(self.get_parameter("x_m").value)
        message.point.y = float(self.get_parameter("y_m").value)
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = NextWaypointFixtureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
