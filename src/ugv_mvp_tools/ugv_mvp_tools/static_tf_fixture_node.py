from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTfFixtureNode(Node):
    def __init__(self) -> None:
        super().__init__("static_tf_fixture")
        self.declare_parameter("production_mode", True)
        if bool(self.get_parameter("production_mode").value):
            raise RuntimeError(
                "static_tf_fixture refuses to run with production_mode=true"
            )
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "livox_frame")

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(
            self.get_parameter("parent_frame").value
        )
        transform.child_frame_id = str(self.get_parameter("child_frame").value)
        transform.transform.rotation.w = 1.0

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = StaticTfFixtureNode()
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
