from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .common import scenario_points


class PointCloudFixtureNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_fixture")
        self.declare_parameter("production_mode", True)
        if bool(self.get_parameter("production_mode").value):
            raise RuntimeError("pointcloud_fixture refuses to run in production mode")

        self.declare_parameter("topic", "/livox/lidar")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("scenario", "front")
        self.declare_parameter("rate_hz", 10.0)
        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("topic").value), 10
        )
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

    def tick(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("frame_id").value)
        msg = point_cloud2.create_cloud(
            header,
            self.fields,
            scenario_points(str(self.get_parameter("scenario").value)),
        )
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = PointCloudFixtureNode()
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
