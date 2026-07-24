from __future__ import annotations

import math
import time

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
        self.declare_parameter("stop_after_s", -1.0)
        self.declare_parameter("freeze_stamp_after_s", -1.0)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.stop_after_s = float(self.get_parameter("stop_after_s").value)
        self.freeze_stamp_after_s = float(
            self.get_parameter("freeze_stamp_after_s").value
        )
        if (
            not math.isfinite(rate_hz)
            or rate_hz <= 0.0
            or not math.isfinite(self.stop_after_s)
            or not math.isfinite(self.freeze_stamp_after_s)
        ):
            raise ValueError("fixture rates and fault times must be finite")
        self.started_at = time.monotonic()
        self.frozen_stamp = None
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
        elapsed = time.monotonic() - self.started_at
        if self.stop_after_s >= 0.0 and elapsed >= self.stop_after_s:
            return
        header = Header()
        if self.freeze_stamp_after_s >= 0.0 and elapsed >= self.freeze_stamp_after_s:
            if self.frozen_stamp is None:
                self.frozen_stamp = self.get_clock().now().to_msg()
            header.stamp = self.frozen_stamp
        else:
            header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("frame_id").value)
        msg = point_cloud2.create_cloud(
            header,
            self.fields,
            scenario_points(str(self.get_parameter("scenario").value)),
        )
        try:
            self.publisher.publish(msg)
        except Exception:
            if not rclpy.ok():
                return
            raise


def main() -> None:
    rclpy.init()
    node = PointCloudFixtureNode()
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
