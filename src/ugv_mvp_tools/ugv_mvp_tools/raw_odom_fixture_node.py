from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .common import integrate_pose


class RawOdomFixtureNode(Node):
    def __init__(self) -> None:
        super().__init__("raw_odom_fixture")
        self.declare_parameter("production_mode", True)
        if bool(self.get_parameter("production_mode").value):
            raise RuntimeError("raw_odom_fixture refuses to run with production_mode=true")

        self.declare_parameter("topic", "/livox_odometry_mapped")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", "livox_frame")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("linear_speed_mps", 0.0)
        self.declare_parameter("yaw_rate_radps", 0.0)
        self.declare_parameter("stop_after_s", -1.0)
        self.declare_parameter("inject_jump_after_s", -1.0)
        self.declare_parameter("jump_distance_m", 5.0)

        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("topic").value), 10
        )
        self.period_s = 1.0 / rate_hz
        self.started_ns = self.get_clock().now().nanoseconds
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.jump_injected = False
        self.timer = self.create_timer(self.period_s, self.tick)

    def tick(self) -> None:
        elapsed_s = (
            self.get_clock().now().nanoseconds - self.started_ns
        ) / 1.0e9
        stop_after_s = float(self.get_parameter("stop_after_s").value)
        if stop_after_s >= 0.0 and elapsed_s >= stop_after_s:
            return

        self.x_m, self.y_m, self.yaw_rad = integrate_pose(
            self.x_m,
            self.y_m,
            self.yaw_rad,
            float(self.get_parameter("linear_speed_mps").value),
            float(self.get_parameter("yaw_rate_radps").value),
            self.period_s,
        )
        jump_after_s = float(self.get_parameter("inject_jump_after_s").value)
        if (
            not self.jump_injected
            and jump_after_s >= 0.0
            and elapsed_s >= jump_after_s
        ):
            self.x_m += float(self.get_parameter("jump_distance_m").value)
            self.jump_injected = True

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.child_frame_id = str(self.get_parameter("child_frame_id").value)
        msg.pose.pose.position.x = self.x_m
        msg.pose.pose.position.y = self.y_m
        msg.pose.pose.orientation.z = math.sin(0.5 * self.yaw_rad)
        msg.pose.pose.orientation.w = math.cos(0.5 * self.yaw_rad)
        msg.twist.twist.linear.x = float(
            self.get_parameter("linear_speed_mps").value
        )
        msg.twist.twist.angular.z = float(
            self.get_parameter("yaw_rate_radps").value
        )
        try:
            self.publisher.publish(msg)
        except Exception:
            if not rclpy.ok():
                return
            raise


def main() -> None:
    rclpy.init()
    node = RawOdomFixtureNode()
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
