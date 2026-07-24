from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class NominalCommandFixtureNode(Node):
    def __init__(self) -> None:
        super().__init__("nominal_cmd_fixture")
        self.declare_parameter("production_mode", True)
        if bool(self.get_parameter("production_mode").value):
            raise RuntimeError(
                "nominal_cmd_fixture refuses to run with production_mode=true"
            )

        self.declare_parameter("topic", "/subject1/nominal_cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("linear_x", 0.23)
        self.declare_parameter("angular_z", -0.07)
        self.declare_parameter("stop_after_s", -1.0)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.linear_x = float(self.get_parameter("linear_x").value)
        self.angular_z = float(self.get_parameter("angular_z").value)
        self.stop_after_s = float(self.get_parameter("stop_after_s").value)
        if (
            not math.isfinite(rate_hz)
            or rate_hz <= 0.0
            or not math.isfinite(self.stop_after_s)
        ):
            raise ValueError("fixture rate and stop time must be finite")

        self.started_at = time.monotonic()
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("topic").value), 10
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

    def tick(self) -> None:
        if (
            self.stop_after_s >= 0.0
            and time.monotonic() - self.started_at >= self.stop_after_s
        ):
            return
        message = Twist()
        message.linear.x = self.linear_x
        message.angular.z = self.angular_z
        try:
            self.publisher.publish(message)
        except Exception:
            if not rclpy.ok():
                return
            raise


def main() -> None:
    rclpy.init()
    node = NominalCommandFixtureNode()
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
