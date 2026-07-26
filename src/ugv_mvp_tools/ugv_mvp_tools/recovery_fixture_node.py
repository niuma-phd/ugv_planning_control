from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32, UInt64
from std_srvs.srv import Trigger


class RecoveryFixtureNode(Node):
    """Fixture-only GPS source and deterministic fake LIO restart service."""

    def __init__(self) -> None:
        super().__init__("recovery_fixture")
        self.declare_parameter("production_mode", True)
        if bool(self.get_parameter("production_mode").value):
            raise RuntimeError("recovery_fixture refuses to run in production mode")

        self.declare_parameter("scenario", "success")
        self.declare_parameter("gps_start_after_s", 7.0)
        self.declare_parameter("gps_x_m", 4.0)
        self.declare_parameter("post_restart_gps_x_m", 5.0)
        self.declare_parameter("gps_y_m", 0.0)
        self.declare_parameter("gps_yaw_rad", 0.0)
        self.declare_parameter("restart_service_delay_s", 0.20)
        self.declare_parameter("rate_hz", 20.0)
        scenario = str(self.get_parameter("scenario").value)
        if scenario not in {"success", "no_gps", "restart_failed"}:
            raise ValueError("scenario must be success, no_gps, or restart_failed")
        rate_hz = float(self.get_parameter("rate_hz").value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive and finite")
        restart_service_delay_s = float(
            self.get_parameter("restart_service_delay_s").value
        )
        if not math.isfinite(restart_service_delay_s) or restart_service_delay_s < 0.0:
            raise ValueError("restart_service_delay_s must be finite and non-negative")
        self.scenario = scenario
        self.started_ns = self.get_clock().now().nanoseconds
        self.restart_calls = 0
        self.generation = 0

        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gps_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/gps_pose", reliable
        )
        self.gps_valid_pub = self.create_publisher(
            Bool, "/localization/gps_valid", reliable
        )
        self.restart_count_pub = self.create_publisher(
            UInt32, "/fixture/restart_call_count", latched
        )
        self.restart_started_pub = self.create_publisher(
            UInt64, "/fixture/restart_started_ns", latched
        )
        self.generation_pub = self.create_publisher(
            UInt32, "/localization/lio_generation", latched
        )
        self.process_alive_pub = self.create_publisher(
            Bool, "/localization/lio_process_alive", latched
        )
        self.restart_service = self.create_service(
            Trigger, "/localization/restart_lio", self.on_restart
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.publish_status()

    def elapsed_s(self) -> float:
        return (self.get_clock().now().nanoseconds - self.started_ns) / 1.0e9

    def publish_status(self) -> None:
        count = UInt32()
        count.data = self.restart_calls
        self.restart_count_pub.publish(count)
        generation = UInt32()
        generation.data = self.generation
        self.generation_pub.publish(generation)
        alive = Bool()
        alive.data = self.elapsed_s() < 6.0 or self.generation > 0
        self.process_alive_pub.publish(alive)

    def on_restart(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        self.restart_calls += 1
        restart_started = UInt64()
        restart_started.data = self.get_clock().now().nanoseconds
        self.restart_started_pub.publish(restart_started)
        # Publish the call count before the delay so the raw-odom fixture can
        # retain old-generation samples produced during service execution.
        self.publish_status()
        time.sleep(float(self.get_parameter("restart_service_delay_s").value))
        if self.scenario == "restart_failed":
            response.success = False
            response.message = "fixture requested deterministic restart failure"
        else:
            self.generation += 1
            response.success = True
            response.message = "fixture started a fresh LIO generation"
        self.publish_status()
        return response

    def tick(self) -> None:
        self.publish_status()
        if self.scenario == "no_gps" or self.elapsed_s() < float(
            self.get_parameter("gps_start_after_s").value
        ):
            return

        valid = Bool()
        valid.data = True
        self.gps_valid_pub.publish(valid)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        gps_x_parameter = "post_restart_gps_x_m" if self.generation > 0 else "gps_x_m"
        pose.pose.pose.position.x = float(self.get_parameter(gps_x_parameter).value)
        pose.pose.pose.position.y = float(self.get_parameter("gps_y_m").value)
        yaw = float(self.get_parameter("gps_yaw_rad").value)
        pose.pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.pose.orientation.w = math.cos(0.5 * yaw)
        pose.pose.covariance[0] = 0.04
        pose.pose.covariance[7] = 0.04
        pose.pose.covariance[14] = 0.09
        pose.pose.covariance[35] = 0.01
        self.gps_pose_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = RecoveryFixtureNode()
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
