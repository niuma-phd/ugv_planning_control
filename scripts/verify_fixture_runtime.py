#!/usr/bin/env python3
"""Verify the isolated Subject 2 fixture graph and fail-closed behavior."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path as FilePath
import sys
import time

import rclpy
from geometry_msgs.msg import (
    PointStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32
from tf2_msgs.msg import TFMessage


ODOM_FAULT_MODES = {
    "subject2_odom_timeout",
    "subject2_odom_jump",
    "subject2_odom_invalid_stamp",
}
PATH_ALWAYS_INVALID_MODES = {
    "subject2_path_wrong_frame",
    "subject2_path_zero_stamp",
    "subject2_path_negative_stamp",
    "subject2_path_invalid_nanosec",
    "subject2_path_empty",
    "subject2_path_wrong_pose_frame",
    "subject2_path_nonfinite",
}
PATH_FAIL_AFTER_DRIVING_MODES = {
    "subject2_path_timeout",
    "subject2_path_replay",
}
RECOVERY_MODES = {
    "subject2_recovery_success",
    "subject2_recovery_no_gps",
    "subject2_recovery_restart_failed",
}
MAX_RECOVERY_STOP_LATENCY_S = 0.50


class FixtureMonitor(Node):
    def __init__(self, mode: str) -> None:
        super().__init__(f"verify_{mode}_fixture_runtime")
        self.mode = mode
        self.state: dict[str, object] = {
            "static_tf": False,
            "map_odom_identity": False,
            "map_odom_identity_seen": False,
            "map_odom_nonidentity_seen": False,
            "post_restart_alignment_uses_x5": False,
            "valid_seen": False,
            "current_valid": False,
            "fault_invalid": False,
            "path_messages": 0,
            "path_frame": "",
            "last_path_stamp_ns": 0,
            "path_stamps_strict": True,
            "path_points": [],
            "target_messages": 0,
            "target_frame": "",
            "saw_positive": False,
            "saw_left": False,
            "saw_right": False,
            "saw_straight": False,
            "zero_command": False,
            "zero_after_path_count": 0,
            "zero_after_positive": False,
            "zero_after_fault": False,
            "linear_x": 0.0,
            "angular_z": 0.0,
            "last_trusted_position": None,
            "last_trusted_orientation": None,
            "last_trusted_stamp_ns": 0,
            "fault_trusted_position": None,
            "fault_trusted_orientation": None,
            "fault_trusted_stamp_ns": 0,
            "pre_fault_trusted_samples": 0,
            "pre_fault_trusted_monotonic": True,
            "pre_fault_first_x": None,
            "pre_fault_last_x": None,
            "pre_fault_raw_samples": 0,
            "pre_fault_raw_monotonic": True,
            "pre_fault_raw_nonzero_speed": True,
            "pre_fault_raw_last_x": None,
            "queued_old_after_generation_count": 0,
            "navigation_enabled": False,
            "navigation_true_seen": False,
            "navigation_false_after_true": False,
            "recovery_states": [],
            "restart_call_count": 0,
            "lio_generation": 0,
            "lio_process_alive": False,
            "gps_messages": 0,
            "gps_valid": False,
            "gps_valid_seen": False,
            "gps_contract_valid": True,
            "gps_stamps_strict": True,
            "last_gps_stamp_ns": 0,
            "last_gps_pose": None,
            "pre_restart_gps_x4_seen": False,
            "post_restart_gps_x5_seen": False,
            "zero_during_recovery_count": 0,
            "nonzero_during_recovery": False,
            "recovery_stop_latency_s": None,
            "positive_after_recovery": False,
            "recovery_completed": False,
            "trusted_gps_continuous": False,
        }
        self.recovery_stop_started_at: float | None = None
        self.first_recovery_zero_at: float | None = None
        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry,
            "/fixture/livox_odometry_mapped",
            self._on_raw_odom,
            reliable,
        )
        self.create_subscription(
            TFMessage, "/fixture/tf_static", self._on_static_tf, transient
        )
        self.create_subscription(
            TFMessage, "/fixture/tf", self._on_dynamic_tf, reliable
        )
        self.create_subscription(
            TransformStamped,
            "/fixture/localization/map_odom",
            self._on_map_odom,
            transient,
        )
        self.create_subscription(
            Bool,
            "/fixture/localization/odom_valid",
            self._on_valid,
            transient,
        )
        self.create_subscription(
            Odometry,
            "/fixture/localization/last_trusted_odom",
            self._on_last_trusted,
            transient,
        )
        self.create_subscription(
            PathMessage, "/fixture/subject2/path", self._on_path, reliable
        )
        self.create_subscription(
            PointStamped,
            "/fixture/subject2/target_point",
            self._on_target,
            reliable,
        )
        self.create_subscription(Twist, "/fixture/cmd_vel", self._on_command, reliable)
        self.create_subscription(
            Bool,
            "/fixture/localization/navigation_enabled",
            self._on_navigation_enabled,
            transient,
        )
        self.create_subscription(
            String,
            "/fixture/localization/recovery_state",
            self._on_recovery_state,
            transient,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/fixture/localization/gps_pose",
            self._on_gps_pose,
            reliable,
        )
        self.create_subscription(
            Bool,
            "/fixture/localization/gps_valid",
            self._on_gps_valid,
            reliable,
        )
        self.create_subscription(
            UInt32,
            "/fixture/restart_call_count",
            self._on_restart_count,
            transient,
        )
        self.create_subscription(
            UInt32,
            "/fixture/localization/lio_generation",
            self._on_lio_generation,
            transient,
        )
        self.create_subscription(
            Bool,
            "/fixture/localization/lio_process_alive",
            self._on_lio_alive,
            transient,
        )

    def _on_static_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == "base_link"
                and transform.child_frame_id == "livox_frame"
            ):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.state["static_tf"] = (
                    abs(translation.x) < 1.0e-9
                    and abs(translation.y) < 1.0e-9
                    and abs(translation.z) < 1.0e-9
                    and abs(rotation.x) < 1.0e-9
                    and abs(rotation.y) < 1.0e-9
                    and abs(rotation.z) < 1.0e-9
                    and abs(rotation.w - 1.0) < 1.0e-9
                )

    def _on_raw_odom(self, message: Odometry) -> None:
        position_x = float(message.pose.pose.position.x)
        if (
            self.mode == "subject2_recovery_success"
            and int(self.state["lio_generation"]) > 0
            and position_x < 10.0
        ):
            self.state["queued_old_after_generation_count"] = (
                int(self.state["queued_old_after_generation_count"]) + 1
            )
        if self.mode not in RECOVERY_MODES or bool(self.state["fault_invalid"]):
            return
        previous_x = self.state["pre_fault_raw_last_x"]
        if previous_x is not None and position_x <= float(previous_x):
            self.state["pre_fault_raw_monotonic"] = False
        if abs(float(message.twist.twist.linear.x) - 0.25) > 1.0e-9:
            self.state["pre_fault_raw_nonzero_speed"] = False
        self.state["pre_fault_raw_last_x"] = position_x
        self.state["pre_fault_raw_samples"] = (
            int(self.state["pre_fault_raw_samples"]) + 1
        )

    def _on_dynamic_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == "map"
                and transform.child_frame_id == "odom"
            ):
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                identity = (
                    abs(translation.x) < 1.0e-9
                    and abs(translation.y) < 1.0e-9
                    and abs(translation.z) < 1.0e-9
                    and abs(rotation.x) < 1.0e-9
                    and abs(rotation.y) < 1.0e-9
                    and abs(rotation.z) < 1.0e-9
                    and abs(rotation.w - 1.0) < 1.0e-9
                )
                self.state["map_odom_identity"] = identity
                self.state["map_odom_identity_seen"] |= identity
                self.state["map_odom_nonidentity_seen"] |= not identity

    def _on_map_odom(self, message: TransformStamped) -> None:
        transform = message.transform
        translation = transform.translation
        rotation = transform.rotation
        identity = (
            abs(translation.x) < 1.0e-6
            and abs(translation.y) < 1.0e-6
            and abs(translation.z) < 1.0e-6
            and abs(rotation.x) < 1.0e-6
            and abs(rotation.y) < 1.0e-6
            and abs(rotation.z) < 1.0e-6
            and abs(rotation.w - 1.0) < 1.0e-6
        )
        self.state["map_odom_identity"] = identity
        self.state["map_odom_identity_seen"] |= identity
        self.state["map_odom_nonidentity_seen"] |= not identity
        if (
            int(self.state["lio_generation"]) == 1
            and bool(self.state["post_restart_gps_x5_seen"])
            and -16.0 < translation.x < -14.0
            and abs(translation.y) < 0.25
        ):
            self.state["post_restart_alignment_uses_x5"] = True
        self.map_odom_transform = transform
        self._check_trusted_gps_continuity()

    def _on_valid(self, message: Bool) -> None:
        if message.data:
            self.state["valid_seen"] = True
        elif bool(self.state["valid_seen"]):
            self.state["fault_invalid"] = True
            self._mark_recovery_stop_start()
            if self.state["fault_trusted_position"] is None:
                position = self.state["last_trusted_position"]
                orientation = self.state["last_trusted_orientation"]
                if isinstance(position, list) and isinstance(orientation, list):
                    self.state["fault_trusted_position"] = list(position)
                    self.state["fault_trusted_orientation"] = list(orientation)
        self.state["current_valid"] = message.data

    def _on_last_trusted(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        self.state["last_trusted_stamp_ns"] = stamp_ns
        self.state["last_trusted_position"] = [
            position.x,
            position.y,
            position.z,
        ]
        self.state["last_trusted_orientation"] = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        if self.mode in RECOVERY_MODES and position.x < 10.0:
            # The fixture's old generation is below x=10 and its new generation
            # starts at x=20. Keep consuming already queued old-generation
            # last-trusted samples even if odom_valid=false arrived first on a
            # different DDS topic, so the snapshot comparison is order-robust.
            self.state["fault_trusted_position"] = [
                position.x,
                position.y,
                position.z,
            ]
            self.state["fault_trusted_orientation"] = [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
            self.state["fault_trusted_stamp_ns"] = stamp_ns
        if self.mode in RECOVERY_MODES and not bool(self.state["fault_invalid"]):
            previous_x = self.state["pre_fault_last_x"]
            if previous_x is None:
                self.state["pre_fault_first_x"] = position.x
            elif position.x <= float(previous_x):
                self.state["pre_fault_trusted_monotonic"] = False
            self.state["pre_fault_last_x"] = position.x
            self.state["pre_fault_trusted_samples"] = (
                int(self.state["pre_fault_trusted_samples"]) + 1
            )
        self._check_trusted_gps_continuity()

    def _on_path(self, message: PathMessage) -> None:
        self.state["path_messages"] = int(self.state["path_messages"]) + 1
        self.state["path_frame"] = message.header.frame_id
        stamp_ns = int(
            message.header.stamp.sec
        ) * 1_000_000_000 + int(message.header.stamp.nanosec)
        old_stamp = int(self.state["last_path_stamp_ns"])
        if old_stamp and stamp_ns <= old_stamp:
            self.state["path_stamps_strict"] = False
        self.state["last_path_stamp_ns"] = stamp_ns
        self.state["path_points"] = [
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in message.poses
        ]

    def _on_target(self, message: PointStamped) -> None:
        self.state["target_messages"] = int(self.state["target_messages"]) + 1
        self.state["target_frame"] = message.header.frame_id

    def _on_command(self, message: Twist) -> None:
        linear_x = message.linear.x
        angular_z = message.angular.z
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            return
        zero = abs(linear_x) < 1.0e-12 and abs(angular_z) < 1.0e-12
        if linear_x > 0.0:
            self.state["saw_positive"] = True
            self.state["linear_x"] = linear_x
            self.state["angular_z"] = angular_z
            if angular_z > 1.0e-6:
                self.state["saw_left"] = True
            elif angular_z < -1.0e-6:
                self.state["saw_right"] = True
            else:
                self.state["saw_straight"] = True
            if bool(self.state["recovery_completed"]):
                self.state["positive_after_recovery"] = True
        if (
            self.mode in RECOVERY_MODES
            and self.recovery_stop_started_at is not None
            and not bool(self.state["recovery_completed"])
        ):
            if zero:
                self.state["zero_during_recovery_count"] = (
                    int(self.state["zero_during_recovery_count"]) + 1
                )
                if self.first_recovery_zero_at is None:
                    self.first_recovery_zero_at = time.monotonic()
                    self.state["recovery_stop_latency_s"] = (
                        self.first_recovery_zero_at - self.recovery_stop_started_at
                    )
            elif self.first_recovery_zero_at is not None:
                # A non-zero Twist may already be queued when the fault marker is
                # delivered on another topic. The first observed zero is the stop
                # barrier; no command may re-enable motion after that barrier.
                self.state["nonzero_during_recovery"] = True
        if zero:
            self.state["zero_command"] = True
            if int(self.state["path_messages"]) > 0:
                self.state["zero_after_path_count"] = (
                    int(self.state["zero_after_path_count"]) + 1
                )
            if bool(self.state["saw_positive"]):
                self.state["zero_after_positive"] = True
            if bool(self.state["fault_invalid"]):
                self.state["zero_after_fault"] = True

    def _on_navigation_enabled(self, message: Bool) -> None:
        if message.data:
            if bool(self.state["navigation_false_after_true"]):
                self.state["recovery_completed"] = True
            self.state["navigation_true_seen"] = True
        elif bool(self.state["navigation_true_seen"]):
            self.state["navigation_false_after_true"] = True
        self.state["navigation_enabled"] = message.data

    def _on_recovery_state(self, message: String) -> None:
        states = self.state["recovery_states"]
        assert isinstance(states, list)
        if not states or states[-1] != message.data:
            states.append(message.data)
        if message.data == "WAIT_GPS":
            self._mark_recovery_stop_start()

    def _mark_recovery_stop_start(self) -> None:
        if self.mode in RECOVERY_MODES and self.recovery_stop_started_at is None:
            self.recovery_stop_started_at = time.monotonic()

    def _on_restart_count(self, message: UInt32) -> None:
        self.state["restart_call_count"] = int(message.data)

    def _on_lio_generation(self, message: UInt32) -> None:
        self.state["lio_generation"] = int(message.data)

    def _on_lio_alive(self, message: Bool) -> None:
        self.state["lio_process_alive"] = message.data

    def _on_gps_pose(self, message: PoseWithCovarianceStamped) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        old_stamp = int(self.state["last_gps_stamp_ns"])
        if old_stamp and stamp_ns <= old_stamp:
            self.state["gps_stamps_strict"] = False
        self.state["last_gps_stamp_ns"] = stamp_ns
        self.state["gps_messages"] = int(self.state["gps_messages"]) + 1
        covariance = message.pose.covariance
        self.state["gps_contract_valid"] = bool(
            self.state["gps_contract_valid"]
        ) and bool(
            message.header.frame_id == "map"
            and stamp_ns > 0
            and 0.0 < float(covariance[0]) <= 4.0
            and 0.0 < float(covariance[7]) <= 4.0
            and 0.0 < float(covariance[14]) <= 4.0
            and 0.0 < float(covariance[35]) <= 0.25
        )
        pose = message.pose.pose
        self.state["last_gps_pose"] = [
            pose.position.x,
            pose.position.y,
            2.0 * math.atan2(pose.orientation.z, pose.orientation.w),
        ]
        if abs(pose.position.x - 4.0) < 1.0e-9:
            self.state["pre_restart_gps_x4_seen"] = True
        if abs(pose.position.x - 5.0) < 1.0e-9:
            self.state["post_restart_gps_x5_seen"] = True
        self._check_trusted_gps_continuity()

    def _on_gps_valid(self, message: Bool) -> None:
        self.state["gps_valid"] = bool(message.data)
        self.state["gps_valid_seen"] = bool(
            self.state["gps_valid_seen"]
        ) or bool(message.data)

    def _check_trusted_gps_continuity(self) -> None:
        if not hasattr(self, "map_odom_transform"):
            return
        trusted = self.state["last_trusted_position"]
        gps = self.state["last_gps_pose"]
        if not isinstance(trusted, list) or not isinstance(gps, list):
            return
        transform = self.map_odom_transform
        yaw = 2.0 * math.atan2(transform.rotation.z, transform.rotation.w)
        map_x = (
            transform.translation.x
            + math.cos(yaw) * trusted[0]
            - math.sin(yaw) * trusted[1]
        )
        map_y = (
            transform.translation.y
            + math.sin(yaw) * trusted[0]
            + math.cos(yaw) * trusted[1]
        )
        self.state["trusted_gps_continuous"] = bool(
            self.state["trusted_gps_continuous"]
        ) or (math.hypot(map_x - gps[0], map_y - gps[1]) < 0.25)

    def _common_ready(self) -> bool:
        return (
            bool(self.state["static_tf"])
            and bool(self.state["map_odom_identity_seen"])
            and bool(self.state["valid_seen"])
            and int(self.state["path_messages"]) > 0
        )

    def _recovery_stop_is_verified(self) -> bool:
        latency = self.state["recovery_stop_latency_s"]
        return (
            isinstance(latency, float)
            and 0.0 <= latency <= MAX_RECOVERY_STOP_LATENCY_S
            and int(self.state["zero_during_recovery_count"]) >= 5
            and not bool(self.state["nonzero_during_recovery"])
        )

    def _dynamic_fault_snapshot_source_is_verified(self) -> bool:
        first_x = self.state["pre_fault_first_x"]
        last_x = self.state["pre_fault_last_x"]
        return (
            int(self.state["pre_fault_trusted_samples"]) >= 20
            and bool(self.state["pre_fault_trusted_monotonic"])
            and isinstance(first_x, float)
            and isinstance(last_x, float)
            and last_x > first_x + 0.50
            and last_x > 0.50
            and int(self.state["pre_fault_raw_samples"]) >= 20
            and bool(self.state["pre_fault_raw_monotonic"])
            and bool(self.state["pre_fault_raw_nonzero_speed"])
        )

    def complete(self) -> bool:
        if not self._common_ready():
            return False
        if self.mode == "subject2":
            return (
                bool(self.state["saw_positive"])
                and bool(self.state["saw_left"])
                and int(self.state["target_messages"]) > 0
                and self.state["target_frame"] == "map"
            )
        if self.mode == "subject2_right":
            return bool(self.state["saw_positive"]) and bool(self.state["saw_right"])
        if self.mode == "subject2_line":
            return bool(self.state["saw_positive"]) and bool(self.state["saw_straight"])
        if self.mode == "subject2_waypoint_file":
            expected = [
                [0.0, 0.0, 0.0],
                [1.0, 0.4, 0.0],
                [2.0, 1.0, 0.0],
                [3.0, 2.0, 0.0],
            ]
            return (
                self.state["path_frame"] == "map"
                and self.state["path_points"] == expected
                and bool(self.state["path_stamps_strict"])
                and int(self.state["path_messages"]) >= 2
                and bool(self.state["saw_positive"])
                and bool(self.state["saw_left"])
            )
        if self.mode in PATH_ALWAYS_INVALID_MODES:
            return (
                bool(self.state["current_valid"])
                and bool(self.state["zero_command"])
                and int(self.state["zero_after_path_count"]) >= 5
                and not bool(self.state["saw_positive"])
            )
        if self.mode in PATH_FAIL_AFTER_DRIVING_MODES:
            return (
                bool(self.state["current_valid"])
                and bool(self.state["saw_positive"])
                and bool(self.state["zero_after_positive"])
            )
        if self.mode in ODOM_FAULT_MODES:
            return (
                bool(self.state["saw_positive"])
                and bool(self.state["fault_invalid"])
                and bool(self.state["zero_after_fault"])
            )
        if self.mode == "subject2_recovery_success":
            states = self.state["recovery_states"]
            expected_states = ["RUNNING", "WAIT_GPS", "WAIT_LIO", "RUNNING"]
            state_index = 0
            for state in states:
                if state_index < len(expected_states) and state == expected_states[state_index]:
                    state_index += 1
            return (
                state_index == len(expected_states)
                and "ABORTED" not in states
                and bool(self.state["navigation_enabled"])
                and self._recovery_stop_is_verified()
                and self._dynamic_fault_snapshot_source_is_verified()
                and int(self.state["restart_call_count"]) == 1
                and int(self.state["lio_generation"]) == 1
                and int(self.state["queued_old_after_generation_count"]) >= 5
                and bool(self.state["lio_process_alive"])
                and bool(self.state["map_odom_nonidentity_seen"])
                and bool(self.state["gps_stamps_strict"])
                and bool(self.state["gps_contract_valid"])
                and bool(self.state["gps_valid_seen"])
                and bool(self.state["pre_restart_gps_x4_seen"])
                and bool(self.state["post_restart_gps_x5_seen"])
                and bool(self.state["post_restart_alignment_uses_x5"])
                and isinstance(self.state["last_gps_pose"], list)
                and abs(self.state["last_gps_pose"][0] - 5.0) < 1.0e-9
                and bool(self.state["trusted_gps_continuous"])
                and bool(self.state["positive_after_recovery"])
                and isinstance(self.state["fault_trusted_position"], list)
                and isinstance(self.state["last_trusted_position"], list)
                and abs(
                    self.state["last_trusted_position"][0]
                    - self.state["fault_trusted_position"][0]
                )
                > 10.0
            )
        if self.mode == "subject2_recovery_no_gps":
            return (
                self.state["recovery_states"][-1:] == ["ABORTED"]
                and not bool(self.state["navigation_enabled"])
                and int(self.state["restart_call_count"]) == 0
                and self._recovery_stop_is_verified()
                and self._dynamic_fault_snapshot_source_is_verified()
            )
        if self.mode == "subject2_recovery_restart_failed":
            return (
                self.state["recovery_states"][-1:] == ["ABORTED"]
                and not bool(self.state["navigation_enabled"])
                and int(self.state["restart_call_count"]) == 1
                and int(self.state["lio_generation"]) == 0
                and self._recovery_stop_is_verified()
                and self._dynamic_fault_snapshot_source_is_verified()
            )
        return False

    def assert_isolated(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        canonical = {
            "/livox_odometry_mapped",
            "/localization/odom",
            "/localization/trusted_odom",
            "/localization/odom_valid",
            "/localization/last_trusted_odom",
            "/localization/map_odom_update",
            "/localization/map_odom",
            "/localization/gps_pose",
            "/localization/gps_valid",
            "/localization/navigation_enabled",
            "/localization/recovery_state",
            "/localization/lio_generation",
            "/localization/lio_process_alive",
            "/subject2/path",
            "/subject2/target_point",
            "/cmd_vel",
            "/tf",
            "/tf_static",
        }
        leaked = sorted(canonical.intersection(graph))
        if leaked:
            raise RuntimeError(f"fixture leaked canonical topics: {leaked}")
        final_types = graph.get("/fixture/cmd_vel")
        if final_types != ["geometry_msgs/msg/Twist"]:
            raise RuntimeError(f"fixture final command type mismatch: {final_types!r}")
        publishers = self.get_publishers_info_by_topic("/fixture/cmd_vel")
        if len(publishers) != 1:
            raise RuntimeError(
                "fixture must have exactly one final command publisher; "
                f"found {len(publishers)}"
            )
        path_publishers = self.get_publishers_info_by_topic("/fixture/subject2/path")
        if len(path_publishers) != 1:
            raise RuntimeError(
                "fixture must have exactly one path publisher; "
                f"found {len(path_publishers)}"
            )
        services = {name for name, _ in self.get_service_names_and_types()}
        canonical_services = {
            "/localization/reset_odom_fault",
            "/localization/restart_lio",
        }
        leaked_services = sorted(canonical_services.intersection(services))
        if leaked_services:
            raise RuntimeError(f"fixture leaked canonical services: {leaked_services}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode",
        choices=(
            "subject2",
            "subject2_right",
            "subject2_line",
            "subject2_path_timeout",
            "subject2_path_replay",
            "subject2_path_wrong_frame",
            "subject2_path_zero_stamp",
            "subject2_path_negative_stamp",
            "subject2_path_invalid_nanosec",
            "subject2_path_empty",
            "subject2_path_wrong_pose_frame",
            "subject2_path_nonfinite",
            "subject2_odom_timeout",
            "subject2_odom_jump",
            "subject2_odom_invalid_stamp",
            "subject2_waypoint_file",
            "subject2_recovery_success",
            "subject2_recovery_no_gps",
            "subject2_recovery_restart_failed",
        ),
    )
    parser.add_argument(
        "--expected-fault",
        choices=("stale", "translation_jump", "invalid_stamp"),
        default="stale",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--snapshot",
        type=FilePath,
        required=True,
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be a positive finite value")

    rclpy.init()
    monitor = FixtureMonitor(args.mode)

    def snapshot_is_fresh_and_valid() -> bool:
        if args.mode not in ODOM_FAULT_MODES | RECOVERY_MODES:
            return True
        if not args.snapshot.exists():
            return False
        try:
            contents = json.loads(args.snapshot.read_text())
        except (json.JSONDecodeError, OSError):
            return False
        if args.mode in RECOVERY_MODES:
            last_position = monitor.state["fault_trusted_position"]
            last_orientation = monitor.state["fault_trusted_orientation"]
        else:
            last_position = monitor.state["last_trusted_position"]
            last_orientation = monitor.state["last_trusted_orientation"]
        if last_position is None or last_orientation is None:
            return False

        def close_vector(first, second) -> bool:
            try:
                return (
                    isinstance(first, list)
                    and len(first) == len(second)
                    and all(
                        math.isfinite(float(value))
                        and abs(float(value) - expected) < 1.0e-9
                        for value, expected in zip(first, second)
                    )
                )
            except (TypeError, ValueError, OverflowError):
                return False

        def dynamic_recovery_snapshot() -> bool:
            if args.mode not in RECOVERY_MODES:
                return True
            try:
                velocity = contents.get("linear_velocity_mps")
                position = contents.get("position_m")
                return bool(
                    isinstance(velocity, list)
                    and len(velocity) == 3
                    and isinstance(position, list)
                    and len(position) == 3
                    and all(math.isfinite(float(value)) for value in velocity)
                    and float(position[0]) > 0.50
                )
            except (TypeError, ValueError, OverflowError):
                return False

        return (
            contents.get("fault") == args.expected_fault
            and int(contents.get("stamp_ns", 0))
            == (
                int(monitor.state["fault_trusted_stamp_ns"])
                if args.mode in RECOVERY_MODES
                else int(monitor.state["last_trusted_stamp_ns"])
            )
            and contents.get("frame_id") == "odom"
            and contents.get("child_frame_id") == "base_link"
            and close_vector(contents.get("position_m"), last_position)
            and close_vector(contents.get("orientation_xyzw"), last_orientation)
            and dynamic_recovery_snapshot()
        )

    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not (
            monitor.complete() and snapshot_is_fresh_and_valid()
        ):
            rclpy.spin_once(monitor, timeout_sec=0.1)
        monitor.assert_isolated()
        if not monitor.complete() or not snapshot_is_fresh_and_valid():
            try:
                snapshot_debug = json.loads(args.snapshot.read_text())
            except (json.JSONDecodeError, OSError):
                snapshot_debug = None
            monitor.state["snapshot_contract_valid"] = snapshot_is_fresh_and_valid()
            monitor.state["snapshot_contents"] = snapshot_debug
            print(
                "FIXTURE_RUNTIME_FAILED " + json.dumps(monitor.state, sort_keys=True),
                file=sys.stderr,
            )
            return 1
        print(
            "FIXTURE_RUNTIME_OK "
            + json.dumps(
                {
                    "mode": args.mode,
                    "snapshot": snapshot_is_fresh_and_valid(),
                    **monitor.state,
                },
                sort_keys=True,
            )
        )
        return 0
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
