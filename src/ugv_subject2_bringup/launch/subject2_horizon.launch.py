"""Bring up the real Horizon -> managed LIO -> Subject 2 MVP chain.

The Livox driver and LIO are kept as external packages.  This wrapper only
connects their pinned launch files to the local subject2 bringup.  LIO runs in
a dedicated child launch process owned by ``lio_process_supervisor`` so a
recovery coordinator can restart exactly that process group.
"""

import json

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _lio_supervisor(context):
    def value(name: str) -> str:
        resolved = LaunchConfiguration(name).perform(context).strip()
        if not resolved:
            raise RuntimeError(f"{name} must not be empty when start_lio=true")
        return resolved

    command = [
        "ros2",
        "launch",
        "ugv_subject2_bringup",
        "managed_lio_horizon.launch.py",
        f"lio_config:={value('lio_config')}",
        f"lidar_topic:={value('lio_lidar_topic')}",
        f"imu_topic:={value('lio_imu_topic')}",
    ]
    return [
        Node(
            package="ugv_subject2_bringup",
            executable="lio_process_supervisor.py",
            name="lio_process_supervisor",
            output="screen",
            parameters=[
                {
                    "command_json": json.dumps(command),
                    "termination_timeout_sec": ParameterValue(
                        LaunchConfiguration("lio_termination_timeout_sec"),
                        value_type=float,
                    ),
                    "startup_grace_sec": ParameterValue(
                        LaunchConfiguration("lio_startup_grace_sec"),
                        value_type=float,
                    ),
                    "status_period_sec": ParameterValue(
                        LaunchConfiguration("lio_status_period_sec"),
                        value_type=float,
                    ),
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("ugv_subject2_bringup")
    driver_share = FindPackageShare("livox_ros2_driver_bringup")
    lio_share = FindPackageShare("lio_livox")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [driver_share, "launch", "horizon_pointcloud2.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_driver")),
        launch_arguments={
            "user_config_path": LaunchConfiguration("driver_config"),
            "publish_freq": LaunchConfiguration("driver_publish_freq"),
            "frame_id": "livox_frame",
            "allow_auto_discovery": LaunchConfiguration("driver_allow_auto_discovery"),
        }.items(),
    )

    subject2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "subject2.launch.py"])
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("subject2_config"),
            "odom_snapshot_directory": LaunchConfiguration("odom_snapshot_directory"),
            "publish_lidar_static_tf": LaunchConfiguration("publish_lidar_static_tf"),
            "lidar_extrinsics_provenance": LaunchConfiguration(
                "lidar_extrinsics_provenance"
            ),
            "base_to_lidar_x": LaunchConfiguration("base_to_lidar_x"),
            "base_to_lidar_y": LaunchConfiguration("base_to_lidar_y"),
            "base_to_lidar_z": LaunchConfiguration("base_to_lidar_z"),
            "base_to_lidar_roll": LaunchConfiguration("base_to_lidar_roll"),
            "base_to_lidar_pitch": LaunchConfiguration("base_to_lidar_pitch"),
            "base_to_lidar_yaw": LaunchConfiguration("base_to_lidar_yaw"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_driver", default_value="true"),
            DeclareLaunchArgument("start_lio", default_value="true"),
            DeclareLaunchArgument(
                "driver_config",
                default_value=PathJoinSubstitution(
                    [driver_share, "config", "horizon.json"]
                ),
            ),
            DeclareLaunchArgument("driver_publish_freq", default_value="10.0"),
            DeclareLaunchArgument(
                "driver_allow_auto_discovery",
                default_value="false",
                description=(
                    "Explicit external-config discovery switch. The driver's "
                    "packaged first-use config still discovers automatically, so "
                    "production must pass a validated broadcast-code whitelist."
                ),
            ),
            DeclareLaunchArgument(
                "lio_config",
                default_value=PathJoinSubstitution(
                    [lio_share, "config", "horizon_config.yaml"]
                ),
            ),
            DeclareLaunchArgument("lio_lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("lio_imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument(
                "lio_termination_timeout_sec", default_value="5.0"
            ),
            DeclareLaunchArgument("lio_startup_grace_sec", default_value="3.0"),
            DeclareLaunchArgument("lio_status_period_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "subject2_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "subject2.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "odom_snapshot_directory",
                default_value="/home/sunrise/.ros/ugv_mvp",
            ),
            DeclareLaunchArgument("publish_lidar_static_tf", default_value="false"),
            DeclareLaunchArgument("lidar_extrinsics_provenance", default_value=""),
            DeclareLaunchArgument("base_to_lidar_x", default_value=""),
            DeclareLaunchArgument("base_to_lidar_y", default_value=""),
            DeclareLaunchArgument("base_to_lidar_z", default_value=""),
            DeclareLaunchArgument("base_to_lidar_roll", default_value=""),
            DeclareLaunchArgument("base_to_lidar_pitch", default_value=""),
            DeclareLaunchArgument("base_to_lidar_yaw", default_value=""),
            driver,
            OpaqueFunction(
                function=_lio_supervisor,
                condition=IfCondition(LaunchConfiguration("start_lio")),
            ),
            subject2,
        ]
    )
