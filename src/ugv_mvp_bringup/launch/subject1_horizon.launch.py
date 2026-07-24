"""Bring up the real Horizon -> LIO -> Subject 1 MVP chain.

The external Livox driver and LIO packages remain unmodified.  PointCloud2
mode is forced only inside the LIO group, and raw LIO TF is isolated from the
canonical vehicle TF tree.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("ugv_mvp_bringup")
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
            "allow_auto_discovery": LaunchConfiguration(
                "driver_allow_auto_discovery"
            ),
        }.items(),
    )

    lio = GroupAction(
        [
            SetRemap(src="/tf", dst="/lio_raw/tf"),
            SetRemap(src="/tf_static", dst="/lio_raw/tf_static"),
            SetParameter(name="msg_type", value=1),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([lio_share, "launch", "horizon.launch.py"])
                ),
                launch_arguments={
                    "config_file": LaunchConfiguration("lio_config"),
                    "lidar_topic": "/livox/lidar",
                    "imu_topic": "/livox/imu",
                    "use_rviz": "false",
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("start_lio")),
    )

    subject1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "subject1.launch.py"])
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("subject1_config"),
            "publish_lidar_static_tf": LaunchConfiguration(
                "publish_lidar_static_tf"
            ),
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
                    "Production must pass a validated Horizon broadcast-code "
                    "whitelist instead of relying on auto-discovery."
                ),
            ),
            DeclareLaunchArgument(
                "lio_config",
                default_value=PathJoinSubstitution(
                    [lio_share, "config", "horizon_config.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "subject1_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "subject1.yaml"]
                ),
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
            lio,
            subject1,
        ]
    )
