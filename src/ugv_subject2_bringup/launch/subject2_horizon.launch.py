"""Bring up the real Horizon -> LIO -> Subject 2 MVP chain.

The Livox driver and LIO are kept as external packages.  This wrapper only
connects their pinned launch files to the local subject2 bringup.  ``msg_type``
is scoped to the LIO launch because the upstream launch predates its
PointCloud2 entry and otherwise defaults ScanRegistration to CustomMsg.  Raw
LIO TF is isolated from the canonical vehicle TF tree.
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

    # Keep the raw LIO world->livox_frame broadcaster out of the canonical TF
    # tree, where base_link->livox_frame is the only approved parent.  The
    # parameter override reaches ScanRegistration without modifying LIO.
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

    subject2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "subject2.launch.py"])
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("subject2_config"),
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
            DeclareLaunchArgument(
                "subject2_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "subject2.yaml"]
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
            subject2,
        ]
    )
