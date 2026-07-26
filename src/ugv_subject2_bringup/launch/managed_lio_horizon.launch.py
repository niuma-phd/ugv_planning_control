"""Launch the external Horizon LIO as one supervisor-owned process tree."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    lio_share = FindPackageShare("lio_livox")
    managed_lio = GroupAction(
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
                    "lidar_topic": LaunchConfiguration("lidar_topic"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "use_rviz": "false",
                }.items(),
            ),
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lio_config",
                default_value=PathJoinSubstitution(
                    [lio_share, "config", "horizon_config.yaml"]
                ),
            ),
            DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            managed_lio,
        ]
    )
