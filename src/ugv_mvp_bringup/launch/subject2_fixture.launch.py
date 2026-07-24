from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("ugv_mvp_bringup")
    production = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "subject2.launch.py"])
        ),
        launch_arguments={
            "publish_lidar_static_tf": "true",
            "base_to_lidar_x": "0.0",
            "base_to_lidar_y": "0.0",
            "base_to_lidar_z": "0.0",
            "base_to_lidar_roll": "0.0",
            "base_to_lidar_pitch": "0.0",
            "base_to_lidar_yaw": "0.0",
        }.items(),
    )
    return LaunchDescription(
        [
            production,
            Node(
                package="ugv_mvp_tools",
                executable="path_fixture_node",
                parameters=[{"production_mode": False, "shape": "left"}],
            ),
            Node(
                package="ugv_mvp_tools",
                executable="raw_odom_fixture_node",
                parameters=[{"production_mode": False, "linear_speed_mps": 0.0}],
            ),
        ]
    )

