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
            PathJoinSubstitution([bringup_share, "launch", "subject1.launch.py"])
        )
    )
    return LaunchDescription(
        [
            production,
            Node(
                package="ugv_mvp_tools",
                executable="pointcloud_fixture_node",
                parameters=[
                    {
                        "production_mode": False,
                        "scenario": "front",
                        "frame_id": "base_link",
                    }
                ],
            ),
            Node(
                package="ugv_mvp_tools",
                executable="next_waypoint_fixture_node",
                parameters=[
                    {
                        "production_mode": False,
                        "frame_id": "base_link",
                        "x_m": 4.0,
                        "y_m": 0.0,
                    }
                ],
            ),
        ]
    )

