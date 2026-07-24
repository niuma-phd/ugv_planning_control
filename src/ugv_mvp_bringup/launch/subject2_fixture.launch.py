from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def _isolated_remaps():
    topics = (
        "/livox_odometry_mapped",
        "/localization/odom",
        "/localization/trusted_odom",
        "/localization/odom_valid",
        "/localization/last_trusted_odom",
        "/localization/map_odom_update",
        "/localization/reset_odom_fault",
        "/subject2/path",
        "/subject2/target_point",
        "/control/cmd_vel",
        "/tf",
        "/tf_static",
    )
    return [SetRemap(src=topic, dst=f"/fixture{topic}") for topic in topics]


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("ugv_mvp_bringup")
    production = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "subject2.launch.py"])
        ),
        launch_arguments={
            "publish_lidar_static_tf": "true",
            "lidar_extrinsics_provenance": "fixture-zero-do-not-use-on-vehicle",
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
            GroupAction(
                [
                    *_isolated_remaps(),
                    production,
                    Node(
                        package="ugv_mvp_tools",
                        executable="path_fixture_node",
                        parameters=[{"production_mode": False, "shape": "left"}],
                    ),
                    Node(
                        package="ugv_mvp_tools",
                        executable="raw_odom_fixture_node",
                        parameters=[
                            {"production_mode": False, "linear_speed_mps": 0.0}
                        ],
                    ),
                ]
            )
        ]
    )
