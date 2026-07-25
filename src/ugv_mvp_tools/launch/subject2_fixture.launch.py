from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
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
        "/cmd_vel",
        "/tf",
        "/tf_static",
    )
    return [SetRemap(src=topic, dst=f"/fixture{topic}") for topic in topics]


def generate_launch_description() -> LaunchDescription:
    path_shape = LaunchConfiguration("path_shape")
    stop_after_s = LaunchConfiguration("raw_odom_stop_after_s")
    jump_after_s = LaunchConfiguration("raw_odom_inject_jump_after_s")
    jump_distance_m = LaunchConfiguration("raw_odom_jump_distance_m")
    bringup_share = FindPackageShare("ugv_subject2_bringup")
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
            DeclareLaunchArgument("path_shape", default_value="left"),
            DeclareLaunchArgument("raw_odom_stop_after_s", default_value="-1.0"),
            DeclareLaunchArgument("raw_odom_inject_jump_after_s", default_value="-1.0"),
            DeclareLaunchArgument("raw_odom_jump_distance_m", default_value="5.0"),
            GroupAction(
                [
                    *_isolated_remaps(),
                    production,
                    Node(
                        package="ugv_mvp_tools",
                        executable="path_fixture_node",
                        parameters=[{"production_mode": False, "shape": path_shape}],
                    ),
                    Node(
                        package="ugv_mvp_tools",
                        executable="raw_odom_fixture_node",
                        parameters=[
                            {
                                "production_mode": False,
                                "linear_speed_mps": 0.0,
                                "stop_after_s": ParameterValue(
                                    stop_after_s, value_type=float
                                ),
                                "inject_jump_after_s": ParameterValue(
                                    jump_after_s, value_type=float
                                ),
                                "jump_distance_m": ParameterValue(
                                    jump_distance_m, value_type=float
                                ),
                            }
                        ],
                    ),
                ]
            ),
        ]
    )
