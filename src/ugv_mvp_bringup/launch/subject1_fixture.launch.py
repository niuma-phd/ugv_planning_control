from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _isolated_remaps():
    topics = (
        "/livox/lidar",
        "/livox_odometry_mapped",
        "/localization/odom",
        "/localization/odom_valid",
        "/localization/map_odom_update",
        "/subject1/obstacles",
        "/subject1/obstacle_detected",
        "/subject1/next_waypoint_base",
        "/subject1/avoidance_active",
        "/subject1/avoid_cmd_vel",
        "/subject1/selected_trajectory",
        "/tf",
        "/tf_static",
    )
    return [SetRemap(src=topic, dst=f"/fixture{topic}") for topic in topics]


def generate_launch_description() -> LaunchDescription:
    pointcloud_scenario = LaunchConfiguration("pointcloud_scenario")
    pointcloud_stop_after_s = LaunchConfiguration("pointcloud_stop_after_s")
    pointcloud_freeze_stamp_after_s = LaunchConfiguration(
        "pointcloud_freeze_stamp_after_s"
    )
    bringup_share = FindPackageShare("ugv_mvp_bringup")
    production = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "subject1.launch.py"])
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
            DeclareLaunchArgument("pointcloud_scenario", default_value="right"),
            DeclareLaunchArgument("pointcloud_stop_after_s", default_value="-1.0"),
            DeclareLaunchArgument(
                "pointcloud_freeze_stamp_after_s", default_value="-1.0"
            ),
            GroupAction(
                [
                    *_isolated_remaps(),
                    production,
                    Node(
                        package="ugv_mvp_tools",
                        executable="pointcloud_fixture_node",
                        parameters=[
                            {
                                "production_mode": False,
                                "scenario": pointcloud_scenario,
                                "frame_id": "livox_frame",
                                "stop_after_s": ParameterValue(
                                    pointcloud_stop_after_s, value_type=float
                                ),
                                "freeze_stamp_after_s": ParameterValue(
                                    pointcloud_freeze_stamp_after_s,
                                    value_type=float,
                                ),
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
        ]
    )
