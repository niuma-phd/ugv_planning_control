from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        "/localization/map_odom",
        "/localization/reset_odom_fault",
        "/localization/gps_pose",
        "/localization/gps_valid",
        "/localization/navigation_enabled",
        "/localization/recovery_state",
        "/localization/restart_lio",
        "/localization/lio_generation",
        "/localization/lio_process_alive",
        "/subject2/path",
        "/subject2/target_point",
        "/cmd_vel",
        "/tf",
        "/tf_static",
    )
    return [SetRemap(src=topic, dst=f"/fixture{topic}") for topic in topics]


def generate_launch_description() -> LaunchDescription:
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    raw_odom_frame_id = LaunchConfiguration("raw_odom_frame_id")
    raw_odom_child_frame_id = LaunchConfiguration("raw_odom_child_frame_id")
    stop_after_s = LaunchConfiguration("raw_odom_stop_after_s")
    jump_after_s = LaunchConfiguration("raw_odom_inject_jump_after_s")
    jump_distance_m = LaunchConfiguration("raw_odom_jump_distance_m")
    odom_stamp_mode_after_s = LaunchConfiguration("raw_odom_stamp_mode_after_s")
    odom_stamp_mode_after = LaunchConfiguration("raw_odom_stamp_mode_after")
    raw_odom_linear_speed = LaunchConfiguration("raw_odom_linear_speed_mps")
    queued_old_samples = LaunchConfiguration(
        "raw_odom_queued_old_samples_after_generation"
    )
    recovery_enabled = LaunchConfiguration("recovery_fixture_enabled")
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
            "waypoint_file": LaunchConfiguration("waypoint_file"),
            "automatic_recovery_enabled": recovery_enabled,
            "odom_snapshot_directory": LaunchConfiguration("odom_snapshot_directory"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("waypoint_file"),
            DeclareLaunchArgument(
                "raw_odom_topic", default_value="/livox_odometry_mapped"
            ),
            DeclareLaunchArgument("raw_odom_frame_id", default_value="world"),
            DeclareLaunchArgument(
                "raw_odom_child_frame_id", default_value="livox_frame"
            ),
            DeclareLaunchArgument("raw_odom_stop_after_s", default_value="-1.0"),
            DeclareLaunchArgument("raw_odom_inject_jump_after_s", default_value="-1.0"),
            DeclareLaunchArgument("raw_odom_jump_distance_m", default_value="5.0"),
            DeclareLaunchArgument("raw_odom_stamp_mode_after_s", default_value="-1.0"),
            DeclareLaunchArgument("raw_odom_stamp_mode_after", default_value="current"),
            DeclareLaunchArgument("raw_odom_linear_speed_mps", default_value="0.0"),
            DeclareLaunchArgument(
                "raw_odom_queued_old_samples_after_generation", default_value="0"
            ),
            DeclareLaunchArgument(
                "odom_snapshot_directory",
                default_value="/tmp/ugv_subject2_fixture",
            ),
            DeclareLaunchArgument("recovery_fixture_enabled", default_value="false"),
            DeclareLaunchArgument("recovery_scenario", default_value="success"),
            GroupAction(
                [
                    *_isolated_remaps(),
                    production,
                    Node(
                        package="ugv_mvp_tools",
                        executable="raw_odom_fixture_node",
                        parameters=[
                            {
                                "production_mode": False,
                                "topic": raw_odom_topic,
                                "frame_id": raw_odom_frame_id,
                                "child_frame_id": raw_odom_child_frame_id,
                                "linear_speed_mps": ParameterValue(
                                    raw_odom_linear_speed, value_type=float
                                ),
                                "stop_after_s": ParameterValue(
                                    stop_after_s, value_type=float
                                ),
                                "inject_jump_after_s": ParameterValue(
                                    jump_after_s, value_type=float
                                ),
                                "jump_distance_m": ParameterValue(
                                    jump_distance_m, value_type=float
                                ),
                                "stamp_mode_after_s": ParameterValue(
                                    odom_stamp_mode_after_s, value_type=float
                                ),
                                "stamp_mode_after": odom_stamp_mode_after,
                                "resume_on_lio_generation": ParameterValue(
                                    recovery_enabled, value_type=bool
                                ),
                                "queued_old_samples_after_generation": ParameterValue(
                                    queued_old_samples, value_type=int
                                ),
                                "restarted_origin_x_m": 20.0,
                            }
                        ],
                    ),
                    Node(
                        package="ugv_mvp_tools",
                        executable="recovery_fixture_node",
                        condition=IfCondition(recovery_enabled),
                        parameters=[
                            {
                                "production_mode": False,
                                "scenario": LaunchConfiguration("recovery_scenario"),
                            }
                        ],
                    ),
                ]
            ),
        ]
    )
