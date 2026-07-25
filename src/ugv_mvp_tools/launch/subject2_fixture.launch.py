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
    path_frame = LaunchConfiguration("path_frame")
    path_stop_after_s = LaunchConfiguration("path_stop_after_s")
    path_stamp_mode = LaunchConfiguration("path_stamp_mode")
    path_freeze_stamp_after_s = LaunchConfiguration("path_freeze_stamp_after_s")
    path_empty = LaunchConfiguration("path_empty")
    path_pose_frame_override = LaunchConfiguration("path_pose_frame_override")
    path_inject_nonfinite_x = LaunchConfiguration("path_inject_nonfinite_x")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    raw_odom_frame_id = LaunchConfiguration("raw_odom_frame_id")
    raw_odom_child_frame_id = LaunchConfiguration("raw_odom_child_frame_id")
    stop_after_s = LaunchConfiguration("raw_odom_stop_after_s")
    jump_after_s = LaunchConfiguration("raw_odom_inject_jump_after_s")
    jump_distance_m = LaunchConfiguration("raw_odom_jump_distance_m")
    odom_stamp_mode_after_s = LaunchConfiguration("raw_odom_stamp_mode_after_s")
    odom_stamp_mode_after = LaunchConfiguration("raw_odom_stamp_mode_after")
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
            "odom_snapshot_directory": LaunchConfiguration("odom_snapshot_directory"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("path_shape", default_value="left"),
            DeclareLaunchArgument("path_frame", default_value="map"),
            DeclareLaunchArgument("path_stop_after_s", default_value="-1.0"),
            DeclareLaunchArgument("path_stamp_mode", default_value="current"),
            DeclareLaunchArgument("path_freeze_stamp_after_s", default_value="3.0"),
            DeclareLaunchArgument("path_empty", default_value="false"),
            DeclareLaunchArgument("path_pose_frame_override", default_value=""),
            DeclareLaunchArgument("path_inject_nonfinite_x", default_value="false"),
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
            DeclareLaunchArgument(
                "raw_odom_stamp_mode_after_s", default_value="-1.0"
            ),
            DeclareLaunchArgument(
                "raw_odom_stamp_mode_after", default_value="current"
            ),
            DeclareLaunchArgument(
                "odom_snapshot_directory",
                default_value="/tmp/ugv_subject2_fixture",
            ),
            GroupAction(
                [
                    *_isolated_remaps(),
                    production,
                    Node(
                        package="ugv_mvp_tools",
                        executable="path_fixture_node",
                        parameters=[
                            {
                                "production_mode": False,
                                "shape": path_shape,
                                "frame_id": path_frame,
                                "stop_after_s": ParameterValue(
                                    path_stop_after_s, value_type=float
                                ),
                                "stamp_mode": path_stamp_mode,
                                "freeze_stamp_after_s": ParameterValue(
                                    path_freeze_stamp_after_s, value_type=float
                                ),
                                "empty_path": ParameterValue(
                                    path_empty, value_type=bool
                                ),
                                "pose_frame_override": path_pose_frame_override,
                                "inject_nonfinite_x": ParameterValue(
                                    path_inject_nonfinite_x, value_type=bool
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="ugv_mvp_tools",
                        executable="raw_odom_fixture_node",
                        parameters=[
                            {
                                "production_mode": False,
                                "topic": raw_odom_topic,
                                "frame_id": raw_odom_frame_id,
                                "child_frame_id": raw_odom_child_frame_id,
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
                                "stamp_mode_after_s": ParameterValue(
                                    odom_stamp_mode_after_s, value_type=float
                                ),
                                "stamp_mode_after": odom_stamp_mode_after,
                            }
                        ],
                    ),
                ]
            ),
        ]
    )
