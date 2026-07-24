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
        "/subject1/nominal_cmd_vel",
        "/subject1/avoidance_active",
        "/subject1/avoid_cmd_vel",
        "/subject1/selected_trajectory",
        "/cmd_vel",
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
    pointcloud_scenario_after_s = LaunchConfiguration(
        "pointcloud_scenario_after_s"
    )
    pointcloud_scenario_after = LaunchConfiguration("pointcloud_scenario_after")
    nominal_cmd_stop_after_s = LaunchConfiguration("nominal_cmd_stop_after_s")
    nominal_linear_x = LaunchConfiguration("nominal_linear_x")
    nominal_angular_z = LaunchConfiguration("nominal_angular_z")
    bringup_share = FindPackageShare("ugv_mvp_bringup")
    production = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "subject1.launch.py"])
        ),
        launch_arguments={
            "publish_lidar_static_tf": "false",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("pointcloud_scenario", default_value="right"),
            DeclareLaunchArgument("pointcloud_stop_after_s", default_value="-1.0"),
            DeclareLaunchArgument(
                "pointcloud_freeze_stamp_after_s", default_value="-1.0"
            ),
            DeclareLaunchArgument(
                "pointcloud_scenario_after_s", default_value="-1.0"
            ),
            DeclareLaunchArgument(
                "pointcloud_scenario_after", default_value="none"
            ),
            DeclareLaunchArgument("nominal_cmd_stop_after_s", default_value="-1.0"),
            DeclareLaunchArgument("nominal_linear_x", default_value="0.23"),
            DeclareLaunchArgument("nominal_angular_z", default_value="-0.07"),
            GroupAction(
                [
                    *_isolated_remaps(),
                    production,
                    Node(
                        package="ugv_mvp_tools",
                        executable="static_tf_fixture_node",
                        parameters=[{"production_mode": False}],
                    ),
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
                                "scenario_after_s": ParameterValue(
                                    pointcloud_scenario_after_s,
                                    value_type=float,
                                ),
                                "scenario_after": pointcloud_scenario_after,
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
                    Node(
                        package="ugv_mvp_tools",
                        executable="nominal_cmd_fixture_node",
                        parameters=[
                            {
                                "production_mode": False,
                                "linear_x": ParameterValue(
                                    nominal_linear_x, value_type=float
                                ),
                                "angular_z": ParameterValue(
                                    nominal_angular_z, value_type=float
                                ),
                                "stop_after_s": ParameterValue(
                                    nominal_cmd_stop_after_s, value_type=float
                                ),
                            }
                        ],
                    ),
                ]
            )
        ]
    )
