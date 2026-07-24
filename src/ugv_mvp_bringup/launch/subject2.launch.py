from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    publish_tf = LaunchConfiguration("publish_lidar_static_tf")
    x = LaunchConfiguration("base_to_lidar_x")
    y = LaunchConfiguration("base_to_lidar_y")
    z = LaunchConfiguration("base_to_lidar_z")
    roll = LaunchConfiguration("base_to_lidar_roll")
    pitch = LaunchConfiguration("base_to_lidar_pitch")
    yaw = LaunchConfiguration("base_to_lidar_yaw")

    arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ugv_mvp_bringup"), "config", "subject2.yaml"]
            ),
        ),
        DeclareLaunchArgument("publish_lidar_static_tf", default_value="false"),
        DeclareLaunchArgument("base_to_lidar_x", default_value="0.0"),
        DeclareLaunchArgument("base_to_lidar_y", default_value="0.0"),
        DeclareLaunchArgument("base_to_lidar_z", default_value="0.0"),
        DeclareLaunchArgument("base_to_lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("base_to_lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("base_to_lidar_yaw", default_value="0.0"),
    ]

    extrinsic_parameters = {
        "extrinsics_valid": ParameterValue(publish_tf, value_type=bool),
        "base_to_lidar.x": ParameterValue(x, value_type=float),
        "base_to_lidar.y": ParameterValue(y, value_type=float),
        "base_to_lidar.z": ParameterValue(z, value_type=float),
        "base_to_lidar.roll": ParameterValue(roll, value_type=float),
        "base_to_lidar.pitch": ParameterValue(pitch, value_type=float),
        "base_to_lidar.yaw": ParameterValue(yaw, value_type=float),
    }

    nodes = [
        Node(
            package="ugv_localization_mvp",
            executable="lio_odom_adapter_node",
            name="lio_odom_adapter",
            output="screen",
            parameters=[config_file, extrinsic_parameters],
        ),
        Node(
            package="ugv_localization_mvp",
            executable="map_odom_manager_node",
            name="map_odom_manager",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="ugv_localization_mvp",
            executable="odom_guard_node",
            name="odom_guard",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="ugv_subject2_mvp",
            executable="waypoint_controller_node",
            name="waypoint_controller_node",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_livox_static_tf",
            condition=IfCondition(publish_tf),
            arguments=[
                "--x", x,
                "--y", y,
                "--z", z,
                "--roll", roll,
                "--pitch", pitch,
                "--yaw", yaw,
                "--frame-id", "base_link",
                "--child-frame-id", "livox_frame",
            ],
        ),
    ]
    return LaunchDescription(arguments + nodes)
