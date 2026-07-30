import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("true", "1"):
        return True
    if value in ("false", "0"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _positive_int(context, name):
    value = LaunchConfiguration(name).perform(context).strip()
    try:
        parsed = int(value)
    except ValueError as error:
        raise RuntimeError(f"{name} must be an integer") from error
    if parsed <= 0:
        raise RuntimeError(f"{name} must be positive")
    return parsed


def _nodes(context):
    track_file = LaunchConfiguration("track_file").perform(context).strip()
    if not track_file.startswith("/"):
        raise RuntimeError("track_file must be an explicit Linux absolute path")
    if not os.path.isfile(track_file):
        raise RuntimeError(f"track_file must be an existing regular file: {track_file!r}")

    serial_device = LaunchConfiguration("gps_serial_device").perform(context).strip()
    if not serial_device.startswith("/"):
        raise RuntimeError("gps_serial_device must be an explicit absolute path")
    parity = LaunchConfiguration("gps_serial_parity").perform(context).strip()
    if parity not in ("none", "even", "odd"):
        raise RuntimeError("gps_serial_parity must be none, even, or odd")
    heading = LaunchConfiguration("initial_heading").perform(context).strip()
    if not heading:
        raise RuntimeError("initial_heading must be supplied as EAST/SOUTH/WEST/NORTH")

    config_file = LaunchConfiguration("config_file")
    return [
        Node(
            package="ugv_localization_mvp",
            executable="gga_serial_node",
            name="gga_serial",
            output="screen",
            parameters=[
                config_file,
                {
                    "device": serial_device,
                    "baud_rate": _positive_int(context, "gps_serial_baud_rate"),
                    "data_bits": _positive_int(context, "gps_serial_data_bits"),
                    "parity": parity,
                    "stop_bits": _positive_int(context, "gps_serial_stop_bits"),
                    "accepted_sentence_ids": ["GNGGA"],
                },
            ],
        ),
        Node(
            package="ugv_gps_waypoint_control",
            executable="gps_waypoint_controller_node",
            name="gps_waypoint_controller",
            output="screen",
            parameters=[
                config_file,
                {
                    "track_file": track_file,
                    "initial_heading": heading,
                    "motion_enabled": _bool(context, "motion_enabled"),
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ugv_gps_waypoint_control"),
                        "config",
                        "gps_subject2.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("track_file", default_value=""),
            DeclareLaunchArgument("initial_heading", default_value=""),
            DeclareLaunchArgument("motion_enabled", default_value="false"),
            DeclareLaunchArgument("gps_serial_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("gps_serial_baud_rate", default_value="115200"),
            DeclareLaunchArgument("gps_serial_data_bits", default_value="8"),
            DeclareLaunchArgument("gps_serial_parity", default_value="none"),
            DeclareLaunchArgument("gps_serial_stop_bits", default_value="1"),
            OpaqueFunction(function=_nodes),
        ]
    )
