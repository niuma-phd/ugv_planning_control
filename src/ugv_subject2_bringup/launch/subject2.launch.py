import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _explicit_finite_float(context, name: str) -> float:
    raw = LaunchConfiguration(name).perform(context).strip()
    if not raw:
        raise RuntimeError(
            f"{name} must be supplied explicitly when lidar extrinsics are enabled"
        )
    try:
        value = float(raw)
    except ValueError as error:
        raise RuntimeError(f"{name} is not a floating-point value: {raw!r}") from error
    if not math.isfinite(value):
        raise RuntimeError(f"{name} must be finite")
    return value


def _explicit_positive_int(context, name: str) -> int:
    raw = LaunchConfiguration(name).perform(context).strip()
    if not raw:
        raise RuntimeError(f"{name} must be supplied when gps_serial_device is set")
    try:
        value = int(raw)
    except ValueError as error:
        raise RuntimeError(f"{name} is not an integer: {raw!r}") from error
    if value <= 0:
        raise RuntimeError(f"{name} must be positive")
    return value


def _launch_nodes(context):
    config_file = LaunchConfiguration("config_file")
    snapshot_directory = LaunchConfiguration("odom_snapshot_directory")
    waypoint_file = LaunchConfiguration("waypoint_file").perform(context)
    if not waypoint_file:
        raise RuntimeError("waypoint_file must be supplied explicitly")
    if not os.path.isabs(waypoint_file):
        raise RuntimeError("waypoint_file must be an explicit absolute path")
    extrinsics_valid = IfCondition(
        LaunchConfiguration("lidar_extrinsics_valid")
    ).evaluate(context)
    publish_tf = IfCondition(LaunchConfiguration("publish_lidar_static_tf")).evaluate(
        context
    )
    extrinsic_parameters = {"extrinsics_valid": extrinsics_valid}
    static_tf_nodes = []
    gps_nodes = []

    gps_device = LaunchConfiguration("gps_serial_device").perform(context).strip()
    if gps_device:
        if not gps_device.startswith("/"):
            raise RuntimeError("gps_serial_device must be an explicit absolute path")
        parity = LaunchConfiguration("gps_serial_parity").perform(context).strip()
        if parity not in ("none", "even", "odd"):
            raise RuntimeError(
                "gps_serial_parity must be none, even, or odd when GPS is enabled"
            )
        gps_nodes.append(
            Node(
                package="ugv_localization_mvp",
                executable="gga_serial_node",
                name="gga_serial",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "device": gps_device,
                        "baud_rate": _explicit_positive_int(
                            context, "gps_serial_baud_rate"
                        ),
                        "data_bits": _explicit_positive_int(
                            context, "gps_serial_data_bits"
                        ),
                        "parity": parity,
                        "stop_bits": _explicit_positive_int(
                            context, "gps_serial_stop_bits"
                        ),
                    },
                ],
            )
        )

    if extrinsics_valid or publish_tf:
        provenance = (
            LaunchConfiguration("lidar_extrinsics_provenance").perform(context).strip()
        )
        if not provenance:
            raise RuntimeError(
                "lidar_extrinsics_provenance is required when lidar extrinsics "
                "are enabled"
            )
        values = {
            name: _explicit_finite_float(context, name)
            for name in (
                "base_to_lidar_x",
                "base_to_lidar_y",
                "base_to_lidar_z",
                "base_to_lidar_roll",
                "base_to_lidar_pitch",
                "base_to_lidar_yaw",
            )
        }
        extrinsic_parameters.update(
            {
                "base_to_lidar.x": values["base_to_lidar_x"],
                "base_to_lidar.y": values["base_to_lidar_y"],
                "base_to_lidar.z": values["base_to_lidar_z"],
                "base_to_lidar.roll": values["base_to_lidar_roll"],
                "base_to_lidar.pitch": values["base_to_lidar_pitch"],
                "base_to_lidar.yaw": values["base_to_lidar_yaw"],
            }
        )
        if publish_tf:
            static_tf_nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="base_to_livox_static_tf",
                    arguments=[
                        "--x",
                        str(values["base_to_lidar_x"]),
                        "--y",
                        str(values["base_to_lidar_y"]),
                        "--z",
                        str(values["base_to_lidar_z"]),
                        "--roll",
                        str(values["base_to_lidar_roll"]),
                        "--pitch",
                        str(values["base_to_lidar_pitch"]),
                        "--yaw",
                        str(values["base_to_lidar_yaw"]),
                        "--frame-id",
                        "base_link",
                        "--child-frame-id",
                        "livox_frame",
                    ],
                )
            )

    return [
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
            parameters=[
                config_file,
                {"snapshot_directory": snapshot_directory},
            ],
        ),
        Node(
            package="ugv_localization_mvp",
            executable="recovery_coordinator_node",
            name="recovery_coordinator",
            output="screen",
            parameters=[
                config_file,
                {
                    "automatic_recovery_enabled": IfCondition(
                        LaunchConfiguration("automatic_recovery_enabled")
                    ).evaluate(context)
                },
            ],
        ),
        Node(
            package="ugv_subject2_mvp",
            executable="waypoint_controller_node",
            name="waypoint_controller_node",
            output="screen",
            parameters=[config_file, {"waypoint_file": waypoint_file}],
        ),
        *gps_nodes,
        *static_tf_nodes,
    ]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ugv_subject2_bringup"), "config", "subject2.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "odom_snapshot_directory",
            default_value="/home/sunrise/.ros/ugv_mvp",
        ),
        DeclareLaunchArgument("waypoint_file", default_value=""),
        DeclareLaunchArgument("publish_lidar_static_tf", default_value="false"),
        DeclareLaunchArgument(
            "lidar_extrinsics_valid",
            default_value=LaunchConfiguration("publish_lidar_static_tf"),
        ),
        DeclareLaunchArgument("lidar_extrinsics_provenance", default_value=""),
        DeclareLaunchArgument("base_to_lidar_x", default_value=""),
        DeclareLaunchArgument("base_to_lidar_y", default_value=""),
        DeclareLaunchArgument("base_to_lidar_z", default_value=""),
        DeclareLaunchArgument("base_to_lidar_roll", default_value=""),
        DeclareLaunchArgument("base_to_lidar_pitch", default_value=""),
        DeclareLaunchArgument("base_to_lidar_yaw", default_value=""),
        DeclareLaunchArgument("gps_serial_device", default_value=""),
        DeclareLaunchArgument("gps_serial_baud_rate", default_value=""),
        DeclareLaunchArgument("gps_serial_data_bits", default_value=""),
        DeclareLaunchArgument("gps_serial_parity", default_value=""),
        DeclareLaunchArgument("gps_serial_stop_bits", default_value=""),
        DeclareLaunchArgument("automatic_recovery_enabled", default_value="false"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_nodes)])
