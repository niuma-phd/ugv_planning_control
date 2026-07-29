import importlib.util
from pathlib import Path
import re
import sys
import types

import pytest


PACKAGE_ROOT = Path(__file__).parents[1]
WAYPOINT_FILE = "/tmp/subject2-waypoints.csv"


class _LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return str(context[self.name])


class _IfCondition:
    def __init__(self, expression):
        self.expression = expression

    def evaluate(self, context):
        value = self.expression.perform(context).strip().lower()
        if value in ("true", "1"):
            return True
        if value in ("false", "0"):
            return False
        raise ValueError(f"invalid boolean value: {value!r}")


class _Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        for name, value in kwargs.items():
            setattr(self, name, value)


class _DeclareLaunchArgument(_Action):
    def __init__(self, name, default_value=None, **kwargs):
        super().__init__(name, default_value=default_value, **kwargs)
        self.name = name
        self.default_value = default_value


class _LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


@pytest.fixture
def launch_modules(monkeypatch):
    launch = types.ModuleType("launch")
    launch.LaunchDescription = _LaunchDescription
    actions = types.ModuleType("launch.actions")
    actions.DeclareLaunchArgument = _DeclareLaunchArgument
    actions.IncludeLaunchDescription = _Action
    actions.OpaqueFunction = _Action
    conditions = types.ModuleType("launch.conditions")
    conditions.IfCondition = _IfCondition
    substitutions = types.ModuleType("launch.substitutions")
    substitutions.LaunchConfiguration = _LaunchConfiguration
    substitutions.PathJoinSubstitution = _Action
    sources = types.ModuleType("launch.launch_description_sources")
    sources.PythonLaunchDescriptionSource = _Action

    launch_ros = types.ModuleType("launch_ros")
    ros_actions = types.ModuleType("launch_ros.actions")
    ros_actions.Node = _Action
    ros_substitutions = types.ModuleType("launch_ros.substitutions")
    ros_substitutions.FindPackageShare = _Action
    parameter_descriptions = types.ModuleType("launch_ros.parameter_descriptions")
    parameter_descriptions.ParameterValue = _Action

    modules = {
        "launch": launch,
        "launch.actions": actions,
        "launch.conditions": conditions,
        "launch.substitutions": substitutions,
        "launch.launch_description_sources": sources,
        "launch_ros": launch_ros,
        "launch_ros.actions": ros_actions,
        "launch_ros.substitutions": ros_substitutions,
        "launch_ros.parameter_descriptions": parameter_descriptions,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    def load(filename):
        path = PACKAGE_ROOT / "launch" / filename
        spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module

    return load


def _resolved_context(description, overrides=None):
    context = dict(overrides or {})
    for entity in description.entities:
        if not isinstance(entity, _DeclareLaunchArgument) or entity.name in context:
            continue
        default = entity.default_value
        context[entity.name] = (
            default.perform(context)
            if isinstance(default, _LaunchConfiguration)
            else default
        )
    return context


def _enabled_context(module, **overrides):
    values = {
        "waypoint_file": WAYPOINT_FILE,
        "lidar_extrinsics_provenance": "survey-2026-07-28",
        "base_to_lidar_x": "1.0",
        "base_to_lidar_y": "2.0",
        "base_to_lidar_z": "3.0",
        "base_to_lidar_roll": "0.1",
        "base_to_lidar_pitch": "0.2",
        "base_to_lidar_yaw": "0.3",
    }
    values.update(overrides)
    return _resolved_context(module.generate_launch_description(), values)


def _adapter_parameters(nodes):
    adapter = next(node for node in nodes if node.name == "lio_odom_adapter")
    return adapter.parameters[1]


def _static_tf_nodes(nodes):
    return [
        node
        for node in nodes
        if getattr(node, "executable", None) == "static_transform_publisher"
    ]


def _gps_nodes(nodes):
    return [
        node for node in nodes if getattr(node, "executable", None) == "gga_serial_node"
    ]


def _waypoint_controller_parameters(nodes):
    controller = next(node for node in nodes if node.name == "waypoint_controller_node")
    return controller.parameters[1]


def test_production_config_default_cruise_speed_is_half_meter_per_second():
    config = (PACKAGE_ROOT / "config" / "subject2.yaml").read_text()
    controller_config = config.split("waypoint_controller_node:", maxsplit=1)[1]
    nominal_speed = float(
        re.search(r"^\s+nominal_speed:\s+([0-9.]+)\s*$", controller_config, re.MULTILINE)[1]
    )
    max_speed = float(
        re.search(r"^\s+max_speed:\s+([0-9.]+)\s*$", controller_config, re.MULTILINE)[1]
    )

    assert nominal_speed == 0.5
    assert max_speed == 1.0


@pytest.mark.parametrize("waypoint_file", ["", "relative/waypoints.csv"])
def test_waypoint_file_is_required_and_must_be_absolute(
    launch_modules, waypoint_file
):
    module = launch_modules("subject2.launch.py")
    context = _resolved_context(
        module.generate_launch_description(), {"waypoint_file": waypoint_file}
    )

    with pytest.raises(RuntimeError, match="waypoint_file"):
        module._launch_nodes(context)


def test_absolute_waypoint_file_is_forwarded_unchanged(launch_modules):
    module = launch_modules("subject2.launch.py")
    waypoint_file = "/data/routes/subject2 course.csv"
    context = _resolved_context(
        module.generate_launch_description(), {"waypoint_file": waypoint_file}
    )

    assert _waypoint_controller_parameters(module._launch_nodes(context)) == {
        "waypoint_file": waypoint_file
    }


def test_default_disables_extrinsics_and_static_tf(launch_modules):
    module = launch_modules("subject2.launch.py")
    context = _resolved_context(
        module.generate_launch_description(), {"waypoint_file": WAYPOINT_FILE}
    )

    assert context["publish_lidar_static_tf"] == "false"
    assert context["lidar_extrinsics_valid"] == "false"
    nodes = module._launch_nodes(context)
    assert _adapter_parameters(nodes) == {"extrinsics_valid": False}
    assert not _static_tf_nodes(nodes)
    assert not _gps_nodes(nodes)
    assert {node.name for node in nodes} == {
        "lio_odom_adapter",
        "map_odom_manager",
        "waypoint_controller_node",
    }


def test_explicit_gps_serial_configuration_starts_position_only_adapter(launch_modules):
    module = launch_modules("subject2.launch.py")
    context = _resolved_context(
        module.generate_launch_description(),
        {
            "waypoint_file": WAYPOINT_FILE,
            "gps_serial_device": "/dev/serial/by-id/example",
            "gps_serial_baud_rate": "115200",
            "gps_serial_data_bits": "8",
            "gps_serial_parity": "none",
            "gps_serial_stop_bits": "1",
        },
    )

    nodes = module._launch_nodes(context)
    gps = _gps_nodes(nodes)
    assert len(gps) == 1
    assert gps[0].parameters[1] == {
        "device": "/dev/serial/by-id/example",
        "baud_rate": 115200,
        "data_bits": 8,
        "parity": "none",
        "stop_bits": 1,
    }


@pytest.mark.parametrize(
    "overrides",
    [
        {"gps_serial_device": "ttyUSB0"},
        {"gps_serial_baud_rate": ""},
        {"gps_serial_baud_rate": "auto"},
        {"gps_serial_data_bits": "0"},
        {"gps_serial_parity": "mark"},
        {"gps_serial_stop_bits": ""},
    ],
)
def test_enabled_gps_rejects_incomplete_or_implicit_serial_configuration(
    launch_modules, overrides
):
    module = launch_modules("subject2.launch.py")
    values = {
        "waypoint_file": WAYPOINT_FILE,
        "gps_serial_device": "/dev/ttyUSB0",
        "gps_serial_baud_rate": "115200",
        "gps_serial_data_bits": "8",
        "gps_serial_parity": "none",
        "gps_serial_stop_bits": "1",
    }
    values.update(overrides)
    context = _resolved_context(module.generate_launch_description(), values)
    with pytest.raises(RuntimeError):
        module._launch_nodes(context)


def test_production_launch_has_no_guard_or_recovery_switches(launch_modules):
    module = launch_modules("subject2.launch.py")
    description = module.generate_launch_description()
    declared = {
        entity.name
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }

    assert "automatic_recovery_enabled" not in declared
    assert "odom_snapshot_directory" not in declared


def test_legacy_publish_true_also_enables_adapter_extrinsics(launch_modules):
    module = launch_modules("subject2.launch.py")
    context = _enabled_context(module, publish_lidar_static_tf="true")

    assert context["lidar_extrinsics_valid"] == "true"
    parameters = _adapter_parameters(module._launch_nodes(context))
    assert parameters["extrinsics_valid"] is True
    assert parameters["base_to_lidar.x"] == 1.0
    assert len(_static_tf_nodes(module._launch_nodes(context))) == 1


def test_external_lio_uses_extrinsics_without_publishing_static_tf(launch_modules):
    module = launch_modules("subject2.launch.py")
    context = _enabled_context(
        module,
        lidar_extrinsics_valid="true",
        publish_lidar_static_tf="false",
    )

    nodes = module._launch_nodes(context)
    assert _adapter_parameters(nodes)["extrinsics_valid"] is True
    assert not _static_tf_nodes(nodes)


def test_explicit_publish_only_does_not_mark_adapter_extrinsics_valid(launch_modules):
    module = launch_modules("subject2.launch.py")
    context = _enabled_context(
        module,
        lidar_extrinsics_valid="false",
        publish_lidar_static_tf="true",
    )

    nodes = module._launch_nodes(context)
    assert _adapter_parameters(nodes)["extrinsics_valid"] is False
    assert len(_static_tf_nodes(nodes)) == 1


@pytest.mark.parametrize(
    "overrides",
    [
        {"lidar_extrinsics_valid": "true", "lidar_extrinsics_provenance": ""},
        {"publish_lidar_static_tf": "true", "base_to_lidar_x": ""},
        {"lidar_extrinsics_valid": "true", "base_to_lidar_y": "not-a-number"},
        {"publish_lidar_static_tf": "true", "base_to_lidar_z": "nan"},
        {"lidar_extrinsics_valid": "true", "base_to_lidar_yaw": "inf"},
    ],
)
def test_enabled_extrinsics_reject_missing_provenance_and_nonfinite_axes(
    launch_modules, overrides
):
    module = launch_modules("subject2.launch.py")
    context = _enabled_context(module, **overrides)
    with pytest.raises(RuntimeError):
        module._launch_nodes(context)


def test_horizon_wrapper_declares_and_forwards_validity_switch(launch_modules):
    module = launch_modules("subject2_horizon.launch.py")
    description = module.generate_launch_description()
    declaration = next(
        entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
        and entity.name == "lidar_extrinsics_valid"
    )
    assert isinstance(declaration.default_value, _LaunchConfiguration)
    assert declaration.default_value.name == "publish_lidar_static_tf"

    subject2_include = next(
        entity
        for entity in description.entities
        if "launch_arguments" in entity.kwargs
        and "lidar_extrinsics_valid" in dict(entity.launch_arguments)
    )
    forwarded = dict(subject2_include.launch_arguments)["lidar_extrinsics_valid"]
    assert isinstance(forwarded, _LaunchConfiguration)
    assert forwarded.name == "lidar_extrinsics_valid"


def test_horizon_wrapper_forwards_gps_without_recovery_switches(launch_modules):
    module = launch_modules("subject2_horizon.launch.py")
    description = module.generate_launch_description()
    subject2_include = next(
        entity
        for entity in description.entities
        if "launch_arguments" in entity.kwargs
        and "gps_serial_device" in dict(entity.launch_arguments)
    )
    forwarded = dict(subject2_include.launch_arguments)
    for name in (
        "gps_serial_device",
        "gps_serial_baud_rate",
        "gps_serial_data_bits",
        "gps_serial_parity",
        "gps_serial_stop_bits",
    ):
        assert isinstance(forwarded[name], _LaunchConfiguration)
        assert forwarded[name].name == name
    assert "automatic_recovery_enabled" not in forwarded
    assert "odom_snapshot_directory" not in forwarded


def test_horizon_wrapper_declares_and_forwards_waypoint_file(launch_modules):
    module = launch_modules("subject2_horizon.launch.py")
    description = module.generate_launch_description()
    declaration = next(
        entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
        and entity.name == "waypoint_file"
    )
    assert declaration.default_value == ""

    subject2_include = next(
        entity
        for entity in description.entities
        if "launch_arguments" in entity.kwargs
        and "waypoint_file" in dict(entity.launch_arguments)
    )
    forwarded = dict(subject2_include.launch_arguments)["waypoint_file"]
    assert isinstance(forwarded, _LaunchConfiguration)
    assert forwarded.name == "waypoint_file"
