import importlib.util
from pathlib import Path
import sys
import types

import pytest


PACKAGE_ROOT = Path(__file__).parents[1]


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


def test_default_disables_extrinsics_and_static_tf(launch_modules):
    module = launch_modules("subject2.launch.py")
    context = _resolved_context(module.generate_launch_description())

    assert context["publish_lidar_static_tf"] == "false"
    assert context["lidar_extrinsics_valid"] == "false"
    nodes = module._launch_nodes(context)
    assert _adapter_parameters(nodes) == {"extrinsics_valid": False}
    assert not _static_tf_nodes(nodes)


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
