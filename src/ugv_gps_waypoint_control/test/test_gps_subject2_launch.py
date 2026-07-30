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
def launch_module(monkeypatch):
    launch = types.ModuleType("launch")
    launch.LaunchDescription = _LaunchDescription
    actions = types.ModuleType("launch.actions")
    actions.DeclareLaunchArgument = _DeclareLaunchArgument
    actions.OpaqueFunction = _Action
    substitutions = types.ModuleType("launch.substitutions")
    substitutions.LaunchConfiguration = _LaunchConfiguration
    substitutions.PathJoinSubstitution = _Action
    launch_ros = types.ModuleType("launch_ros")
    ros_actions = types.ModuleType("launch_ros.actions")
    ros_actions.Node = _Action
    ros_substitutions = types.ModuleType("launch_ros.substitutions")
    ros_substitutions.FindPackageShare = _Action
    for name, module in {
        "launch": launch,
        "launch.actions": actions,
        "launch.substitutions": substitutions,
        "launch_ros": launch_ros,
        "launch_ros.actions": ros_actions,
        "launch_ros.substitutions": ros_substitutions,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = PACKAGE_ROOT / "launch" / "gps_subject2.launch.py"
    spec = importlib.util.spec_from_file_location("gps_subject2_launch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _context(track_file: Path, **overrides):
    values = {
        "config_file": "/tmp/gps_subject2.yaml",
        "track_file": str(track_file),
        "initial_heading": "EAST",
        "motion_enabled": "false",
        "gps_serial_device": "/dev/ttyUSB0",
        "gps_serial_baud_rate": "115200",
        "gps_serial_data_bits": "8",
        "gps_serial_parity": "none",
        "gps_serial_stop_bits": "1",
    }
    values.update(overrides)
    return values


def test_defaults_are_confirmed_serial_and_motion_is_off(launch_module):
    description = launch_module.generate_launch_description()
    values = {
        item.name: item.default_value
        for item in description.entities
        if isinstance(item, _DeclareLaunchArgument)
    }
    assert values["gps_serial_device"] == "/dev/ttyUSB0"
    assert values["gps_serial_baud_rate"] == "115200"
    assert values["gps_serial_data_bits"] == "8"
    assert values["gps_serial_parity"] == "none"
    assert values["gps_serial_stop_bits"] == "1"
    assert values["motion_enabled"] == "false"


def test_launch_starts_only_gngga_and_gps_controller(launch_module, monkeypatch):
    track = "/tmp/track_output.txt"
    monkeypatch.setattr(launch_module.os.path, "isfile", lambda path: path == track)
    nodes = launch_module._nodes(_context(track, motion_enabled="true"))
    assert [node.name for node in nodes] == ["gga_serial", "gps_waypoint_controller"]
    serial_overrides = nodes[0].parameters[1]
    assert serial_overrides == {
        "device": "/dev/ttyUSB0",
        "baud_rate": 115200,
        "data_bits": 8,
        "parity": "none",
        "stop_bits": 1,
        "accepted_sentence_ids": ["GNGGA"],
    }
    controller = nodes[1].parameters[1]
    assert controller["track_file"] == track
    assert controller["initial_heading"] == "EAST"
    assert controller["motion_enabled"] is True


@pytest.mark.parametrize("track", ["", "relative/track_output.txt", "/missing.txt"])
def test_track_must_be_existing_absolute_file(launch_module, track):
    with pytest.raises(RuntimeError, match="track_file"):
        launch_module._nodes(_context(Path(track)))
