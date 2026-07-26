import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import pytest
import rclpy
from rclpy.parameter import Parameter


SCRIPT = Path(__file__).parents[1] / "scripts" / "lio_process_supervisor.py"
SPEC = importlib.util.spec_from_file_location("lio_process_supervisor", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_command_json_requires_nonempty_string_array():
    assert MODULE.parse_command_json('["ros2", "launch", "pkg", "file.py"]') == (
        "ros2",
        "launch",
        "pkg",
        "file.py",
    )
    for invalid in ("", "{}", "[]", '["ros2", ""]', '["ros2", 3]'):
        with pytest.raises(ValueError):
            MODULE.parse_command_json(invalid)


def test_command_json_rejects_nul_and_whitespace_only_arguments():
    for invalid in ('["ros2", "   "]', '"ros2"', '["ros2", "\\u0000"]'):
        with pytest.raises(ValueError):
            MODULE.parse_command_json(invalid)


def test_seconds_validation_is_finite_and_respects_zero_policy():
    assert MODULE.require_finite_seconds("grace", 0.0, allow_zero=True) == 0.0
    assert MODULE.require_finite_seconds("timeout", 0.1, allow_zero=False) == 0.1
    for invalid in (-1.0, math.inf, -math.inf, math.nan, True, "1.0"):
        with pytest.raises(ValueError):
            MODULE.require_finite_seconds("timeout", invalid, allow_zero=True)
    with pytest.raises(ValueError):
        MODULE.require_finite_seconds("timeout", 0.0, allow_zero=False)


def test_proc_stat_parser_handles_parentheses_in_command_name():
    state, pgid = MODULE.parse_proc_stat_state_and_pgid(
        "123 (lio worker (managed)) S 42 987 987 0 -1"
    )
    assert state == "S"
    assert pgid == 987
    with pytest.raises(ValueError):
        MODULE.parse_proc_stat_state_and_pgid("malformed")


def _write_proc_stat(root: Path, pid: int, state: str, pgid: int) -> None:
    process_directory = root / str(pid)
    process_directory.mkdir()
    (process_directory / "stat").write_text(
        f"{pid} (test process) {state} 1 {pgid} {pgid} 0 -1",
        encoding="utf-8",
    )


def test_process_group_membership_ignores_zombies(tmp_path):
    _write_proc_stat(tmp_path, 100, "Z", 77)
    _write_proc_stat(tmp_path, 101, "S", 88)
    assert not MODULE.process_group_has_live_members(77, str(tmp_path))
    assert MODULE.process_group_has_live_members(88, str(tmp_path))

    _write_proc_stat(tmp_path, 102, "D", 77)
    assert MODULE.process_group_has_live_members(77, str(tmp_path))


def test_process_group_membership_rejects_invalid_pgid(tmp_path):
    for pgid in (0, -1, True, "7"):
        with pytest.raises(ValueError):
            MODULE.process_group_has_live_members(pgid, str(tmp_path))


def test_process_group_membership_fails_closed_on_malformed_proc_entry(tmp_path):
    process_directory = tmp_path / "100"
    process_directory.mkdir()
    (process_directory / "stat").write_text("malformed", encoding="utf-8")
    assert MODULE.process_group_has_live_members(77, str(tmp_path))


def _wait_for_run_records(path: Path, count: int, timeout: float = 3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.exists():
            records = [
                tuple(map(int, line.split(",")))
                for line in path.read_text(encoding="utf-8").splitlines()
                if line
            ]
            if len(records) >= count:
                return records
        time.sleep(0.01)
    raise AssertionError(f"expected {count} process records in {path}")


def test_real_restart_kills_term_ignoring_process_group_and_starts_new_generation(
    tmp_path,
):
    child_code = tmp_path / "term_ignoring_child.py"
    child_code.write_text(
        """\
import os
import signal
import sys
import time

signal.signal(signal.SIGTERM, signal.SIG_IGN)
with open(sys.argv[1], "w", encoding="utf-8") as marker:
    marker.write(str(os.getpid()))
    marker.flush()
    os.fsync(marker.fileno())
while True:
    time.sleep(1.0)
""",
        encoding="utf-8",
    )
    leader_code = tmp_path / "term_ignoring_leader.py"
    leader_code.write_text(
        """\
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

signal.signal(signal.SIGTERM, signal.SIG_IGN)
state_dir = Path(sys.argv[1])
marker = state_dir / f"child-{os.getpid()}.ready"
child = subprocess.Popen([sys.executable, sys.argv[2], str(marker)])
deadline = time.monotonic() + 2.0
while not marker.exists() and time.monotonic() < deadline:
    time.sleep(0.01)
if not marker.exists():
    raise RuntimeError("child did not install its SIGTERM handler")
with (state_dir / "runs.csv").open("a", encoding="utf-8") as records:
    records.write(f"{os.getpid()},{child.pid},{os.getpgrp()}\\n")
    records.flush()
    os.fsync(records.fileno())
while True:
    time.sleep(1.0)
""",
        encoding="utf-8",
    )
    records_path = tmp_path / "runs.csv"
    command = [sys.executable, str(leader_code), str(tmp_path), str(child_code)]
    overrides = [
        Parameter("command_json", value=json.dumps(command)),
        Parameter("termination_timeout_sec", value=0.15),
        Parameter("startup_grace_sec", value=0.1),
        Parameter("status_period_sec", value=10.0),
    ]

    node = None
    recorded_signals = []
    rclpy.init()
    try:
        node = MODULE.LioProcessSupervisor(parameter_overrides=overrides)
        first_records = _wait_for_run_records(records_path, 1)
        first_leader, first_child, first_pgid = first_records[0]
        assert first_leader == first_pgid == node._process.pid
        assert os.getpgid(first_child) == first_pgid
        assert MODULE.process_group_has_live_members(first_pgid)
        assert node._generation == 1

        original_signal_process_group = node._signal_process_group

        def record_and_signal(pgid, signum):
            recorded_signals.append((pgid, signum))
            original_signal_process_group(pgid, signum)

        node._signal_process_group = record_and_signal
        old_process = node._process
        response = node._restart_callback(
            MODULE.Trigger.Request(), MODULE.Trigger.Response()
        )

        assert response.success, response.message
        assert recorded_signals[:2] == [
            (first_pgid, signal.SIGTERM),
            (first_pgid, signal.SIGKILL),
        ]
        assert old_process.returncode == -signal.SIGKILL
        assert not MODULE.process_group_has_live_members(first_pgid)

        second_records = _wait_for_run_records(records_path, 2)
        second_leader, second_child, second_pgid = second_records[1]
        assert second_leader == second_pgid == node._process.pid
        assert second_pgid != first_pgid
        assert os.getpgid(second_child) == second_pgid
        assert MODULE.process_group_has_live_members(second_pgid)
        assert node._generation == 2

        node.destroy_node()
        node = None
        assert recorded_signals[-2:] == [
            (second_pgid, signal.SIGTERM),
            (second_pgid, signal.SIGKILL),
        ]
        assert not MODULE.process_group_has_live_members(second_pgid)
    finally:
        if node is not None:
            node.destroy_node()
        # If an assertion interrupts the normal path, forcibly clean every
        # process group recorded by the temporary command before leaving.
        if records_path.exists():
            for record in records_path.read_text(encoding="utf-8").splitlines():
                if not record:
                    continue
                pgid = int(record.split(",")[2])
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if rclpy.ok():
            rclpy.shutdown()
