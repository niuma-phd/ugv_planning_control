#!/usr/bin/env python3
"""Own and restart the external LIO launch process without name-based killing."""

import json
import math
import os
import signal
import subprocess
import threading
import time
from typing import Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32
from std_srvs.srv import Trigger


def parse_command_json(raw: str) -> tuple[str, ...]:
    """Parse a command encoded as a JSON string array, rejecting shell forms."""
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as error:
        raise ValueError(f"command_json is not valid JSON: {error.msg}") from error
    if not isinstance(value, list) or not value:
        raise ValueError("command_json must be a non-empty JSON array")
    if any(not isinstance(item, str) or not item.strip() for item in value):
        raise ValueError("every command_json item must be a non-empty string")
    if "\x00" in "".join(value):
        raise ValueError("command_json must not contain NUL bytes")
    return tuple(value)


def require_finite_seconds(name: str, value: float, *, allow_zero: bool) -> float:
    """Validate timeout-like parameters before they affect process control."""
    if (
        isinstance(value, bool)
        or not isinstance(value, (float, int))
        or not math.isfinite(value)
        or value < 0.0
        or (value == 0.0 and not allow_zero)
    ):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be a finite {qualifier} number")
    return value


def parse_proc_stat_state_and_pgid(stat_line: str) -> tuple[str, int]:
    """Extract state and process group from Linux ``/proc/PID/stat`` text."""
    closing_parenthesis = stat_line.rfind(")")
    if closing_parenthesis < 0:
        raise ValueError("proc stat line has no closing command parenthesis")
    fields = stat_line[closing_parenthesis + 1:].split()
    if len(fields) < 3 or len(fields[0]) != 1:
        raise ValueError("proc stat line is missing state, parent PID, or process group")
    try:
        process_group = int(fields[2])
    except ValueError as error:
        raise ValueError("proc stat process group is not an integer") from error
    return fields[0], process_group


def process_group_has_live_members(pgid: int, proc_root: str = "/proc") -> bool:
    """Return whether a PGID has any non-zombie Linux process members."""
    if isinstance(pgid, bool) or not isinstance(pgid, int) or pgid <= 0:
        raise ValueError("pgid must be a positive integer")
    with os.scandir(proc_root) as entries:
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                with open(os.path.join(entry.path, "stat"), encoding="utf-8") as stat:
                    state, member_pgid = parse_proc_stat_state_and_pgid(stat.read())
            except (FileNotFoundError, ProcessLookupError):
                # Processes can disappear between scandir and open.
                continue
            except (PermissionError, ValueError):
                # Failing open here could start a new LIO while an unreadable old
                # process still owns the same group.  Treat uncertainty as live.
                return True
            if member_pgid == pgid and state != "Z":
                return True
    return False


class LioProcessSupervisor(Node):
    """Own exactly one LIO child launch process group and restart it safely."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "lio_process_supervisor", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("command_json", "")
        self.declare_parameter("termination_timeout_sec", 5.0)
        self.declare_parameter("startup_grace_sec", 3.0)
        self.declare_parameter("status_period_sec", 0.5)

        self._command = parse_command_json(
            self.get_parameter("command_json").get_parameter_value().string_value
        )
        self._termination_timeout = require_finite_seconds(
            "termination_timeout_sec",
            self.get_parameter("termination_timeout_sec").value,
            allow_zero=False,
        )
        self._startup_grace = require_finite_seconds(
            "startup_grace_sec",
            self.get_parameter("startup_grace_sec").value,
            allow_zero=True,
        )
        status_period = require_finite_seconds(
            "status_period_sec",
            self.get_parameter("status_period_sec").value,
            allow_zero=False,
        )

        self._lock = threading.RLock()
        self._process: Optional[subprocess.Popen] = None
        self._generation = 0
        self._last_alive: Optional[bool] = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._generation_publisher = self.create_publisher(
            UInt32, "/localization/lio_generation", status_qos
        )
        self._alive_publisher = self.create_publisher(
            Bool, "/localization/lio_process_alive", status_qos
        )
        self._restart_service = self.create_service(
            Trigger, "/localization/restart_lio", self._restart_callback
        )
        self._status_timer = self.create_timer(status_period, self._publish_status)

        started, detail = self._start_and_verify()
        if started:
            self.get_logger().info(detail)
        else:
            self.get_logger().error(detail)
        self._publish_status()

    def _spawn(self) -> subprocess.Popen:
        self.get_logger().info(f"Starting managed LIO command: {list(self._command)!r}")
        return subprocess.Popen(
            list(self._command),
            shell=False,
            start_new_session=True,
        )

    def _start_and_verify(self) -> tuple[bool, str]:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return False, "managed LIO process is already running"
            try:
                candidate = self._spawn()
            except OSError as error:
                self._process = None
                return False, f"failed to start managed LIO process: {error}"
            self._process = candidate

        deadline = time.monotonic() + self._startup_grace
        while True:
            return_code = candidate.poll()
            if return_code is not None:
                cleaned = self._terminate_process_group(candidate)
                with self._lock:
                    if cleaned and self._process is candidate:
                        self._process = None
                return (
                    False,
                    "managed LIO process exited during startup grace "
                    f"with code {return_code}; process group cleanup "
                    f"{'completed' if cleaned else 'failed'}",
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.05, remaining))

        with self._lock:
            if self._process is not candidate or candidate.poll() is not None:
                return False, "managed LIO process did not survive startup grace"
            self._generation = (self._generation + 1) & 0xFFFFFFFF
            generation = self._generation
        return True, f"managed LIO generation {generation} is running"

    def _signal_process_group(self, pgid: int, signum: int) -> None:
        try:
            os.killpg(pgid, signum)
        except ProcessLookupError:
            pass

    def _wait_for_empty_process_group(self, pgid: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while process_group_has_live_members(pgid):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.05, remaining))
        return True

    def _terminate_process_group(self, process: subprocess.Popen) -> bool:
        pgid = process.pid
        self._signal_process_group(pgid, signal.SIGTERM)
        empty = self._wait_for_empty_process_group(pgid, self._termination_timeout)
        if not empty:
            self.get_logger().warning(
                "managed LIO process group ignored SIGTERM; sending SIGKILL"
            )
            self._signal_process_group(pgid, signal.SIGKILL)
            empty = self._wait_for_empty_process_group(
                pgid, self._termination_timeout
            )

        # poll()/wait() reaps the direct launch child even when other members
        # kept its process group alive after the leader exited.
        if process.poll() is None:
            try:
                process.wait(timeout=self._termination_timeout)
            except subprocess.TimeoutExpired:
                self._signal_process_group(pgid, signal.SIGKILL)
                try:
                    process.wait(timeout=self._termination_timeout)
                except subprocess.TimeoutExpired:
                    empty = False
        else:
            process.wait()
        return empty

    def _stop_owned_process(self) -> bool:
        with self._lock:
            process = self._process
        if process is None:
            return True
        stopped = self._terminate_process_group(process)
        with self._lock:
            if stopped and self._process is process:
                self._process = None
        if not stopped:
            self.get_logger().error(
                f"managed LIO process group {process.pid} still has live members"
            )
        return stopped

    def _restart_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self.get_logger().warning("Restarting the owned LIO process group")
        if not self._stop_owned_process():
            response.success = False
            response.message = "old managed LIO process group could not be stopped"
        else:
            response.success, response.message = self._start_and_verify()
        self._publish_status()
        return response

    def _publish_status(self) -> None:
        with self._lock:
            process = self._process
            alive = process is not None and process.poll() is None
            generation = self._generation
        if self._last_alive is not None and self._last_alive and not alive:
            self.get_logger().error("managed LIO process is no longer alive")
        self._last_alive = alive
        self._alive_publisher.publish(Bool(data=alive))
        self._generation_publisher.publish(UInt32(data=generation))

    def destroy_node(self) -> bool:
        self._status_timer.cancel()
        if not self._stop_owned_process():
            self.get_logger().fatal("failed to clean up the managed LIO process group")
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[LioProcessSupervisor] = None
    try:
        node = LioProcessSupervisor()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except (TypeError, ValueError) as error:
        if node is None:
            rclpy.logging.get_logger("lio_process_supervisor").fatal(str(error))
        else:
            node.get_logger().fatal(str(error))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
