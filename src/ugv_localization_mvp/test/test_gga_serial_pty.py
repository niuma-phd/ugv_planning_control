import os
from pathlib import Path
import pty
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String


def _nmea(payload: str) -> str:
    checksum = 0
    for byte in payload.encode("ascii"):
        checksum ^= byte
    return f"${payload}*{checksum:02X}"


class _Monitor(Node):
    def __init__(self):
        super().__init__("gga_serial_pty_monitor")
        self.fixes = []
        self.valid = []
        self.raw = []
        self.create_subscription(NavSatFix, "/gps/fix", self.fixes.append, 10)
        self.create_subscription(
            Bool,
            "/gps/gga_position_valid",
            lambda message: self.valid.append(message.data),
            10,
        )
        self.create_subscription(
            String,
            "/gps/gga_sentence",
            lambda message: self.raw.append(message.data),
            10,
        )


def _wait(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return predicate()


def _gga_executable() -> Path:
    return (
        Path(get_package_prefix("ugv_localization_mvp"))
        / "lib"
        / "ugv_localization_mvp"
        / "gga_serial_node"
    )


def _start_gga_process(device: str, log, quality_profile_valid: bool):
    return subprocess.Popen(
        [
            str(_gga_executable()),
            "--ros-args",
            "-p",
            f"device:={device}",
            "-p",
            "baud_rate:=115200",
            "-p",
            "data_bits:=8",
            "-p",
            "parity:=none",
            "-p",
            "stop_bits:=1",
            "-p",
            f"quality_profile_valid:={str(quality_profile_valid).lower()}",
            "-p",
            "minimum_satellites:=4",
            "-p",
            "maximum_hdop:=2.0",
            "-p",
            "poll_rate_hz:=100.0",
            "-p",
            "status_rate_hz:=20.0",
            "-p",
            "fix_timeout_sec:=0.35",
            "-p",
            "reconnect_period_sec:=0.1",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _process_has_open_path(process, target: str) -> bool:
    try:
        descriptors = Path(f"/proc/{process.pid}/fd").iterdir()
        return any(os.path.realpath(descriptor) == target for descriptor in descriptors)
    except (FileNotFoundError, PermissionError):
        return False


def test_position_only_gga_node_reads_pty_and_fails_closed():
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(200 + os.getpid() % 20)
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.close(slave_fd)
    serial_directory = tempfile.TemporaryDirectory()
    serial_path = Path(serial_directory.name) / "gps-serial"
    os.symlink(slave_path, serial_path)
    reconnect_master_fd = -1
    log = tempfile.TemporaryFile(mode="w+")
    process = None
    monitor = None
    executor = None

    try:
        process = _start_gga_process(str(serial_path), log, True)

        rclpy.init(args=[])
        monitor = _Monitor()
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        assert _wait(executor, lambda: monitor.count_publishers("/gps/fix") == 1)
        assert _wait(executor, lambda: _process_has_open_path(process, slave_path))
        assert monitor.count_publishers("/localization/gps_pose") == 0
        assert monitor.count_publishers("/localization/gps_valid") == 0
        assert _wait(executor, lambda: bool(monitor.valid))
        assert monitor.valid[-1] is False

        first = _nmea(
            "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,99,AAAA"
        )
        encoded = (first + "\r\n").encode("ascii")
        os.write(master_fd, encoded[:17])
        executor.spin_once(timeout_sec=0.1)
        assert not monitor.fixes
        os.write(master_fd, encoded[17:])
        assert _wait(executor, lambda: len(monitor.fixes) == 1 and monitor.valid[-1])
        fix = monitor.fixes[-1]
        assert fix.header.frame_id == "gps_link"
        assert fix.status.status == NavSatStatus.STATUS_FIX
        assert fix.status.service == NavSatStatus.SERVICE_GPS
        assert fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
        assert fix.latitude == pytest.approx(31.174489838333, abs=1.0e-12)
        assert fix.longitude == pytest.approx(121.387702825, abs=1.0e-12)
        assert fix.altitude == pytest.approx(57.0924, abs=1.0e-12)
        assert _wait(executor, lambda: bool(monitor.raw))
        assert monitor.raw[-1] == first

        os.write(master_fd, encoded)
        assert _wait(executor, lambda: monitor.valid[-1] is False)
        assert len(monitor.fixes) == 1

        second = _nmea(
            "GPGGA,024942.00,3110.4693903,N,12123.2621695,E,4,16,0.5,57.0924,M,0.00,M,1,0001"
        )
        os.write(master_fd, ("noise" + second + "\r\n").encode("ascii"))
        assert _wait(executor, lambda: len(monitor.fixes) == 2 and monitor.valid[-1])
        assert monitor.fixes[-1].status.status == NavSatStatus.STATUS_GBAS_FIX

        assert _wait(executor, lambda: monitor.valid[-1] is False, timeout=2.0)
        before_bad_checksum = len(monitor.fixes)
        os.write(master_fd, (second[:-2] + "00\r\n").encode("ascii"))
        executor.spin_once(timeout_sec=0.2)
        assert len(monitor.fixes) == before_bad_checksum
        assert monitor.valid[-1] is False

        os.close(master_fd)
        master_fd = -1
        reconnect_master_fd, reconnect_slave_fd = pty.openpty()
        reconnect_slave_path = os.ttyname(reconnect_slave_fd)
        os.close(reconnect_slave_fd)
        replacement = Path(serial_directory.name) / "gps-serial.new"
        os.symlink(reconnect_slave_path, replacement)
        os.replace(replacement, serial_path)
        assert _wait(
            executor,
            lambda: _process_has_open_path(process, reconnect_slave_path),
        )

        # This UTC is earlier than the last pre-disconnect epoch. A new serial
        # session must establish a fresh UTC baseline rather than reject it for hours.
        after_reconnect = _nmea(
            "GPGGA,010000.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        os.write(reconnect_master_fd, (after_reconnect + "\r\n").encode("ascii"))
        assert _wait(executor, lambda: len(monitor.fixes) == 3 and monitor.valid[-1])
        assert process.poll() is None
    finally:
        if executor is not None and monitor is not None:
            executor.remove_node(monitor)
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        if master_fd >= 0:
            os.close(master_fd)
        if reconnect_master_fd >= 0:
            os.close(reconnect_master_fd)
        log.close()
        serial_directory.cleanup()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain


def test_navsat_status_preserves_receiver_fix_when_local_profile_is_unapproved():
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(180 + os.getpid() % 20)
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.close(slave_fd)
    log = tempfile.TemporaryFile(mode="w+")
    process = None
    monitor = None
    executor = None

    try:
        process = _start_gga_process(slave_path, log, False)
        rclpy.init(args=[])
        monitor = _Monitor()
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        assert _wait(executor, lambda: monitor.count_publishers("/gps/fix") == 1)
        assert _wait(executor, lambda: _process_has_open_path(process, slave_path))
        assert _wait(executor, lambda: bool(monitor.valid))

        sentence = _nmea(
            "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        os.write(master_fd, (sentence + "\r\n").encode("ascii"))
        assert _wait(executor, lambda: len(monitor.fixes) == 1)
        assert monitor.fixes[-1].status.status == NavSatStatus.STATUS_FIX
        assert monitor.valid[-1] is False
    finally:
        if executor is not None and monitor is not None:
            executor.remove_node(monitor)
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        os.close(master_fd)
        log.close()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
