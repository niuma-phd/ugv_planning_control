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
    def __init__(self, validated_fix_topic="/gps/validated_fix"):
        super().__init__("gga_serial_pty_monitor")
        self.fixes = []
        self.validated_fixes = []
        self.valid = []
        self.raw = []
        self.create_subscription(NavSatFix, "/gps/fix", self.fixes.append, 10)
        self.create_subscription(
            NavSatFix,
            validated_fix_topic,
            self.validated_fixes.append,
            10,
        )
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


def _start_gga_process(
    device: str,
    log,
    quality_profile_valid: bool,
    validated_fix_topic=None,
    accepted_sentence_ids=None,
):
    arguments = [
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
    ]
    if validated_fix_topic is not None:
        arguments.extend(["-p", f"validated_fix_topic:={validated_fix_topic}"])
    if accepted_sentence_ids is not None:
        values = ",".join(accepted_sentence_ids)
        arguments.extend(["-p", f"accepted_sentence_ids:=[{values}]"])
    return subprocess.Popen(
        arguments,
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


def test_validated_fix_topic_must_not_alias_legacy_fix_topic():
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.close(slave_fd)
    log = tempfile.TemporaryFile(mode="w+")
    process = _start_gga_process(
        slave_path,
        log,
        True,
        validated_fix_topic="/gps/fix",
    )

    try:
        process.wait(timeout=3.0)
        assert process.returncode != 0
        log.seek(0)
        assert "fix_topic and validated_fix_topic must be different" in log.read()
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        os.close(master_fd)
        log.close()


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
        process = _start_gga_process(
            str(serial_path),
            log,
            True,
            accepted_sentence_ids=["GNGGA"],
        )

        rclpy.init(args=[])
        monitor = _Monitor()
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        assert _wait(executor, lambda: monitor.count_publishers("/gps/fix") == 1)
        assert _wait(
            executor,
            lambda: monitor.count_publishers("/gps/validated_fix") == 1,
        )
        assert _wait(executor, lambda: _process_has_open_path(process, slave_path))
        assert monitor.count_publishers("/localization/gps_pose") == 0
        assert monitor.count_publishers("/localization/gps_valid") == 0
        assert _wait(executor, lambda: bool(monitor.valid))
        assert monitor.valid[-1] is False

        first = _nmea(
            "GNGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,99,AAAA"
        )
        encoded = (first + "\r\n").encode("ascii")
        os.write(master_fd, encoded[:17])
        executor.spin_once(timeout_sec=0.1)
        assert not monitor.fixes
        os.write(master_fd, encoded[17:])
        assert _wait(
            executor,
            lambda: len(monitor.fixes) == 1
            and len(monitor.validated_fixes) == 1
            and monitor.valid[-1],
        )
        fix = monitor.fixes[-1]
        validated_fix = monitor.validated_fixes[-1]
        assert fix.header.frame_id == "gps_link"
        assert fix.status.status == NavSatStatus.STATUS_FIX
        assert fix.status.service == 0
        assert fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
        assert fix.latitude == pytest.approx(31.174489838333, abs=1.0e-12)
        assert fix.longitude == pytest.approx(121.387702825, abs=1.0e-12)
        assert fix.altitude == pytest.approx(57.0924, abs=1.0e-12)
        assert validated_fix == fix
        assert _wait(executor, lambda: bool(monitor.raw))
        assert monitor.raw[-1] == first

        fresh_before_txt = _nmea(
            "GNGGA,024942.00,3110.4693903,N,12123.2621695,E,4,16,0.5,57.0924,M,0.00,M,1,0001"
        )
        txt_sentinel = _nmea(
            "GNGGA,024943.00,3110.4893903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        valid_count_before_txt = len(monitor.valid)
        txt_sentences = [
            _nmea("GNTXT,01,01,01,NMEA unknown msg") for _ in range(10)
        ]
        interleaved = (
            "\r\n".join([fresh_before_txt, *txt_sentences, txt_sentinel])
            + "\r\n"
        )
        os.write(master_fd, interleaved.encode("ascii"))
        txt_sentinel_latitude = 31.0 + 10.4893903 / 60.0
        assert _wait(
            executor,
            lambda: len(monitor.fixes) >= 3
            and monitor.fixes[-1].latitude
            == pytest.approx(txt_sentinel_latitude, abs=1.0e-12),
        )
        txt_drain_deadline = time.monotonic() + 0.1
        while time.monotonic() < txt_drain_deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(monitor.fixes) == 3
        assert len(monitor.validated_fixes) == 3
        assert len(monitor.raw) == 3
        assert all(monitor.valid[valid_count_before_txt:])
        assert monitor.raw[-1] == txt_sentinel
        assert monitor.fixes[-2].status.status == NavSatStatus.STATUS_GBAS_FIX
        assert monitor.validated_fixes[-1] == monitor.fixes[-1]

        checksum_candidate = _nmea(
            "GNGGA,024944.00,3110.4993903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        replacement_checksum = "00" if checksum_candidate[-2:] != "00" else "FF"
        bad_checksum = checksum_candidate[:-2] + replacement_checksum
        checksum_sentinel = _nmea(
            "GNGGA,024945.00,3110.5093903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        valid_count_before_bad_checksum = len(monitor.valid)
        os.write(master_fd, (bad_checksum + "\r\n" + checksum_sentinel + "\r\n").encode("ascii"))
        checksum_sentinel_latitude = 31.0 + 10.5093903 / 60.0
        assert _wait(
            executor,
            lambda: len(monitor.fixes) >= 4
            and monitor.fixes[-1].latitude
            == pytest.approx(checksum_sentinel_latitude, abs=1.0e-12),
        )
        checksum_drain_deadline = time.monotonic() + 0.1
        while time.monotonic() < checksum_drain_deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(monitor.fixes) == 4
        assert len(monitor.validated_fixes) == 4
        assert False in monitor.valid[valid_count_before_bad_checksum:]
        assert monitor.valid[-1] is True
        assert monitor.validated_fixes[-1] == monitor.fixes[-1]

        os.write(master_fd, (checksum_sentinel + "\r\n").encode("ascii"))
        assert _wait(executor, lambda: monitor.valid[-1] is False)
        assert len(monitor.fixes) == 4
        assert len(monitor.validated_fixes) == 4

        rejected_by_field_profile = _nmea(
            "GNGGA,024946.00,3110.4693903,N,12123.2621695,E,1,3,2.1,57.0924,M,0.00,M,,0001"
        )
        os.write(master_fd, (rejected_by_field_profile + "\r\n").encode("ascii"))
        assert _wait(
            executor,
            lambda: len(monitor.fixes) == 5 and monitor.valid[-1] is False,
        )
        assert len(monitor.validated_fixes) == 4

        wrong_sentence_id = _nmea(
            "GPGGA,024947.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        sentinel = _nmea(
            "GNGGA,024948.00,3110.5693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        os.write(master_fd, (wrong_sentence_id + "\r\n" + sentinel + "\r\n").encode("ascii"))
        sentinel_latitude = 31.0 + 10.5693903 / 60.0
        assert _wait(
            executor,
            lambda: len(monitor.fixes) >= 6
            and len(monitor.validated_fixes) >= 5
            and monitor.valid[-1]
            and monitor.fixes[-1].latitude
            == pytest.approx(sentinel_latitude, abs=1.0e-12)
            and monitor.validated_fixes[-1].latitude
            == pytest.approx(sentinel_latitude, abs=1.0e-12),
        )
        assert len(monitor.fixes) == 6
        assert len(monitor.validated_fixes) == 5
        assert monitor.validated_fixes[-1] == monitor.fixes[-1]
        assert monitor.valid[-1] is True

        assert _wait(executor, lambda: monitor.valid[-1] is False, timeout=2.0)
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
            "GNGGA,010000.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        os.write(reconnect_master_fd, (after_reconnect + "\r\n").encode("ascii"))
        assert _wait(
            executor,
            lambda: len(monitor.fixes) == 7
            and len(monitor.validated_fixes) == 6
            and monitor.valid[-1],
        )
        assert monitor.validated_fixes[-1] == monitor.fixes[-1]
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
        custom_validated_topic = "/test/gps/validated_fix"
        process = _start_gga_process(
            slave_path,
            log,
            False,
            validated_fix_topic=custom_validated_topic,
        )
        rclpy.init(args=[])
        monitor = _Monitor(custom_validated_topic)
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        assert _wait(executor, lambda: monitor.count_publishers("/gps/fix") == 1)
        assert _wait(
            executor,
            lambda: monitor.count_publishers(custom_validated_topic) == 1,
        )
        assert _wait(executor, lambda: _process_has_open_path(process, slave_path))
        assert _wait(executor, lambda: bool(monitor.valid))

        sentence = _nmea(
            "GPGGA,024941.00,3110.4693903,N,12123.2621695,E,1,16,0.6,57.0924,M,0.00,M,,0001"
        )
        os.write(master_fd, (sentence + "\r\n").encode("ascii"))
        assert _wait(executor, lambda: len(monitor.fixes) == 1)
        assert monitor.fixes[-1].status.status == NavSatStatus.STATUS_FIX
        assert monitor.valid[-1] is False
        assert not monitor.validated_fixes
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
