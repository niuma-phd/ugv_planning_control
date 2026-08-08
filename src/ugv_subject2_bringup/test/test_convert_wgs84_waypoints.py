import csv
import importlib.util
import math
from pathlib import Path
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "convert_wgs84_waypoints.py"
SPEC = importlib.util.spec_from_file_location("convert_wgs84_waypoints", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_input(path: Path, rows: list[str]) -> None:
    path.write_text("\r\n".join(rows), encoding="utf-8", newline="")


def two_point_input(tmp_path: Path) -> Path:
    path = tmp_path / "global.txt"
    write_input(
        path,
        [
            "1;0.000000;0.000000;0.0;ignored",
            "2;0.001000;0.000000;0.0;anything",
        ],
    )
    return path


def north_displacement_input(tmp_path: Path) -> Path:
    path = tmp_path / "north.txt"
    write_input(
        path,
        [
            "1;105.000000;39.000000;0.0;ignored",
            "2;105.000000;39.001000;0.0;ignored",
        ],
    )
    return path


def track_output_input(tmp_path: Path) -> Path:
    path = tmp_path / "track_output.txt"
    write_input(
        path,
        [
            "序号;经度;纬度;高程",
            "1;120.00000000;30.00000000;0",
            "2;120.00010000;30.00000000;0",
        ],
    )
    return path


def test_track_output_format_and_cli_defaults(tmp_path: Path) -> None:
    source = MODULE.read_global_waypoints(track_output_input(tmp_path))
    assert [waypoint.sequence for waypoint in source] == [1, 2]
    assert source[0].longitude_deg == pytest.approx(120.0)
    assert source[0].latitude_deg == pytest.approx(30.0)

    args = MODULE.build_parser().parse_args(["--heading", "E"])
    assert args.input == Path("track_output.txt")
    assert args.output == Path("watpoints_odom.csv")


@pytest.mark.parametrize(
    ("heading", "x_sign", "y_sign"),
    [
        ("E", 1, 0),
        ("N", 0, -1),
        ("S", 0, 1),
        ("W", -1, 0),
    ],
)
def test_cardinal_heading_rotates_east_displacement(
    tmp_path: Path, heading: str, x_sign: int, y_sign: int
) -> None:
    source = MODULE.read_global_waypoints(two_point_input(tmp_path))
    converted = MODULE.convert_to_initial_odom(
        source, MODULE.heading_alias_degrees(heading)
    )

    assert converted[0].sequence == 1
    assert converted[0].x_m == 0.0
    assert converted[0].y_m == 0.0
    assert math.hypot(converted[1].x_m, converted[1].y_m) == pytest.approx(
        111.31949, rel=1.0e-5
    )
    if x_sign == 0:
        assert converted[1].x_m == pytest.approx(0.0, abs=1.0e-8)
    else:
        assert math.copysign(1.0, converted[1].x_m) == x_sign
    if y_sign == 0:
        assert converted[1].y_m == pytest.approx(0.0, abs=1.0e-8)
    else:
        assert math.copysign(1.0, converted[1].y_m) == y_sign


@pytest.mark.parametrize(
    ("heading", "x_sign", "y_sign"),
    [
        ("N", 1, 0),
        ("E", 0, 1),
        ("S", -1, 0),
        ("W", 0, -1),
    ],
)
def test_cardinal_heading_rotates_north_displacement_at_nonzero_latitude(
    tmp_path: Path, heading: str, x_sign: int, y_sign: int
) -> None:
    source = MODULE.read_global_waypoints(north_displacement_input(tmp_path))
    converted = MODULE.convert_to_initial_odom(
        source, MODULE.heading_alias_degrees(heading)
    )

    assert math.hypot(converted[1].x_m, converted[1].y_m) == pytest.approx(
        111.015, rel=2.0e-4
    )
    if x_sign == 0:
        assert converted[1].x_m == pytest.approx(0.0, abs=1.0e-8)
    else:
        assert math.copysign(1.0, converted[1].x_m) == x_sign
    if y_sign == 0:
        assert converted[1].y_m == pytest.approx(0.0, abs=1.0e-8)
    else:
        assert math.copysign(1.0, converted[1].y_m) == y_sign


def test_chinese_heading_aliases_and_arbitrary_angle() -> None:
    assert MODULE.heading_alias_degrees("\u5317") == 0.0
    assert MODULE.heading_alias_degrees("\u4e1c") == 90.0
    assert MODULE.heading_alias_degrees("\u5357") == 180.0
    assert MODULE.heading_alias_degrees("\u897f") == 270.0
    assert MODULE.arbitrary_heading_degrees("-90") == 270.0
    assert MODULE.arbitrary_heading_degrees("450") == 90.0


def test_arbitrary_heading_rotates_into_x_forward_y_left(tmp_path: Path) -> None:
    source = MODULE.read_global_waypoints(two_point_input(tmp_path))
    converted = MODULE.convert_to_initial_odom(source, 45.0)

    assert converted[1].x_m > 0.0
    assert converted[1].y_m < 0.0
    assert abs(converted[1].x_m) == pytest.approx(
        abs(converted[1].y_m), rel=1.0e-8
    )


def test_conversion_retains_every_row_and_writes_controller_csv(tmp_path: Path) -> None:
    source_path = tmp_path / "global.txt"
    output_path = tmp_path / "waypoints.csv"
    write_input(
        source_path,
        [
            "10;105.00000;39.00000;100.0;0",
            "20;105.00001;39.00000;100.0;unused",
            "30;105.00002;39.00000;101.0;",
        ],
    )

    report = MODULE.convert_file(source_path, output_path, 90.0)

    assert [point.sequence for point in report.waypoints] == [10, 20, 30]
    assert report.waypoints[0].x_m == 0.0
    assert report.waypoints[0].y_m == 0.0
    with output_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    assert rows[0] == ["x_m", "y_m"]
    assert len(rows) == 4
    assert rows[1] == ["0.000000", "0.000000"]
    assert float(rows[2][0]) > 0.0


def test_altitude_is_used_in_ecef() -> None:
    origin = MODULE.GlobalWaypoint(1, 105.0, 39.0, 0.0)
    elevated = MODULE.GlobalWaypoint(2, 105.0, 39.0, 10.0)
    origin_ecef = MODULE.geodetic_to_ecef(origin)
    elevated_ecef = MODULE.geodetic_to_ecef(elevated)
    delta = tuple(
        elevated_value - origin_value
        for elevated_value, origin_value in zip(elevated_ecef, origin_ecef)
    )
    east, north, up = MODULE.ecef_delta_to_enu(delta, origin)

    assert math.sqrt(sum(component * component for component in delta)) == pytest.approx(
        10.0, abs=1.0e-9
    )
    assert east == pytest.approx(0.0, abs=1.0e-9)
    assert north == pytest.approx(0.0, abs=1.0e-9)
    assert up == pytest.approx(10.0, abs=1.0e-9)


def test_altitude_is_not_written(tmp_path: Path) -> None:
    source_path = tmp_path / "global.txt"
    output_path = tmp_path / "waypoints.csv"
    write_input(
        source_path,
        [
            "1;105.00000;39.00000;0.0;ignored",
            "2;105.00001;39.00000;10.0;ignored",
        ],
    )

    report = MODULE.convert_file(source_path, output_path, 0.0)

    assert report.origin.altitude_m == 0.0
    with output_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    assert rows[0] == ["x_m", "y_m"]
    assert all(len(row) == 2 for row in rows)
    assert rows[1] == ["0.000000", "0.000000"]


def test_report_omits_altitude(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    source_path = two_point_input(tmp_path)
    output_path = tmp_path / "waypoints.csv"
    report = MODULE.convert_file(source_path, output_path, 90.0)

    MODULE.print_report(report, output_path)

    lines = capsys.readouterr().out.splitlines()
    origin_line = next(
        line for line in lines if line.startswith("odom_origin_wgs84_lon_lat=")
    )
    assert origin_line == "odom_origin_wgs84_lon_lat=(0.0000000000, 0.0000000000)"


def test_existing_output_requires_force(tmp_path: Path) -> None:
    source_path = two_point_input(tmp_path)
    output_path = tmp_path / "waypoints.csv"
    output_path.write_text("keep me", encoding="utf-8")

    with pytest.raises(FileExistsError, match="--force"):
        MODULE.convert_file(source_path, output_path, 90.0)
    assert output_path.read_text(encoding="utf-8") == "keep me"

    MODULE.convert_file(source_path, output_path, 90.0, force=True)
    assert output_path.read_text(encoding="utf-8").startswith("x_m,y_m\n")


def test_controller_rejects_are_caught_before_writing(tmp_path: Path) -> None:
    source_path = tmp_path / "coincident.txt"
    output_path = tmp_path / "waypoints.csv"
    write_input(
        source_path,
        [
            "1;105.00000;39.00000;0.0;ignored",
            "2;105.00000;39.00000;10.0;ignored",
        ],
    )

    with pytest.raises(ValueError, match="non-zero-length planar segment"):
        MODULE.convert_file(source_path, output_path, 90.0)
    assert not output_path.exists()

    too_many = [
        MODULE.LocalWaypoint(index, float(index), 0.0)
        for index in range(MODULE.MAXIMUM_WAYPOINT_COUNT + 1)
    ]
    with pytest.raises(ValueError, match="10000-waypoint"):
        MODULE.render_local_waypoint_csv(too_many)

    quantized_to_zero = [
        MODULE.LocalWaypoint(1, 0.0, 0.0),
        MODULE.LocalWaypoint(2, -0.4e-6, 0.4e-6),
    ]
    with pytest.raises(ValueError, match="non-zero-length planar segment"):
        MODULE.render_local_waypoint_csv(quantized_to_zero)

    non_finite = [
        MODULE.LocalWaypoint(1, 0.0, 0.0),
        MODULE.LocalWaypoint(2, math.inf, 1.0),
    ]
    with pytest.raises(ValueError, match="finite"):
        MODULE.render_local_waypoint_csv(non_finite)


def test_one_kilometre_segment_is_retained(tmp_path: Path) -> None:
    source_path = tmp_path / "long_segment.txt"
    output_path = tmp_path / "waypoints.csv"
    write_input(
        source_path,
        [
            "10;105.00000;39.00000;0.0;ignored",
            "20;105.01200;39.00000;0.0;ignored",
        ],
    )

    report = MODULE.convert_file(source_path, output_path, 90.0)

    assert [point.sequence for point in report.waypoints] == [10, 20]
    assert report.segment_lengths_m[0] > 1_000.0
    assert output_path.is_file()


@pytest.mark.parametrize(
    ("rows", "message"),
    [
        (["1;105;39"], "expected 4"),
        (["1;181;39;0;x", "2;105;39;0;x"], "longitude"),
        (["1;105;91;0;x", "2;105;39;0;x"], "latitude"),
        (["2;105;39;0;x", "1;105.1;39;0;x"], "must be greater"),
        (["1;105;39;0;x"], "at least two"),
    ],
)
def test_invalid_input_is_rejected(
    tmp_path: Path, rows: list[str], message: str
) -> None:
    path = tmp_path / "bad.txt"
    write_input(path, rows)
    with pytest.raises(ValueError, match=message):
        MODULE.read_global_waypoints(path)


def test_input_and_output_must_differ(tmp_path: Path) -> None:
    path = two_point_input(tmp_path)
    with pytest.raises(ValueError, match="must be different"):
        MODULE.convert_file(path, path, 90.0, force=True)
