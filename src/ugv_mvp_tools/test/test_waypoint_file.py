import math

import pytest

from ugv_mvp_tools.waypoint_file import load_waypoints


def write_csv(tmp_path, content: str):
    path = tmp_path / "waypoints.csv"
    path.write_text(content, encoding="utf-8")
    return path


def test_loads_minimal_csv_and_derives_yaw(tmp_path) -> None:
    path = write_csv(tmp_path, "x_m,y_m\n0,0\n1,1\n1,2\n")
    points = load_waypoints(path)
    assert [(point.x_m, point.y_m, point.z_m) for point in points] == [
        (0.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (1.0, 2.0, 0.0),
    ]
    assert points[0].yaw_rad == pytest.approx(math.pi / 4.0)
    assert points[1].yaw_rad == pytest.approx(math.pi / 2.0)
    assert points[2].yaw_rad == pytest.approx(math.pi / 2.0)


def test_ignores_blank_lines_and_full_line_comments(tmp_path) -> None:
    path = write_csv(
        tmp_path,
        "# test route\n\n  # another comment\nx_m,y_m\n0,0\n\n1,0\n",
    )
    assert len(load_waypoints(path)) == 2


def test_loads_optional_z_and_yaw_columns(tmp_path) -> None:
    path = write_csv(tmp_path, "yaw_rad,x_m,z_m,y_m\n0.2,0,3,0\n0.4,1,4,0\n")
    points = load_waypoints(path)
    assert points[0].z_m == 3.0
    assert points[0].yaw_rad == 0.2
    assert points[1].z_m == 4.0
    assert points[1].yaw_rad == 0.4


@pytest.mark.parametrize(
    "content, message",
    [
        ("x_m\n0\n1\n", "missing columns"),
        ("x_m,y_m,speed\n0,0,1\n1,0,1\n", "unsupported columns"),
        ("x_m,y_m\nnan,0\n1,0\n", "must be finite"),
        ("x_m,y_m\n0,inf\n1,0\n", "must be finite"),
        ("x_m,y_m\n0,0\n", "at least two points"),
        ("x_m,y_m\n1,2\n1,2\n1,2\n", "non-coincident segment"),
    ],
)
def test_rejects_invalid_csv(tmp_path, content: str, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        load_waypoints(write_csv(tmp_path, content))


def test_rejects_unreadable_path(tmp_path) -> None:
    with pytest.raises(ValueError, match="cannot read waypoint file"):
        load_waypoints(tmp_path / "missing.csv")


def test_duplicate_points_use_next_non_coincident_segment(tmp_path) -> None:
    path = write_csv(tmp_path, "x_m,y_m\n0,0\n0,0\n0,1\n0,1\n")
    points = load_waypoints(path)
    assert [point.yaw_rad for point in points] == pytest.approx(
        [math.pi / 2.0] * 4
    )
