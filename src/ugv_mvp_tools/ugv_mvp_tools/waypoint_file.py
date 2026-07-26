"""Strict CSV waypoint loading shared by the file path publisher and tests."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


REQUIRED_COLUMNS = frozenset({"x_m", "y_m"})
OPTIONAL_COLUMNS = frozenset({"z_m", "yaw_rad"})
ALLOWED_COLUMNS = REQUIRED_COLUMNS | OPTIONAL_COLUMNS


@dataclass(frozen=True)
class Waypoint:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float


def _content_lines(path: Path) -> list[str]:
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise ValueError(f"cannot read waypoint file {path}: {error}") from error
    return [
        line
        for line in text.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


def _parse_finite(row: dict[str, str], column: str, line_number: int) -> float:
    raw_value = row.get(column)
    if raw_value is None or not raw_value.strip():
        raise ValueError(f"line {line_number}: {column} must not be empty")
    try:
        value = float(raw_value)
    except ValueError as error:
        raise ValueError(
            f"line {line_number}: {column} must be a number"
        ) from error
    if not math.isfinite(value):
        raise ValueError(f"line {line_number}: {column} must be finite")
    return value


def _derived_yaws(points: list[tuple[float, float]]) -> list[float]:
    result: list[float | None] = [None] * len(points)
    for index, (x_m, y_m) in enumerate(points[:-1]):
        for next_x_m, next_y_m in points[index + 1 :]:
            delta_x = next_x_m - x_m
            delta_y = next_y_m - y_m
            if delta_x != 0.0 or delta_y != 0.0:
                result[index] = math.atan2(delta_y, delta_x)
                break

    last_yaw: float | None = None
    for index, yaw_rad in enumerate(result):
        if yaw_rad is not None:
            last_yaw = yaw_rad
        elif last_yaw is not None:
            result[index] = last_yaw

    first_yaw = next((yaw for yaw in result if yaw is not None), None)
    if first_yaw is None:
        raise ValueError("waypoints must contain at least one non-coincident segment")
    return [first_yaw if yaw is None else yaw for yaw in result]


def load_waypoints(path_value: str | Path) -> tuple[Waypoint, ...]:
    """Load and completely validate one UTF-8 waypoint CSV file."""

    path = Path(path_value).expanduser()
    lines = _content_lines(path)
    if not lines:
        raise ValueError("waypoint file must contain a CSV header")

    reader = csv.DictReader(lines, skipinitialspace=True)
    if reader.fieldnames is None:
        raise ValueError("waypoint file must contain a CSV header")
    columns = [column.strip() for column in reader.fieldnames]
    if len(columns) != len(set(columns)):
        raise ValueError("waypoint CSV header contains duplicate columns")
    missing = REQUIRED_COLUMNS - set(columns)
    unexpected = set(columns) - ALLOWED_COLUMNS
    if missing:
        raise ValueError(f"waypoint CSV is missing columns: {sorted(missing)}")
    if unexpected:
        raise ValueError(f"waypoint CSV has unsupported columns: {sorted(unexpected)}")
    reader.fieldnames = columns

    raw_points: list[tuple[float, float, float, float | None]] = []
    for line_number, row in enumerate(reader, start=2):
        if None in row:
            raise ValueError(f"line {line_number}: too many CSV fields")
        x_m = _parse_finite(row, "x_m", line_number)
        y_m = _parse_finite(row, "y_m", line_number)
        z_m = (
            _parse_finite(row, "z_m", line_number) if "z_m" in columns else 0.0
        )
        yaw_rad = (
            _parse_finite(row, "yaw_rad", line_number)
            if "yaw_rad" in columns
            else None
        )
        raw_points.append((x_m, y_m, z_m, yaw_rad))

    if len(raw_points) < 2:
        raise ValueError("waypoint file must contain at least two points")
    xy_points = [(point[0], point[1]) for point in raw_points]
    if all(point == xy_points[0] for point in xy_points[1:]):
        raise ValueError("waypoints must contain at least one non-coincident segment")

    if "yaw_rad" in columns:
        yaws = [point[3] for point in raw_points]
    else:
        yaws = _derived_yaws(xy_points)
    return tuple(
        Waypoint(x_m, y_m, z_m, yaw_rad)
        for (x_m, y_m, z_m, _), yaw_rad in zip(raw_points, yaws, strict=True)
    )
