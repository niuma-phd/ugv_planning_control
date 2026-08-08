#!/usr/bin/env python3
"""Convert track_output.txt WGS84 waypoints into the UGV's initial odom frame.

The first input waypoint defines the WGS84 position of the odom origin and is
also retained as the first output row. Consequently, its local coordinate is
mathematically (0, 0).
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import math
import os
from dataclasses import dataclass
from pathlib import Path
import tempfile
from typing import Iterable, Sequence


WGS84_SEMI_MAJOR_AXIS_M = 6_378_137.0
WGS84_FLATTENING = 1.0 / 298.257_223_563
WGS84_ECCENTRICITY_SQUARED = (
    WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)
)
MAXIMUM_WAYPOINT_FILE_BYTES = 2 * 1024 * 1024
MAXIMUM_WAYPOINT_LINE_BYTES = 4096
MAXIMUM_WAYPOINT_COUNT = 10_000
DEFAULT_INPUT_PATH = Path("track_output.txt")
# Keep the spelling requested by the upstream/downstream file contract.
DEFAULT_OUTPUT_PATH = Path("watpoints_odom.csv")
TRACK_OUTPUT_HEADERS = {
    ("序号", "经度", "纬度", "高程"),
    ("sequence", "longitude", "latitude", "altitude"),
}


@dataclass(frozen=True)
class GlobalWaypoint:
    sequence: int
    longitude_deg: float
    latitude_deg: float
    altitude_m: float


@dataclass(frozen=True)
class LocalWaypoint:
    sequence: int
    x_m: float
    y_m: float


@dataclass(frozen=True)
class ConversionReport:
    source_sha256: str
    origin: GlobalWaypoint
    heading_deg: float
    waypoints: tuple[LocalWaypoint, ...]
    segment_lengths_m: tuple[float, ...]

    @property
    def total_length_m(self) -> float:
        return sum(self.segment_lengths_m)


HEADING_ALIASES_DEG = {
    "N": 0.0,
    "NORTH": 0.0,
    "\u5317": 0.0,
    "E": 90.0,
    "EAST": 90.0,
    "\u4e1c": 90.0,
    "S": 180.0,
    "SOUTH": 180.0,
    "\u5357": 180.0,
    "W": 270.0,
    "WEST": 270.0,
    "\u897f": 270.0,
}


def finite_float(text: str, field: str, line_number: int) -> float:
    try:
        value = float(text.strip())
    except ValueError as error:
        raise ValueError(
            f"line {line_number}: {field} is not a number: {text!r}"
        ) from error
    if not math.isfinite(value):
        raise ValueError(f"line {line_number}: {field} must be finite")
    return value


def heading_alias_degrees(text: str) -> float:
    normalized = text.strip().upper()
    try:
        return HEADING_ALIASES_DEG[normalized]
    except KeyError as error:
        options = (
            "N/E/S/W, NORTH/EAST/SOUTH/WEST, or "
            "\u5317/\u4e1c/\u5357/\u897f"
        )
        raise argparse.ArgumentTypeError(
            f"unsupported heading {text!r}; use {options}"
        ) from error


def arbitrary_heading_degrees(text: str) -> float:
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            f"heading angle is not a number: {text!r}"
        ) from error
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("heading angle must be finite")
    return value % 360.0


def read_global_waypoints(path: Path) -> list[GlobalWaypoint]:
    waypoints: list[GlobalWaypoint] = []
    previous_sequence: int | None = None
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            fields = [field.strip() for field in line.split(";")]
            normalized_header = tuple(field.casefold() for field in fields)
            if not waypoints and normalized_header in TRACK_OUTPUT_HEADERS:
                continue
            if len(fields) not in (4, 5):
                raise ValueError(
                    f"line {line_number}: expected 4 semicolon-separated "
                    "track_output fields (or 5 legacy fields), "
                    f"found {len(fields)}"
                )

            try:
                sequence = int(fields[0], 10)
            except ValueError as error:
                raise ValueError(
                    f"line {line_number}: sequence is not an integer: {fields[0]!r}"
                ) from error
            if previous_sequence is not None and sequence <= previous_sequence:
                raise ValueError(
                    f"line {line_number}: sequence {sequence} must be greater than "
                    f"the previous sequence {previous_sequence}"
                )

            longitude = finite_float(fields[1], "longitude", line_number)
            latitude = finite_float(fields[2], "latitude", line_number)
            altitude = finite_float(fields[3], "altitude", line_number)
            if not -180.0 <= longitude <= 180.0:
                raise ValueError(
                    f"line {line_number}: longitude must be in [-180, 180] degrees"
                )
            if not -90.0 <= latitude <= 90.0:
                raise ValueError(
                    f"line {line_number}: latitude must be in [-90, 90] degrees"
                )

            waypoints.append(
                GlobalWaypoint(sequence, longitude, latitude, altitude)
            )
            if len(waypoints) > MAXIMUM_WAYPOINT_COUNT:
                raise ValueError(
                    "input exceeds the controller's 10000-waypoint limit"
                )
            previous_sequence = sequence

    if len(waypoints) < 2:
        raise ValueError("input must contain at least two waypoint rows")
    return waypoints


def geodetic_to_ecef(waypoint: GlobalWaypoint) -> tuple[float, float, float]:
    longitude = math.radians(waypoint.longitude_deg)
    latitude = math.radians(waypoint.latitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    prime_vertical_radius = WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    x = (
        prime_vertical_radius + waypoint.altitude_m
    ) * cos_latitude * math.cos(longitude)
    y = (
        prime_vertical_radius + waypoint.altitude_m
    ) * cos_latitude * math.sin(longitude)
    z = (
        prime_vertical_radius * (1.0 - WGS84_ECCENTRICITY_SQUARED)
        + waypoint.altitude_m
    ) * sin_latitude
    return x, y, z


def ecef_delta_to_enu(
    delta_ecef: tuple[float, float, float], origin: GlobalWaypoint
) -> tuple[float, float, float]:
    longitude = math.radians(origin.longitude_deg)
    latitude = math.radians(origin.latitude_deg)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    dx, dy, dz = delta_ecef

    east = -sin_longitude * dx + cos_longitude * dy
    north = (
        -sin_latitude * cos_longitude * dx
        - sin_latitude * sin_longitude * dy
        + cos_latitude * dz
    )
    up = (
        cos_latitude * cos_longitude * dx
        + cos_latitude * sin_longitude * dy
        + sin_latitude * dz
    )
    return east, north, up


def convert_to_initial_odom(
    waypoints: Iterable[GlobalWaypoint], heading_deg: float
) -> list[LocalWaypoint]:
    ordered = list(waypoints)
    if not ordered:
        raise ValueError("at least one waypoint is required for conversion")
    if not math.isfinite(heading_deg):
        raise ValueError("heading must be finite")

    origin = ordered[0]
    origin_ecef = geodetic_to_ecef(origin)
    heading = math.radians(heading_deg % 360.0)
    sin_heading = math.sin(heading)
    cos_heading = math.cos(heading)
    converted: list[LocalWaypoint] = []

    for waypoint in ordered:
        waypoint_ecef = geodetic_to_ecef(waypoint)
        delta = tuple(
            waypoint_value - origin_value
            for waypoint_value, origin_value in zip(waypoint_ecef, origin_ecef)
        )
        east, north, _up = ecef_delta_to_enu(delta, origin)

        # Heading is a compass bearing measured clockwise from true north.
        # The odom convention is x forward and y left.
        x_m = east * sin_heading + north * cos_heading
        y_m = -east * cos_heading + north * sin_heading
        if abs(x_m) < 5.0e-10:
            x_m = 0.0
        if abs(y_m) < 5.0e-10:
            y_m = 0.0
        converted.append(LocalWaypoint(waypoint.sequence, x_m, y_m))
    return converted


def segment_lengths(waypoints: Sequence[LocalWaypoint]) -> tuple[float, ...]:
    return tuple(
        math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
        for first, second in zip(waypoints, waypoints[1:])
    )


def render_local_waypoint_csv(waypoints: Sequence[LocalWaypoint]) -> str:
    if len(waypoints) < 2:
        raise ValueError("output must contain at least two waypoints")
    if len(waypoints) > MAXIMUM_WAYPOINT_COUNT:
        raise ValueError("output exceeds the controller's 10000-waypoint limit")

    formatted: list[tuple[str, str]] = []
    controller_values: list[tuple[float, float]] = []
    for waypoint in waypoints:
        if not math.isfinite(waypoint.x_m) or not math.isfinite(waypoint.y_m):
            raise ValueError("output coordinates must be finite")
        text_values = (f"{waypoint.x_m:.6f}", f"{waypoint.y_m:.6f}")
        formatted.append(text_values)
        # Compare the same six-decimal values that the C++ loader will parse.
        # In particular, 0.000000 and -0.000000 are the same controller point.
        controller_values.append((float(text_values[0]), float(text_values[1])))
    if not any(
        second != first
        for first, second in zip(controller_values, controller_values[1:])
    ):
        raise ValueError(
            "output must contain at least one non-zero-length planar segment"
        )

    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(("x_m", "y_m"))
    writer.writerows(formatted)
    rendered = buffer.getvalue()
    if len(rendered.encode("utf-8")) > MAXIMUM_WAYPOINT_FILE_BYTES:
        raise ValueError("output exceeds the controller's 2 MiB file-size limit")
    if any(
        len(line.encode("utf-8")) > MAXIMUM_WAYPOINT_LINE_BYTES
        for line in rendered.splitlines()
    ):
        raise ValueError("output exceeds the controller's 4096-byte line limit")
    return rendered


def write_local_waypoints(
    path: Path, waypoints: Sequence[LocalWaypoint], force: bool
) -> None:
    if path.exists() and not force:
        raise FileExistsError(
            f"output already exists: {path}; pass --force to replace it"
        )
    if not path.parent.is_dir():
        raise FileNotFoundError(f"output directory does not exist: {path.parent}")
    rendered = render_local_waypoint_csv(waypoints)

    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            prefix=f".{path.name}.",
            suffix=".tmp",
            dir=path.parent,
            delete=False,
        ) as stream:
            temporary_name = stream.name
            stream.write(rendered)
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass


def convert_file(
    input_path: Path, output_path: Path, heading_deg: float, force: bool = False
) -> ConversionReport:
    if input_path.resolve() == output_path.resolve():
        raise ValueError("input and output paths must be different")
    source_bytes = input_path.read_bytes()
    waypoints = read_global_waypoints(input_path)
    converted = convert_to_initial_odom(waypoints, heading_deg)
    write_local_waypoints(output_path, converted, force)
    return ConversionReport(
        source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        origin=waypoints[0],
        heading_deg=heading_deg % 360.0,
        waypoints=tuple(converted),
        segment_lengths_m=segment_lengths(converted),
    )


def print_report(report: ConversionReport, output_path: Path) -> None:
    print(f"source_sha256={report.source_sha256}")
    print(
        "odom_origin_wgs84_lon_lat="
        f"({report.origin.longitude_deg:.10f}, "
        f"{report.origin.latitude_deg:.10f})"
    )
    print(
        f"initial_heading_deg_clockwise_from_true_north={report.heading_deg:.6f}"
    )
    for index, waypoint in enumerate(report.waypoints):
        suffix = ""
        if index > 0:
            suffix = f", segment={report.segment_lengths_m[index - 1]:.3f} m"
        print(
            f"waypoint {waypoint.sequence}: "
            f"x={waypoint.x_m:.6f} m, y={waypoint.y_m:.6f} m{suffix}"
        )
    print(f"total_length={report.total_length_m:.3f} m")
    print(f"output={output_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert track_output.txt semicolon-separated WGS84 waypoints "
            "into x-forward, "
            "y-left coordinates. The first waypoint is both the odom origin "
            "and the retained first output point."
        )
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT_PATH,
        help="upstream input file (default: ./track_output.txt)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help="controller CSV output (default: ./watpoints_odom.csv)",
    )
    heading_group = parser.add_mutually_exclusive_group(required=True)
    heading_group.add_argument(
        "--heading",
        type=heading_alias_degrees,
        metavar="N|E|S|W",
        help="initial vehicle direction; Chinese and full English names are accepted",
    )
    heading_group.add_argument(
        "--heading-deg",
        type=arbitrary_heading_degrees,
        metavar="DEGREES",
        help="initial true heading in degrees clockwise from north",
    )
    parser.add_argument(
        "--force", action="store_true", help="replace an existing output file"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    heading_deg = args.heading if args.heading is not None else args.heading_deg
    try:
        report = convert_file(args.input, args.output, heading_deg, args.force)
    except (OSError, ValueError) as error:
        parser.exit(2, f"conversion failed: {error}\n")
    print_report(report, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
