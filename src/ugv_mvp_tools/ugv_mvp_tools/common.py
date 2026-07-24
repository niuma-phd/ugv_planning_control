"""Pure fixture geometry helpers."""

from __future__ import annotations

import math
from typing import Iterable


def path_points(
    shape: str, length_m: float, spacing_m: float, radius_m: float
) -> list[tuple[float, float, float]]:
    if length_m <= 0.0 or spacing_m <= 0.0 or radius_m <= 0.0:
        raise ValueError("length, spacing and radius must be positive")
    if shape not in {"line", "left", "right"}:
        raise ValueError(f"unsupported path shape: {shape}")

    count = max(2, int(math.floor(length_m / spacing_m)) + 1)
    result: list[tuple[float, float, float]] = []
    sign = 1.0 if shape == "left" else -1.0
    for index in range(count):
        distance = min(index * spacing_m, length_m)
        if shape == "line":
            result.append((distance, 0.0, 0.0))
        else:
            angle = distance / radius_m
            result.append(
                (
                    radius_m * math.sin(angle),
                    sign * radius_m * (1.0 - math.cos(angle)),
                    sign * angle,
                )
            )
    if result[-1][0] != length_m and shape == "line":
        result.append((length_m, 0.0, 0.0))
    return result


def integrate_pose(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    linear_speed_mps: float,
    yaw_rate_radps: float,
    dt_s: float,
) -> tuple[float, float, float]:
    if dt_s < 0.0:
        raise ValueError("dt must be non-negative")
    if abs(yaw_rate_radps) < 1.0e-9:
        return (
            x_m + linear_speed_mps * math.cos(yaw_rad) * dt_s,
            y_m + linear_speed_mps * math.sin(yaw_rad) * dt_s,
            yaw_rad,
        )
    new_yaw = yaw_rad + yaw_rate_radps * dt_s
    radius = linear_speed_mps / yaw_rate_radps
    return (
        x_m + radius * (math.sin(new_yaw) - math.sin(yaw_rad)),
        y_m - radius * (math.cos(new_yaw) - math.cos(yaw_rad)),
        math.atan2(math.sin(new_yaw), math.cos(new_yaw)),
    )


def scenario_points(scenario: str) -> list[tuple[float, float, float, float]]:
    centers: list[tuple[float, float]]
    if scenario == "none":
        centers = []
    elif scenario == "front":
        centers = [(2.0, 0.0)]
    elif scenario == "left":
        centers = [(2.0, 0.8)]
    elif scenario == "right":
        centers = [(2.0, -0.8)]
    elif scenario == "blocked":
        centers = [(2.0, -1.0), (2.0, -0.5), (2.0, 0.0), (2.0, 0.5), (2.0, 1.0)]
    else:
        raise ValueError(f"unsupported point-cloud scenario: {scenario}")

    result: list[tuple[float, float, float, float]] = []
    offsets: Iterable[float] = (-0.08, -0.04, 0.0, 0.04, 0.08)
    for center_x, center_y in centers:
        for dx in offsets:
            for dy in offsets:
                result.append((center_x + dx, center_y + dy, 0.35, 100.0))
    return result

