"""Pure fixture motion helpers."""

from __future__ import annotations

import math


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
