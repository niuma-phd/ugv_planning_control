import math

import pytest

from ugv_mvp_tools.common import integrate_pose, path_points, scenario_points


def test_path_directions() -> None:
    assert path_points("line", 2.0, 0.5, 4.0)[-1][:2] == (2.0, 0.0)
    assert path_points("left", 2.0, 0.5, 4.0)[-1][1] > 0.0
    assert path_points("right", 2.0, 0.5, 4.0)[-1][1] < 0.0


def test_integrate_straight_and_turn() -> None:
    x_m, y_m, yaw_rad = integrate_pose(0.0, 0.0, 0.0, 1.0, 0.0, 1.0)
    assert x_m == pytest.approx(1.0)
    assert y_m == pytest.approx(0.0)
    assert yaw_rad == pytest.approx(0.0)
    x_m, y_m, yaw_rad = integrate_pose(0.0, 0.0, 0.0, 1.0, 0.5, 1.0)
    assert x_m > 0.0
    assert y_m > 0.0
    assert yaw_rad == pytest.approx(0.5)


def test_scenarios_are_deterministic() -> None:
    assert scenario_points("none") == []
    assert len(scenario_points("front")) == 25
    assert len(scenario_points("blocked")) == 125
    with pytest.raises(ValueError):
        scenario_points("unknown")
    assert math.isfinite(scenario_points("left")[0][0])

