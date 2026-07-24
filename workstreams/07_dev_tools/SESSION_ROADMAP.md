# Session 07 — development fixtures

## Purpose

Provide deterministic standard-message inputs for actuator-disconnected tests.
These nodes are not a simulator and are never part of production launch.

## Package

`src/ugv_mvp_tools`

Nodes refuse to start unless `production_mode:=false` is explicitly set:

- `path_fixture_node`: line/left/right `nav_msgs/Path`;
- `raw_odom_fixture_node`: scripted raw LIO-style odometry, optional stop/jump;
- `pointcloud_fixture_node`: deterministic body-frame obstacle clusters.

## Quick examples

```bash
ros2 run ugv_mvp_tools path_fixture_node --ros-args \
  -p production_mode:=false -p shape:=left

ros2 run ugv_mvp_tools raw_odom_fixture_node --ros-args \
  -p production_mode:=false -p linear_speed_mps:=0.1

ros2 run ugv_mvp_tools pointcloud_fixture_node --ros-args \
  -p production_mode:=false -p scenario:=front
```

## Next tasks for a low-cost session

1. Add a fixture manifest containing exact expected command signs.
2. Add launch tests after the production packages stabilize.
3. Add CSV capture of `/control/cmd_vel` and `/subject1/avoid_cmd_vel`.
4. Keep every new fixture deterministic and production-refusing.

