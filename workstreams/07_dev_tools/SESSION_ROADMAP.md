# Session 07 — development fixtures

## Purpose

Provide deterministic standard-message inputs for actuator-disconnected tests.
These nodes are not a simulator and are never part of production launch.

Subject 2 is the priority. Its fixture represents the confirmed internal
Horizon/LIO dataflow, identity `map→odom`, and a two-track differential vehicle
receiving `geometry_msgs/msg/Twist` on fixture-isolated `/fixture/cmd_vel`.
It does **not** represent a confirmed upstream interface.

## Package

`src/ugv_mvp_tools`

Nodes refuse to start unless `production_mode:=false` is explicitly set:

- `path_fixture_node`: line/left/right `nav_msgs/Path`;
- `raw_odom_fixture_node`: scripted raw LIO-style odometry, optional stop/jump;
- `pointcloud_fixture_node`: deterministic lidar-frame obstacle/clear/invalid
  clouds, optional cutoff, and optional repeated-stamp injection.
- `next_waypoint_fixture_node`: deterministic body-frame next waypoint.

## Quick examples

```bash
ros2 run ugv_mvp_tools path_fixture_node --ros-args \
  -p production_mode:=false -p shape:=left

ros2 run ugv_mvp_tools raw_odom_fixture_node --ros-args \
  -p production_mode:=false -p linear_speed_mps:=0.1

ros2 run ugv_mvp_tools pointcloud_fixture_node --ros-args \
  -p production_mode:=false -p scenario:=front

ros2 run ugv_mvp_tools next_waypoint_fixture_node --ros-args \
  -p production_mode:=false -p x_m:=4.0 -p y_m:=0.0
```

## Low-cost session tasks and acceptance

Take one task per session. Only modify `src/ugv_mvp_tools/**` and this workstream.
Every fixture must remain deterministic, standard-message-only, and refuse
`production_mode=true`.

1. **P0 — keep Subject 2 nominal/stale/jump fixtures reproducible.**

   ```bash
   colcon build --packages-select ugv_mvp_tools ugv_mvp_bringup
   colcon test --packages-select ugv_mvp_tools
   colcon test-result --verbose
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_fault
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_jump
   ```

   Acceptance: all pass twice on two unused domains; no fixture publishes
   canonical `/cmd_vel`, `/tf`, `/localization/odom`, or `/subject2/path`.

2. **P1 — add deterministic command CSV capture.**
   Capture fixture-isolated `/fixture/cmd_vel` with receive time, `linear.x`,
   `angular.z`, and mode metadata. Do not add an actuator bridge.
   Acceptance: a unit test checks header/order and identical fixture runs produce
   identical command sequences within explicit floating-point tolerance.

3. **P1 — add launch tests after production launch contracts stabilize.**
   Assert fixtures require `production_mode=false`, canonical topics remain
   absent, and process shutdown leaves no survivors.

4. **Blocked — upstream path fixture changes.**
   Do not model a guessed upstream protocol. `path_fixture_node` remains only an
   internal `nav_msgs/Path` source until a real upstream sample is supplied.
