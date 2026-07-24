# Session 08 — bringup profiles

## Package

`src/ugv_mvp_bringup`

## Production launches

- `subject2.launch.py`
- `subject2_horizon.launch.py`
- `subject1.launch.py`
- `subject1_horizon.launch.py`

All default `publish_lidar_static_tf=false`. The LIO adapter therefore refuses
canonical odom by default. Enabling the transform requires all six
`base_to_lidar_*` launch values plus a non-empty
`lidar_extrinsics_provenance` measurement-record identifier. Empty values,
non-finite values, or a missing provenance string abort launch. The launch
arguments are the single runtime source of truth; YAML cannot silently mark
candidate values as approved.

Both launch files publish `base_link→livox_frame`, matching the pinned driver
default for PointCloud2 and IMU. Do not rename that child to a sensor model
unless the driver header configuration changes at the same reviewed revision.

Subject 2 is the field P0 profile and has these confirmed MVP decisions:

- two-track differential tracked chassis;
- Horizon + LIO localization;
- identity `map→odom`;
- controller output `/cmd_vel` as `geometry_msgs/msg/Twist`, using only
  `linear.x` in m/s and `angular.z` in rad/s.

`subject2_horizon.launch.py` starts the installed Horizon PointCloud2 driver and
LIO by default, scopes `msg_type=1` to LIO, and remaps raw LIO TF to
`/lio_raw/tf`. Its `start_driver` and `start_lio` switches support separately
supervised external layers. A production run must pass an absolute validated
Horizon broadcast-code whitelist; the packaged config is automatic-discovery
first-use material only. No launch starts an upstream path gateway: the other
team's actual deployment/data format is unknown, so adding a guessed adapter is
prohibited.

Subject 1 has a separate minimal closed profile:

- `subject1_horizon.launch.py` starts Horizon PointCloud2, scopes LIO
  `msg_type=1`, isolates raw TF on `/lio_raw/tf`, and includes the local stack;
- Horizon perception remains active and transforms clouds to `base_link`;
- the partner supplies `/subject1/nominal_cmd_vel` and
  `/subject1/next_waypoint_base`;
- `local_avoidance_node` atomically selects nominal, safe avoidance, or zero in
  one timer callback and is the sole `/cmd_vel` publisher in this profile;
- active/candidate/trajectory topics remain diagnostics;
- `map_odom_manager` starts with `initial_transform_valid=false` and does not
  publish `map→odom` before an explicit reviewed update. The local body-frame
  loop does not depend on that TF.

## Fixture launches

- `subject2_fixture.launch.py`
- `subject1_fixture.launch.py`

They explicitly activate development fixtures and remap the complete dataflow,
including `/tf` and command topics, below `/fixture`. They therefore never
publish canonical command or Subject 1 diagnostic topics.
Keep using a separate `ROS_DOMAIN_ID`; isolation is defense in depth, not a
reason to connect actuators during a fixture test.

`scripts/run_fixture_smoke.sh` owns exact process-group startup/shutdown and
calls `verify_fixture_runtime.py`. Modes cover S2 nominal/stale/jump and S1
avoid/clear/blocked/cutoff/replay/all-NaN/release/nominal-stale/nominal-invalid
behavior. They prove expected command selection or zero, one fixture final
publisher with `geometry_msgs/msg/Twist`, valid/detected state, static TF, full
canonical-topic isolation, fresh last-good snapshot for S2 faults, and no
survivor processes.

## Low-cost session tasks and acceptance

Take one item per session and keep changes inside
`src/ugv_mvp_bringup/**` plus this workstream.

1. **P0 — run the complete Subject 2 software gate on RDK.**

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select \
     ugv_localization_mvp ugv_subject2_mvp ugv_mvp_tools ugv_mvp_bringup \
     --cmake-args -DCMAKE_BUILD_TYPE=Release
   colcon test --packages-select \
     ugv_localization_mvp ugv_subject2_mvp ugv_mvp_tools ugv_mvp_bringup
   colcon test-result --verbose
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_fault
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_jump
   ```

   Acceptance: all commands pass; fixture graph contains no canonical `/cmd_vel`
   publisher; process-group cleanup leaves no survivors.

2. **P0 — create the reviewed Horizon production invocation.**
   Pass six measured `base_link→livox_frame` values and a measurement-record ID
   via launch arguments; never place unapproved defaults in YAML.
   With actuators disconnected, acceptance is:

   ```bash
   ros2 launch ugv_mvp_bringup subject2_horizon.launch.py \
     driver_config:=/absolute/path/to/validated_horizon_whitelist.json \
     driver_allow_auto_discovery:=false \
     publish_lidar_static_tf:=true \
     lidar_extrinsics_provenance:=<record-id> \
     base_to_lidar_x:=<x> base_to_lidar_y:=<y> base_to_lidar_z:=<z> \
     base_to_lidar_roll:=<r> base_to_lidar_pitch:=<p> base_to_lidar_yaw:=<yaw>
   ros2 run tf2_ros tf2_echo map odom
   ros2 run tf2_ros tf2_echo odom base_link
   ros2 topic info /cmd_vel --verbose
   ```

   `map→odom` must be identity and `/cmd_vel` must have exactly one planning
   publisher before any downstream adapter is connected.

3. **P0 — verify the installed driver/LIO wrapper after every external update.**
   Confirm the included launch arguments still exist, `/livox/lidar` is
   `sensor_msgs/msg/PointCloud2`, `/scan_registration` reports `msg_type=1`,
   and raw `world→livox_frame` appears only on `/lio_raw/tf`. Never modify
   either external repository here.

4. **P1 — add launch contract tests.**
   Assert identity `map→odom`, one canonical `/cmd_vel` publisher, extrinsic
   provenance gating, and zero command after localization timeout.

5. **Blocked — upstream gateway.**
   Do not add it until the upstream team's executable output and real sample are
   available. Then implement the thinnest separately owned adapter into
   `/subject2/path`.

6. **P1 — run the complete Subject 1 software gate on RDK.**

   ```bash
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_none
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_blocked
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_fault
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_replay
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_invalid
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_release
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_nominal_stale
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject1_nominal_invalid
   ```

   Acceptance: every mode passes; fixture graph has no canonical topic leak;
   `/fixture/cmd_vel` has one publisher; Subject 1 publishes no `map→odom`
   before an explicit update; process cleanup leaves no survivors.
