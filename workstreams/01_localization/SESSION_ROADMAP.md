# 01 Localization MVP — session roadmap

## Read this first

This session owns only:

- `src/ugv_localization_mvp/**`
- `workstreams/01_localization/**`

Do not edit root integration files, bringup, other packages, the Livox driver, or
`LIO_Livox_ROS2`. Subject 2 (Horizon + LIO) is the first integration target. The current
implementation deliberately avoids Nav2, PCL and custom messages.

Confirmed Subject 2 assumptions for the MVP:

- the vehicle is a two-track differential tracked chassis;
- Horizon and LIO provide localization; no chassis odom fallback is planned;
- `map→odom` is the identity transform because the physical start and the global
  path start are assumed coincident;
- the controller publishes `geometry_msgs/msg/Twist` on `/cmd_vel`;
- the real upstream path delivery format is unknown. Do not implement a gateway
  or claim that another team will publish ROS 2 messages until its actual output
  is inspected.

## Implemented data flow

```text
/livox_odometry_mapped (raw world -> livox_frame pose)
  -> lio_odom_adapter_node
  -> /localization/odom (odom -> base_link pose) + odom -> base_link TF
  -> odom_guard_node
  -> /localization/trusted_odom + /localization/odom_valid

fixed identity assumption
  -> map_odom_manager_node
  -> map -> odom TF (x=y=z=roll=pitch=yaw=0)
```

The adapter implements exactly:

```text
T_odom_base = T_world_lidar * inverse(T_base_lidar)
```

It publishes nothing unless `extrinsics_valid=true`. This flag means the six
extrinsic numbers were measured, reviewed and approved; it is not a convenience
switch for accepting zero defaults.

The map manager supports two modes, but Subject 2 currently uses only mode 1
with all-zero values:

1. `auto_align_from_path=false`: repeatedly broadcasts the parameterized initial
   `map->odom` transform.
2. `auto_align_from_path=true`: optional capability, not the current Subject 2
   contract. It does not broadcast until it has both a valid path
   and canonical odom. It uses the first path point and the tangent of the first
   non-coincident path segment as the map-frame base start pose, computes
   `T_map_odom=T_map_base_start*inverse(T_odom_base_first)`, and latches it.

A valid `/localization/map_odom_update` overrides either mode and is latched.
Subject 2 sets `require_odom_invalid_for_update=true`, so a future explicit
update is accepted only after the guard has published false. Do not enable path
auto-alignment merely to compensate for an unverified upstream format.

The guard is fail-closed. Non-finite data, malformed quaternion, stale/future,
repeated/backward stamp, or a single-frame position/yaw jump immediately latches
the fault. Trusted odom stops. `/localization/odom_valid=false` is transient-local,
and the fault predecessor is atomically replaced at:

```text
<snapshot_directory>/<snapshot_basename>.json
<snapshot_directory>/<snapshot_basename>.csv
```

Reset only clears the latch. Status remains false until a new valid sample is
accepted.

## Node parameters

### `lio_odom_adapter_node`

| Parameter | Default | Meaning |
|---|---:|---|
| `input_topic` | `/livox_odometry_mapped` | raw LIO Odometry |
| `output_topic` | `/localization/odom` | canonical odom |
| `raw_world_frame` | `world` | required raw parent frame |
| `raw_lidar_frame` | `livox_frame` | required raw child frame |
| `odom_frame` | `odom` | canonical parent |
| `base_frame` | `base_link` | canonical child |
| `extrinsics_valid` | `false` | fail-closed external approval |
| `base_to_lidar.{x,y,z}` | `0` | metres, base to selected lidar |
| `base_to_lidar.{roll,pitch,yaw}` | `0` | radians, fixed-axis RPY |

The adapter deliberately outputs zero twist: the current LIO raw odometry does
not provide a trustworthy base-frame twist. Do not silently reuse a lidar-frame
twist without implementing and testing the rigid-body twist transform.

### `map_odom_manager_node`

| Parameter | Default | Meaning |
|---|---:|---|
| `map_frame`, `odom_frame`, `base_frame` | `map`, `odom`, `base_link` | canonical frames |
| `initial.{x,y,z,roll,pitch,yaw}` | `0` | explicit initial transform |
| `auto_align_from_path` | `false` | enable one-shot path/odom alignment |
| `path_topic` | `/subject2/path` | only declared in auto mode |
| `odom_topic` | `/localization/trusted_odom` | only declared in auto mode |
| `require_odom_invalid_for_update` | follows `auto_align_from_path` | require explicit invalid state before map update |
| `path_segment_epsilon_m` | `0.05` | ignore coincident leading points |
| `publish_rate_hz` | `20` | TF refresh rate |

### `odom_guard_node`

| Parameter | Default | Tuning direction |
|---|---:|---|
| `max_age_s` | `0.30` | must exceed measured healthy transport jitter |
| `future_tolerance_s` | `0.05` | keep small; fix clock sync instead of enlarging |
| `quaternion_norm_tolerance` | `0.05` | normally do not loosen |
| `max_translation_jump_m` | `1.50` | set above maximum healthy one-frame displacement |
| `max_yaw_jump_rad` | `0.80` | set above maximum healthy one-frame yaw change |
| `watchdog_rate_hz` | `20` | should be several times odom rate |
| `expected_frame_id`, `expected_child_frame_id` | `odom`, `base_link` | exact canonical frame contract |
| `snapshot_directory` | `/tmp/ugv_odom_guard` | use persistent writable RDK path in bringup |
| `snapshot_basename` | `last_good_odom` | fixed replacement target |

## Build and unit test

On a ROS 2 Humble host/RDK workspace:

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_localization_mvp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select ugv_localization_mvp --event-handlers console_direct+
colcon test-result --verbose
```

## Safe synthetic smoke test (actuators disconnected)

Use measured Horizon extrinsics, not the zero example. Start each node in a separate
shell after sourcing the workspace:

```bash
ros2 run ugv_localization_mvp lio_odom_adapter_node --ros-args \
  -p extrinsics_valid:=true \
  -p base_to_lidar.x:=<MEASURED_X> -p base_to_lidar.y:=<MEASURED_Y> \
  -p base_to_lidar.z:=<MEASURED_Z> -p base_to_lidar.roll:=<MEASURED_ROLL> \
  -p base_to_lidar.pitch:=<MEASURED_PITCH> -p base_to_lidar.yaw:=<MEASURED_YAW>
ros2 run ugv_localization_mvp map_odom_manager_node
ros2 run ugv_localization_mvp odom_guard_node
```

Then verify, without connecting the actuator bridge:

```bash
ros2 topic echo --once /localization/odom
ros2 topic echo --once /localization/trusted_odom
ros2 topic echo --once /localization/odom_valid
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

Fault injection: stop raw odom for longer than `max_age_s`, or replay the same
stamp. Confirm status becomes false, trusted odom stops, both snapshot files
contain the preceding good pose, and the latch remains after healthy odom
returns. Reset only after external recovery:

```bash
ros2 service call /localization/reset_odom_fault std_srvs/srv/Trigger '{}'
```

## RDK deployment checklist

1. Copy/checkout the repository into a disposable RDK workspace; never modify the
   installed Livox/LIO workspaces.
2. Record `uname -a`, ROS distro, repository commit, driver commit and LIO commit.
3. Build Release and run package tests.
4. Confirm only this adapter broadcasts `odom->base_link`; raw LIO TF must be
   disabled or isolated to avoid two TF authorities.
5. Configure Horizon measured `base_link->livox_frame` extrinsics and raw frame names.
6. With wheels/actuators disconnected, run LIO and record at least 60 seconds of
   raw/canonical/trusted odom and `/tf`.
7. Verify transform numerically at several poses and physically verify axes:
   vehicle forward is +x, left is +y, yaw-left is positive.
8. Tune guard thresholds from healthy recordings, then inject silence/repeated
   stamps/jumps and verify fail-closed behavior.
9. Only integration ownership may wire production bringup or commands.

## TODO / blocked facts — do not guess

- **P0:** Measure and approve Horizon `base_link->livox_frame` xyz/RPY and uncertainty.
- **P0:** Confirm deployed LIO actually labels raw frames `world` and
  `livox_frame`; configure parameters if names differ.
- **P0:** Disable/isolate the raw LIO TF broadcaster so TF authority is unique.
- **P0:** Resolve the known driver/LIO IMU acceleration double-scaling before a
  live localization claim.
- **P0:** Obtain real Horizon PointCloud2+IMU+raw odom bag and characterize odom rate,
  jitter, jumps, restart behavior and clock domain.
- Confirm whether path altitude should participate in 3-D map alignment. Current
  auto alignment uses first path z but a planar tangent yaw.
- GPS conversion, heading semantics and LIO restart supervisor are deliberately
  deferred. Recovery must remain stopped if full global pose or restart success
  cannot be proven.
- Tune snapshot path to persistent storage and define log collection/rotation;
  guard currently atomically replaces one last-fault pair by design.
- Add launch/integration configuration only in the bringup-owner session.

## Cheap-model tasks, in execution order

Each session takes exactly one item, stays inside this workstream/package, and
records the command output in its handoff.

1. **P0 — verify the fixed Subject 2 TF contract on RDK.**
   Confirm `map→odom` is numerically identity and that only one node publishes
   `odom→base_link`. Do not edit the driver/LIO packages.
   Acceptance:

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run tf2_ros tf2_echo map odom
   ros2 run tf2_ros tf2_echo odom base_link
   ros2 topic info /tf --verbose
   ```

2. **P0 — characterize Horizon/LIO odom health from a real recording.**
   Report rate, 99th-percentile inter-arrival interval, repeated/backward stamps,
   maximum healthy one-frame translation/yaw, and clock domain. Change guard
   thresholds only from this evidence.
   Acceptance:

   ```bash
   ros2 topic hz /livox_odometry_mapped
   ros2 topic echo --once /livox_odometry_mapped
   colcon test --packages-select ugv_localization_mvp
   colcon test-result --verbose
   ```

3. **P0 — re-run deterministic guard faults after any threshold change.**
   Acceptance:

   ```bash
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_fault
   ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_jump
   ```

4. **Blocked — GPS/LIO restart recovery.**
   Do not implement until GPS pose/heading validity, timestamps, LIO restart
   command and post-restart frame behavior are supplied.

## Completion boundary for the next cheap-model session

A follow-up localization session is complete only when its assigned item has
fresh RDK Release build/test output and the listed acceptance evidence. A
synthetic fixture is software validation, not live-vehicle localization proof.
