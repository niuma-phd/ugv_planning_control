# 01 Localization MVP — session roadmap

## Read this first

This session owns only:

- `src/ugv_localization_mvp/**`
- `workstreams/01_localization/**`

Do not edit root integration files, bringup, other packages, the Livox driver, or
`LIO_Livox_ROS2`. Subject 2 (Avia) is the first integration target. The current
implementation deliberately avoids Nav2, PCL and custom messages.

## Implemented data flow

```text
/livox_odometry_mapped (raw world -> livox_frame pose)
  -> lio_odom_adapter_node
  -> /localization/odom (odom -> base_link pose) + odom -> base_link TF
  -> odom_guard_node
  -> /localization/trusted_odom + /localization/odom_valid

/subject2/path + first /localization/odom
  -> map_odom_manager_node
  -> map -> odom TF
```

The adapter implements exactly:

```text
T_odom_base = T_world_lidar * inverse(T_base_lidar)
```

It publishes nothing unless `extrinsics_valid=true`. This flag means the six
extrinsic numbers were measured, reviewed and approved; it is not a convenience
switch for accepting zero defaults.

The map manager has two modes:

1. `auto_align_from_path=false`: repeatedly broadcasts the parameterized initial
   `map->odom` transform.
2. `auto_align_from_path=true`: does not broadcast until it has both a valid path
   and canonical odom. It uses the first path point and the tangent of the first
   non-coincident path segment as the map-frame base start pose, computes
   `T_map_odom=T_map_base_start*inverse(T_odom_base_first)`, and latches it.

A valid `/localization/map_odom_update` overrides either mode and is latched.

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
| `map_frame`, `odom_frame` | `map`, `odom` | canonical frames |
| `initial.{x,y,z,roll,pitch,yaw}` | `0` | explicit initial transform |
| `auto_align_from_path` | `false` | enable one-shot path/odom alignment |
| `path_topic` | `/subject2/path` | only declared in auto mode |
| `odom_topic` | `/localization/odom` | only declared in auto mode |
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

Use measured test extrinsics, not the zero example. Start each node in a separate
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
5. Configure Avia measured `base_link->lidar` extrinsics and raw frame names.
6. With wheels/actuators disconnected, run LIO and record at least 60 seconds of
   raw/canonical/trusted odom and `/tf`.
7. Verify transform numerically at several poses and physically verify axes:
   vehicle forward is +x, left is +y, yaw-left is positive.
8. Tune guard thresholds from healthy recordings, then inject silence/repeated
   stamps/jumps and verify fail-closed behavior.
9. Only integration ownership may wire production bringup or commands.

## TODO / blocked facts — do not guess

- **P0:** Measure and approve Avia `base_link->livox_frame` xyz/RPY and uncertainty.
- **P0:** Confirm deployed LIO actually labels raw frames `world` and
  `livox_frame`; configure parameters if names differ.
- **P0:** Disable/isolate the raw LIO TF broadcaster so TF authority is unique.
- **P0:** Resolve the known driver/LIO IMU acceleration double-scaling before a
  live localization claim.
- **P0:** Obtain real Avia PointCloud2+IMU+raw odom bag and characterize odom rate,
  jitter, jumps, restart behavior and clock domain.
- Confirm whether path altitude should participate in 3-D map alignment. Current
  auto alignment uses first path z but a planar tangent yaw.
- GPS conversion, heading semantics and LIO restart supervisor are deliberately
  deferred. Recovery must remain stopped if full global pose or restart success
  cannot be proven.
- Tune snapshot path to persistent storage and define log collection/rotation;
  guard currently atomically replaces one last-fault pair by design.
- Add launch/integration configuration only in the bringup-owner session.

## Completion boundary for the next cheap-model session

A follow-up localization session is complete only when it provides fresh RDK
Release build/test output, a measured transform configuration supplied through
bringup ownership, TF single-authority evidence, and recorded fail-closed smoke
results. Unit tests or synthetic publishers are not live-vehicle validation.
