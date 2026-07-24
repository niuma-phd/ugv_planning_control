# RDK deployment

## Dedicated workspace

Use:

```text
/home/sunrise/ugv_planning_control_ws
```

Do not build this project inside `/home/sunrise/livox_driver` or
`/home/sunrise/ws_livox`, and do not modify those repositories.

## Source deployment

Preferred after a GitHub remote exists:

```bash
git clone <private-repository-url> /home/sunrise/ugv_planning_control_ws
git checkout <reviewed-commit-or-tag>
```

For a pre-remote development snapshot, use `rsync` or a tar stream while
excluding `.git`, `build`, `install`, `log`, bags and artifacts. Record a
snapshot SHA-256 and the local Git commit.

## Build

```bash
cd /home/sunrise/ugv_planning_control_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/livox_driver/install/setup.bash
# As verified on 2026-07-24; update this one path when the LIO tuning owner
# promotes a newer reviewed install.
source /home/sunrise/ws_livox/tunable-final/install/setup.bash
rm -rf build install log   # only inside this dedicated workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
python3 scripts/verify_repository.py
```

## Runtime layering

In each terminal source only:

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/livox_driver/install/setup.bash
source /home/sunrise/ws_livox/tunable-final/install/setup.bash
source /home/sunrise/ugv_planning_control_ws/install/setup.bash
```

The Subject 1 and Subject 2 Horizon wrappers start the installed external
packages without modifying or overlaying their workspaces. They force the
PointCloud2 LIO mode and remap raw LIO TF to `/lio_raw/tf`.
`start_driver:=false` and/or `start_lio:=false` select separately supervised
external processes; an external LIO must apply equivalent `msg_type=1` and
raw-TF isolation.

Production bringup remains disabled until approved extrinsics are supplied:

```bash
ros2 launch ugv_mvp_bringup subject2_horizon.launch.py \
  driver_config:=/absolute/path/to/validated_horizon_whitelist.json \
  driver_allow_auto_discovery:=false \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<MEASUREMENT_RECORD_ID> \
  base_to_lidar_x:=<M> base_to_lidar_y:=<M> base_to_lidar_z:=<M> \
  base_to_lidar_roll:=<RAD> base_to_lidar_pitch:=<RAD> \
  base_to_lidar_yaw:=<RAD>
```

The packaged Horizon config is automatic-discovery first-use material and is not
a production whitelist. Subject 1 uses the equivalent reviewed invocation:

```bash
ros2 launch ugv_mvp_bringup subject1_horizon.launch.py \
  driver_config:=/absolute/path/to/validated_horizon_whitelist.json \
  driver_allow_auto_discovery:=false \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<MEASUREMENT_RECORD_ID> \
  base_to_lidar_x:=<M> base_to_lidar_y:=<M> base_to_lidar_z:=<M> \
  base_to_lidar_roll:=<RAD> base_to_lidar_pitch:=<RAD> \
  base_to_lidar_yaw:=<RAD>
```

Merely setting the boolean without all six finite values and provenance aborts
either local stack launch. Subject 1 starts `map_odom_manager` with
`initial_transform_valid=false`; it does not publish `map→odom` until an
explicit reviewed `/localization/map_odom_update` arrives. Before connecting
actuators, verify the partner inputs `/subject1/nominal_cmd_vel` and
`/subject1/next_waypoint_base`, confirm exactly one `/cmd_vel` publisher, and
inject stale/invalid/blocked cases that must produce zero.

## Production restrictions

- Development fixture nodes must remain stopped.
- Lidar extrinsics must be explicitly marked valid.
- Vehicle limits must be measured, reviewed and loaded.
- Start with actuators disconnected.
- Run exactly one subject profile so only one final `/cmd_vel` publisher exists.
- Subject 1 requires a measured footprint/height/ROI profile and an independent
  downstream `/cmd_vel` watchdog.
- The current MVP does not automatically restart LIO or consume raw GNSS.
