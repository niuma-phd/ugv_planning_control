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
source /home/sunrise/ugv_planning_control_ws/install/setup.bash
```

The Livox driver and LIO are launched from their existing workspaces in
separate terminals. This repository consumes their topics; it does not overlay
or replace their packages.

Production bringup remains disabled until approved extrinsics are supplied:

```bash
ros2 launch ugv_mvp_bringup subject2.launch.py \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<MEASUREMENT_RECORD_ID> \
  base_to_lidar_x:=<M> base_to_lidar_y:=<M> base_to_lidar_z:=<M> \
  base_to_lidar_roll:=<RAD> base_to_lidar_pitch:=<RAD> \
  base_to_lidar_yaw:=<RAD>
```

Use `subject1.launch.py` with the independently reviewed Horizon measurement.
Merely setting the boolean without all six finite values and provenance aborts
the launch.

## Production restrictions

- Development fixture nodes must remain stopped.
- Lidar extrinsics must be explicitly marked valid.
- Vehicle limits must be measured, reviewed and loaded.
- Start with actuators disconnected.
- The current MVP does not automatically restart LIO or consume raw GNSS.
