# UGV Planning and Control MVP

ROS 2 Humble MVP for a two-sided differential-drive tracked vehicle. Work in
this repository directory; the surrounding `越野车规划控制/` tree is read-only
reference material.

## Current product direction

- **Subject 2 first:** Horizon PointCloud2 plus the existing LIO with
  `ScanRegistration.msg_type=1`, odom guarding, temporary identity
  `map→odom`, and Pure Pursuit.
- **Final Subject 2 command:** `/cmd_vel`, `geometry_msgs/msg/Twist`;
  `linear.x` is m/s forward and `angular.z` is rad/s left turn.
- **Downstream boundary:** the base converts body `v/omega` to left/right track
  actuation and independently enforces its hardware watchdog.
- **Subject 1 unchanged:** Horizon obstacle detection and
  `/subject1/avoidance_active` plus `/subject1/avoid_cmd_vel` candidate output.
- **Upstream unknown:** `/subject2/path` is canonical input only. No partner
  protocol, deployment form, global pose, or update behavior is assumed.

The identity `map→odom` profile is only for a reviewed closed-course path whose
origin and axes already match odometry. It is not production global alignment.

## Repository layout

```text
src/          ROS 2 packages
workstreams/  one roadmap per Codex session
docs/         architecture, interfaces, launch, testing, and gates
scripts/      repository and RDK verification helpers
dependencies/ pinned external source references
```

## Build and fixture smoke

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
python3 scripts/verify_repository.py
ROS_DOMAIN_ID=171 scripts/run_fixture_smoke.sh subject2
```

Fixtures remap the final command to `/fixture/cmd_vel` and never authorize real
vehicle output.

## Subject 2 launch

The one-command wrapper starts the Horizon PointCloud2 driver, scopes
`msg_type=1` to LIO, and starts the local stack:

```bash
ros2 launch ugv_mvp_bringup subject2_horizon.launch.py \
  driver_config:=/absolute/path/to/validated_horizon_whitelist.json \
  driver_allow_auto_discovery:=false \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<APPROVED_RECORD_ID> \
  base_to_lidar_x:=<M> base_to_lidar_y:=<M> base_to_lidar_z:=<M> \
  base_to_lidar_roll:=<RAD> base_to_lidar_pitch:=<RAD> base_to_lidar_yaw:=<RAD>
```

Those placeholders must be replaced by approved measurements. For local-only or
partially externalized startup, use the layered commands in
[TESTING_AND_TUNING.md](docs/TESTING_AND_TUNING.md).
The packaged driver config is automatic-discovery first-use material; it is not
the production whitelist.

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [ROS interfaces](docs/INTERFACES.md)
- [Testing, launch, and tuning](docs/TESTING_AND_TUNING.md)
- [Roadmap](docs/ROADMAP.md)
- [Known gaps and field gates](docs/KNOWN_GAPS.md)
- [Parallel Codex workflow](docs/CODEX_PARALLEL_WORKFLOW.md)

## Safety status

The software MVP has fixture evidence but is **not approved for non-zero vehicle
control**. Close every Subject 2 gate in [KNOWN_GAPS.md](docs/KNOWN_GAPS.md), keep
actuators disconnected through fault testing, and obtain human field approval
before any low-speed straight-line test.
