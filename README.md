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
- **Subject 1 minimum loop:** Horizon PointCloud2 stays active; the stack
  transforms it into `base_link`, extracts obstacle cells, and atomically
  selects either the other team's `/subject1/nominal_cmd_vel` or a safe local
  avoidance command for final `/cmd_vel`.
- **Subject 1 fails closed:** blocked, stale, non-finite, non-planar, or
  otherwise invalid selected input produces zero `/cmd_vel`.
- **Upstream unknown:** `/subject2/path` is canonical input only. No partner
  protocol, deployment form, global pose, or update behavior is assumed.

The identity `map→odom` profile is only for a reviewed closed-course path whose
origin and axes already match odometry, and applies only to Subject 2. Subject 1
starts its manager uninitialized and publishes no `map→odom` until an explicit
reviewed update arrives; its local `base_link` loop does not require that
transform.

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
ROS_DOMAIN_ID=174 scripts/run_fixture_smoke.sh subject1
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

## Subject 1 launch

`subject1_horizon.launch.py` starts the same Horizon PointCloud2 and LIO
external layers, scopes `msg_type=1`, isolates raw LIO TF, and starts the local
perception/avoidance selector. Pass the independently approved Horizon
extrinsic exactly as shown for Subject 2:

```bash
ros2 launch ugv_mvp_bringup subject1_horizon.launch.py \
  driver_config:=/absolute/path/to/validated_horizon_whitelist.json \
  driver_allow_auto_discovery:=false \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<APPROVED_RECORD_ID> \
  base_to_lidar_x:=<M> base_to_lidar_y:=<M> base_to_lidar_z:=<M> \
  base_to_lidar_roll:=<RAD> base_to_lidar_pitch:=<RAD> base_to_lidar_yaw:=<RAD>
```

The partner supplies fresh planar `geometry_msgs/msg/Twist` commands on
`/subject1/nominal_cmd_vel` and fresh `base_link` waypoints on
`/subject1/next_waypoint_base`. The Subject 1 launch must not run concurrently
with another `/cmd_vel` publisher.

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [ROS interfaces](docs/INTERFACES.md)
- [Testing, launch, and tuning](docs/TESTING_AND_TUNING.md)
- [Roadmap](docs/ROADMAP.md)
- [Known gaps and field gates](docs/KNOWN_GAPS.md)
- [Parallel Codex workflow](docs/CODEX_PARALLEL_WORKFLOW.md)

## Safety status

The software MVP has fixture evidence but is **not approved for non-zero vehicle
control**. Close the applicable Subject 1 or Subject 2 gates in
[KNOWN_GAPS.md](docs/KNOWN_GAPS.md), keep actuators disconnected through fault
testing, and obtain human field approval before any low-speed test.
