# Testing, launch, and tuning guide

Subject 2 is validated before Subject 1. Every stage must pass before the next;
fixture and disconnected-actuator evidence never authorizes non-zero vehicle
motion.

## 1. Build and automated tests

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
python3 scripts/verify_repository.py
```

## 2. Deterministic fixtures

Use an unused ROS domain for each run:

```bash
ROS_DOMAIN_ID=171 scripts/run_fixture_smoke.sh subject2
ROS_DOMAIN_ID=172 scripts/run_fixture_smoke.sh subject2_fault
ROS_DOMAIN_ID=173 scripts/run_fixture_smoke.sh subject2_jump
ROS_DOMAIN_ID=174 scripts/run_fixture_smoke.sh subject1
ROS_DOMAIN_ID=175 scripts/run_fixture_smoke.sh subject1_none
ROS_DOMAIN_ID=176 scripts/run_fixture_smoke.sh subject1_blocked
ROS_DOMAIN_ID=177 scripts/run_fixture_smoke.sh subject1_fault
ROS_DOMAIN_ID=178 scripts/run_fixture_smoke.sh subject1_replay
ROS_DOMAIN_ID=179 scripts/run_fixture_smoke.sh subject1_invalid
ROS_DOMAIN_ID=180 scripts/run_fixture_smoke.sh subject1_release
ROS_DOMAIN_ID=181 scripts/run_fixture_smoke.sh subject1_nominal_stale
ROS_DOMAIN_ID=182 scripts/run_fixture_smoke.sh subject1_nominal_invalid
```

The Subject 2 fixture verifies identity `map→odom`, canonical odom, zero on
fault, and remapped `/fixture/cmd_vel` as `geometry_msgs/msg/Twist`. It does not
run the real Horizon driver or LIO.

The Subject 1 modes verify obstacle-triggered local selection, clear-road
nominal passthrough, fully blocked zero, perception cutoff/replay/all-NaN
fail-closed behavior, avoidance release back to nominal, and stale/invalid
nominal zero. They also verify `/fixture/cmd_vel` has exactly one publisher and
type `geometry_msgs/msg/Twist`. Fixtures use synthetic point clouds and a
fixture-only static TF; they do not approve vehicle parameters or motion.

## 3. Subject 2 launch modes

### One-command Horizon chain

`subject2_horizon.launch.py` includes the Horizon PointCloud2 driver, scopes
`ScanRegistration.msg_type=1` to the LIO launch, remaps the raw LIO broadcaster
to `/lio_raw/tf`, and starts the local Subject 2 stack. Source the installed
driver and LIO underlays before this repository. Replace every `<...>` value
with an approved measurement record:

```bash
source /opt/ros/humble/setup.bash
source <LIVOX_DRIVER_INSTALL>/setup.bash
source <LIO_INSTALL>/setup.bash
source install/setup.bash
ros2 launch ugv_mvp_bringup subject2_horizon.launch.py \
  start_driver:=true \
  start_lio:=true \
  driver_config:=/absolute/path/to/validated_horizon_whitelist.json \
  driver_allow_auto_discovery:=false \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<APPROVED_RECORD_ID> \
  base_to_lidar_x:=<METERS> \
  base_to_lidar_y:=<METERS> \
  base_to_lidar_z:=<METERS> \
  base_to_lidar_roll:=<RADIANS> \
  base_to_lidar_pitch:=<RADIANS> \
  base_to_lidar_yaw:=<RADIANS>
```

The repository cannot supply those values. With the default
`publish_lidar_static_tf:=false`, the adapter remains invalid and motion is
blocked. The driver's packaged `horizon.json` intentionally permits first-use
automatic discovery; use it only in an isolated, actuator-disconnected setup.
Production must pass an absolute config containing the reviewed 15-character
broadcast-code whitelist.

### Layered integration

Use the local-only launch when the approved Horizon driver and LIO are already
running in another supervised layer:

```bash
ros2 launch ugv_mvp_bringup subject2.launch.py \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<APPROVED_RECORD_ID> \
  base_to_lidar_x:=<METERS> \
  base_to_lidar_y:=<METERS> \
  base_to_lidar_z:=<METERS> \
  base_to_lidar_roll:=<RADIANS> \
  base_to_lidar_pitch:=<RADIANS> \
  base_to_lidar_yaw:=<RADIANS>
```

Alternatively, reuse one already-running external layer through the wrapper:

```bash
# Existing Horizon driver; start LIO and local Subject 2.
ros2 launch ugv_mvp_bringup subject2_horizon.launch.py \
  start_driver:=false start_lio:=true <APPROVED_EXTRINSIC_ARGUMENTS>

# Existing driver and LIO; start only local Subject 2.
ros2 launch ugv_mvp_bringup subject2_horizon.launch.py \
  start_driver:=false start_lio:=false <APPROVED_EXTRINSIC_ARGUMENTS>
```

`<APPROVED_EXTRINSIC_ARGUMENTS>` abbreviates the same seven explicit arguments
from the complete command; it is not a literal shell token. The exact commands
used to start externally managed driver/LIO processes remain owner-provided and
must not be invented here.

## 4. Disconnected-actuator inspection

Before connecting vehicle actuation:

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/imu
ros2 topic info -v /livox_odometry_mapped
ros2 topic info -v /cmd_vel
ros2 topic type /cmd_vel
ros2 param get /scan_registration msg_type
ros2 topic hz /livox/lidar
ros2 topic hz /livox_odometry_mapped
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_tools view_frames
```

Required observations:

- `/livox/lidar` is Horizon `sensor_msgs/msg/PointCloud2`;
- the deployed LIO process is demonstrably using `ScanRegistration.msg_type=1`;
- the resulting `/livox_full_cloud` and odometry are validated rather than
  assuming CustomMsg feature-extraction tuning transfers to PointCloud2;
- raw `world→livox_frame` appears only on `/lio_raw/tf`, not canonical `/tf`;
- `/cmd_vel` type is exactly `geometry_msgs/msg/Twist`;
- only `linear.x` and `angular.z` can be non-zero, with m/s and rad/s units;
- `map→odom` is exactly identity for this temporary Subject 2 profile;
- canonical TF has one authority per edge and raw LIO TF is isolated;
- stopping the path, trusted odom, odom-valid status, LIO, or driver produces
  zero `/cmd_vel` within the controller timeout;
- the downstream watchdog independently stops the two-track chassis when
  `/cmd_vel` stops.

Record at least 60 seconds of PointCloud2, IMU, raw/canonical/trusted odom, TF,
odom-valid status, path, target point, and `/cmd_vel`. Store bags outside Git and
record a SHA-256 manifest.

## 5. Subject 2 tuning order

1. Confirm Horizon/LIO units, approved 6DoF extrinsic, and identity-map test
   geometry. Never tune around a wrong TF.
2. Confirm the two-sided differential tracked base interprets positive
   `linear.x` as forward and positive `angular.z` as left turn.
3. Set the first-test speed from approved vehicle evidence.
4. Tune fixed lookahead on a straight path, then left and right arcs.
5. Set yaw-rate/curvature limits from measured stable behavior.
6. Tune terminal slowdown and goal tolerance.
7. Set odom timeouts and jump limits from healthy p99 data, not fixtures.

Do not enable GPS/global recovery while upstream pose, datum, heading, clock, and
LIO restart semantics remain unknown.

## 6. Subject 1 launch, inspection, and tuning

### One-command Horizon chain

`subject1_horizon.launch.py` includes the Horizon PointCloud2 driver, scopes
`ScanRegistration.msg_type=1` to LIO, remaps raw LIO TF to `/lio_raw/tf`, and
starts the local Subject 1 stack:

```bash
source /opt/ros/humble/setup.bash
source <LIVOX_DRIVER_INSTALL>/setup.bash
source <LIO_INSTALL>/setup.bash
source install/setup.bash
ros2 launch ugv_mvp_bringup subject1_horizon.launch.py \
  driver_config:=/absolute/path/to/validated_horizon_whitelist.json \
  driver_allow_auto_discovery:=false \
  publish_lidar_static_tf:=true \
  lidar_extrinsics_provenance:=<APPROVED_RECORD_ID> \
  base_to_lidar_x:=<METERS> \
  base_to_lidar_y:=<METERS> \
  base_to_lidar_z:=<METERS> \
  base_to_lidar_roll:=<RADIANS> \
  base_to_lidar_pitch:=<RADIANS> \
  base_to_lidar_yaw:=<RADIANS>
```

The local-only equivalent is `subject1.launch.py` with the same seven
extrinsic/provenance arguments. Subject 1 starts `map_odom_manager` with
`initial_transform_valid=false`: it must not publish `map→odom` before an
explicit reviewed `/localization/map_odom_update`. The local loop remains usable
because the partner supplies `/subject1/next_waypoint_base` in `base_link`.

### Disconnected-actuator inspection

With a safe external nominal-command fixture or the partner program connected:

```bash
ros2 topic type /livox/lidar
ros2 param get /scan_registration msg_type
ros2 topic info -v /subject1/nominal_cmd_vel
ros2 topic info -v /subject1/next_waypoint_base
ros2 topic info -v /cmd_vel
ros2 topic type /cmd_vel
ros2 topic echo /subject1/avoidance_active
ros2 topic echo /subject1/avoid_cmd_vel
ros2 topic echo /subject1/selected_trajectory
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Required observations:

- `/livox/lidar` is Horizon PointCloud2 and LIO uses `msg_type=1`;
- raw LIO TF is isolated and measured `base_link→livox_frame` is available;
- no `map→odom` appears before the explicit partner-reviewed update;
- `/cmd_vel` is `geometry_msgs/msg/Twist` with exactly one publisher;
- no relevant obstacle passes through fresh planar nominal `linear.x` and
  `angular.z`;
- a relevant obstacle with a safe trajectory selects the avoidance candidate;
- obstacle release returns to nominal without a second command publisher;
- blocked, stale perception/waypoint, stale nominal, and invalid selected input
  all produce zero final `/cmd_vel`.

### Tuning order

1. Measure the independent lidar extrinsic, footprint, obstacle height band,
   ROI, and self crop; validate known obstacle positions.
2. Tune grid thresholds and inflation against real Horizon PointCloud2 bags.
3. Load approved low-speed and curvature/yaw-rate limits; verify signs with
   actuators disconnected.
4. Tune rollout horizon, sampling, goal/heading score, and clearance score.
5. Measure healthy input rates and freeze perception, waypoint, and nominal
   timeouts from evidence.
6. Verify no-obstacle passthrough, avoidance, release, blocked stop, and every
   stale/invalid stop branch. If no candidate is collision-free, stopping is
   correct.

## 7. Non-zero vehicle gate

All items below are mandatory:

- physical emergency stop, remote stop, observer, and exclusion zone;
- approved Horizon 6DoF extrinsic and vehicle geometry;
- verified two-track differential-drive command signs and downstream watchdog;
- resolved IMU/LIO double-gravity issue with stationary evidence;
- live proof of PointCloud2 LIO `msg_type=1` and isolated raw TF;
- approved speed, yaw-rate, acceleration, braking, and first-test limits;
- a reviewed local test path whose origin/axes justify temporary identity
  `map→odom`; unknown upstream data is not used;
- every stale/disconnect/jump test produces zero while actuators are disconnected;
- parameter/source revisions and evidence manifest recorded;
- human authorization for straight-line low-speed testing before curves.

For Subject 1, additionally require its measured footprint/height/ROI profile,
approved speed and curvature, partner nominal/waypoint producers, one final
publisher, and the full nominal/avoidance/release/fail-closed sequence.
