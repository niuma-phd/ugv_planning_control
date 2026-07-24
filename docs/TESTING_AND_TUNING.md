# Testing and tuning guide

This sequence is intentionally conservative. Each stage must pass before the
next one starts.

## 1. Build and unit tests

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
python3 scripts/verify_repository.py
```

## 2. Deterministic fixture smoke

Fixtures require an explicit `production_mode:=false`. They do not model real
vehicle physics and may only prove topic wiring, command signs and fault
handling.

The preferred RDK entry point starts an isolated process group, verifies the
result, shuts down only that group, and rejects survivors:

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
```

Use a currently unused domain for every invocation. `subject1_none` is a
non-empty finite cloud whose returns are outside the ROI; it is the valid
clear-road case. The last three S1 modes prove fail-closed behavior for cloud
cutoff, repeated timestamps, and all-NaN input.

### Subject 2

Launch the isolated Subject 2 fixture graph:

```bash
# Terminal 1
ROS_DOMAIN_ID=71 ros2 launch ugv_mvp_bringup subject2_fixture.launch.py

# Terminal 2, from the repository root
ROS_DOMAIN_ID=71 python3 scripts/verify_fixture_runtime.py subject2
```

The fixture never publishes canonical `/control/cmd_vel`. For focused
controller tests, repeat `path_shape:=right` and verify `angular.z` changes
sign. Set `raw_odom_stop_after_s:=2.0` or
`raw_odom_inject_jump_after_s:=2.0`; command must become zero and the persisted
last-good file must contain the sample immediately before the fault.

The integrated stale-odom check is:

```bash
# Terminal 1
ROS_DOMAIN_ID=73 ros2 launch ugv_mvp_bringup subject2_fixture.launch.py \
  raw_odom_stop_after_s:=3.0

# Terminal 2, start before the three-second stop
ROS_DOMAIN_ID=73 python3 scripts/verify_fixture_runtime.py subject2_fault
```

### Subject 1

```bash
# Terminal 1
ROS_DOMAIN_ID=72 ros2 launch ugv_mvp_bringup subject1_fixture.launch.py

# Terminal 2, from the repository root
ROS_DOMAIN_ID=72 python3 scripts/verify_fixture_runtime.py subject1
```

The cloud fixture uses `livox_frame`, so this path also exercises the static TF
lookup. Its default obstacle is on the vehicle's right, so the expected
avoidance command has positive `angular.z` (left turn). For focused
detector/planner tests, repeat `none`, `front`, `left` and `blocked`; with the
conservative candidate footprint, a centered front obstacle may correctly
produce a zero command until measured vehicle dimensions replace it.
Malformed, all-non-finite, replayed, TF-failed, or missing clouds do not
publish a fresh empty scene; downstream input timeout requests control with a
zero command.

## 3. Real sensors, actuators disconnected

### Common checks

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/imu
ros2 topic info -v /livox_odometry_mapped
ros2 topic hz /livox/lidar
ros2 topic hz /livox_odometry_mapped
ros2 run tf2_tools view_frames
```

Verify:

- PointCloud2 fields and `frame_id`;
- LIO `msg_type=1`;
- canonical `map→odom→base_link→lidar`;
- no duplicate canonical TF authority;
- raw LIO TF is isolated;
- command forward/left/right signs;
- every timeout/fault produces zero within one control cycle.

Record at least 60 seconds of each lidar PointCloud2 and IMU plus TF and
commands. Save bags outside Git and commit a SHA-256 manifest.

## 4. Subject 2 parameter order

1. **Extrinsics and alignment first.** Do not tune control around a wrong TF.
2. Set `nominal_speed` to the approved first-test speed.
3. Start with a fixed lookahead:
   - left/right oscillation: increase lookahead;
   - slow convergence or excessive corner cutting: decrease lookahead.
4. Set curvature/yaw-rate limits from measured stable vehicle behavior.
5. Tune slowdown distance and goal tolerance on a straight path.
6. Tune odom timeout just above measured worst healthy period/latency.
7. Set jump limits above healthy p99 motion increments but below observed
   failure jumps.

Do not enable automatic GPS recovery in this phase.

## 5. Subject 1 parameter order

1. Measure lidar static TF and vehicle footprint.
2. Place known boxes front/left/right; validate transformed obstacle position.
3. Tune self crop.
4. Tune z band on the real ground profile.
5. Tune grid resolution and minimum points.
6. Set footprint inflation from localization, TF and tracking errors.
7. Tune rollout horizon and the inflated footprint that define whether an
   obstacle intersects the nominal forward sweep.
8. Tune curvature sample range, then goal/heading/clearance weights.

If no candidate is collision-free, stopping is the correct output.

## 6. Field gates

- physical emergency stop and remote stop available;
- observer and exclusion zone present;
- parameter and source revision recorded;
- straight line before curves;
- Subject 2 before Subject 1;
- soft obstacles only;
- no automatic unattended test.
