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

### Subject 2

Run the Subject 2 bringup with approved test-only zero lidar extrinsics, then:

```bash
ros2 run ugv_mvp_tools path_fixture_node --ros-args \
  -p production_mode:=false -p shape:=left

ros2 run ugv_mvp_tools raw_odom_fixture_node --ros-args \
  -p production_mode:=false -p linear_speed_mps:=0.05

ros2 topic echo /control/cmd_vel
```

Repeat `shape:=right` and verify `angular.z` changes sign. Set
`stop_after_s:=2.0` or `inject_jump_after_s:=2.0`; command must become zero and
the persisted last-good file must contain the sample immediately before the
fault.

### Subject 1

```bash
ros2 run ugv_mvp_tools pointcloud_fixture_node --ros-args \
  -p production_mode:=false -p scenario:=front

ros2 topic echo /subject1/obstacles
ros2 topic echo /subject1/avoidance_active
ros2 topic echo /subject1/avoid_cmd_vel
```

Repeat `none`, `left`, `right` and `blocked`.

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
7. Tune trigger distance at the approved avoidance speed.
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
