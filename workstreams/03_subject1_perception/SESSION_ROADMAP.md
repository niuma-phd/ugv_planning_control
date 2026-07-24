# Subject 1 perception MVP session roadmap

## Outcome
Turn Horizon `PointCloud2` into occupied 2-D cell centers in `base_link` and a conservative front-corridor Boolean. No PCL, Nav2 costmap, tracking, terrain model, or negative-obstacle claim.

## Implemented baseline
- Subscribe `/livox/lidar` with sensor-data QoS.
- Require `x/y/z` FLOAT32 count-1 fields; validate offsets, point/row stride, and exact data size before access.
- Transform finite points using TF at the cloud timestamp.
- Apply ROI, body self-filter, height band, and grid count threshold in `base_link`.
- Publish `/subject1/obstacles` and `/subject1/obstacle_detected`.
- Bad input/TF fails closed by withholding output; repeated/backward stamps and
  clouds with too few finite points do not refresh validity. The avoidance
  input timeout then requests active zero. Only a successfully decoded cloud
  may publish false with an empty obstacle array.

## Real Horizon bag tuning
1. Record stationary and slow bags with `/livox/lidar`, `/tf`, `/tf_static`, measured obstacles, clear-road segments, driver config, and bag SHA-256.
2. Confirm PointCloud2 mode; measure actual rate, frame, fields, stamps, and dropout. Never infer these from synthetic clouds.
3. Measure and approve full 6-DoF `base_link -> livox_frame` for the Horizon
   deployment. **TODO: no placeholder TF for vehicle motion.**
4. Measure vehicle envelope; tune `self_min/max_x/y` with margin without deleting nearby obstacles.
5. Plot ground/obstacle height histograms and tune `min_z/max_z` on slopes, vegetation, and pitch. **TODO: height band needs real evidence.**
6. Tune ROI/cell size for the planner; `min_points` against noise and smallest required obstacle; corridor against footprint/stopping distance.
7. Replay normal/delayed/dropped input; verify timeout clears stale detections and record RDK CPU/memory.

## Build, test, and run
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_subject1_perception_mvp --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select ugv_subject1_perception_mvp
colcon test-result --verbose
source install/setup.bash
ros2 run ugv_subject1_perception_mvp obstacle_detector_node --ros-args \
  -p roi_max_x:=8.0 -p roi_min_y:=-3.0 -p roi_max_y:=3.0 \
  -p min_z:=-0.4 -p max_z:=1.5 -p cell_size:=0.25 -p min_points:=3
ros2 bag play <HORIZON_BAG>
ros2 topic echo /subject1/obstacle_detected
ros2 topic echo /subject1/obstacles
```
On RDK use a disposable workspace and the same package-select commands. Bag replay with actuators disconnected proves compilation/replay only, not live-vehicle safety.

## Exit criteria
- Measured Horizon TF approved and published by bringup.
- Real bag shows transform-at-stamp without extrapolation.
- Clear-road false positives and required-obstacle misses quantified.
- Tuned parameters plus bag evidence recorded.
- RDK Release build/test and replay resource usage recorded.
