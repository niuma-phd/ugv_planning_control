# Session 06 — RDK test and tuning

## Mission

Build the exact Git revision on RDK S100, validate topics/TF with actuators
disconnected, then provide a conservative tuning sequence. Do not edit
algorithm code until a repeatable failing test has been captured.

## RDK workspace

Use a dedicated path such as:

```text
/home/sunrise/ugv_planning_control_ws
```

Do not place build/install/log inside the existing Livox driver or LIO
workspace. Do not modify either dependency repository.

## Static checks

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test
colcon test-result --verbose
```

Record CPU architecture, OS, ROS distribution, source commit and full parameter
files.

## Subject 2 tuning order

1. Verify TF axes and command signs with actuators disconnected.
2. Keep nominal speed at the approved first-test speed.
3. Tune `lookahead_distance_m`:
   - oscillation: increase;
   - cuts corners/slow convergence: decrease.
4. Tune yaw-rate and curvature limits from measured vehicle capability.
5. Tune terminal slowdown and goal tolerance.
6. Capture odom timeout, repeated stamp, jump and disconnect tests.
7. Only after all zero-command cases pass, perform a closed-course straight
   line, then left/right arcs.

## Subject 1 tuning order

1. Static vehicle, known boxes at front/left/right; tune lidar TF first.
2. Tune self crop and z height band.
3. Tune grid resolution/min points until static false positives are bounded.
4. Set footprint and inflation from measured geometry.
5. With actuator disconnected, tune trigger distance and curvature costs.
6. Closed course at the approved avoidance speed with soft obstacles.

## Required artifacts

- source commit;
- parameter snapshot;
- rosbag URI and SHA-256;
- command/odom/obstacle CSV;
- operator, location, maximum speed and result;
- every failed run, not only successful runs.

