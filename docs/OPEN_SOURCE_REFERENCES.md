# Open-source algorithm references

The MVP independently implements small algorithms and does not link full Nav2,
Autoware or PythonRobotics at runtime.

## Subject 2

- Nav2 Humble Regulated Pure Pursuit, Apache-2.0:
  <https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller>
- PythonRobotics Pure Pursuit, MIT:
  <https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit>

The core independently implemented relation is:

```text
curvature = 2 * target_y_in_base / lookahead_distance²
omega = speed * curvature
```

## Subject 1 perception

- Autoware Euclidean Cluster processing order, Apache-2.0:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_euclidean_cluster/>
- PCL crop/voxel/clustering concepts, BSD-3-Clause:
  <https://pointclouds.org/>

The first implementation avoids the large PCL development dependency and uses
`sensor_msgs::PointCloud2Iterator` plus a deterministic 2-D grid. PCL may be
evaluated later if true Euclidean clustering becomes necessary.

## Subject 1 avoidance

- Nav2 DWB sampling/scoring structure, Apache-2.0:
  <https://github.com/ros-navigation/navigation2/tree/humble/nav2_dwb_controller>
- PythonRobotics Dynamic Window Approach, MIT:
  <https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/DynamicWindowApproach>

The MVP samples constant curvature rather than importing the plugin/costmap
framework.

## Deferred terrain segmentation

Patchwork++ may be evaluated only if fixed height filtering fails on the real
course: <https://github.com/url-kaist/patchwork-plusplus>.

The upstream repository has mixed licensing signals: the core repository is
BSD-2-Clause while its ROS package metadata declares GPL-3.0. Do not copy or
vendor its ROS wrapper without a file-level license review.

