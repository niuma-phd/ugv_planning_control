# MVP ROS interfaces

These are internal canonical topics. An upstream or downstream adapter may be
added later without changing the algorithm packages.

## Common localization

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/livox_odometry_mapped` | `nav_msgs/msg/Odometry` | existing LIO | `lio_odom_adapter` |
| `/localization/odom` | `nav_msgs/msg/Odometry` | `lio_odom_adapter` | guard/debug |
| `/localization/trusted_odom` | `nav_msgs/msg/Odometry` | `odom_guard` | S2 controller |
| `/localization/odom_valid` | `std_msgs/msg/Bool` | `odom_guard` | S2 controller/integration |
| `/localization/last_trusted_odom` | `nav_msgs/msg/Odometry` | `odom_guard` | recovery/debug |
| `/localization/map_odom_update` | `geometry_msgs/msg/TransformStamped` | future recovery/manual tool | `map_odom_manager` |
| `/localization/reset_odom_fault` | `std_srvs/srv/Trigger` | integration/recovery | `odom_guard` |

Canonical frames are `map`, `odom` and `base_link`.

The LIO adapter requires an explicit approved `base_link→lidar` transform. It
computes:

```text
T_odom_base = T_raw_world_lidar × inverse(T_base_lidar)
```

## Subject 2

| Topic | Type | Meaning |
|---|---|---|
| `/subject2/path` | `nav_msgs/msg/Path` | ordered global waypoints; first MVP places the vehicle at its first point and along its first non-coincident segment |
| `/control/cmd_vel` | `geometry_msgs/msg/TwistStamped` | final MVP command; `linear.x` m/s, `angular.z` rad/s |
| `/subject2/target_point` | `geometry_msgs/msg/PointStamped` | selected Pure Pursuit lookahead point for tuning |

If path or trusted odom is stale/invalid, output is zero.

## Subject 1

| Topic | Type | Meaning |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Horizon PointCloud2 |
| `/subject1/obstacles` | `geometry_msgs/msg/PoseArray` | body-frame centers of occupied obstacle cells |
| `/subject1/obstacle_detected` | `std_msgs/msg/Bool` | relevant obstacle presence |
| `/subject1/next_waypoint_base` | `geometry_msgs/msg/PointStamped` | next global-route waypoint already expressed in `base_link` |
| `/subject1/avoidance_active` | `std_msgs/msg/Bool` | local planner requests control |
| `/subject1/avoid_cmd_vel` | `geometry_msgs/msg/TwistStamped` | local avoidance candidate; zero while inactive |
| `/subject1/selected_trajectory` | `nav_msgs/msg/Path` | selected rollout for tuning/visualization |

The other team's control handoff consumes `avoidance_active` and
`avoid_cmd_vel`. Its arbitration remains outside this repository.

The pinned Livox driver uses `livox_frame` in both Horizon and Avia
PointCloud2/IMU headers. Bringup therefore publishes the approved
`base_link→livox_frame` transform; model-specific frame names are not invented.

## Command convention

```text
linear.x  > 0  forward
angular.z > 0  left turn
linear.y       always zero
all other Twist components zero
```

The first version does not command reverse.
