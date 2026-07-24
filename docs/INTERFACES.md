# MVP ROS interfaces

These topics are the current internal contract. Unknown upstream or downstream
systems require explicit adapters; their transport and deployment must not be
inferred.

## Subject 2 sensor and localization chain

| Topic | Type | Producer | Consumer / meaning |
|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Horizon driver | Existing LIO; production LIO must use `ScanRegistration.msg_type=1` |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Horizon driver | Existing LIO; unit compatibility remains a field gate |
| `/livox_odometry_mapped` | `nav_msgs/msg/Odometry` | Existing LIO | `lio_odom_adapter` raw lidar pose |
| `/localization/odom` | `nav_msgs/msg/Odometry` | `lio_odom_adapter` | Canonical `odom→base_link` pose |
| `/localization/trusted_odom` | `nav_msgs/msg/Odometry` | `odom_guard` | Subject 2 controller |
| `/localization/odom_valid` | `std_msgs/msg/Bool` | `odom_guard` | Controller and integration gate |
| `/localization/last_trusted_odom` | `nav_msgs/msg/Odometry` | `odom_guard` | Recovery/debug evidence |
| `/localization/map_odom_update` | `geometry_msgs/msg/TransformStamped` | Future confirmed recovery/manual tool | `map_odom_manager`; gated while odom is invalid |
| `/localization/reset_odom_fault` | `std_srvs/srv/Trigger` | Integration/recovery | Explicit guard reset |

Canonical odom uses `header.frame_id=odom`, `child_frame_id=base_link`, and a
strictly increasing non-zero stamp. The adapter requires an approved
`base_link→livox_frame` transform and computes:

```text
T_odom_base = T_raw_world_lidar × inverse(T_base_lidar)
```

Subject 2 currently publishes an identity `map→odom`. This is a temporary
closed-course assumption, not evidence that upstream map coordinates are known.

## Subject 2 path and final command

| Topic | Type | Meaning |
|---|---|---|
| `/subject2/path` | `nav_msgs/msg/Path` | Canonical ordered path in `map`; actual upstream producer/adapter and production timing remain unknown |
| `/subject2/target_point` | `geometry_msgs/msg/PointStamped` | Pure Pursuit lookahead diagnostic |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Final Subject 2 planar command |

`/cmd_vel` convention:

```text
linear.x  [m/s]    forward positive
angular.z [rad/s]  left turn positive
all other components zero
```

The vehicle is a two-sided differential-drive tracked chassis. This repository
does not publish wheel/track RPM, PWM, steering angle, throttle, or CAN frames.
If path, TF, trusted odom, or odom-valid status is stale/invalid, `/cmd_vel` is
zero. Because `geometry_msgs/msg/Twist` has no header or expiry field, the
downstream vehicle interface must independently enforce its approved watchdog.

## Subject 1 — unchanged

| Topic | Type | Meaning |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Horizon PointCloud2 |
| `/subject1/obstacles` | `geometry_msgs/msg/PoseArray` | Body-frame occupied-cell centers |
| `/subject1/obstacle_detected` | `std_msgs/msg/Bool` | Diagnostic obstacle presence |
| `/subject1/next_waypoint_base` | `geometry_msgs/msg/PointStamped` | Next route waypoint already expressed in `base_link` |
| `/subject1/avoidance_active` | `std_msgs/msg/Bool` | Local planner requests control |
| `/subject1/avoid_cmd_vel` | `geometry_msgs/msg/TwistStamped` | Local avoidance candidate; zero while inactive |
| `/subject1/selected_trajectory` | `nav_msgs/msg/Path` | Selected rollout diagnostic |

The external Subject 1 takeover/return protocol remains unknown. Do not replace
`/subject1/avoid_cmd_vel` with the Subject 2 final `/cmd_vel` contract.
