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

## Subject 1 local closed loop

| Topic | Type | Meaning |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Horizon PointCloud2 |
| `/subject1/obstacles` | `geometry_msgs/msg/PoseArray` | Body-frame occupied-cell centers |
| `/subject1/obstacle_detected` | `std_msgs/msg/Bool` | Diagnostic obstacle presence |
| `/subject1/next_waypoint_base` | `geometry_msgs/msg/PointStamped` | Other-team input: next route waypoint already expressed in `base_link` |
| `/subject1/nominal_cmd_vel` | `geometry_msgs/msg/Twist` | Other-team input: nominal cruise command |
| `/subject1/avoidance_active` | `std_msgs/msg/Bool` | Diagnostic: avoidance branch selected or fail-closed stop requested |
| `/subject1/avoid_cmd_vel` | `geometry_msgs/msg/TwistStamped` | Diagnostic local avoidance candidate; zero while inactive or blocked |
| `/subject1/selected_trajectory` | `nav_msgs/msg/Path` | Selected rollout diagnostic |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Final Subject 1 command; only `local_avoidance_node` publishes it in the Subject 1 graph |

Both Subject 1 inputs must arrive continuously. The waypoint must use
`header.frame_id=base_link` and a non-zero, strictly increasing timestamp.
Obstacles must also use `base_link` with a non-zero, strictly increasing
timestamp. The nominal command has no header, so freshness is measured from its
local receipt time; it must contain only finite `linear.x` and `angular.z`, with
`linear.y`, `linear.z`, `angular.x`, and `angular.y` exactly zero.

Final selection is fail-closed:

| Selected condition | Final `/cmd_vel` |
|---|---|
| No relevant obstacle; nominal fresh and planar | Pass through nominal `linear.x` and `angular.z` |
| Relevant obstacle; safe local trajectory exists | Select local avoidance `linear.x` and `angular.z` |
| Relevant obstacle but all trajectories blocked | Zero |
| Perception or waypoint missing, stale, wrong-frame, replayed, or invalid | Zero; never fall back to nominal |
| Nominal selected but missing, stale, non-finite, or non-planar | Zero |

The selector computes the local plan, source choice, diagnostics, and final
command in one timer callback. Run only one subject profile at a time: Subject 1
and Subject 2 each own `/cmd_vel` in their respective launch graph.

Subject 1 starts `map_odom_manager` with no valid initial transform. It does not
publish `map→odom` before an explicit reviewed update, and its local contract
ends at the already-transformed `base_link` waypoint.
