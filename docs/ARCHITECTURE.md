# MVP architecture

## Current decisions

This ROS 2 Humble MVP targets a two-sided differential-drive tracked vehicle. The
controller publishes body velocity, not track speed: the downstream base converts
`linear.x [m/s]` and `angular.z [rad/s]` to left/right track actuation.

Subject 2 remains the field-validation priority. Its current sensor chain is
Horizon PointCloud2 plus the existing LIO with
`ScanRegistration.msg_type=1`. Subject 1 now also has a minimal closed software
loop: it selects the partner's nominal body command or a local body-frame
avoidance command and is the only `/cmd_vel` publisher in the Subject 1 launch
graph.

The upstream system is still unknown. This repository defines a canonical ROS
Path input but does not assume a network protocol, process layout, route source,
or production update rate.

## Packages

| Package | Responsibility |
|---|---|
| `ugv_localization_mvp` | Adapt raw LIO lidar pose to `odom→base_link`, guard odometry, publish temporary `map→odom`, and persist the last trusted sample |
| `ugv_subject2_mvp` | Follow `/subject2/path` with Pure Pursuit and publish final `/cmd_vel` |
| `ugv_subject1_perception_mvp` | Transform Horizon PointCloud2 to `base_link`, filter it, and publish occupied obstacle cells |
| `ugv_subject1_avoidance_mvp` | Sample low-speed constant-curvature avoidance candidates and atomically select the final Subject 1 command |
| `ugv_mvp_bringup` | Subject launches, parameters, external Horizon/LIO wrapper, and fixtures |

No custom message package is required by this MVP.

## Subject 2 critical path

```text
Horizon /livox/lidar PointCloud2 + /livox/imu
  → existing LIO, ScanRegistration.msg_type=1
  → /livox_odometry_mapped (world→livox_frame pose)
  → lio_odom_adapter
  → /localization/odom + odom→base_link
  → odom_guard
  → /localization/trusted_odom + /localization/odom_valid

map→odom = identity (temporary closed-course assumption)
/subject2/path (canonical input; upstream adapter unknown)
  → subject2 waypoint controller
  → /cmd_vel geometry_msgs/msg/Twist
```

The temporary Subject 2 alignment is exactly the identity transform: zero
translation and unit quaternion from `map` to `odom`. `map` and `odom` remain
separate frame names. The assumption is valid only when the supplied test path
is already expressed in the same origin and axes as canonical odometry. It is
not a global-position solution and must be replaced once upstream/global-pose
semantics are confirmed.

The odom guard:

1. Rejects wrong-frame, stale, non-finite, malformed, repeated/backward, or
   implausibly jumping odometry.
2. Stops forwarding trusted odom, causing the controller to publish zero.
3. Publishes invalid status and atomically persists the last trusted sample.
4. Keeps the fault latched until explicit reset after external recovery.

GPS-assisted LIO restart remains deferred. The repository does not know the
GPS/global-pose contract or the approved LIO restart boundary.

## Subject 1 interface

```text
Horizon /livox/lidar PointCloud2 + base_link→livox_frame
  → body-frame ROI/height/self filter
  → /subject1/obstacles

/subject1/obstacles + /subject1/next_waypoint_base
  → sampled-curvature local avoidance
  → local plan result

other-team cruise → /subject1/nominal_cmd_vel

local plan result + nominal command
  → one timer callback atomically selects one source
  → /cmd_vel geometry_msgs/msg/Twist

diagnostics:
  /subject1/avoidance_active
  /subject1/avoid_cmd_vel
  /subject1/selected_trajectory
```

The detector runs continuously. With fresh valid perception and waypoint data,
no relevant obstacle means the selector passes through a fresh finite planar
nominal command. A relevant obstacle with a safe rollout selects the avoidance
command. A blocked rollout, stale/invalid perception or waypoint, or a
stale/non-finite/non-planar nominal command when nominal is selected produces
zero `/cmd_vel`. The decision and final publish happen in the same timer
callback; there is no separate takeover edge on which two publishers can race.

Subject 1 starts `map_odom_manager` with `initial_transform_valid=false`. It
does not auto-align or publish `map→odom` until a reviewed
`/localization/map_odom_update` arrives. The partner's global pose, map origin,
axes, heading, and timing are not confirmed. The required waypoint is already
expressed in `base_link`, so perception and local avoidance do not depend on
`map→odom`.

## TF authority

```text
map → odom → base_link → livox_frame
```

- `map→odom`: `map_odom_manager`; identity only in the temporary Subject 2
  profile. Subject 1 keeps the same manager uninitialized until an explicit
  reviewed update.
- `odom→base_link`: `lio_odom_adapter` from LIO lidar pose and the approved
  `base_link→livox_frame` extrinsic.
- `base_link→livox_frame`: measured static transform supplied to bringup;
  disabled by default until approved.
- Raw LIO `world→livox_frame`: both Horizon wrappers remap its `/tf` output to
  `/lio_raw/tf`, so it never gives `livox_frame` a second parent in the canonical
  tree. An externally launched LIO must provide equivalent isolation.

## Safety boundary

The repository currently publishes `/cmd_vel`; it does not implement the
physical emergency stop, remote stop, downstream hardware watchdog, or
left/right track actuator conversion. Non-zero vehicle testing remains blocked
until every gate in [KNOWN_GAPS.md](KNOWN_GAPS.md) is closed.
