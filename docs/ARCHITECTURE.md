# MVP architecture

## Design decision

This repository deliberately replaces the earlier contract-heavy design with a
small demonstrator architecture. The goal is not a complete autonomy stack. It
is a tunable ROS 2 system that can show useful behavior quickly and stop safely
when its few required inputs disappear.

The partner-facing historical material remains useful reference, but it is not
the implementation dependency graph for this MVP.

## Packages

| Package | Immediate responsibility |
|---|---|
| `ugv_localization_mvp` | Convert raw LIO lidar pose to canonical base pose; publish `odom→base_link`; publish initial or updated `map→odom`; guard odom and persist the last trusted sample |
| `ugv_subject2_mvp` | Follow a `nav_msgs/Path` with a compact Pure Pursuit controller and publish planar speed/yaw-rate commands |
| `ugv_subject1_perception_mvp` | Convert Livox `PointCloud2` to `base_link`, apply finite/shape/ROI/self/height filters, and publish occupied body-frame obstacle cells |
| `ugv_subject1_avoidance_mvp` | When obstacles are present, sample low-speed constant-curvature trajectories toward the next body-frame waypoint |
| `ugv_mvp_bringup` | Parameter files, subject-specific launches and integration smoke tests |

No custom message package is used in the first version.

## Subject 2

```text
Avia LIO raw Odometry
  → lio_odom_adapter
  → /localization/odom + odom→base_link

identity map→odom at physical/path start
  → map_odom_manager

/localization/odom
  → odom_guard
  → /localization/trusted_odom
  → subject2_waypoint_controller
  → /control/cmd_vel
```

The initial assumption is explicit: the physical start pose and the path start
pose share the same origin and heading, so `map→odom` begins as identity.
Parameters allow a measured initial x/y/yaw offset without changing the
controller.

The odom guard is intentionally small:

1. Reject stale, non-finite, malformed or implausibly jumping odometry.
2. Immediately stop forwarding trusted odom.
3. Publish invalid status and persist the last trusted odom atomically.
4. Keep the fault latched until an explicit reset after external recovery.

GPS-assisted LIO restart is not guessed in this first implementation. The
future recovery session will consume the persisted sample and a confirmed
full global pose, call a confirmed LIO supervisor interface, update
`map→odom`, then reset the guard. If GPS or heading is unavailable, the vehicle
remains stopped and retires.

## Subject 1

```text
Horizon PointCloud2 + base_link→livox_horizon
  → body-frame ROI/height/self filter + 2-D occupied cells
  → /subject1/obstacles

/subject1/obstacles + /subject1/next_waypoint_base
  → sampled-curvature local avoidance
  → /subject1/avoidance_active
  → /subject1/avoid_cmd_vel
```

The detector is always allowed to run. The avoidance controller only asserts
`avoidance_active=true` when a relevant obstacle exists. When inactive it
publishes a zero candidate; the other team's normal controller keeps control.
This repository does not guess their arbitration or transport protocol.

The first detector uses fixed body-frame ROI and height limits instead of a
large terrain-segmentation dependency. The first planner samples constant
curvatures, rolls them forward, rejects collisions against inflated obstacle
cells, and selects the lowest simple goal/heading/curvature/clearance cost.

## TF

Required tree:

```text
map → odom → base_link → livox_horizon
                       └→ livox_avia
```

- `map→odom`: `map_odom_manager`, initially identity for the MVP assumption.
- `odom→base_link`: `lio_odom_adapter`.
- lidar static transforms: measured values supplied to bringup; invalid by
  default until explicitly approved.
- raw LIO `world→livox_frame` remains a private/raw relation and is not used as
  the canonical tree.

## Deliberately deferred

- Any upstream TCP, NDJSON, shared-memory or vendor-specific gateway.
- Full GPS adapter, geodetic projection and GPS-heading interpretation.
- Exact LIO process supervisor/restart implementation.
- Vehicle actuation conversion and hardware watchdog.
- Dynamic obstacle tracking, negative obstacles, terrain classification,
  Nav2 behavior trees, costmaps and lifecycle orchestration.

