# Implementation roadmap

Subject 2 remains the field-validation priority until its Horizon/LIO chain
passes a disconnected-actuator end-to-end test and the non-zero vehicle gate is
approved. Subject 1's minimal software loop is now implemented in parallel, but
its vehicle gates remain open.

## Current baseline

Implemented MVP pieces:

- Horizon PointCloud2/LIO wrapper with `ScanRegistration.msg_type=1`;
- raw LIO pose adapter, odom guard, and last-trusted persistence;
- temporary identity `map→odom` Subject 2 profile;
- Pure Pursuit controller publishing `/cmd_vel` as `geometry_msgs/msg/Twist`;
- one-command Horizon/LIO wrappers and local-only launches for both subjects;
- Subject 1 body-frame obstacle grid, constant-curvature planner, nominal/local
  atomic selector, and one final `/cmd_vel` publisher;
- isolated fixtures for S2 nominal/fault/jump and S1
  avoid/clear/blocked/fault/replay/invalid/release/nominal-fault behavior.

The target chassis is two-sided differential-drive tracked. Track-level actuator
conversion and hardware watchdog remain downstream responsibilities.

## Wave 1 — Subject 2 disconnected-actuator evidence

```text
Horizon PointCloud2 + IMU
→ LIO with msg_type=1
→ canonical/trusted odom
→ identity map→odom
→ reviewed local test path
→ /cmd_vel Twist
→ driver/LIO/path/odom fault injection
```

Required evidence:

1. Live topic types, frames, rates, units, and single TF authorities.
2. Resolved double-gravity issue and healthy stationary/motion statistics.
3. Approved Horizon 6DoF extrinsic.
4. Straight/left/right command signs and terminal stop.
5. Zero command for all stale, disconnect, timestamp, and jump faults.
6. No actuator connection during this wave.

## Wave 2 — Subject 2 low-speed field gate

- Confirm downstream `/cmd_vel` consumer, independent watchdog, emergency stop,
  remote stop, and differential track sign mapping.
- Approve vehicle limits and first-test speed.
- Use a reviewed closed-course path already aligned with the identity-map
  assumption; do not use unknown upstream data.
- Run straight line before left/right arcs, then terminal braking.
- Record evidence and stop on any unexplained TF, odom, or command behavior.

## Wave 3 — Subject 1 disconnected-actuator closure

The local software loop is implemented:

```text
Horizon PointCloud2 + approved S1 extrinsic
→ obstacle detector
→ curvature avoidance plan

other team /subject1/nominal_cmd_vel
+ /subject1/next_waypoint_base
→ atomic nominal/avoidance selection
→ /cmd_vel Twist
```

Subject 1 starts `map_odom_manager` uninitialized and does not publish
`map→odom` before an explicit reviewed update; its waypoint is already in
`base_link`. Complete the measured extrinsic, footprint/height/ROI,
speed/curvature, final watchdog, single-publisher, and all fail-closed tests
before connecting actuators. Keep `/subject1/avoidance_active`,
`/subject1/avoid_cmd_vel`, and `/subject1/selected_trajectory` as diagnostics.

## Wave 4 — confirmed global alignment and recovery

Replace temporary identity `map→odom` only after all of the following exist:

- actual upstream path/global-pose topics and transport;
- coordinate reference, datum, origin, heading, covariance, and timing;
- approved LIO restart/supervisor interface;
- post-restart valid-odom criterion;
- stopped update/recovery procedure.

Until then, odom failure means zero command, saved evidence, and manual recovery.

## Merge and verification order

1. Horizon/LIO wrapper and localization.
2. Subject 2 final command interface and fixtures.
3. Subject 2 bringup and RDK disconnected-actuator evidence.
4. Subject 2 approved low-speed gate.
5. Subject 1 perception and avoidance.
6. Confirmed upstream/global recovery adapter.

Parallel sessions may own disjoint packages, but integration and RDK checks
follow this order.
