# UGV MVP repository guidance

## Goal

Deliver the smallest ROS 2 Humble system that can demonstrate both vehicle
subjects on RDK S100. Subject 2 is always the first integration priority.

## Scope and architecture rules

- Work only inside this repository unless a task explicitly asks for read-only
  evidence from `../00原文档`, `../docs`, the Livox driver, or the LIO repository.
- Do not modify or vendor `livox_ros2_driver` or `LIO_Livox_ROS2`.
- Prefer standard ROS 2 messages and small packages. Do not introduce Nav2,
  Autoware, MPPI, a custom framework, or a custom message package unless an RFC
  proves it is necessary.
- Do not implement an upstream network gateway until the upstream program and
  deployment form are confirmed. Algorithms consume canonical local ROS topics.
- Commands are planar `v` and `omega`: `linear.x` in m/s and `angular.z` in
  rad/s, x forward, y left, positive omega left.
- Unknown vehicle limits, extrinsics, GPS semantics, or restart commands must
  remain explicit TODOs. Production launch must fail closed rather than invent
  values.
- No task may claim live-vehicle success from unit tests, synthetic publishers,
  bag replay, or an actuator-disconnected smoke test.

## Priority

1. Localization adapter and TF foundation.
2. Subject 2 odom guard and waypoint follower.
3. Subject 2 recovery only after GPS and LIO restart interfaces are confirmed.
4. Subject 1 point-cloud obstacle detection.
5. Subject 1 body-frame local avoidance.
6. Full bringup, RDK and closed-course tuning.

## Ownership

Each implementation session owns exactly one package or one explicitly named
file set. Shared root files, CI, localization, production bringup and
cross-package interfaces are integration-owner only. Parallel agents must not
edit the same package.

## Required checks

From the repository root:

```bash
scripts/build_subject2.sh --clean
scripts/build_subject1.sh --clean
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test
colcon test-result --verbose
python3 scripts/verify_repository.py
```

Run the narrow package tests first, then the complete checks. RDK builds must
use a disposable or dedicated workspace and must not alter the installed Livox
driver/LIO workspaces.

## Commit protocol

Commit messages are decision records:

```text
<intent: why this change exists>

Constraint: <constraint>
Rejected: <alternative> | <reason>
Confidence: <low|medium|high>
Scope-risk: <narrow|moderate|broad>
Directive: <future warning>
Tested: <fresh evidence>
Not-tested: <known gap>
```
