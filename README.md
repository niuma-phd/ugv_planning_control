# UGV Planning and Control MVP

Clean ROS 2 Humble repository for the off-road vehicle planning/control MVP.
The surrounding `越野车规划控制/` directory is reference material; new Codex
implementation sessions should start in this directory to avoid loading the
large historical task set by default.

## Product direction

- **Subject 2 first:** Avia LIO odometry, a small odom validity guard, and a
  parameterized Pure Pursuit waypoint follower. At the first invalid odom
  sample the command becomes zero and the last trusted odom is persisted.
- **Subject 1 second:** Horizon point cloud to body-frame obstacle points, then
  a small sampled-curvature local avoidance controller. When no obstacle is
  active this project does not take control from the other team.
- **Upstream gateway later:** the repository defines canonical ROS inputs but
  does not assume the partner's network protocol or deployment form.
- **Downstream kept simple:** the system publishes planar target speed and yaw
  rate. Vehicle-specific actuation remains outside this repository.

## Repository layout

```text
src/          ROS 2 packages
workstreams/  one concise roadmap per future Codex session
docs/         architecture, interfaces, deployment and tuning
scripts/      repository and RDK verification helpers
dependencies/ pinned external source references (not vendored code)
```

See `docs/ROADMAP.md` for integration order and `AGENTS.md` for non-negotiable
repository rules.

## Status

MVP implementation in progress. No profile is approved for non-zero
live-vehicle control until the gaps in `docs/KNOWN_GAPS.md` are closed.

