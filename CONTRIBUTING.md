# Contributing

## Branch and ownership

- `main` contains integrated, freshly verified revisions.
- Use short-lived branches named `feat/<workstream>-<purpose>` or
  `fix/<workstream>-<purpose>`.
- One writable Codex session owns one package/worktree.
- Shared architecture, interfaces and bringup are integration-owner files.

## Before opening a PR

```bash
python3 scripts/verify_repository.py
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test
colcon test-result --verbose
```

The PR description must identify:

- goal and owned directories;
- source/base commit;
- parameters added or changed;
- exact tests and platform;
- real data used, if any;
- known gaps and whether actuators were connected.

Do not describe fixture/bag/disconnected-actuator evidence as a successful
live-vehicle test.

