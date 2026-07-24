# Session 00 — repository and integration owner

## Mission

Keep the clean repository reproducible and integrate Subject 2 before any
Subject 1 field work.

## Read first

- `AGENTS.md`
- `docs/ARCHITECTURE.md`
- `docs/INTERFACES.md`
- `docs/KNOWN_GAPS.md`
- every package's `workstreams/*/SESSION_ROADMAP.md`

## Owned files

- root governance, CI and scripts;
- `docs/**`;
- `src/ugv_mvp_bringup/**`;
- `src/ugv_mvp_tools/**`;
- this workstream directory.

Do not rewrite algorithm packages while their owning session is active.

## Execution order

1. Run `python3 scripts/verify_repository.py`.
2. Build/test localization alone.
3. Build/test Subject 2 alone.
4. Add Subject 2 bringup and fixture smoke.
5. Deploy a clean source snapshot to an isolated RDK workspace.
6. Run Release build/test and actuator-disconnected topic smoke.
7. Only then integrate Subject 1 packages.

## Completion evidence

- clean Git status;
- exact commit;
- local static verifier output;
- RDK architecture/ROS version;
- complete `colcon build/test/test-result` output;
- smoke commands and observed topic values;
- explicit list of missing real-vehicle evidence.

