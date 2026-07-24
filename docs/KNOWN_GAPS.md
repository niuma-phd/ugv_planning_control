# Known gaps and field gates

## Blocks non-zero Subject 2 vehicle testing

- Approved Horizon `base_link→livox_frame` 6DoF, uncertainty, reviewers, and
  measurement-record identifier.
- Confirmed Horizon PointCloud2 driver profile and live proof that LIO
  `ScanRegistration.msg_type=1` consumes it.
- Real-course validation of LIO's PointCloud2 `FeatureExtract_Mid` path; it is
  not the CustomMsg per-point timing/segmentation path and must not inherit
  tuning evidence from that path without measurement.
- Resolution of the RDK driver/LIO double-gravity scaling conflict, with static
  and motion evidence.
- Verified isolation of raw LIO `world→livox_frame` TF and one canonical
  `odom→base_link` authority.
- Approved two-sided differential tracked vehicle limits, track sign mapping,
  first-test speed, braking evidence, emergency/remote stop, and independent
  downstream `/cmd_vel` watchdog.
- A real reviewed closed-course Path whose origin and axes intentionally match
  the temporary identity `map→odom` assumption.
- Disconnected-actuator proof that path/LIO/odom/status faults produce zero
  `/cmd_vel` within the required timeout.

The current identity `map→odom` is temporary. It does not validate an unknown
upstream map, global position, or heading.

## Blocks production upstream integration

- Actual upstream process and deployment owner.
- Path topic/transport, message, frame, origin/datum, update rate, ordering, and
  restart/version behavior.
- Global-pose topic/type, position and heading source, covariance/quality rules,
  and cross-machine clock relationship.

No network protocol or adapter may be assumed until these are supplied.

## Blocks GPS/global-pose-assisted recovery

- Confirmed global-pose contract and stopped-update procedure.
- Exact LIO supervisor restart service or process boundary.
- Post-restart first-valid-odom criterion and TF behavior.
- Evidence that a non-identity `map→odom` update cannot race vehicle motion.

Until these are confirmed, odom failure stops the vehicle and saves the last
trusted sample; automatic restart is disabled.

## Blocks Subject 1 vehicle testing

- Subject 1's independently approved Horizon extrinsic, footprint, and safety
  inflation.
- Real Horizon PointCloud2 bag and body-axis verification.
- Ground/terrain thresholds on the actual course.
- External team's exact takeover/return mechanism.
- Actual next-waypoint source expressed in `base_link`.

Subject 1 interfaces remain `/subject1/avoidance_active` and
`/subject1/avoid_cmd_vel`; the Subject 2 `/cmd_vel` change does not replace them.

## Does not block coding and fixtures

- Unknown production upstream transport.
- Final track actuator protocol.
- Exact global recovery implementation.
- Dynamic obstacles, negative obstacles, or advanced terrain segmentation.
