# Known gaps and gates

## Blocks non-zero Subject 2 vehicle testing

- Subject 2 Avia `base_link→livox_frame` measured 6DoF, uncertainty and
  measurement-record identifier.
- Actual vehicle speed/yaw-rate limits and first-test safe speed.
- RDK driver/LIO double gravity scaling resolution and static norm evidence.
- Confirmed Avia LIO parameter set and `ScanRegistration.msg_type=1`.
- Real `/vehicle/twist` feedback if it is required for field acceptance.
- Real path sample and confirmation that start origin/heading match the vehicle.
- LIO raw TF isolation and canonical adapter live verification.

## Blocks GPS-assisted recovery

- Exact GPS/global-pose topic, message, frame, datum and heading source.
- Position/heading validity and covariance thresholds.
- GPS/RDK/LIO clock relationship.
- Exact LIO supervisor restart service or process boundary.
- Post-restart first-valid-odom criterion.

Until these are confirmed, the implemented guard stops and saves the last
trusted odom; it does not attempt an unsafe automatic restart.

## Blocks Subject 1 vehicle testing

- Subject 1 Horizon `base_link→livox_frame` measured 6DoF, uncertainty and
  measurement-record identifier.
- Vehicle footprint and safety inflation.
- Real Horizon PointCloud2 bag and body-axis verification.
- Ground/terrain height thresholds on the actual course.
- External team's exact takeover/return mechanism.
- Actual next-waypoint source expressed in `base_link`.

## Does not block coding/unit tests

- Unknown upstream network/deployment form.
- Final downstream actuator protocol.
- Exact GPS integration.
- Dynamic obstacles, negative obstacles or advanced terrain segmentation.
