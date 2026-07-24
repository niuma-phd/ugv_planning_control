# Session 08 — bringup profiles

## Package

`src/ugv_mvp_bringup`

## Production launches

- `subject2.launch.py`
- `subject1.launch.py`

Both default `publish_lidar_static_tf=false`. The LIO adapter therefore refuses
canonical odom by default. Enabling the transform requires all six
`base_to_lidar_*` launch values plus a non-empty
`lidar_extrinsics_provenance` measurement-record identifier. Empty values,
non-finite values, or a missing provenance string abort launch. The launch
arguments are the single runtime source of truth; YAML cannot silently mark
candidate values as approved.

Both launch files publish `base_link→livox_frame`, matching the pinned driver
default for PointCloud2 and IMU. Do not rename that child to a sensor model
unless the driver header configuration changes at the same reviewed revision.

The launch files do not start the Livox driver or LIO. Their final Avia/Horizon
launch parameters, including LIO `msg_type=1` and raw TF isolation, remain
external until confirmed.

## Fixture launches

- `subject2_fixture.launch.py`
- `subject1_fixture.launch.py`

They explicitly activate development fixtures and remap the complete dataflow,
including `/tf` and command topics, below `/fixture`. They therefore never
publish the canonical `/control/cmd_vel` or `/subject1/avoid_cmd_vel` topics.
Keep using a separate `ROS_DOMAIN_ID`; isolation is defense in depth, not a
reason to connect actuators during a fixture test.

`scripts/verify_fixture_runtime.py subject1|subject2` subscribes directly with
rclpy and proves the expected non-zero command, valid/detected state, static
TF, and absence of the canonical command topic without perturbing the RDK with
multiple discovery-heavy CLI processes.

## Next low-cost session

1. Run fixture launches on a new ROS domain and assert command signs/topics.
2. Add subject-specific reviewed wrapper launches that pass the approved
   Avia/Horizon six-axis values and measurement record. Do not overwrite
   candidate defaults or add implicit approval to YAML.
3. Add wrapper includes for the actual driver/LIO only after their exact launch
   files and Avia parameters are frozen.
4. Add launch tests that assert one canonical publisher per TF edge and command
   topic.
