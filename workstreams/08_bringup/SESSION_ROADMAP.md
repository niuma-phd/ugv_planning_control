# Session 08 — bringup profiles

## Package

`src/ugv_mvp_bringup`

## Production launches

- `subject2.launch.py`
- `subject1.launch.py`

Both default `publish_lidar_static_tf=false`. The LIO adapter therefore refuses
canonical odom by default. Pass the measured lidar transform and explicitly
enable the static TF only after its evidence is approved.

Both launch files publish `base_link→livox_frame`, matching the pinned driver
default for PointCloud2 and IMU. Do not rename that child to a sensor model
unless the driver header configuration changes at the same reviewed revision.

The launch files do not start the Livox driver or LIO. Their final Avia/Horizon
launch parameters, including LIO `msg_type=1` and raw TF isolation, remain
external until confirmed.

## Fixture launches

- `subject2_fixture.launch.py`
- `subject1_fixture.launch.py`

They explicitly activate development fixtures and must never be used with
connected actuators.

## Next low-cost session

1. Run fixture launches on a new ROS domain and assert command signs/topics.
2. Add the approved Avia/Horizon measured profiles as separate YAML files; do
   not overwrite candidate defaults.
3. Add wrapper includes for the actual driver/LIO only after their exact launch
   files and Avia parameters are frozen.
4. Add launch tests that assert one canonical publisher per TF edge and command
   topic.
