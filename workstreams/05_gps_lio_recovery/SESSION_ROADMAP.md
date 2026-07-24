# Session 05 — GPS/LIO recovery (deferred)

## Start condition

Do not implement this session until all of the following are real, recorded and
stable:

1. GPS/global pose ROS type, topic, frame and timestamp.
2. Projection/datum and a full vehicle heading source.
3. Position and heading covariance/validity rules.
4. Exact LIO restart service or supervisor interface.
5. Evidence describing the first valid odom after restart.

`sensor_msgs/NavSatFix` position alone is insufficient to recover a 2-D rigid
transform because it does not define vehicle yaw.

## Minimal intended behavior

1. Odom guard faults, command is already zero, and last trusted odom is saved.
2. Wait a short configured time for a valid full global pose.
3. If none arrives, remain stopped and report retirement.
4. Call the confirmed LIO restart interface once.
5. Wait for a fresh stable LIO odom sample.
6. Compute:

   ```text
   T_map_odom_new = T_map_base_global × inverse(T_odom_new_base)
   ```

7. Publish an accepted `/localization/map_odom_update`.
8. Reset the odom guard and resume at a reduced configured speed.
9. Any second fault remains stopped; no repeated restart loop.

## Constraints

- No shell `pkill`, no fuzzy process matching and no whole-machine reboot.
- Never reset while the vehicle is moving.
- Never infer yaw from a single GNSS position.
- Keep implementation private and small; no public multi-stage state contract
  is required.

## Required tests

- missing GPS;
- invalid/old GPS;
- restart service failure/timeout;
- no new odom;
- transform numerical consistency;
- second fault;
- all failure paths keep command zero.

