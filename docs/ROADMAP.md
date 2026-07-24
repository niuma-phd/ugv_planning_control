# Implementation roadmap

Subject 2 is the only critical path until its disconnected-actuator end-to-end
test passes.

## Wave 0 — repository foundation

- Clean Git repository, root guidance and ignored artifact policy.
- Concise architecture/interfaces rather than importing the historical task
  tree.
- Pinned external driver/LIO source references.
- Repository verifier and ROS 2 Humble CI.

## Wave 1 — immediately parallel

1. **Localization**
   - raw LIO lidar-pose to canonical base-pose adapter;
   - TF publishing;
   - identity/updateable `map→odom`;
   - odom guard and last-trusted persistence.
2. **S2 control**
   - path cache and nearest-progress tracking;
   - fixed-lookahead Pure Pursuit;
   - command limits and terminal slowdown;
   - input timeout zero output.
3. **S1 perception**
   - robust PointCloud2 layout checks;
   - base-frame transform;
   - ROI/self/height filter;
   - 2-D occupied-cell output.
4. **S1 avoidance**
   - obstacle trigger;
   - constant-curvature rollout/collision test;
   - simple scoring and zero-on-no-candidate.

## Wave 2 — S2 integration

```text
localization + S2 control
→ bringup and fixed synthetic path
→ Avia PointCloud2/LIO live topics
→ odom disconnect/jump tests
→ disconnected-actuator command sign test
→ low-speed closed-course tuning
```

GPS/LIO recovery is added only after the global pose type/frame, heading
quality, restart service/process boundary and post-restart odom behavior are
confirmed.

## Wave 3 — S1 integration

```text
Horizon TF + point cloud detector
→ static boxes at known body-frame positions
→ curvature planner with disconnected actuator
→ external takeover handshake
→ low-speed soft-obstacle course
```

## Merge order

1. Localization foundation.
2. Subject 2 controller.
3. Subject 2 bringup and RDK evidence.
4. Subject 1 perception.
5. Subject 1 avoidance.
6. Confirmed recovery adapter.

Parallel sessions may implement disjoint packages, but integration merges and
RDK checks follow this order.

