# Twist Propagation Axis Alignment Bug Analysis

## Objective

Identify the root cause of the axis misalignment in twist propagation where moving forward causes the prediction to move up (or other incorrect axis mappings).

## Findings

### Bug Identified: Twist (velocity) is NOT transformed to the cloud frame before propagation

**Location:** `src/twist_propagation/twist_propagation/twist_propagation_node.py:1361-1367`

The core issue is a **frame mismatch** between the pose and the twist during propagation. Here's the flow:

1. **Pose** is received in `pose_frame` (typically `"marker_map"` from OpenVINS via `odom_to_pose_relay.py`)
2. **Pose position** is transformed to `cloud_frame` via `_transform_pose_to_cloud_frame()` (line 1352)
3. **Twist (velocity)** is estimated from pose differences **in `pose_frame` coordinates** (lines 816-818 in `_estimate_twist`)
4. **Propagation** uses the cloud-frame position but the **pose-frame twist** — the twist is NEVER transformed to the cloud frame

### Why this causes axis misalignment

When `pose_frame` (`"marker_map"`) and `cloud_frame` (`"marker_map"`) are the same frame, the twist is already correct and there's no bug. **But when they differ**, the velocity components (vx, vy, vz) are in the wrong coordinate system.

The critical code path in `_run_idle_cycle` (lines 1361-1367):

```python
# Position is in cloud_frame
hit_result = self._propagate_and_find_hit(
    (px_cloud, py_cloud, pz_cloud, qx, qy, qz, qw),  # cloud frame
    self._twist,  # STILL in pose_frame — NOT transformed!
)
```

The `_propagate_pose` function (lines 174-207) adds `vx*dt` to the cloud-frame x position, but `vx` is the velocity in the pose frame's x-axis, not the cloud frame's x-axis. If the frames have any rotational offset, this causes the predicted path to go in the wrong direction.

### When this manifests

This bug manifests whenever `pose_frame != cloud_frame` with a non-trivial rotation between them. Common scenarios:

1. **Cloud in optical frame** (`camera_depth_optical_frame`) — optical frame has Z-forward, X-right, Y-down, while `marker_map` has standard ROS conventions (X-forward, Y-left, Z-up). A forward motion in `marker_map` (+X) would become +Z in optical frame, making the prediction go "up" instead of "forward".

2. **Cloud from a rotated sensor** — if the fused cloud is published in a frame that has any rotation relative to `marker_map`.

3. **Using the external twist from odometry** — OpenVINS odometry twist is in the **body frame** of the camera (`arm_cam0`), not in `marker_map`. The `_on_hand_twist` callback stores it directly (line 738-741) without any frame transformation.

### Additional issue: Orientation is also not transformed

At line 1356, the orientation `(qx, qy, qz, qw)` from the pose is published in `cloud_frame`, but it was never transformed from `pose_frame`. Similarly, the angular velocity `(wx, wy, wz)` is in `pose_frame` coordinates but applied to an orientation that's being propagated in the cloud frame context.

### The `_transform_pose_to_cloud_frame` only handles position

Looking at `_transform_pose_to_cloud_frame` (lines 960-1008), it only transforms the **position** (x, y, z) — it does NOT transform the orientation quaternion. The orientation is passed through unchanged. This means the propagation starts with a position in the cloud frame but an orientation in the pose frame.

## Implementation Plan

### Phase 1: Transform the full pose (position + orientation) to cloud frame

- [ ] **1.1** Modify `_transform_pose_to_cloud_frame` to return both position AND orientation in the cloud frame. Rename to `_transform_full_pose_to_cloud_frame` or add a new method. The method should:
  - Look up the TF from `pose_frame` to `cloud_frame`
  - Apply the rotation to the position (already done)
  - Also apply the rotation to the orientation quaternion via quaternion multiplication
  - Return `(px, py, pz, qx, qy, qz, qw)` all in cloud frame

- [ ] **1.2** Update `_run_idle_cycle` to use the transformed orientation for propagation instead of the raw pose orientation.

### Phase 2: Transform the twist (linear + angular velocity) to cloud frame

- [ ] **2.1** Add a new method `_transform_twist_to_cloud_frame` that transforms a twist `(vx, vy, vz, wx, wy, wz)` from `pose_frame` to `cloud_frame`. This requires:
  - Looking up the rotation matrix R from `pose_frame` to `cloud_frame`
  - Transforming linear velocity: `v_cloud = R * v_pose`
  - Transforming angular velocity: `w_cloud = R * w_pose`
  - (No translation component needed for velocity — it's a vector, not a point)

- [ ] **2.2** In `_run_idle_cycle`, after estimating the twist and before calling `_propagate_and_find_hit`, transform the twist to the cloud frame:
  ```python
  twist_cloud = self._transform_twist_to_cloud_frame(self._twist, pose_frame)
  hit_result = self._propagate_and_find_hit(
      (px_cloud, py_cloud, pz_cloud, qx_cloud, qy_cloud, qz_cloud, qw_cloud),
      twist_cloud,
  )
  ```

- [ ] **2.3** Cache the rotation matrix from `pose_frame` to `cloud_frame` to avoid redundant TF lookups (the same transform is needed for both pose and twist).

### Phase 3: Handle external twist frame correctly

- [ ] **3.1** When using the external twist from odometry (via `_on_hand_twist`), the twist is in the **body frame** of the tracked object (e.g., `arm_cam0`), not in `marker_map`. This needs to be transformed to `marker_map` first (using the orientation from the latest pose), and then to `cloud_frame`.
  - Option A: Transform external twist to `marker_map` in `_on_hand_twist` callback using the latest pose orientation
  - Option B: Store the external twist's frame_id and transform it in `_estimate_twist` before returning
  - Option C: Since the external twist and self-estimated twist should be in the same frame, ensure both are in `pose_frame` before the cloud-frame transformation in Phase 2

### Phase 4: Add frame mismatch logging

- [ ] **4.1** Add a startup warning or info log when `pose_frame != cloud_frame` to make this visible during debugging.

- [ ] **4.2** Add a unit test that verifies propagation produces the correct path when `pose_frame` and `cloud_frame` have a 90-degree rotation between them.

## Verification Criteria

- [ ] When moving the hand forward in `marker_map`, the predicted path in the cloud frame also moves forward (not up, not sideways)
- [ ] When `pose_frame == cloud_frame`, behavior is unchanged from current
- [ ] When `pose_frame != cloud_frame` with a known rotation, the predicted path is rotated correctly
- [ ] External twist from odometry produces the same propagation direction as self-estimated twist
- [ ] Angular velocity is correctly transformed between frames

## Potential Risks and Mitigations

1. **TF lookup overhead**: Additional TF lookups per cycle
   Mitigation: Cache the rotation matrix; reuse the same lookup for both pose and twist transforms. The TF lookup is already done once for position — just extract the rotation too.

2. **External twist frame ambiguity**: OpenVINS body-frame twist may not have a clear frame_id in the TwistStamped header
   Mitigation: Check the `frame_id` of the incoming TwistStamped. If it's empty or matches the child_frame of the odom, treat it as body-frame and transform accordingly.

3. **Breaking existing behavior when frames match**: The transformation is identity when `pose_frame == cloud_frame`, so existing correct behavior is preserved.
   Mitigation: Add an early return in the transform methods when frames match (already done for position at line 971-972).

## Alternative Approaches

1. **Transform everything to `marker_map` and keep propagation in `marker_map`**: Instead of transforming to cloud frame, transform the cloud to `marker_map`. This avoids the issue entirely but requires transforming the entire point cloud each cycle (expensive).

2. **Estimate twist in cloud frame directly**: Instead of estimating twist from pose differences in `pose_frame`, transform all poses to `cloud_frame` first, then compute finite differences. This avoids the separate twist transformation but requires transforming every pose in the buffer.

3. **Ensure `pose_frame == cloud_frame` always**: Configure the system so the hand pose and cloud are always in the same frame (e.g., both in `marker_map`). This is the simplest approach but limits flexibility.
