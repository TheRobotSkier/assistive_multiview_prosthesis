# Fix: Proximity Controller Frame Mismatch

## Objective

The proximity controller compares the current hand position (`/hand_pose`, which is the camera/IMU position) against the planner's target hand position. Both are in the camera frame. However, the 0.08m threshold is designed for fingertip-to-target distance, not camera-to-camera distance. The camera is ~30cm from the fingertips, so even when the fingertips are at the object, the camera-to-camera distance can be 0.26m+.

The fix is to apply the same `propagation_origin_offset` that twist propagation uses, transforming the comparison from camera-frame to grasp-contact-frame. This way the proximity controller compares "where the fingertips are" vs. "where the fingertips should be", and the 0.08m threshold becomes meaningful.

## Root Cause Analysis

### Frame chain
```
marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> screw_frame -> palm_frame -> grasp_contact_frame
```

- `/hand_pose` reports `arm_cam0` position in `marker_map`
- Planner's `target_position` is the best SMC particle's position — also in `arm_cam0` frame (camera position at grasp time)
- The offset from `arm_cam0` to `grasp_contact_frame` is `[0.1543, -0.1485, 0.1352]` (~26cm magnitude)
- When the user brings their hand to the object, the camera is ~26cm from the object
- The planner's target camera position is also ~26cm from the object (from the predicted approach direction)
- The distance between current camera and target camera depends on approach angle — minimum observed was 0.263m

### Why the threshold never triggers
- `enter_thresh = 0.08m` — designed for fingertip proximity
- Minimum observed distance = 0.263m — camera-to-camera distance
- Even if the user perfectly aligns with the target, the camera offset adds ~0.15-0.30m to the distance

## Implementation Plan

- [ ] **1. Add `grasp_contact_offset` parameter to proximity controller**
  - Add a new parameter `grasp_contact_offset` (double array, default `[0.0, 0.0, 0.0]`)
  - This is the offset from the tracked pose origin (camera) to the grasp contact point (fingertips), expressed in the pose's local frame
  - Use the same value as twist propagation's `propagation_origin_offset`: `[0.1543, -0.1485, 0.1352]`
  - File: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

- [ ] **2. Apply the offset in the control loop**
  - In `_control_loop`, before computing distance:
    1. Get the current camera pose position and orientation
    2. Rotate the `grasp_contact_offset` by the camera's orientation (to transform from local to world frame)
    3. Add the rotated offset to the camera position → this gives the grasp contact position
    4. Similarly, apply the same offset to the planned hand frame position (using the planned orientation)
    5. Compute distance between the two grasp contact positions
  - This ensures both positions are compared at the fingertips, not at the camera

- [ ] **3. Add the offset to the config file**
  - Add `grasp_contact_offset: [0.1543, -0.1485, 0.1352]` to the proximity controller parameters in `config/prosthesis_config.yaml`
  - Use the same value as twist propagation's `propagation_origin_offset`

- [ ] **4. Log the grasp contact distance alongside the raw distance**
  - In the FAR/NEAR mode log messages, include both the raw distance and the grasp contact distance for debugging
  - This helps verify the offset is working correctly

## Verification Criteria

- When the hand is at the object, the proximity controller should report a grasp contact distance near 0m (not 0.26m)
- The "Entered near zone" message should appear when the fingertips are within 8cm of the target grasp contact position
- The FAR mode log should show both raw and contact distances for debugging

## Potential Risks and Mitigations

1. **Offset orientation mismatch**: The offset must be rotated by the pose orientation before being added. If the orientation is wrong, the offset will point in the wrong direction.
   Mitigation: Use the same rotation logic as twist propagation's `_apply_origin_offset` method.

2. **Planner target orientation may differ from current orientation**: The planner's target has its own orientation (the planned wrist rotation). Applying the offset with the planned orientation gives the planned grasp contact position, which is correct.
   Mitigation: Apply offset to both current and planned poses using their respective orientations.

3. **Config drift**: The offset value must match between twist propagation and proximity controller.
   Mitigation: Document that both must use the same value, computed from `camera_mounts.yaml`.

## Alternative Approaches

1. **Increase the threshold instead**: Change `enter_thresh` from 0.08m to 0.35m to account for the camera offset. Simpler but less precise — the threshold would depend on approach angle.
2. **Use TF lookup for grasp_contact_frame**: Look up `marker_map -> grasp_contact_frame` dynamically instead of using a static offset. More robust but requires the TF chain to be connected (which it sometimes isn't due to Mia Hand disconnects).
3. **Change the planner to output grasp contact position**: Modify the planner to output the target position in grasp contact frame instead of camera frame. More invasive change.
