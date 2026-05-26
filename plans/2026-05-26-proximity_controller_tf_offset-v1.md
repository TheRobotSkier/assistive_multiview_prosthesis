# Replace Hardcoded Grasp Contact Offset with TF Lookup in Proximity Controller

## Objective

Replace the hardcoded `grasp_contact_offset` parameter in the proximity controller node with a runtime TF2 lookup that resolves the distance from the tracked pose frame (camera) to `grasp_contact_frame`. This eliminates the error-prone manual offset precomputation and fixes the existing Z-sign discrepancy bug (`config/prosthesis_config.yaml:155` has `-0.1352` while all other references use `+0.1352`).

## Background & Problem Analysis

### Current Approach (Broken)

The proximity controller at `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:438-457` computes the Euclidean distance between the current hand pose and the planned target pose. Both poses are shifted by a hardcoded 3D offset (`[0.1543, -0.1485, -0.1352]` from `config/prosthesis_config.yaml:155`) that represents the camera-to-fingertip translation, rotated into each pose's local frame via `_apply_offset()` (line 460-480).

**Problems:**
1. **Z-sign bug**: The proximity controller uses `-0.1352` (line 155) while twist propagation uses `+0.1352` (line 332). The comment says "Same value" but it is not.
2. **Manual recomputation**: The offset must be hand-calculated from the camera mount geometry chain whenever the mount changes.
3. **No single source of truth**: The same offset appears in 4 places in `prosthesis_config.yaml` and must be kept in sync manually.

### What Already Exists

- `publish_camera_mounts.py` (`src/sensor_fusion_bringup/scripts/publish_camera_mounts.py:296-303`) already publishes `palm_frame → grasp_contact_frame` as a TF transform.
- The full TF chain is available at runtime: `arm_d435i_arm_link → screw_frame → palm_frame → grasp_contact_frame`.
- The hand pose publisher (`src/camera/camera/hand_pose_publisher.py`) already does `world → wrist_link` TF lookups using `tf2_ros.Buffer` + `TransformListener`.
- The twist propagation node (`src/twist_propagation/twist_propagation/twist_propagation_node.py:644-646`) already has a TF2 setup with the `_HAS_TF2` guard pattern.
- DDS reliability workaround: `publish_camera_mounts.py` sends transforms on both `/tf_static` AND `/tf` (lines 219, 241-242) to handle CycloneDDS latch issues.

### Key Insight: Frame Semantics

The `/hand_pose` topic carries the `wrist_link` pose in the `world` frame (published by `hand_pose_publisher.py`). The `/grasp_preshaping/target_hand_pose` carries the planner's target position with the same `frame_id` as the input pose (`preshaping_service_bridge_node.cpp:562`).

The distance needs to be measured **from the grasp contact point** (fingertips) of the current hand to the **grasp contact point** of the planned target. The TF chain `arm_d435i_arm_link → ... → grasp_contact_frame` provides the camera-to-grasp-contact transform. However, the poses on `/hand_pose` and `/target_hand_pose` are already in the `world` frame, so the approach is:

1. Transform the current hand pose from `world` frame to `grasp_contact_frame` position.
2. Transform the planned target pose from `world` frame to `grasp_contact_frame` position.
3. Compute Euclidean distance between these two grasp-contact positions.

**Better approach**: Since both poses are in `world` frame, we can look up the static transform `arm_d435i_arm_link → grasp_contact_frame` once, then apply it to both poses (rotating the offset by each pose's orientation). This is essentially what `_apply_offset` already does, but the offset would come from TF instead of a hardcoded parameter.

## Implementation Plan

### Phase 1: Add TF2 Infrastructure to Proximity Controller

- [ ] **1.1** Add `tf2_ros` imports to `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`. Import `Buffer`, `TransformListener`, `ConnectivityException`, and `LookupException` from `tf2_ros`. Also import `rclpy.time` and `rclpy.duration`. Follow the pattern from `src/camera/camera/hand_pose_publisher.py:15-16`.

- [ ] **1.2** Add new ROS parameters for TF frame names. Declare parameters:
  - `source_frame` (default: `""`) — the tracked pose frame (e.g., `arm_d435i_arm_link`), which is the camera link frame. If empty, fall back to the hardcoded offset.
  - `contact_frame` (default: `"grasp_contact_frame"`) — the grasp contact frame published by `publish_camera_mounts.py`.
  - `tf_lookup_timeout_s` (default: `0.5`) — timeout for the initial TF lookup.
  - Keep the existing `grasp_contact_offset` parameter as a fallback.

- [ ] **1.3** Initialize `tf2_ros.Buffer` and `TransformListener` in `__init__()`, following the established pattern from `src/twist_propagation/twist_propagation/twist_propagation_node.py:641-646`. Use a `_HAS_TF2` guard for robustness (try/except the import).

- [ ] **1.4** Add a one-time TF offset resolution method `_resolve_grasp_contact_offset()`. This method:
  1. Checks if `source_frame` parameter is non-empty.
  2. Calls `self._tf_buffer.lookup_transform(source_frame, contact_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=tf_lookup_timeout_s))`.
  3. Extracts the translation `(x, y, z)` from the result — this is the offset from the camera link origin to the grasp contact point, expressed in the camera link's local frame.
  4. Stores it as `self._grasp_contact_offset` (replacing the parameter value).
  5. Logs the resolved offset for verification.
  6. On failure (exception), falls back to the hardcoded `grasp_contact_offset` parameter and logs a warning.

- [ ] **1.5** Call `_resolve_grasp_contact_offset()` from a startup timer (1-2 second delay after node init). This gives the TF tree time to populate from `publish_camera_mounts.py`. Use `self.create_timer(delay, self._resolve_grasp_contact_offset, one_shot=True)` or the `rclpy` equivalent (create a one-shot timer that cancels itself). This avoids the DDS race condition where the static transforms may not be available at node startup.

### Phase 2: Modify Distance Computation

- [ ] **2.1** The `_compute_proximity_distance()` method at line 438 and `_apply_offset()` at line 460 remain functionally unchanged. The only change is the **source** of `self._grasp_contact_offset` — it now comes from TF instead of the YAML parameter. No modification to these methods is needed if Phase 1 correctly populates `self._grasp_contact_offset`.

- [ ] **2.2** Add a diagnostic log at startup that prints the resolved offset value and its source (TF vs. hardcoded fallback). This helps verify correctness during integration testing.

### Phase 3: Configuration Updates

- [ ] **3.1** Add the new TF frame parameters to `config/prosthesis_config.yaml` under `proximity_controller.ros__parameters` (around line 133). Add:
  ```yaml
  source_frame: "arm_d435i_arm_link"
  contact_frame: "grasp_contact_frame"
  tf_lookup_timeout_s: 0.5
  ```
  Keep `grasp_contact_offset` as-is (it becomes the fallback).

- [ ] **3.2** Fix the Z-sign bug in `config/prosthesis_config.yaml:155`. Change `grasp_contact_offset: [0.1543, -0.1485, -0.1352]` to `grasp_contact_offset: [0.1543, -0.1485, 0.1352]`. This ensures the fallback value is correct even if TF lookup fails.

- [ ] **3.3** Add a comment above `grasp_contact_offset` noting that it is now a **fallback** value, and that the primary source is the TF lookup from `source_frame` to `contact_frame`.

### Phase 4: Robustness & DDS Considerations

- [ ] **4.1** Handle the DDS `TRANSIENT_LOCAL` reliability issue. The `publish_camera_mounts.py` node already sends transforms on both `/tf_static` AND `/tf` (lines 219, 241-242). However, if the proximity controller starts before the mount publisher, the TF buffer may be empty. The startup timer delay (step 1.5) mitigates this. Additionally, add a retry mechanism: if the initial lookup fails, retry every 2 seconds up to 5 times before falling back to the hardcoded value.

- [ ] **4.2** Add a validation step after resolving the offset: compare the TF-resolved offset against the hardcoded fallback value. If they differ by more than a small epsilon (e.g., 1 cm), log a warning. This catches configuration drift between `camera_mounts.yaml` and `prosthesis_config.yaml`.

- [ ] **4.3** Ensure the node degrades gracefully. If TF is unavailable entirely (e.g., `tf2_ros` not installed, or `source_frame` is empty), the node should use the hardcoded offset and log a single info message. The node must never crash or refuse to start due to TF unavailability.

### Phase 5: Testing & Verification

- [ ] **5.1** Verify the TF-resolved offset matches the expected value. With `8_cm_cam_mount`, the full chain `arm_d435i_arm_link → screw_frame → palm_frame → grasp_contact_frame` should yield approximately `[0.1543, -0.1485, 0.1352]` (the corrected value). Log the resolved offset at startup for manual verification.

- [ ] **5.2** Test with the proximity controller running in isolation (no TF tree) to confirm the fallback path works correctly.

- [ ] **5.3** Test with the full pipeline running to confirm the TF-resolved offset produces correct proximity distances. Compare against the twist propagation node's offset usage to ensure consistency.

- [ ] **5.4** Test mount switching: change the active mount (e.g., from `8_cm_cam_mount` to `5_cm_cam_mount`), restart `publish_camera_mounts.py`, and verify the proximity controller picks up the new offset on restart.

## Verification Criteria

1. **Offset correctness**: The TF-resolved offset matches the corrected hardcoded value `[0.1543, -0.1485, 0.1352]` (within 1mm) when using `8_cm_cam_mount`.
2. **Fallback works**: When `source_frame` is empty or TF is unavailable, the node uses the hardcoded `grasp_contact_offset` from the parameter server without crashing.
3. **Z-sign bug fixed**: The hardcoded fallback value in `config/prosthesis_config.yaml:155` now uses `+0.1352` instead of `-0.1352`.
4. **No regression**: Proximity distances computed with the TF-based offset match those from the corrected hardcoded offset (within floating-point precision).
5. **Startup robustness**: The node does not crash or hang if the TF tree is not yet available at startup. It retries and falls back gracefully.
6. **DDS compatibility**: The TF lookup works reliably with CycloneDDS, leveraging the existing `/tf_static` + `/tf` dual-publishing pattern from `publish_camera_mounts.py`.

## Potential Risks and Mitigations

1. **TF tree not available at startup**
   - Mitigation: Use a delayed one-shot timer (1-2s) for the initial lookup, with up to 5 retries at 2-second intervals. Fall back to the hardcoded parameter if all retries fail.

2. **DDS reliability with CycloneDDS `TRANSIENT_LOCAL`**
   - Mitigation: The `publish_camera_mounts.py` node already publishes on both `/tf_static` and `/tf`. The retry mechanism provides additional resilience. The hardcoded fallback ensures the node never blocks indefinitely.

3. **Frame name mismatch between `source_frame` config and actual TF tree**
   - Mitigation: Log the resolved offset at startup. The validation step (4.2) compares against the hardcoded value and warns on significant discrepancy. The `frame_id` from the incoming `/hand_pose` messages could also be used as an alternative source frame (but this requires confirming it matches the TF tree frame).

4. **Performance impact of TF listener in a 10 Hz control loop**
   - Mitigation: The offset is resolved **once** at startup (it is a static transform). The TF listener is only used during initialization, not in the control loop. The `Buffer` + `TransformListener` have negligible overhead when not actively querying.

5. **Breaking existing deployments that rely on the hardcoded offset**
   - Mitigation: The hardcoded `grasp_contact_offset` parameter is kept as a fallback. If `source_frame` is empty (the default in the parameter declaration), the node uses the hardcoded value — maintaining backward compatibility.

## Alternative Approaches

1. **Dynamic per-cycle TF lookup**: Instead of resolving the offset once at startup, look up the transform on every control cycle. This would handle dynamic mount changes without restart, but adds latency and TF-dependency to the critical control path. Not recommended for a 10 Hz control loop with a static mount.

2. **Read `camera_mounts.yaml` directly**: Parse the YAML file in the proximity controller and compute the offset from the mount geometry. This avoids the TF system entirely but duplicates the transform chain logic and creates a second source of truth for the mount configuration.

3. **Dedicated offset topic**: Have `publish_camera_mounts.py` publish the computed offset as a `Vector3Stamped` on a dedicated topic. The proximity controller subscribes to it. This avoids TF2 dependency but adds a new topic and coupling between the nodes.

4. **Fix only the Z-sign bug**: The minimal fix is to correct `-0.1352` to `+0.1352` in `config/prosthesis_config.yaml:155`. This addresses the immediate symptom but does not solve the root cause (manual offset maintenance). Recommended as a prerequisite regardless of which approach is chosen.

## Recommended Approach

**Phase 1-4 as described above** (one-time TF lookup with hardcoded fallback). This provides:
- Automatic offset derivation from the canonical `camera_mounts.yaml` via the TF tree
- Backward compatibility via the hardcoded fallback
- No performance impact on the control loop
- Consistency with existing TF usage patterns in the codebase
- DDS resilience through the retry + fallback mechanism
