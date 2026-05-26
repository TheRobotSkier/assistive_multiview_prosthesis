# Proximity Controller: TF-Based Grasp Contact Position

## Objective

Replace the hardcoded `grasp_contact_offset` rotation in the proximity controller with a TF2 lookup of `marker_map → grasp_contact_frame`. This gives the exact position of the grasp contact point (fingertips) in `marker_map`, which is then compared against the twist propagation contact pose (also in `marker_map`) to compute proximity distance.

## Verification Summary

### The TF chain is confirmed complete and consistent

Every edge from `marker_map` to `grasp_contact_frame` exists at runtime:

```
marker_map
  └→ arm_imu                                        [Edge 1: DYNAMIC, ~200 Hz, openvins_odom_tf_relay]
      └→ arm_cam0                                   [Edge 2: STATIC, openvins_odom_tf_relay]
          └→ arm_d435i_arm_link                     [Edge 3: STATIC, openvins_realsense_tf_bridge]
              └→ d435i_arm_bottom_screw_frame_8_cm_cam_mount  [Edge 4: STATIC, publish_camera_mounts.py]
                  └→ palm_frame                     [Edge 5: STATIC, publish_camera_mounts.py]
                      └→ grasp_contact_frame        [Edge 6: STATIC, publish_camera_mounts.py]
```

All frame names verified consistent across nodes. The `--link-frame arm_d435i_arm_link` argument is correctly passed in the pipeline launch.

### Both positions are in `marker_map`

| Quantity | Frame | Source |
|---|---|---|
| TF lookup `marker_map → grasp_contact_frame` | `marker_map` | 6-edge TF chain above |
| Contact pose (`/grasp_preshaping/contact_pose`) | `marker_map` | Twist propagation publishes with `frame_id = self._cloud_frame = marker_map` |

### What each point represents

- **TF `grasp_contact_frame`**: The physical fingertip position in `marker_map`. This is the **current** position of the fingertips, updated in real time as the hand moves (because edge 1 is dynamic at 200 Hz).
- **Contact pose**: The predicted hit point on the **object surface** in `marker_map`, computed by twist propagation. This is where the fingertips are predicted to contact the object.

The distance between these two points is exactly what we want: "how far are the fingertips right now from where they'll contact the object."

### Why the previous hardcoded offset approach failed

The old code at `grasp_proximity_controller_node.py:495` did:
```python
cur_pos = self._apply_offset(current.pose, self._grasp_contact_offset)
```
This rotates the hardcoded offset `[0.1543, -0.1485, 0.1352]` by the pose quaternion. But:
1. The offset was precomputed for the `cam0` local frame, while `/hand_pose` tracks `arm_imu` (the IMU frame).
2. The IMU orientation may differ from the camera optical frame orientation, causing the ~24cm offset to rotate in the wrong direction.
3. The TF chain already computes this transform correctly — we just need to use it.

## Implementation Plan

### Step 1: Add TF2 imports and initialization

- [ ] **1.1** Add `tf2_ros` imports at the top of `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`. Add:
  ```
  from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException
  import rclpy.time
  ```
  Follow the pattern from `src/camera/camera/hand_pose_publisher.py:15-16`. Wrap in try/except with a `_HAS_TF2` flag for robustness (like `twist_propagation_node.py:644`).

- [ ] **1.2** Add new parameters for the TF frames:
  - `grasp_contact_frame` (default: `"grasp_contact_frame"`) — the child frame to look up
  - `map_frame` (default: `"marker_map"`) — the parent frame (same as the hand pose frame)
  - `tf_lookup_timeout_s` (default: `0.5`) — timeout for initial TF resolution

  Declare these alongside the existing parameters (around line 80-90).

- [ ] **1.3** Initialize TF2 `Buffer` + `TransformListener` in `__init__()`, guarded by `_HAS_TF2`:
  ```python
  self._tf_buffer = None
  self._tf_listener = None
  if _HAS_TF2:
      self._tf_buffer = Buffer()
      self._tf_listener = TransformListener(self._tf_buffer, self)
  ```

### Step 2: Replace `_compute_proximity_distance` with TF-based approach

- [ ] **2.1** Replace the `_compute_proximity_distance` method. Instead of applying a hardcoded offset, look up `marker_map → grasp_contact_frame` via TF to get the current fingertip position. The new logic:

  1. Look up `marker_map → grasp_contact_frame` via `self._tf_buffer.lookup_transform(map_frame, grasp_contact_frame, rclpy.time.Time())`.
  2. Extract the translation `(x, y, z)` from the result — this is the grasp contact position in `marker_map`.
  3. Extract the contact pose position `(x, y, z)` — this is the hit point on the object surface in `marker_map`.
  4. Compute Euclidean distance between the two.

  The method signature stays the same but the body changes. The `current: PoseStamped` parameter is no longer used for the offset — the TF lookup replaces it entirely. However, keep the parameter for potential future use and for the log messages.

- [ ] **2.2** Wrap the TF lookup in try/except. On failure (`LookupException`, `ConnectivityException`, or `ExtrapolationException`), log a warning (throttled) and return a large sentinel distance or skip the control cycle. Follow the pattern from `hand_pose_publisher.py:46`.

- [ ] **2.3** Remove the `_apply_offset` call from `_compute_proximity_distance`. The `_apply_offset` static method and `_grasp_contact_offset` parameter can remain for now (unused but harmless) — they can be cleaned up in a follow-up.

### Step 3: Update control loop for TF lookup

- [ ] **3.1** In `_control_loop()`, the readiness check currently requires `_contact_pose` (the twist propagation hit point). Add an additional check that the TF from `map_frame` to `grasp_contact_frame` is available. Use `self._tf_buffer.can_transform(map_frame, grasp_contact_frame, rclpy.time.Time())` for a non-blocking check. If the transform is not available, log "waiting for: grasp_contact TF" and return.

- [ ] **3.2** The TF lookup happens on every control cycle (10 Hz). Since the chain is mostly static (only edge 1 is dynamic at 200 Hz), this is lightweight. The `lookup_transform` call with `rclpy.time.Time()` (latest available) is non-blocking and fast — same pattern used by `hand_pose_publisher.py` at 50 Hz.

### Step 4: Configuration updates

- [ ] **4.1** Add the new TF frame parameters to `config/prosthesis_config.yaml` under `proximity_controller.ros__parameters`:
  ```yaml
  grasp_contact_frame: "grasp_contact_frame"
  map_frame: "marker_map"
  tf_lookup_timeout_s: 0.5
  ```

- [ ] **4.2** The `grasp_contact_offset` parameter can remain in the config as a deprecated fallback. Add a comment noting it is no longer used by the proximity controller (TF lookup replaces it). It is still used by twist propagation's `propagation_origin_offset` but that's a different node/parameter.

### Step 5: Threshold adjustment

- [ ] **5.1** The distance now measures fingertip-to-object-surface (via TF), not camera-to-object-surface (via offset). The thresholds may need adjustment. The current values (`enter: 0.08m`, `exit: 0.20m`) were designed for fingertip-to-target distance, which is what the TF-based approach computes. They should work as-is, but verify during testing and adjust if needed.

## Verification Criteria

1. **TF chain resolves**: `lookup_transform("marker_map", "grasp_contact_frame", ...)` succeeds and returns a translation consistent with the expected camera-to-fingertip offset (~24cm from the tracked position).
2. **Distance correctness**: When the hand is physically at the contact point, the computed distance should be near zero (within a few cm), not 30-40cm.
3. **Real-time updates**: As the hand moves, the distance changes smoothly at 10 Hz (the control loop rate), driven by the dynamic `marker_map → arm_imu` edge updating at 200 Hz.
4. **Graceful degradation**: If TF is unavailable (e.g., `publish_camera_mounts.py` not running), the node logs a warning and skips control cycles without crashing.
5. **Frame consistency**: Both the TF-based grasp contact position and the twist propagation contact pose are in `marker_map`.

## Potential Risks and Mitigations

1. **TF not available at startup**
   - Mitigation: The `can_transform` check in the control loop handles this. The control loop simply waits until the TF chain is ready. All static edges are published on both `/tf_static` and `/tf` with liveness timers, so they become available quickly.

2. **DDS reliability with CycloneDDS**
   - Mitigation: All three nodes in the TF chain use liveness timers (2-10 Hz re-send on `/tf`). The proximity controller's `can_transform` check handles temporary gaps. No special DDS configuration needed.

3. **TF lookup latency at 10 Hz**
   - Mitigation: The lookup uses `rclpy.time.Time()` (latest available, no timeout), which is essentially a buffer read — sub-microsecond. This is the same pattern used by `hand_pose_publisher.py` at 50 Hz and `twist_propagation_node.py` at ~50 Hz.

4. **Stale contact pose + moving grasp_contact_frame**
   - Mitigation: The contact pose freshness check (5 seconds, already implemented) prevents using a stale hit point. The grasp contact frame position updates in real time via TF, so the distance reflects the current hand position.

5. **Mount change requires TF tree update**
   - Mitigation: This is automatic — `publish_camera_mounts.py` reads the mount config from `camera_mounts.yaml` and publishes the correct TF chain. No manual offset update needed.
