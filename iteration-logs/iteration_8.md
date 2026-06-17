# Iteration 8 — Fix coordinate-frame mismatch in keyframe buffer (camera → world)

## What I saw in the logs

### Unit test results (logs/test-results/)
- `test_tsdf_fusion_pipeline` → **FAIL**: "Expected TSDF volume to contain geometry, but got NO GEOMETRY"
- `test_gtsam_optimization` → **FAIL**: KITTI odometry error too large (ATE=0.029 > 0.025)
- `test_scene_preview` → **PASS** (but this is a simpler smoke test)
- All pointcloud fusion and keyframe buffer unit tests → **PASS**

### Replay test results (logs/replay-results/replay_20260617_030742/)
- **bag_analysis.txt**: 5 topics with 0 msgs (STARVED): `/diagnostics`, `/gtsam/head_pose`, `/gtsam/arm_pose`, `/tf_static`, `/parameter_events`
- **log_analysis.txt**: No ERROR-level messages, one WARN about `/parameter_events`
- **bag_metrics.json**: `/camera/points` effective_hz=19.6 (close to 20 Hz nominal)
- **log_metrics.json**: `/gtsam_tracker/pose` → no messages logged (the gtsam_tracker node didn't publish)
- **metrics.json**: Overall pipeline appears short-lived (~3.0s bag)

> Root cause of short bag: the VIO odometry TF relay (`openvins_odom_tf_relay`) uses `lookupTransform("odom", "camera")` which returns the transform *from camera to odom*, but `transform_to_pose` treats it as `T_odom_camera` — previous iterations fixed this.

## Decision: What to fix this iteration

The "NO GEOMETRY" TSDF test failure is a **critical blocking issue** — if the pipeline reaches the fusion stage with real sensor data, it would produce an empty object cloud. Fixing this directly advances the goal of *measurably better fused pointclouds*.

### Root cause analysis

After tracing the code paths:

1. **The convention**: `keyframe.py:44` and `cloud_utils.py:11` explicitly state that `cloud_xyz` is in the **world frame**:
   - `cloud_utils.py:11`: "cloud_xyz for unorganized clouds: (N,3) float array in the *world* frame."
   - `keyframe.py:8`: "the point cloud (xyz + rgb) in the *world* (marker_map) frame"

2. **The reality**: `keyframe_buffer_node.py:289` stores `cloud_xyz` **as-is from the sensor message** (camera frame) — no transformation to world frame:
   ```python
   cloud_xyz=np.asarray(cloud_xyz),  # ← camera frame, not world frame!
   ```

3. **The impact**: All downstream code in `cloud_utils.py` and `tsdf_fusion_core.py` assumes world-frame points:
   - `build_depth_image()` at `cloud_utils.py:232` projects world→camera via `pose_inv @ cloud_xyz`
   - If `cloud_xyz` is already in camera frame, applying `pose_inv` produces garbage coordinates
   - The depth image ends up with no valid pixels → TSDF volume gets "NO GEOMETRY"
   - This explains `test_tsdf_fusion_pipeline` failure

4. **Why unit tests pass**: The TSDF unit tests in `test_tsdf_core.py:194` correctly construct synthetic clouds in world frame:
   ```python
   cloud_xyz = (pose @ homog.T).T[:, :3]  # camera→world transform
   ```
   So they test the downstream code with correct input, and miss the production bug.

### Changes made

**File**: `src/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py` (lines 286–298)

Added coordinate frame transformation inside `try_create_keyframe()`, between the pose-freshness check and Keyframe construction:

```python
# Transform cloud_xyz from camera frame → world frame using pose.
xyz_in = np.asarray(cloud_xyz)
orig_shape = xyz_in.shape
flat = xyz_in.reshape(-1, 3)
N = flat.shape[0]
homog = np.empty((N, 4), dtype=np.float64)
homog[:, :3] = flat
homog[:, 3] = 1.0
cloud_xyz = (pose @ homog.T).T[:, :3]
cloud_xyz = cloud_xyz.reshape(orig_shape).astype(xyz_in.dtype)
```

This uses the GTSAM-smoothed `pose` (T_world_camera) to transform the raw sensor points from camera optical frame to the world frame, matching the documented convention.

### Expected impact

1. **TSDF fusion**: `build_depth_image()` will now receive world-frame points and correctly project them to camera coordinates using `pose_inv` → depth images will have valid pixels → TSDF volume will contain geometry → `test_tsdf_fusion_pipeline` should pass
2. **Masked clouds**: `mask_unorganized_cloud()` will correctly project world-frame points to pixels for mask lookup (instead of garbage coordinates)
3. **Cross-camera features**: `lookup_depth_3d()` for organized clouds returns `cloud[v,u]` directly — now returns world-frame points as expected
4. **No memory impact**: The transformed array uses the same storage as the original; the temporary homogeneous matrix is garbage-collected
5. **Performance**: ~307k × 4 × 4 = ~5M float ops on a 640×480 organized cloud — well under 1 ms on modern CPU
