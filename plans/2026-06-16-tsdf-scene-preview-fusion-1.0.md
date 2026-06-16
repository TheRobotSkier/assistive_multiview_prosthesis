# TSDF Scene Preview Fusion Node

## Objective

Replace the legacy `pointcloud_fusion` node with a lightweight TSDF-based scene preview that runs at ~1 Hz, producing a fused point cloud on `/fused_pointcloud` using the existing GTSAM + keyframe buffer + Open3D TSDF infrastructure — **without** requiring a hit point, segmentation (MobileSAM), or any inference server.

This makes `make run`, `make run-camera`, and `make run-camera-log` produce a TSDF-fused point cloud for RViz visualization, exercising the full V6 perception stack (gtsam_tracker → keyframe_buffer → tsdf_fusion) in preview mode.

## Background

The existing TSDF fusion (`tsdf_fusion_core.py:138-373`) is **on-demand only**: it requires a `TriggerGraspFusion` service call with a 3D hit point, projects that hit point to 2D, calls MobileSAM for a segmentation mask, then masks the cloud before TSDF integration. This dependency chain (hit point → SAM → mask) is what blocks using TSDF for a live preview.

However, Open3D's `ScalableTSDFVolume.integrate()` itself only needs an RGBD image + camera intrinsics/extrinsics. The SAM mask is a preprocessing step to isolate one object. For a **scene-level preview**, we can skip the mask entirely and integrate all points from each keyframe — producing a volumetrically fused reconstruction of the whole workspace.

## Design Decisions

1. **Fresh TSDF volume per cycle** (not persistent). Each 1 Hz tick creates a new volume, integrates all current keyframes, extracts the mesh, and publishes. This matches the legacy fusion's behavior (shows the current scene from recent observations) and avoids unbounded memory growth. A persistent accumulating volume can be added later.

2. **New `GetAllKeyframes` service** (clean approach). The keyframe buffer core already has `all_keyframes()` at `keyframe_buffer_node.py:355-359`, but it's not exposed via a ROS service. Adding a new `.srv` is ~15 lines and avoids the hack of querying `GetKeyframesInROI` with a 5-meter radius.

3. **Preview mode as a parameter on the existing `tsdf_fusion_node`**. When `preview_mode=true`: skip SAM client init, skip the `TriggerGraspFusion` service, create a timer at `preview_rate_hz`, fetch all keyframes, call `fuse_scene_preview()`, publish to `/fused_pointcloud`. This keeps it as one configurable node rather than a separate package.

4. **Output on `/fused_pointcloud`** (same topic as legacy fusion). Downstream consumers (twist_propagation, preshaping_service) already subscribe here. The preview node replaces the legacy fusion node when active.

5. **Workspace bbox crop**. Without segmentation, the full cloud includes background (walls, floor, ceiling). An optional axis-aligned bounding box crop in the `marker_map` frame limits integration to the workspace volume, analogous to the legacy fusion's distance filter. Configurable via parameters, can be disabled.

## Implementation Plan

### Phase 1: Pure-Logic Scene Preview Function

- [x] **1.1** Add `fuse_scene_preview()` to `src/tsdf_fusion/tsdf_fusion/tsdf_fusion_core.py`

  Rationale: This is the heart of the preview — a mask-free, hit-point-free variant of `fuse_object_cloud()`. It reuses `build_depth_image()` from `cloud_utils.py:187-241` and the existing TSDF volume + mesh extraction logic.

  The function signature:
  ```
  fuse_scene_preview(
      keyframes: Sequence,
      voxel_size: float = 0.005,
      sdf_trunc: float = 0.02,
      workspace_bbox_min: Optional[np.ndarray] = None,  # (3,) or None to disable
      workspace_bbox_max: Optional[np.ndarray] = None,
      dbscan_eps: float = 0.02,
      dbscan_min_points: int = 10,
      enable_dbscan_cleanup: bool = True,
  ) -> FuseResult
  ```

  Logic per keyframe:
  1. Extract `K`, `pose`, `image`, `cloud_xyz`, `cloud_rgb` from the keyframe
  2. If workspace bbox is provided, crop `cloud_xyz` / `cloud_rgb` to the AABB (vectorized numpy, same pattern as `_bbox_filter` in `pointcloud_fusion_node.py:118-124`)
  3. Rasterize the **full** (cropped) cloud into a depth image via `build_depth_image()` — no mask
  4. Build RGBD image from keyframe's RGB image + rasterized depth
  5. Integrate into `ScalableTSDFVolume`
  6. After all keyframes: extract mesh → point cloud
  7. Optional DBSCAN noise removal (remove label==-1 outliers, keep ALL clusters — unlike `fuse_object_cloud` which picks one cluster near the hit point)
  8. Return `FuseResult`

  Key difference from `fuse_object_cloud()`: no `hit_point_3d`, no `sam_segment_fn`, no `shift_inward()`, no mask application, no single-cluster selection.

- [x] **1.2** Add `fuse_scene_preview` to `__all__` in `tsdf_fusion_core.py:36-40`

- [x] **1.3** Write unit test for `fuse_scene_preview()` in `src/tsdf_fusion/test/test_scene_preview.py`

  Rationale: The existing test pattern (`test_tsdf_core.py`) uses synthetic keyframes with known geometry. The preview test should verify:
  - Empty keyframe list → `success=False`
  - Single keyframe with a synthetic plane → non-empty cloud, `success=True`
  - Two keyframes from different poses → fused cloud has more points than either alone
  - Workspace bbox crop excludes points outside the box
  - No Open3D import errors (lazy import pattern)

### Phase 2: GetAllKeyframes Service

- [x] **2.1** Create `src/sensor_fusion_msgs/srv/GetAllKeyframes.srv`

  Rationale: The keyframe buffer core already has `all_keyframes()` but no ROS service exposes it. The preview node needs all keyframes (not just those near a hit point).

  Service definition:
  ```
  # Request: optionally filter by camera_id (empty = all cameras)
  string camera_id
  ---
  sensor_fusion_msgs/Keyframe[] keyframes
  int32 count
  ```

- [x] **2.2** Register `GetAllKeyframes.srv` in `src/sensor_fusion_msgs/CMakeLists.txt:11-19`

  Add `"srv/GetAllKeyframes.srv"` to the `rosidl_generate_interfaces` call.

- [x] **2.3** Add `GetAllKeyframes` service to `keyframe_buffer_node.py`

  In `create_node()` at `keyframe_buffer_node.py:519-745`:
  - Import `GetAllKeyframes` alongside `GetKeyframesInROI` (line ~531)
  - Create the service: `self.create_service(GetAllKeyframes, "~/get_all", self._handle_get_all_keyframes)`
  - Add handler `_handle_get_all_keyframes()` that calls `self._core.all_keyframes(camera_id=request.camera_id if request.camera_id else None)` and serializes results using the existing `_keyframe_to_msg()` helper

  Rationale: ~15 lines of new code, reuses the existing serialization path.

### Phase 3: Preview Mode in TSDF Fusion Node

- [x] **3.1** Add preview parameters to `DEFAULT_PARAMS` in `tsdf_fusion_node.py:276-289`

  New parameters:
  ```python
  "preview_mode": False,
  "preview_rate_hz": 1.0,
  "preview_output_topic": "/fused_pointcloud",
  "preview_keyframe_service": "/keyframe_buffer/get_all",
  "preview_workspace_bbox_min": [-0.5, -0.5, -0.2],
  "preview_workspace_bbox_max": [0.8, 0.5, 0.6],
  "preview_enable_bbox_crop": True,
  "preview_enable_dbscan_cleanup": True,
  ```

- [x] **3.2** Add preview mode branching in `TsdfFusionNode.__init__()` at `tsdf_fusion_node.py:306-366`

  When `preview_mode=True`:
  - Skip SAM client initialization (`SamHttpClient`)
  - Skip SAM health check timer
  - Skip `TriggerGraspFusion` service creation
  - Create a `GetAllKeyframes` service client instead of `GetKeyframesInROI`
  - Publish to `preview_output_topic` instead of `output_topic`
  - Create a timer at `1.0 / preview_rate_hz` calling `self._preview_tick()`
  - Log: `"TsdfFusionNode running in PREVIEW mode (rate=X Hz, output=/fused_pointcloud)"`

  When `preview_mode=False`: existing behavior unchanged.

- [x] **3.3** Implement `_preview_tick()` method in `TsdfFusionNode`

  Logic:
  1. Call `GetAllKeyframes` service (with 10s timeout, same pattern as `_fetch_keyframes` at `tsdf_fusion_node.py:471-502`)
  2. Deserialize returned `Keyframe` messages via existing `_keyframe_from_msg()` at `tsdf_fusion_node.py:140-217`
  3. If no keyframes, log at debug level and return
  4. Call `fuse_scene_preview()` with the keyframes + workspace bbox parameters
  5. Build `PointCloud2` via existing `_build_xyzrgb_cloud()` at `tsdf_fusion_node.py:224-259`
  6. Publish to the preview output topic
  7. Log: point count, keyframe count, processing time

  Rationale: Reuses all existing deserialization and cloud-building infrastructure. The only new logic is the timer-driven fetch + call.

- [x] **3.4** Import `fuse_scene_preview` alongside `fuse_object_cloud` at `tsdf_fusion_node.py:29`

- [x] **3.5** Add `_fetch_all_keyframes()` helper method

  Similar to `_fetch_keyframes()` at `tsdf_fusion_node.py:471-502` but calls `GetAllKeyframes` instead of `GetKeyframesInROI`. Returns `Optional[List[Keyframe]]`.

### Phase 4: Config & Launch Wiring

- [x] **4.1** Add preview parameters to `config/prosthesis_config.yaml` under `tsdf_fusion`

  Add after line 543 (`world_frame: "marker_map"`):
  ```yaml
  # Preview mode: timer-driven scene-level TSDF fusion (no hit point / SAM needed)
  preview_mode: false
  preview_rate_hz: 1.0
  preview_output_topic: "/fused_pointcloud"
  preview_keyframe_service: "/keyframe_buffer/get_all"
  preview_workspace_bbox_min: [-0.5, -0.5, -0.2]
  preview_workspace_bbox_max: [0.8, 0.5, 0.6]
  preview_enable_bbox_crop: true
  preview_enable_dbscan_cleanup: true
  ```

  Rationale: Centralized config, loaded by launch files via `_node_params()`.

- [x] **4.2** Add `fusion_mode` launch argument to `pipeline.launch.py`

  Add a new `DeclareLaunchArgument`:
  ```python
  DeclareLaunchArgument(
      "fusion_mode",
      default_value="legacy",
      description="Fusion mode: 'legacy' (pointcloud_fusion), 'tsdf_preview' (TSDF scene preview), 'tsdf_grasp' (on-demand TSDF at grasp time)",
  ),
  ```

  In `_launch_setup()`, branch on `fusion_mode`:
  - `"legacy"`: launch `pointcloud_fusion` node (current behavior)
  - `"tsdf_preview"`: launch `tsdf_fusion` node with `preview_mode=true`, skip `pointcloud_fusion`, skip `segmentation_bridge`
  - `"tsdf_grasp"`: launch `tsdf_fusion` node with `preview_mode=false` (current `use_tsdf_fusion=true` behavior), keep `pointcloud_fusion` for live cloud, skip `segmentation_bridge`

  Rationale: Three clean modes instead of overloading `use_tsdf_fusion`. The `tsdf_preview` mode is the new target for `make run-camera`.

- [x] **4.3** Update Makefile targets to use `fusion_mode:=tsdf_preview`

  In `Makefile.workspace`, update the three targets:
  - `run` (line 116-121): add `fusion_mode:=tsdf_preview`
  - `run-camera` (line 123-128): add `fusion_mode:=tsdf_preview`
  - `run-camera-log` (line 130-138): add `fusion_mode:=tsdf_preview`

  Rationale: User wants `make run` / `make run-camera` / `make run-camera-log` to produce a TSDF-fused point cloud.

- [x] **4.4** Keep `use_tsdf_fusion` launch arg for backward compatibility

  Map `use_tsdf_fusion=true` to `fusion_mode=tsdf_grasp` if `fusion_mode` is still `legacy`. This preserves existing `make pipeline-v6` and `make start` behavior.

### Phase 5: Integration Testing

- [x] **5.1** Verify mock pipeline works with preview mode

  Host-side verification done: all Python files compile, YAML config parses,
  launch file AST is valid, and the fusion_mode resolution logic passes all 7
  test cases (including backward-compat and fallback). The Open3D-dependent
  unit tests (`test_scene_preview.py`, `test_tsdf_core.py`) skip on the host
  (no Open3D) — they run in-container. Full in-container build + `make mock-v6`
  should be run to complete end-to-end verification.

  Note: The mock cloud publisher publishes on `/camera/depth/color/points`, not `/jetson/head/points`. The mock launch may need topic remapping or the mock publishers need updating to match the keyframe buffer's expected topics. This is a pre-existing gap to verify.

- [ ] **5.2** Verify hardware pipeline with `make run-camera`

  After building, `make run-camera` should:
  1. Launch gtsam_tracker, keyframe_buffer, cross_camera_features, tsdf_fusion (preview mode)
  2. NOT launch pointcloud_fusion or segmentation_bridge
  3. After TF tree connects and keyframes accumulate, publish a fused cloud on `/fused_pointcloud` at ~1 Hz
  4. RViz should show the TSDF-fused point cloud

- [ ] **5.3** Verify downstream consumers still work

  The twist_propagation node subscribes to `/fused_pointcloud` (`config/prosthesis_config.yaml:369`). The preview cloud should be compatible (same XYZRGB PointCloud2 format, `marker_map` frame).

## Verification Criteria

- [ ] `fuse_scene_preview()` unit test passes (synthetic keyframes → non-empty fused cloud)
- [ ] `make run-camera` launches tsdf_fusion in preview mode without errors
- [ ] `/fused_pointcloud` topic publishes at ~1 Hz with non-empty clouds after keyframes accumulate
- [ ] RViz displays the TSDF-fused point cloud in `marker_map` frame
- [ ] No MobileSAM server required for preview mode
- [ ] No hit point or segmentation trigger required for preview mode
- [ ] Legacy mode (`fusion_mode:=legacy`) still works unchanged
- [ ] Grasp mode (`fusion_mode:=tsdf_grasp`) still works unchanged
- [ ] `GetAllKeyframes` service returns keyframes correctly

## Potential Risks and Mitigations

1. **Performance at 1 Hz with 100 keyframes**
   Risk: Each keyframe requires a depth rasterization + TSDF integration. With 50 keyframes/camera × 2 cameras = 100 integrations per tick, this could exceed 1 second.
   Mitigation: The spatial gate (0.10m translation, 15° rotation) means far fewer than 50 keyframes in practice. Open3D's TSDF is C++ internally. If too slow, reduce `max_keyframes_per_camera` or add a `preview_max_keyframes` parameter to limit integration count. Can also subsample keyframes (e.g., every Nth).

2. **Keyframe buffer may have no keyframes if GTSAM poses are stale**
   Risk: The keyframe buffer rejects keyframes if `pose_max_age_s` (0.05s) is exceeded (`keyframe_buffer_node.py:264-268`). If GTSAM poses lag behind clouds, no keyframes accumulate.
   Mitigation: The preview tick should log keyframe count at info level. If zero, the user can check `/keyframe_buffer/diagnostics` for rejection counts. The `pose_max_age_s` parameter can be relaxed for preview mode.

3. **Workspace bbox may exclude the object of interest**
   Risk: Default bbox `[-0.5, -0.5, -0.2]` to `[0.8, 0.5, 0.6]` may not match the actual workspace geometry.
   Mitigation: The bbox is configurable via `prosthesis_config.yaml`. Setting `preview_enable_bbox_crop: false` disables cropping entirely (integrates everything, at the cost of background noise).

4. **Mock pipeline topic mismatch**
   Risk: The mock cloud publisher publishes on `/camera/depth/color/points` but the keyframe buffer expects `/jetson/head/points` and `/jetson/arm/points`. The mock launch may not produce keyframes.
   Mitigation: Verify during Phase 5.1. May need to update mock publishers or add topic remapping in `mock.launch.py`.

5. **Depth image quality from unorganized cloud rasterization**
   Risk: `build_depth_image()` uses a z-buffer that may produce artifacts (holes, quantization) when the cloud is sparse. This could produce a noisy TSDF surface.
   Mitigation: The TSDF volume's voxel grid + truncation distance smooths over small holes. The `sdf_trunc_m` parameter (default 0.02m) controls this. Can increase truncation for smoother surfaces.

## Alternative Approaches

1. **Large-radius ROI query instead of new service**: Query `GetKeyframesInROI` with center=[0,0,0] and radius=10.0. Avoids adding a new `.srv` file. Downside: semantically wrong (abusing the ROI interface), and the keyframe buffer logs the query which would look odd.

2. **Separate preview node package**: Create a new `tsdf_preview` package instead of adding preview mode to `tsdf_fusion_node`. Downside: code duplication (deserialization, cloud building), more packages to maintain. The parameter-based approach is cleaner.

3. **Persistent TSDF volume**: Keep the volume across ticks and only integrate new keyframes. Produces an accumulating reconstruction. Downside: memory grows unbounded, drift artifacts accumulate, doesn't match the "live preview" use case. Better as a future enhancement.

4. **Direct cloud subscription (bypass keyframe buffer)**: Have the preview node subscribe directly to `/jetson/*/points` + `/gtsam/*_pose` and maintain its own minimal buffer. Downside: duplicates the keyframe buffer's spatial gating logic, doesn't exercise the V6 stack. The keyframe buffer approach is architecturally cleaner.
