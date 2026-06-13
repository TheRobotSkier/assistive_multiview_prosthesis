# Localization Rework V6 — Phase 3: TSDF Fusion + Cross-Camera Features

**Date:** 2026-06-13
**Parent plan:** `plans/2026-06-13-localization-rework-plan-v6.md`
**Depends on:** Phase 1 (all tasks), Phase 2 (both tasks)
**Blocks:** Phase 4 (integration)
**Estimated time:** 3–4 days
**Parallelism:** 2 agents (Tasks A and B are independent after their shared deps)

---

## Objective

Build the two highest-complexity components:
- **A:** `tsdf_fusion` ROS node — on-demand TSDF fusion triggered at grasp time. Calls MobileSAM, masks clouds, rasterizes depth, integrates into Open3D TSDF volume, extracts + cleans the fused cloud, publishes to `/segmentation/object_cloud`.
- **B:** `cross_camera_features` ROS node — SIFT-based 3D-3D alignment between head and arm cameras. Publishes `/vis/head_arm_pose` (PoseWithCovariance) consumed by the GTSAM tracker.

Both depend on the Phase 1 pure-logic libraries (`cloud_utils`, `se3_helpers`, `umeyama`) and are independent of each other.

---

## Task A: `tsdf_fusion` ROS Node

**Agent:** 1 (~2 days)
**Depends on:** Phase 1 Task B (`cloud_utils`), Phase 2 Task A (keyframe buffer service), Phase 2 Task B (MobileSAM server)
**Blocks:** Phase 4 (twist propagation integration)

### A.1 Package structure

```
src/tsdf_fusion/
├── tsdf_fusion/
│   ├── __init__.py
│   ├── tsdf_fusion_core.py       # Pure-logic fusion pipeline (no ROS)
│   └── tsdf_fusion_node.py       # ROS wrapper + service handler
├── srv/
│   └── TriggerGraspFusion.srv    # or define in sensor_fusion_msgs
├── test/
│   ├── test_tsdf_core.py         # Pure-logic tests with synthetic scene
│   └── test_tsdf_node.py         # ROS smoke test
├── resource/
│   └── tsdf_fusion
├── setup.py
└── package.xml
```

- [ ] Create the package skeleton following `src/pointcloud_fusion/setup.py` and `package.xml` patterns.

### A.2 Service definition

Create `TriggerGraspFusion.srv`:

```
# Request
geometry_msgs/Point hit_point           # 3D click in marker_map frame
string camera_id                        # "head" or "arm" (which camera saw the click)
float64 roi_radius                      # search radius for keyframes (default 0.15)
---
# Response
bool success
sensor_msgs/PointCloud2 object_cloud    # fused, cleaned object cloud
int32 num_points
float64 processing_time_ms
string message
```

- [ ] If `ament_python` can't generate services, define this in `sensor_fusion_msgs` (coordinate with Phase 1/2). Same decision point as the keyframe buffer service.

### A.3 Core fusion pipeline (pure logic — `tsdf_fusion_core.py`)

This is the heart of the V6 plan (§5.6, §6.5). Extract it as a pure-Python function so it's testable without ROS:

```python
def fuse_object_cloud(keyframes, hit_point_3d, K_click, pose_click,
                      sam_segment_fn, voxel_size=0.005, sdf_trunc=0.02,
                      dbscan_eps=0.02, dbscan_min_points=10,
                      hit_point_shift=0.015, mask_dilation=15):
    """
    Pure-logic TSDF fusion. No ROS imports.
    keyframes: list of Keyframe objects (from keyframe_buffer)
    hit_point_3d: (3,) hit point in world frame
    sam_segment_fn: callable(image, uv, dilation_px) -> (H,W) bool mask
    Returns: (object_cloud_xyz (M,3), object_cloud_rgb (M,3))
    """
```

- [ ] **Step 1 — Shift hit point inward** (V6 §4.4): Move the hit point 1.5cm along the camera viewing ray (toward the camera origin). This biases the segmentation toward the object interior.

- [ ] **Step 2 — Query keyframes:** Already provided as input (the node does the service call).

- [ ] **Step 3 — Initialize TSDF volume:**
  ```python
  volume = o3d.pipelines.integration.ScalableTSDFVolume(
      voxel_length=voxel_size, sdf_trunc=sdf_trunc,
      color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
  ```

- [ ] **Step 4 — Per-keyframe loop** (V6 §6.5, the critical unorganized-safe path):
  For each keyframe:
  1. Project the shifted hit point to 2D: `uv = project_3d_to_2d(hit_internal, kf.K, kf.pose)` (from `cloud_utils`).
  2. Call SAM: `mask = sam_segment_fn(kf.image, uv, mask_dilation)`.
  3. Mask the cloud — **organization-aware**:
     - If `kf.organized`: `masked_xyz = kf.cloud_xyz[mask]` (fast path).
     - Else: `keep = mask_unorganized_cloud(kf.cloud_xyz, mask, kf.K, kf.pose)` (default path from `cloud_utils`).
  4. Build depth image by rasterizing masked points: `depth = build_depth_image(masked_xyz, kf.K, kf.pose, H, W)` (from `cloud_utils`).
  5. Create RGBD image (RGB from keyframe image, depth from rasterized cloud).
  6. Integrate into TSDF: `volume.integrate(rgbd, intrinsic, inv(kf.pose))`.

- [ ] **Step 5 — Extract mesh/point cloud:**
  ```python
  mesh = volume.extract_triangle_mesh()
  cloud = o3d.geometry.PointCloud()
  cloud.points = mesh.vertices
  cloud.colors = mesh.vertex_colors
  ```

- [ ] **Step 6 — DBSCAN cleanup** (V6 §4.4):
  ```python
  labels = np.array(cloud.cluster_dbscan(eps=dbscan_eps, min_points=dbscan_min_points))
  # Keep the largest cluster near the hit point
  ```
  Remove noise (label == -1) and small clusters.

- [ ] **Step 7 — Return** the cleaned `(M, 3)` xyz + `(M, 3)` rgb arrays.

### A.4 ROS node wrapper (`tsdf_fusion_node.py`)

- [ ] **Parameters** (V6 §6.9):
  - `keyframe_service: "/keyframe_buffer/get_in_roi"`
  - `sam_inference_url: "http://127.0.0.1:5679"`
  - `roi_radius_m: 0.15`, `hit_point_shift_m: 0.015`, `mask_dilation_px: 15`
  - `voxel_size_m: 0.005`, `sdf_trunc_m: 0.02`
  - `dbscan_eps_m: 0.02`, `dbscan_min_points: 10`
  - `output_topic: "/segmentation/object_cloud"`

- [ ] **SAM client:** HTTP client (using `requests` or `urllib`) that calls the MobileSAM server's `/segment_2d` endpoint. Wrap it as a `sam_segment_fn(image, uv, dilation_px) -> mask` callable that `fuse_object_cloud` expects.

- [ ] **Service handler:**
  - On `TriggerGraspFusion` request:
    1. Call the keyframe buffer service `GetKeyframesInROI` with the hit point + radius.
    2. Deserialize returned keyframes into `Keyframe` objects.
    3. Call `fuse_object_cloud(...)` with the SAM client as the segmentation function.
    4. Convert the result to `sensor_msgs/PointCloud2`.
    5. Publish to `/segmentation/object_cloud` AND return in the service response.

- [ ] **Health check:** On startup, ping the MobileSAM server `/health`. Log a warning if unavailable (the node can still start; it will fail at trigger time if SAM is down).

- [ ] **Publisher:** `/segmentation/object_cloud` — this is the topic `pipeline_manager_node.py:243` subscribes to (V6 §5.1). Must use the same message type (`sensor_msgs/PointCloud2`).

### A.5 Tests

- [ ] **`test/test_tsdf_core.py` — synthetic scene test (the most important test):**
  This is the offline validation strategy. Build a synthetic scene:
  1. Create a known 3D object (e.g., a cylinder or box at a known position).
  2. Generate 5–10 synthetic keyframes with known poses around the object.
  3. For each keyframe, render the object into a synthetic cloud + image (use simple projection, no OpenGL needed — just project the object's surface points).
  4. Provide a mock `sam_segment_fn` that returns a ground-truth mask (the projected object silhouette).
  5. Call `fuse_object_cloud(...)`.
  6. **Verify:** the fused cloud recovers the object geometry within 2cm RMS of the ground-truth surface.

  This test validates the entire fusion pipeline (masking, depth rasterization, TSDF integration, DBSCAN) without ROS, without SAM, and without hardware. If it passes, the pipeline logic is sound.

- [ ] **Edge case tests:**
  - Empty keyframe list → returns empty cloud, success=False.
  - All keyframes have no points in front of camera → empty result.
  - Hit point outside all keyframe FOVs → graceful handling.

- [ ] **ROS smoke test** (in-container):
  - Launch tsdf_fusion + keyframe_buffer + mock SAM server.
  - Call `TriggerGraspFusion` with a synthetic hit point.
  - Verify a PointCloud2 is published on `/segmentation/object_cloud`.

### A.6 Build and verify

- [ ] `make build-pkg PKG=tsdf_fusion` succeeds.
- [ ] `python3 -m pytest src/tsdf_fusion/test/test_tsdf_core.py -v` passes (the synthetic scene test).
- [ ] Node launches and responds to the service in-container.

---

## Task B: `cross_camera_features` ROS Node (SIFT)

**Agent:** 1 (independent, ~1.5 days)
**Depends on:** Phase 1 Task B (`cloud_utils`), Phase 1 Task C (`se3_helpers`, `umeyama`)
**Blocks:** Phase 4 (GTSAM tracker consumes `/vis/head_arm_pose`)

### B.1 Package structure

```
src/cross_camera_features/
├── cross_camera_features/
│   ├── __init__.py
│   └── sift_feature_node.py     # Main node
├── test/
│   └── test_sift_features.py    # Logic tests
├── resource/
│   └── cross_camera_features
├── setup.py
└── package.xml
```

### B.2 Node implementation

Implement `sift_feature_node.py` following V6 plan §6.6:

- [ ] **Parameters** (V6 §6.9):
  - `head_image_topic`, `arm_image_topic`
  - `head_cloud_topic`, `arm_cloud_topic`
  - `head_info_topic`, `arm_info_topic`
  - `process_rate_hz: 5.0` (V6 correction: process EVERY frame, not every 6th-10th)
  - `sync_slop_s: 0.05`
  - `min_matches: 5`
  - `backend: "sift"`

- [ ] **Synchronization** (V6 §5.8): Use `message_filters.ApproximateTimeSynchronizer` with 50ms slop to sync head+arm image pairs. Register a `TimeSynchronizer` callback.

  > **Note:** Unlike the keyframe buffer (which must NOT sync), this node intentionally syncs head+arm pairs because it needs corresponding features for 3D-3D alignment.

- [ ] **SIFT extraction + matching** (V6 §6.6):
  ```python
  self._sift = cv2.SIFT_create()
  self._matcher = cv2.BFMatcher(cv2.NORM_L2)
  ```
  - Detect + compute on both images.
  - Match descriptors, sort by distance, keep top 100.
  - Apply Lowe's ratio test if using knnMatch (optional, improves quality).

- [ ] **Depth lookup — organization-aware** (V6 §6.6, uses `cloud_utils.lookup_depth_3d`):
  For each match, look up the 3D point at the keypoint pixel in both cameras:
  - Organized: `cloud[v, u]` directly.
  - Unorganized (default): find the nearest 3D point to the ray through pixel `(u, v)` using projection. This is the V6-critical path.

- [ ] **Umeyama alignment** (V6 §3.1, uses Phase 1 Task C `umeyama.py`):
  - Collect matched 3D point pairs `(p_head, p_arm)`.
  - If `len(valid_matches) >= min_matches`: run `umeyama(src_points, dst_points)` to get `T_head_arm` + covariance.
  - Publish as `geometry_msgs/PoseWithCovariance` on `/vis/head_arm_pose`.

- [ ] **Publisher:** `/vis/head_arm_pose` — consumed by the GTSAM tracker as a visual between-factor.

- [ ] **Diagnostics:** Log match count per frame. If consistently < `min_matches`, log a warning suggesting SuperPoint upgrade.

### B.3 Tests

- [ ] **`test/test_sift_features.py` — synthetic image pair test:**
  1. Generate a synthetic textured image (random pattern or known features).
  2. Create a second image by applying a known small homography (simulating a different viewpoint).
  3. Run SIFT detect + match on the pair.
  4. Verify match count > 0 and that matched points are geometrically consistent.
  5. Test the depth-lookup logic with a synthetic unorganized cloud + known intrinsics.

- [ ] **Umeyama integration test:**
  1. Generate known 3D point pairs with a known rigid transform + small noise.
  2. Run `umeyama()`.
  3. Verify recovered transform matches ground truth within tolerance.

- [ ] **Edge cases:**
  - No matches found → node does not publish (no crash).
  - All depth lookups fail (points behind camera) → no publish.

### B.4 Build and verify

- [ ] `make build-pkg PKG=cross_camera_features` succeeds.
- [ ] `python3 -m pytest src/cross_camera_features/test/test_sift_features.py -v` passes.
- [ ] Node launches in-container with mock data (mock.launch.py provides synced image pairs).

---

## Verification Gate

Phase 3 is complete when ALL of the following are true:
1. `tsdf_fusion` core logic passes the synthetic scene test (object recovered within 2cm RMS).
2. `tsdf_fusion` node responds to `TriggerGraspFusion` and publishes `/segmentation/object_cloud`.
3. `cross_camera_features` node extracts SIFT matches and publishes `/vis/head_arm_pose` when matches are sufficient.
4. SIFT depth lookup works on unorganized clouds (the default path).
5. Umeyama alignment recovers known transforms within tolerance.
6. All unit tests pass for both packages.
7. Both nodes build and launch in-container.

---

## Dependency Graph

```
Phase 1 Task B (cloud_utils) ──┬──► Task A (tsdf_fusion)
                               │
                               └──► Task B (SIFT node)

Phase 1 Task C (se3/umeyama) ──┤
                               │
Phase 2 Task A (keyframe buf) ─┤
                               │
Phase 2 Task B (MobileSAM) ────┘

Task A and Task B are independent of each other.
```

---

## Risk Notes

- **SIFT match quality across viewpoints:** Head and arm cameras have very different viewpoints of an object. SIFT may fail to find enough matches. This is a known risk (V6 §9). The node must degrade gracefully (no publish) and log diagnostics. The SuperPoint upgrade path (V6 §6.7) is the fallback, but is NOT in scope for this phase.
- **Unorganized depth lookup is approximate:** For unorganized clouds, looking up the 3D point at a SIFT keypoint pixel requires finding the nearest point to the viewing ray. This is less precise than organized `cloud[v,u]`. Acceptable for the visual between-factor (which has its own noise model), but may introduce error. Mitigation: use a small search radius and reject points that are too far from the ray.
- **TSDF integration with sparse clouds:** The rasterized depth image from an unorganized cloud will have holes (pixels with no projected point). Open3D's TSDF integration handles this by simply not integrating those pixels, but the resulting mesh may be incomplete. This is expected and acceptable — multi-view fusion fills gaps.
- **SAM server latency:** If the MobileSAM server is slow (>300ms for 20 keyframes), the grasp trigger will be delayed. Mitigation: reduce `max_keyframes_per_camera` or skip keyframes with poor viewpoint angles.
- **The synthetic scene test is the key de-risking artifact.** If `test_tsdf_core.py` passes, the fusion math is validated. The remaining risk is purely in the ROS plumbing and live data quality, both of which are addressed in Phase 4.