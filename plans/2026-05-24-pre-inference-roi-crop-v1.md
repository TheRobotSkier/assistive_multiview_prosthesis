# Pre-Inference ROI Crop: 30cm Sphere Filter for Segmentation

## Objective

Add a configurable spherical region-of-interest (ROI) crop in the segmentation bridge node that filters the fused point cloud to only include points within a configurable radius (default 30cm) of the click centroid before sending to the inference server. This reduces inference input from ~50K-100K points to ~5K-15K points, cutting inference latency by an estimated 3-5x and reducing HTTP payload size proportionally.

## Analysis

### Current Performance Baseline

From `camera-test-log-v6.txt`, typical inference cycles show:

| Cloud Size | Inference Time | Latency |
|-----------|---------------|---------|
| ~50K points | ~0.6-0.7s | Full cloud processed |
| ~70K points | ~0.7-0.8s | Full cloud processed |
| ~90K points | ~0.8-1.0s | Full cloud processed |
| ~120K points | ~1.0-1.5s | Full cloud processed |

The bottleneck is `_inseg.prediction()` at `src/segmentation/nodes/inference_server.py:134` — the MinkowskiEngine sparse 3D CNN forward pass scales roughly linearly with point count. Currently the **entire** fused cloud is sent to the server regardless of where the click is.

### Why 30cm Radius Is Appropriate

- The click centroid is the user's (or twist propagation's) intended target object
- InterObject3D segments objects using local geometry context — it doesn't need distant points
- The workspace is within arm's reach (~0.5-1.0m from cameras); a 30cm sphere captures the target object plus ample surrounding context for boundary discrimination
- The existing `cubeedge` parameter (5cm) defines the click interaction radius; 30cm provides 6x margin for context

### Where to Implement

**Best location: `segmentation_ros2_node.py` `_run_inference()` method** (`src/segmentation/segmentation_bridge/segmentation_ros2_node.py:234-281`)

Reasons:
1. This is where the cloud is snapshot and prepared for inference — the crop naturally fits between the snapshot (line 239-243) and payload construction (line 253-258)
2. The click locations are already available in `pos_clicks`/`neg_clicks` lists at this point
3. No changes needed to the fusion node, inference server, or launch files
4. The crop is purely a segmentation optimization — it doesn't affect the fused cloud published for RViz/twist propagation
5. The inference server remains generic — it still accepts arbitrary clouds

### Estimated Impact

| Metric | Before | After (30cm crop) | Improvement |
|--------|--------|-------------------|-------------|
| Points to inference | 50K-100K | ~5K-15K | 5-10x reduction |
| Inference time | 0.6-1.5s | ~0.1-0.3s | 3-5x faster |
| HTTP payload | ~2-3 MB | ~200-500 KB | 5-10x smaller |
| Segmentation quality | Baseline | Similar or better | Less noise from distant points |

---

## Implementation Plan

### Phase 1: Add ROI Crop to Segmentation Bridge

- [ ] **1.1** Add a `roi_radius_m` parameter to `SegmentationNode.__init__()` in `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:110-145`
  - Declare with `self.declare_parameter("roi_radius_m", 0.3)` (default 30cm)
  - Store as `self._roi_radius` in the constructor
  - Add validation: if `roi_radius_m <= 0`, disable the crop (pass full cloud through)
  - Rationale: A parameter allows tuning without code changes; 0 or negative disables the feature

- [ ] **1.2** Add a `_crop_to_roi()` helper method to `SegmentationNode` in `src/segmentation/segmentation_bridge/segmentation_ros2_node.py` (add after `_transform_click_to_cloud_frame` around line 167)
  - Signature: `def _crop_to_roi(self, xyz, rgb, pos_clicks, neg_clicks, radius) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]`
  - Compute the centroid of all positive clicks: `centroid = np.mean(pos_clicks, axis=0)`
  - Compute squared distances from every point to the centroid: `dists_sq = np.sum((xyz - centroid) ** 2, axis=1)`
  - Create a boolean mask: `mask = dists_sq <= radius ** 2`
  - Also keep all points within `cubeedge` of any negative click (so negative click constraints are preserved): extend mask to include those points too
  - Return cropped `xyz`, `rgb`, plus an index mapping array (to map the inference mask back to full-cloud indices)
  - Rationale: Pure numpy operation, O(N) time, negligible compared to inference

- [ ] **1.3** Modify `_run_inference()` in `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:234-281` to apply the ROI crop
  - After snapshotting state (lines 235-243) and before building the payload (line 253):
    - If `self._roi_radius > 0` and `pos_clicks` is non-empty, call `_crop_to_roi()`
    - Use the cropped xyz/rgb for the HTTP payload
    - After receiving the inference mask back, map it to the full cloud indices: create a full-size boolean mask initialized to False, then set the cropped indices where the inference mask is True
  - If `pos_clicks` is empty (edge case: only negative clicks), skip the crop and send the full cloud
  - Log the crop ratio: `"ROI crop: {len(cropped_xyz)}/{len(xyz)} points within {radius}m of click centroid"`
  - Rationale: The mapping step ensures downstream consumers (twist propagation, RViz) see a consistent full-cloud segmentation result

- [ ] **1.4** Update the startup log message at `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:144-145` to include the ROI radius
  - Change to: `f"Segmentation node ready. debounce={self._debounce_s}s, roi_radius={self._roi_radius}m"`
  - Rationale: Makes it immediately visible in logs whether ROI cropping is active

### Phase 2: Configuration

- [ ] **2.1** Add `roi_radius_m` parameter to the segmentation section in `config/prosthesis_config.yaml`
  - Add under a new `segmentation_bridge` section (or extend the existing `_reference.segmentation` section at lines 338-340):
    ```yaml
    segmentation_bridge:
        ros__parameters:
            roi_radius_m: 0.3
    ```
  - Rationale: Makes the parameter visible and tunable in the central config file

- [ ] **2.2** Pass the parameter through `pipeline.launch.py` at `src/prosthesis_launch/launch/pipeline.launch.py:297-306`
  - Add `roi_radius_m` to the segmentation bridge node's `parameters` list, reading from the config file alongside the existing `inference_url` parameter
  - Rationale: Ensures the config value reaches the node at launch time

### Phase 3: Testing

- [ ] **3.1** Add unit tests for `_crop_to_roi()` in `src/segmentation/segmentation_bridge/test/test_segmentation_coalescing.py`
  - Test 1: Points within radius are kept, points outside are excluded
  - Test 2: Negative click points within `cubeedge` are always preserved even if outside radius
  - Test 3: Empty positive clicks list returns full cloud (no crop)
  - Test 4: Radius=0 returns full cloud (disabled)
  - Test 5: Verify index mapping correctly maps cropped mask back to full cloud
  - Rationale: Ensures the crop logic is correct and the mask remapping preserves segmentation fidelity

- [ ] **3.2** Add an integration-style test to verify end-to-end mask remapping
  - Create a synthetic cloud with known foreground/background points
  - Verify that after crop + inference + remap, the full-cloud mask has the correct True/False values at the original indices
  - Rationale: The index remapping is the trickiest part — a dedicated test prevents regressions

### Phase 4: Verification

- [ ] **4.1** Rebuild the segmentation_bridge package: `colcon build --packages-select segmentation_bridge`

- [ ] **4.2** Run `make camera-test` and verify:
  - Log shows `"ROI crop: X/Y points within 0.3m of click centroid"` before each inference call
  - Inference times are measurably reduced (compare timestamps of "Running inference" vs "Published segmented cloud")
  - Segmentation quality is maintained — foreground point counts are similar to pre-crop baseline
  - No errors in segmentation bridge or inference server logs

- [ ] **4.3** Compare before/after inference latency from camera-test logs:
  - Before: typical 0.6-1.5s per inference
  - After: target <0.3s per inference
  - Verify segmented cloud sizes are similar (the model should produce comparable results with less input noise)

---

## Verification Criteria

1. **Inference latency reduced by >= 2x** — measured from log timestamps between "Running inference" and "Published segmented cloud"
2. **Segmentation quality preserved** — foreground point counts within 20% of baseline for similar scenes
3. **ROI crop logged** — every inference call logs the crop ratio
4. **No regression in twist propagation** — segmented cloud triggers preshaping correctly, no "null pointer" errors
5. **Feature is toggleable** — setting `roi_radius_m: 0` disables the crop entirely

## Potential Risks and Mitigations

1. **Crop removes context needed for accurate segmentation**
   - The InterObject3D model uses surrounding geometry to determine object boundaries. If the target object is large (>30cm) or the boundary requires distant context, quality could degrade.
   - Mitigation: The 30cm default is conservative. The parameter is tunable — increase to 0.5 or disable if quality drops. Start with 0.3 and validate empirically.

2. **Negative clicks outside the ROI are lost**
   - If a user places a negative click >30cm from the positive click centroid, the negative click's nearby points may be cropped away, reducing the model's ability to distinguish background.
   - Mitigation: The `_crop_to_roi()` method explicitly preserves all points within `cubeedge` of any negative click, regardless of distance from centroid.

3. **Index remapping bugs cause wrong points in output cloud**
   - The mask remapping (cropped indices → full cloud) is the most error-prone step. A bug would publish wrong points as the segmented object.
   - Mitigation: Dedicated unit tests (Phase 3.1, 3.2) cover the remapping logic. The output cloud size is logged and can be compared to baseline.

4. **Edge case: clicks with no nearby points**
   - If the click centroid has zero points within 30cm (e.g., click in empty space), the crop returns an empty cloud, inference returns no foreground, and an empty cloud is published.
   - Mitigation: This is the same behavior as the current system (click in empty space → no segmentation). No special handling needed.

## Alternative Approaches

1. **Crop in the inference server instead of the bridge**
   - Pros: No changes to ROS node, server handles its own optimization
   - Cons: Still sends full cloud over HTTP (network overhead unchanged), server becomes stateful about crop logic
   - Trade-off: Less network savings, but simpler bridge code. Rejected because the primary goal is reducing both network transfer and inference time.

2. **Crop in the fusion node (add a configurable ROI around a tracked point)**
   - Pros: Reduces the fused cloud size for all consumers (RViz, twist propagation, segmentation)
   - Cons: The ROI center is not known at fusion time (it depends on where the click happens). Would require a dynamic ROI that changes per-click, adding complexity to the fusion node.
   - Trade-off: More impactful but architecturally wrong — the fusion node shouldn't know about segmentation clicks. Rejected.

3. **Crop in a new relay node between fusion and segmentation**
   - Pros: Clean separation of concerns, doesn't modify existing nodes
   - Cons: Adds another node to the launch graph, another subscription/publication hop (adds latency), more inter-process data transfer
   - Trade-off: Cleaner architecture but worse performance. Rejected in favor of the simpler in-bridge approach.

4. **Use a fixed workspace bounding box instead of a click-centered sphere**
   - Pros: Simpler — no need to compute centroid, no index remapping
   - Cons: The workspace is already partially filtered by the fusion node's distance filter (2m) and bbox removal. A fixed box wouldn't adapt to click location. The sphere approach is more targeted.
   - Trade-off: Less adaptive. Could be combined with the sphere approach as an additional filter if needed.
