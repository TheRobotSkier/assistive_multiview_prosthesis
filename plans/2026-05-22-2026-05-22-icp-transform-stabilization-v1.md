# ICP-Based Point Cloud Transform Stabilization

## Objective

Eliminate visible point cloud "jumping" by using Iterative Closest Point (ICP) registration to refine the TF-based transform before merging clouds. The approach uses a lightweight numpy/scipy ICP implementation (no new dependencies) that aligns each incoming cloud against a high-confidence reference, correcting for OpenVINS TF jitter at ~2 Hz update rate.

## Root Cause Recap

The TF chain `marker_map -> *_imu -> *_cam0 -> *_link -> *_depth_optical_frame` depends on OpenVINS tracking published at only ~2 Hz (liveness mode). The fusion node processes at 15 Hz. This causes:

1. **Transform jumps** — When a new OpenVINS update arrives, the transform can differ significantly from the previous frame, shifting the entire fused cloud.
2. **Intermittent TF disconnection** — ~80s cold start with no TF chain at all.
3. **Bbox removal 0% success** — Pruning box frames (`palm_frame`, `screw_frame`) never available.

The prior plan (`plans/2026-05-22-pointcloud-transform-stabilization-v1.md`) proposed EMA smoothing. This plan proposes a more principled alternative: use ICP to compute a geometric correction to the TF transform, anchoring each cloud to the scene geometry rather than trusting the noisy TF chain blindly.

## Why ICP Is Now Appropriate

The prior decision to reject ICP (`plans/2026-05-19-pointcloud_fusion_implementation-v2.md:37`) was based on the assumption that "OpenVINS + ChArUco markers provide accurate TF chain." The test logs (v5, v6) prove this assumption is wrong in practice — the chain is intermittent, noisy, and operates at 2 Hz. ICP provides a geometric ground truth that is independent of the TF chain quality.

## Implementation Plan

### Phase 1: Lightweight ICP Transform Refinement (Core Solution)

- [ ] **Implement a point-to-point ICP function using only numpy + scipy (no new dependencies).** Use `scipy.spatial.KDTree` for nearest-neighbor correspondence (already available in the main container per `docker/Dockerfile:20`) and SVD-based rigid transform estimation. Target: ~5-15 iterations on ~5K-10K downsampled points. Rationale: This is the core alignment engine. A numpy-only implementation avoids adding open3d as a dependency. The SVD solution for optimal rigid transform is a well-known closed-form (Arun et al. 1987) — compute centroid, center points, build cross-covariance matrix, SVD, extract R and t.

- [ ] **Add a pre-alignment step using the current TF transform as the ICP initial guess.** The TF transform provides a coarse alignment (typically within a few cm of correct). ICP then refines it. This is critical — ICP only converges from a reasonable initial guess, and the TF transform provides exactly that. Rationale: Without a good initial guess, ICP can converge to a local minimum. The TF chain is noisy but not wildly wrong — it provides a sufficient initialization.

- [ ] **Integrate ICP refinement into `_process_clouds` at `pointcloud_fusion_node.py:416-438` (Step 1).** After the TF-based transform succeeds, run ICP against a reference cloud to compute a correction. The corrected transform is used instead of the raw TF transform. If ICP fails (low overlap, divergence), fall back to the raw TF transform. Rationale: This is the insertion point where transforms are applied to clouds. Adding ICP here means all downstream steps (concatenation, filtering, downsampling) operate on correctly aligned data.

- [ ] **Add a "reference cloud" mechanism.** Maintain a high-confidence cloud in `marker_map` frame that serves as the ICP target. The reference is updated only when confidence is high (both cameras present, TF chain healthy, ICP convergence good). On first frame or after TF reconnect, the first successfully transformed cloud becomes the reference. Rationale: ICP needs a stable target to align against. A continuously-updated reference would drift. Updating only on high-confidence frames ensures the reference stays anchored to the true scene geometry.

- [ ] **Add ICP convergence quality metrics.** Track: (a) mean squared error (MSE) before/after ICP, (b) fraction of points with correspondences within threshold, (c) transform delta from initial guess. Use these to gate whether the ICP result is accepted or the raw TF is used. Rationale: Not all frames will have good ICP convergence (e.g., scene change, occlusion). Quality metrics allow graceful fallback.

- [ ] **Add parameters for ICP configuration:** `icp_enabled` (bool, default True), `icp_max_iterations` (int, default 15), `icp_correspondence_threshold_m` (float, default 0.03), `icp_convergence_mse` (float, default 1e-6), `icp_min_overlap_ratio` (float, default 0.3), `icp_max_correction_m` (float, default 0.05). Rationale: ICP behavior needs to be tunable per deployment. The max_correction parameter prevents ICP from applying large corrections that could indicate a bad registration rather than a TF error.

### Phase 2: Confidence-Based Reference Management

- [ ] **Implement confidence scoring for the reference cloud.** Score based on: (a) number of contributing cameras (dual > single), (b) TF chain health at time of capture, (c) ICP convergence quality when the reference was created/updated, (d) age of the reference. Rationale: The reference cloud quality directly determines ICP alignment quality. Tracking confidence allows the system to know when the reference is reliable and when it needs replacement.

- [ ] **Add reference cloud update strategy.** When a new frame has higher confidence than the current reference (or the reference is older than a threshold), replace the reference with the new frame's downsampled cloud. Use a weighted blend if both are high-confidence to reduce noise. Rationale: The reference must track slow scene changes (objects moving) while remaining stable enough for ICP to work. A confidence-gated update strategy balances these needs.

- [ ] **Store the reference cloud downsampled to a coarser voxel size (e.g., 1cm vs 5mm).** This reduces ICP computation while preserving enough geometry for alignment. The reference is ~2K-5K points at 1cm voxel size. Rationale: ICP runs every frame at 15 Hz. A smaller reference cloud means faster KDTree queries. 1cm resolution is sufficient for rigid alignment since we're correcting cm-scale TF errors.

- [ ] **Add reference invalidation on scene change detection.** If ICP consistently fails (low overlap, high MSE) for multiple consecutive frames, invalidate the reference and rebuild from the next high-confidence frame. Rationale: Large scene changes (camera moved, major object rearrangement) make the old reference useless. Detecting this prevents the system from forcing bad alignments.

### Phase 3: TF Health-Aware Fallback Chain

- [ ] **Implement a three-tier transform strategy: ICP-corrected > raw TF > last-known-good.** When ICP converges well, use the corrected transform. When ICP fails but TF is fresh, use raw TF. When TF is stale/disconnected, use the last known good transform (cached from the last successful ICP+TF frame). Rationale: This provides graceful degradation. The system always has a best-effort transform available, preventing the "no output for 80s" problem seen in test logs.

- [ ] **Cache the last successful ICP-corrected transform per camera.** Store (R_corrected, t_corrected, timestamp, confidence) for each camera. When TF drops entirely, apply the cached transform to the incoming cloud (which is still in camera optical frame). Rationale: Even when the TF chain is disconnected, the cameras are still producing depth data. The cached transform from the last good frame is a better estimate than nothing — the scene hasn't moved, only the TF tracking was lost.

- [ ] **Add a `transform_max_age_s` parameter (default 1.0).** When the cached transform is older than this, stop publishing rather than serving very stale data. Log a warning. Rationale: A very stale transform could place the cloud far from reality. Better to drop frames than publish garbage.

### Phase 4: Marker Frame Anchoring (Optional Enhancement)

- [ ] **Use ChArUco marker detection frames as "ground truth" anchors.** When the ChArUco board is visible in a camera image (detected by `charuco_tf_node.py`), the resulting pose is highly accurate. Use these frames to establish the reference cloud and validate ICP corrections. Rationale: The ChArUco detection at `src/camera/camera/charuco_tf_node.py` already computes camera-to-board poses. These are more accurate than OpenVINS odometry for the frames where the board is visible. Using them as anchors gives ICP a known-correct starting point.

- [ ] **Publish a diagnostic topic `/pointcloud_fusion/icp_diagnostics`** with: ICP enabled, convergence status, MSE, correction delta, reference age, reference confidence, fallback mode. Rationale: Real-time diagnostics allow tuning ICP parameters and detecting when the system is operating in degraded mode.

## Latency Budget Analysis

| Step | Current | With ICP | Delta |
|------|---------|----------|-------|
| TF lookup (Step 1) | ~0.1 ms | ~0.1 ms | 0 |
| **ICP on downsampled cloud** | — | **~2-5 ms** | +2-5 ms |
| Cloud parsing | ~1-3 ms | ~1-3 ms | 0 |
| Distance filter | ~0.5 ms | ~0.5 ms | 0 |
| Bbox removal | ~1-2 ms | ~1-2 ms | 0 |
| Voxel downsampling | ~2-5 ms | ~2-5 ms | 0 |
| Cloud rebuild | ~1-2 ms | ~1-2 ms | 0 |
| **Total** | ~6-13 ms | **~8-18 ms** | **+2-5 ms** |

ICP computation breakdown (per frame, ~5K reference points, ~10K source points):
- KDTree build (reference): ~0.5 ms (amortized if reference unchanged)
- 15 ICP iterations × KDTree query: ~1.5-3 ms
- SVD per iteration: ~0.1 ms × 15 = ~1.5 ms
- Total: ~2-5 ms

At 15 Hz fusion rate, the frame budget is 66 ms. Adding 2-5 ms for ICP keeps total well under budget.

## Verification Criteria

- [ ] Fused point cloud shows no visible jumping during stable OpenVINS operation
- [ ] Fused point cloud shows significantly reduced jumping during OpenVINS transitions (loss/recovery)
- [ ] ICP convergence rate > 80% during normal operation (measured via diagnostics topic)
- [ ] End-to-end latency increases by no more than 5 ms (measured via header stamp delta)
- [ ] Published cloud rate remains at ~15 Hz
- [ ] When ICP is disabled (`icp_enabled=False`), behavior is identical to current implementation
- [ ] System degrades gracefully when TF chain is disconnected (publishes cached-transform clouds, not nothing)

## Potential Risks and Mitigations

1. **ICP converges to wrong local minimum**
   Mitigation: Use TF transform as initial guess (already close). Add `icp_max_correction_m` parameter to reject large corrections. Fall back to raw TF when ICP quality metrics are poor.

2. **ICP adds too much latency on large clouds**
   Mitigation: Run ICP on pre-downsampled clouds (1cm voxel) rather than full resolution. Typical ICP input: ~5K-10K points. KDTree on 5K points is sub-millisecond.

3. **Reference cloud drifts over time**
   Mitigation: Confidence-gated updates with overlap checking. Invalidate reference after consecutive ICP failures. The ChArUco anchor in Phase 4 provides periodic ground truth resets.

4. **Scene changes cause reference mismatch**
   Mitigation: Detect via ICP overlap ratio and MSE spike. Auto-invalidate and rebuild reference from next high-confidence frame.

5. **scipy KDTree is slower than open3d's FLANN**
   Mitigation: For 5K-10K points, scipy KDTree is adequate (~0.1 ms per query). If profiling shows this is a bottleneck, consider `cKDTree` (C implementation, already in scipy) or batch queries.

6. **ICP correction fights with bbox removal**
   Mitigation: Apply ICP correction only to the main cloud transform (Step 1). Bbox transforms use raw TF lookups as before. The ICP correction is small (mm-scale refinement) and should not significantly affect bbox positioning.

## Alternative Approaches

1. **EMA smoothing only (prior plan Phase 1):** Simpler (~0.1 ms overhead) but purely reactive — doesn't use scene geometry. Works for small jitter but cannot correct systematic TF errors. Trade-off: less computation, less accurate.

2. **Hybrid EMA + ICP:** Use EMA as a fast smoother for small frame-to-frame variations, and ICP only when the EMA detects a large jump (> max_jump threshold). This reduces ICP invocations to ~2 Hz (matching OpenVINS update rate) while still catching the large jumps. Trade-off: more complex logic, but lower average compute.

3. **Open3D ICP in segmentation container:** Route clouds through the segmentation container (which already has open3d) for ICP, then back. Trade-off: adds HTTP round-trip latency (~10-20 ms), architectural complexity, and couples fusion to segmentation availability.

4. **Increase OpenVINS bridge rate from 2 Hz to 15 Hz:** Instead of fixing alignment, make the source more reliable. Trade-off: requires Jetson-side changes, more network bandwidth, doesn't help during tracking loss.

## Recommended Implementation Order

**Start with Phase 1** (ICP core + reference cloud). This is the highest-impact change — it directly corrects the transform error using scene geometry. The implementation is self-contained in `pointcloud_fusion_node.py` with no new dependencies.

**Then Phase 3** (fallback chain) to handle the ~80s cold start and intermittent TF disconnection. This addresses the "zero output" problem.

**Phase 2** (confidence management) can be added incrementally once Phase 1 is validated in testing.

**Phase 4** (ChArUco anchoring) is an optimization that can be deferred unless ICP-only alignment proves insufficient.

## Dependencies

- **No new packages required.** `scipy.spatial.KDTree` (or `cKDTree`) is already installed in the main container (`docker/Dockerfile:20`). `numpy` is already used throughout. SVD is available via `numpy.linalg.svd`.
- **Modified files:** Only `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` and its test file.
- **New parameters:** All ICP parameters declared via `declare_parameter()` — backward compatible with existing config files.
