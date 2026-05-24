# Improve Point Cloud Fusion: Single-Camera Robustness & Temporal Accumulation

## Objective

Improve the point cloud fusion node to produce a richer, more complete environment representation for grasp planning, even when only one camera is available. The current single-camera fallback produces a sparse, single-viewpoint cloud that limits grasp planning quality. A few centimeters of spatial uncertainty in a denser cloud is preferable to a precise but incomplete cloud.

## Analysis

### Current Behavior

The fusion node (`pointcloud_fusion_node.py`) has two modes:
1. **Synchronized mode** (`require_both=True`): Only publishes when both camera clouds arrive within sync tolerance. Blocks entirely if one camera is down.
2. **Timer-based fallback** (`require_both=False`, default): Every ~67ms (15Hz), merges whichever clouds are fresh (<0.5s old). If only one camera is available, it publishes that single cloud.

**Pruning works correctly with single-camera**: The bounding box removal (Step 4 at `pointcloud_fusion_node.py:462-480`) transforms points to each pruning box frame and filters. This is frame-agnostic — it doesn't matter how many cameras contributed. With the camera mount TF fix, pruning works for both single and dual camera cases.

**The real problem**: A single-viewpoint cloud has:
- Severe occlusion — objects behind other objects are invisible
- Limited field of view — the arm camera sees ~87° × 58° (D435i), missing lateral context
- No redundancy — noise in one point can't be averaged away
- Poor grasp planning input — the planner needs to see object geometry from multiple sides to plan approach vectors

### Why Temporal Accumulation Helps

When the cameras are moving (mounted on the hand/head during operation), each frame provides a slightly different viewpoint. Accumulating clouds over a short time window (1-3 seconds) creates a much denser, multi-view representation:

- **Occlusion filling**: As the camera moves, previously hidden surfaces become visible
- **Noise reduction**: Voxel downsampling averages overlapping points
- **Better geometry**: More surface coverage means better normal estimation for grasp planning
- **Graceful degradation**: Even with one camera, you get multi-view coverage over time

The trade-off is **temporal staleness** — points from 2 seconds ago may not reflect the current scene. But for grasp planning:
- Objects don't move fast (typically stationary targets)
- The hand approach vector is computed from current pose, not from the cloud
- A slightly stale but complete object model is better than a fresh but partial one

### Key Design Decision

The user explicitly stated: "A few centimeter uncertainty in a better pointcloud is better than a precise, but one-view point cloud." This validates the temporal accumulation approach.

## Implementation Plan

### Phase 1: Add temporal accumulation to the fusion node

- [ ] **1.1** Add a rolling accumulation buffer to `PointCloudFusionNode`. Store the last N seconds of transformed+filtered XYZ+RGB points in a deque (timestamped). New parameter `accumulation_window_s` (default: 2.0) controls the window. Rationale: Provides multi-view coverage from a single moving camera.

- [ ] **1.2** Modify `_process_clouds` to insert the transformed+filtered points into the accumulation buffer instead of publishing immediately. After insertion, prune entries older than `accumulation_window_s`. Rationale: Decouples accumulation from individual cloud processing.

- [ ] **1.3** Add a publish step that concatenates all points in the accumulation buffer, runs voxel downsampling on the combined cloud, and publishes. This should happen in the `_timer_merge` callback (or a separate timer). Rationale: Voxel downsampling on the accumulated cloud naturally handles duplicate points from overlapping views and reduces noise through averaging.

- [ ] **1.4** Add parameters:
  - `accumulation_window_s` (float, default 2.0): How far back to keep points
  - `enable_accumulation` (bool, default True): Toggle for A/B comparison
  Rationale: Makes the feature configurable and easy to disable if it causes issues.

### Phase 2: Improve single-camera quality

- [ ] **2.1** When only one camera is available, increase the effective accumulation window (e.g., 3s instead of 2s) to compensate for the reduced per-frame coverage. This could be automatic based on how many cameras contributed in the last window. Rationale: Single camera needs more temporal diversity to match dual-camera coverage.

- [ ] **2.2** Add a diagnostic log showing accumulation stats: buffer size, time span, number of source frames. This helps tune the window parameter. Rationale: Visibility into the accumulation behavior for debugging.

### Phase 3: Handle staleness for dynamic scenes

- [ ] **3.1** Add a per-voxel freshness weight. When downsampling the accumulated cloud, weight newer points more heavily in the centroid computation. This gives a soft blend between fresh (precise) and stale (better coverage) data. Rationale: Prevents ghost artifacts from moved objects while keeping coverage benefits.

- [ ] **3.2** Add a `max_accumulation_points` parameter (default: 500000) to cap memory usage. When exceeded, prune oldest entries first. Rationale: Prevents unbounded memory growth in long-running sessions.

## Verification Criteria

1. With a single camera moving, the fused cloud shows significantly more surface coverage than a single frame
2. Bounding box pruning still works correctly (hand/arm points are removed)
3. Memory usage stays bounded under the max points limit
4. Latency from cloud arrival to published fused cloud stays under 100ms
5. Grasp planning produces valid results using the accumulated cloud

## Potential Risks and Mitigations

1. **Ghost artifacts from moved objects**
   Mitigation: Phase 3.1's freshness weighting reduces the influence of stale points. The voxel downsampling also helps by averaging.

2. **Increased memory and CPU usage**
   Mitigation: Phase 3.2's max points cap. The voxel downsampling is already O(N log N) via numpy unique, which handles 500K points in ~50ms.

3. **TF drift causing misaligned accumulated points**
   Mitigation: The accumulation window is short (2-3s). OpenVINS drift over this period is negligible (<1mm). If drift becomes an issue, the window can be shortened.

4. **Increased latency**
   Mitigation: The publish step runs in the same timer callback as before. The only added latency is the concatenation + downsampling of the accumulated buffer, which is fast for reasonable buffer sizes.

## Alternative Approaches

1. **Voxel hashing with explicit occupancy tracking**: Instead of storing raw points, maintain a 3D voxel hash map where each voxel stores the averaged point position, color, observation count, and last-seen time. More memory-efficient and naturally handles deduplication. Trade-off: more complex implementation, harder to debug.

2. **TSDF (Truncated Signed Distance Function) volume**: KinectFusion-style volumetric integration. Provides the highest quality surface reconstruction. Trade-off: significant GPU/memory requirements, fixed volume bounds, complex implementation. Overkill for grasp planning input.

3. **Simply increase the voxel size**: Instead of temporal accumulation, just use a larger voxel (e.g., 1cm instead of 5mm). This gives better noise reduction per frame but doesn't help with occlusion. Trade-off: loses spatial resolution without gaining coverage. Doesn't address the core problem.

4. **Remove single-camera fallback entirely**: Only publish when both cameras are available. Trade-off: the pipeline is completely dead if one camera fails. The user's question suggests they want robustness, not strictness.
