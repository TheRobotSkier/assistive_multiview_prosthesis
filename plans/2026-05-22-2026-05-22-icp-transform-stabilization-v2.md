# ICP-Based Point Cloud Transform Stabilization

## Objective

Eliminate visible point cloud "jumping" by using Iterative Closest Point (ICP) registration to refine the TF-based transform before merging clouds. The approach uses a lightweight numpy/scipy ICP implementation (no new dependencies) that aligns each incoming cloud against a high-confidence reference, correcting for OpenVINS TF jitter.

## Root Cause Analysis (Corrected Understanding)

### Q1: Why do we only have 2 Hz from OpenVINS?

**The 2 Hz is NOT the OpenVINS odometry rate.** OpenVINS runs on the Jetson at ~200 Hz. The 2 Hz is the **liveness re-send rate** of the bridge node on the host (`openvins_realsense_tf_bridge_node.py:419-433`), which re-publishes a **truly static** hardware extrinsic (`cam0 -> link`) on `/tf` as a workaround for CycloneDDS `/tf_static` latch unreliability with late-joining nodes.

The bridge publishes only 2 static transforms (one per camera). The 2 Hz rate is configured at `config/prosthesis_config.yaml:174` (`liveness_rate_hz: 2.0`) and can be trivially increased — these are tiny messages with near-zero computational cost.

**However**, the dynamic portion of the TF tree (`marker_map -> *_imu -> *_cam0`) comes from OpenVINS on the Jetson and is relayed by `odom_to_pose_relay` at full rate. The bridge's 2 Hz liveness is a separate concern.

### Q2: Can we increase the 2 Hz rate?

**Yes, trivially.** It is a ROS parameter (`liveness_rate_hz`). Changing it to 10 or 15 Hz in `config/prosthesis_config.yaml:174` would be a one-line config change with near-zero cost (2 small TransformStamped messages per tick). This would help with TF buffer expiry windows.

### Q3: What about the TF issues — are they getting each other's static TFs?

**The host does NOT depend on Jetson static TFs.** The bridge node at `openvins_realsense_tf_bridge_node.py:325-351` publishes the full 20-transform D435i nominal static chain locally, explicitly to avoid depending on Jetson `/tf_static` crossing the DDS boundary (documented in `signoff-report.md:93-101`).

The CycloneDDS configuration (`config/cyclonedds_peer.xml`) has **multicast completely disabled** (`AllowMulticast: false`) because multicast is unreliable over direct Ethernet + WSL2 mirrored networking. Discovery uses explicit unicast peers only. This is a known limitation of CycloneDDS `TRANSIENT_LOCAL` durability — the latch delivery for `/tf_static` is unreliable when DDS discovery between peers is slow or intermittent.

The timesync fix (`Makefile:322-326`) was applied correctly — it uses epoch seconds (`date +%s.%N`) to avoid timezone misinterpretation. Clock skew should now be within ~0.1s (SSH round-trip latency).

### Q4: Is the 80s cold start because the Jetson wasn't started?

**Very likely yes.** The v5 and v6 logs show:
- Lines 54-74: `cam1: no cloud; cam2: no cloud` — **no point clouds arriving at all** for 70+ seconds
- Line 77: First clouds arrive at ~80s (`cam1: fresh (0.06s); cam2: fresh (0.03s)`)
- The diagnostic says "OpenVINS(head):MISSING | OpenVINS(arm):MISSING" — suggesting the Jetson wasn't running yet

In contrast, v3 shows clouds arriving at ~10s (line 49-50: `cam1: fresh (0.04s); cam2: fresh (0.05s)`) because the Jetson was already running. The 80s gap in v5/v6 is almost certainly the time between starting the host `make camera-test` and manually starting the Jetson software.

### Q5: Does bbox removal only report failures, making us think it never works?

**Partially yes.** The stats ARE accurate — `bbox_removed` is correctly incremented on both primary success and cache-hit success paths. However:

1. **The health check undercounts success** — `_bbox_successes` (used by `_check_bbox_health`) only counts primary-path successes, not cache hits. So the "0% success rate" ERROR is slightly more alarming than reality.

2. **Successful removals are completely silent** — there is no per-removal success log. Only failures produce WARN messages. This creates an impression of total failure.

3. **Bbox removal DID work in earlier tests** — v3 shows `bbox_removed=377607` in one interval (line 56). v5 had one brief success window with `bbox_removed=21354` (line 152). The bbox system works when the TF chain is connected.

4. **The `bbox_skipped` counter is misleading** — it's incremented at the top of `_bbox_fallback()` before the cache check, so even successful cache hits are counted as "skipped." The counter really means "fresh lookup failed," not "box was skipped."

5. **In v6, bbox removal genuinely never worked** — `bbox_removed=0` and `bbox_cache_hits=0` for the entire 400+ second run. The pruning box frames (`palm_frame`, `screw_frame`) were never reachable from `marker_map` because the arm-side OpenVINS chain was intermittent.

### Why the Cloud Actually "Jumps"

With the corrected understanding, the jumping is caused by:
1. **OpenVINS dynamic TF updates** — The `marker_map -> *_cam0` edge updates at OpenVINS rate (200 Hz on Jetson, relayed to host). Between updates, the same transform is reused. When the OpenVINS pose estimate shifts (tracking noise, relocalization), the fused cloud jumps.
2. **Bridge liveness at 2 Hz** — The static `cam0 -> link` edge is re-stamped at 2 Hz. If the TF buffer expires the old entry before the new one arrives, there's a brief window where the full chain is broken.
3. **Arm-side OpenVINS intermittently disconnected** — In v5/v6, the arm OpenVINS chain was frequently MISSING, causing the `marker_map -> arm_*` path to break and reconnect, each reconnection causing a jump.

## Implementation Plan

### Phase 1: Quick Wins (Config Changes, Zero Code)

- [ ] **Increase `liveness_rate_hz` from 2.0 to 10.0 in `config/prosthesis_config.yaml:174`.** Rationale: The bridge re-sends 2 tiny static transforms. At 10 Hz, the TF buffer always has a fresh entry. Near-zero computational cost. This directly reduces the windows where the TF chain appears broken between liveness ticks.

- [ ] **Fix `bbox_skipped` semantics in `pointcloud_fusion_node.py`.** Move the `bbox_skipped` increment to after the cache check fails, or rename to `bbox_fresh_lookup_failed`. Rationale: The current counter is misleading — it counts cache hits as "skipped," making diagnostics harder to interpret.

- [ ] **Include cache hits in `_bbox_successes` for the health check.** Rationale: Cache hits are functionally successful removals. Excluding them from the health metric makes the error message more alarming than reality.

- [ ] **Add a throttled DEBUG log for successful bbox removals.** Log every Nth successful removal with point count. Rationale: Currently only failures are logged, creating the impression that bbox removal never works even when it does.

### Phase 2: ICP Transform Refinement (Core Solution)

- [ ] **Implement a point-to-point ICP function using only numpy + scipy.** Use `scipy.spatial.cKDTree` for nearest-neighbor correspondence (already in the main container per `docker/Dockerfile:20`) and SVD-based rigid transform estimation. Target: ~5-15 iterations on ~5K-10K downsampled points. Rationale: This is the core alignment engine. A numpy-only implementation avoids adding open3d. SVD rigid transform (Arun et al. 1987): compute centroid, center points, build cross-covariance matrix, SVD, extract R and t.

- [ ] **Use the current TF transform as the ICP initial guess.** The TF transform provides a coarse alignment (typically within a few cm). ICP then refines it. Rationale: ICP only converges from a reasonable initial guess. The TF chain is noisy but not wildly wrong — it provides sufficient initialization.

- [ ] **Integrate ICP refinement into `_process_clouds` at `pointcloud_fusion_node.py:416-438` (Step 1).** After the TF-based transform succeeds, run ICP against a reference cloud to compute a correction. If ICP fails (low overlap, divergence), fall back to the raw TF transform. Rationale: This is the insertion point where transforms are applied. Adding ICP here means all downstream steps operate on correctly aligned data.

- [ ] **Add a "reference cloud" mechanism.** Maintain a high-confidence cloud in `marker_map` frame as the ICP target. Updated only when confidence is high (both cameras present, TF chain healthy, ICP convergence good). First successfully transformed cloud becomes the initial reference. Rationale: ICP needs a stable target. Updating only on high-confidence frames prevents drift.

- [ ] **Add ICP convergence quality metrics.** Track: (a) MSE before/after, (b) fraction of points with correspondences within threshold, (c) transform delta from initial guess. Use to gate acceptance. Rationale: Not all frames will have good ICP convergence. Quality metrics enable graceful fallback.

- [ ] **Add parameters:** `icp_enabled` (bool, default True), `icp_max_iterations` (int, default 15), `icp_correspondence_threshold_m` (float, default 0.03), `icp_convergence_mse` (float, default 1e-6), `icp_min_overlap_ratio` (float, default 0.3), `icp_max_correction_m` (float, default 0.05). Rationale: ICP behavior needs to be tunable. The max_correction parameter prevents large corrections that indicate bad registration.

### Phase 3: Confidence-Based Reference Management

- [ ] **Implement confidence scoring for the reference cloud.** Score based on: (a) number of contributing cameras, (b) TF chain health, (c) ICP convergence quality, (d) reference age. Rationale: Reference quality determines ICP alignment quality.

- [ ] **Store the reference cloud downsampled to 1cm voxel (~2K-5K points).** Rationale: ICP runs every frame at 15 Hz. Smaller reference = faster KDTree queries. 1cm is sufficient for correcting cm-scale TF errors.

- [ ] **Add reference invalidation on scene change.** If ICP consistently fails for multiple frames, invalidate and rebuild from next high-confidence frame. Rationale: Large scene changes make old references useless.

### Phase 4: TF Health-Aware Fallback Chain

- [ ] **Implement three-tier transform strategy: ICP-corrected > raw TF > last-known-good.** Rationale: Graceful degradation. Always has a best-effort transform available.

- [ ] **Cache the last successful ICP-corrected transform per camera.** When TF drops, apply the cached transform. Rationale: Even with TF disconnected, cameras still produce depth data. The cached transform from the last good frame is better than nothing.

- [ ] **Add `transform_max_age_s` parameter (default 1.0).** When cached transform exceeds this age, stop publishing rather than serving very stale data. Rationale: Very stale transforms could place the cloud far from reality.

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

Frame budget at 15 Hz = 66 ms. ICP adds 2-5 ms — well within budget.

## Verification Criteria

- [ ] Fused point cloud shows no visible jumping during stable OpenVINS operation
- [ ] Fused point cloud shows significantly reduced jumping during OpenVINS transitions
- [ ] ICP convergence rate > 80% during normal operation
- [ ] End-to-end latency increases by no more than 5 ms
- [ ] Published cloud rate remains at ~15 Hz
- [ ] When ICP is disabled (`icp_enabled=False`), behavior is identical to current
- [ ] Bbox removal stats accurately reflect success/failure (after Phase 1 fixes)

## Potential Risks and Mitigations

1. **ICP converges to wrong local minimum**
   Mitigation: Use TF transform as initial guess. Add `icp_max_correction_m` to reject large corrections. Fall back to raw TF when quality metrics are poor.

2. **ICP adds too much latency**
   Mitigation: Run on pre-downsampled clouds (1cm voxel, ~5K-10K points). scipy cKDTree on 5K points is sub-millisecond per query.

3. **Reference cloud drifts over time**
   Mitigation: Confidence-gated updates. Invalidate after consecutive ICP failures.

4. **Scene changes cause reference mismatch**
   Mitigation: Detect via ICP overlap ratio and MSE spike. Auto-invalidate and rebuild.

5. **Hybrid EMA + ICP as fallback if full ICP is too heavy:** Use EMA for small frame-to-frame variations, invoke ICP only when EMA detects a large jump. Reduces average ICP cost to ~0.3-0.7 ms per frame.

## Recommended Implementation Order

1. **Phase 1 first** (config + reporting fixes) — immediate diagnostic improvement, zero risk
2. **Phase 2** (ICP core) — the main stabilization mechanism
3. **Phase 3** (confidence management) — incremental improvement once Phase 2 is validated
4. **Phase 4** (fallback chain) — handles edge cases and TF disconnection

## Dependencies

- **No new packages.** `scipy.spatial.cKDTree` already in main container (`docker/Dockerfile:20`). `numpy.linalg.svd` already used.
- **Modified files:** `pointcloud_fusion_node.py`, `config/prosthesis_config.yaml`
- **New parameters:** All via `declare_parameter()` — backward compatible.
