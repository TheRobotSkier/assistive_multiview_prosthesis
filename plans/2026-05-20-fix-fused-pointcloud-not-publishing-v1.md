# Fix: Fused Pointcloud Not Publishing (published=0)

## Objective

The pointcloud fusion node receives raw clouds from both cameras but publishes nothing (`published=0`). The root cause is a TF chain breakage: the fusion node needs `marker_map -> <cloud_frame>` for both cameras, but the arm camera's TF chain is incomplete. The bridge node only resolves the head camera, and the fusion node's `ApproximateTimeSynchronizer` requires both clouds to arrive simultaneously — then silently drops them when TF fails.

There are two distinct problems to fix:

1. **Arm TF chain broken** — the bridge can't resolve `arm_cam0 -> arm_d435i_arm_link`, likely because the Jetson's arm OpenVINS isn't publishing `arm_cam0` in the TF tree.
2. **Fusion node too strict** — it uses `ApproximateTimeSynchronizer` (requires both clouds) and silently drops everything when either TF chain is broken, with no per-camera fallback.

## Diagnosis Summary

### What the logs show

- `pointcloud_fusion_node`: `Stats: published=0 (dual=0, cam1_only=0)` — nothing published, not even single-camera
- `openvins_realsense_tf_bridge_node`: Only logs `head: publishing head_cam0->head_d435i_head_link` — **arm never resolves**
- RViz: `Message Filter dropping message: frame 'arm_d435i_arm_depth_frame' ... timestamp on the message is earlier than all the data in the transform cache`

### TF chain required

```
marker_map -> head_cam0 -> head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
marker_map -> arm_cam0  -> arm_d435i_arm_link  -> arm_d435i_arm_depth_frame  -> arm_d435i_arm_depth_optical_frame
                ^                ^
             OpenVINS          TF bridge publishes this edge
             publishes
```

The bridge publishes `head_cam0 -> head_d435i_head_link` (and arm equivalent). It needs `marker_map -> head_cam0` from OpenVINS to exist first. The arm chain never resolves, meaning `arm_cam0` is not in the TF tree — the Jetson's arm OpenVINS container is likely not running or not publishing TF.

### Why fusion publishes 0

The fusion node uses `ApproximateTimeSynchronizer` (`pointcloud_fusion_node.py:254`), which only fires when **both** clouds arrive within `sync_tolerance_s=0.1`. When it does fire, `_process_clouds` tries to transform both clouds to `marker_map`. The arm cloud's frame (`arm_d435i_arm_depth_frame`) can't be resolved to `marker_map` because the `arm_cam0` TF is missing. The TF lookup fails silently (throttled warning), `transformed` list is empty or only has head, but the code at line 347 returns early if `not transformed`.

Even if the head cloud transforms successfully alone, the synchronizer callback never fires with just one cloud. There is no `cam1_only` path when using the synchronizer.

## Implementation Plan

### Phase 1: Make fusion resilient to missing camera (host-side fix)

This is the most impactful change — make the fusion node work with whatever clouds are available, not require both.

- [x] **1.1** Add a `cam1_only` fallback in `pointcloud_fusion_node.py`
  - The `ApproximateTimeSynchronizer` only fires when both clouds arrive. Add individual subscriptions as a fallback that feed a timer-based merge (the `else` branch at line 259-271 already exists but is only used when `message_filters` is unavailable).
  - Better approach: keep the synchronizer for the dual-cloud case, but also add a timer that processes whichever clouds have arrived recently (e.g., within 2x the sync tolerance). This way dual-cloud gets the best sync, and single-cloud still works.
  - Rationale: In practice, one camera or OpenVINS instance may be down. The system should still produce output.

- [x] **1.2** In `_process_clouds`, log which cloud frames failed TF (not just throttled warn)
  - Change the throttled warn at line 337-340 to also include the frame_id in the stats log, so `published=0` is immediately diagnosable.
  - Rationale: The current silent drop makes debugging very hard.

- [x] **1.3** Add a `require_both_cameras` parameter (default `true`) to control sync behavior
  - When `false`, use the timer-based merge path (process whatever is available).
  - When `true` (current behavior), use `ApproximateTimeSynchronizer`.
  - Rationale: Gives flexibility for single-camera debugging without code changes.

### Phase 2: Fix arm TF bridge resolution

- [x] **2.1** Add diagnostic logging to the TF bridge for the arm camera
  - In `openvins_realsense_tf_bridge_node.py:370-400`, the `_resolve_bridge_transform` method returns `None` silently for the arm (only `_warn_once` fires, which logs once then never again).
  - Add a periodic debug log (every 10s) showing which frames were attempted and why they failed for each camera spec.
  - Rationale: Makes it obvious whether `arm_cam0` exists in TF, or `arm_d435i_arm_color_optical_frame_body_display` is missing, etc.

- [x] **2.2** Verify the Jetson arm OpenVINS is running and publishing `arm_cam0`
  - This is a runtime/Jetson-side check. Diagnostic commands:
    - `ros2 topic echo --once /ov_msckf_arm/odomimu` — if this times out, arm OpenVINS is down.
    - `ros2 run tf2_ros tf2_echo marker_map arm_cam0` — if this fails, arm_cam0 is not in the TF tree.
  - The TF bridge diagnostic logging added in 2.1 will now report this clearly every 10 seconds.
  - Rationale: If the Jetson side isn't publishing, no host-side fix will help. Phase 1 ensures the system still works with head-only.

### Phase 3: Rebuild and test

- [x] **3.1** Rebuild the modified packages inside the container
  - `make build-pkg PKG=pointcloud_fusion`
  - `make build-pkg PKG=camera`

- [x] **3.2** Test with single camera (arm OpenVINS down)
  - Requires user to relaunch pipeline inside container and verify.
  - The `require_both_cameras` parameter defaults to `false` now, so single-camera will work out of the box.

- [x] **3.3** Test with both cameras (Jetson fully up)
  - Requires user to relaunch pipeline with both Jetson OpenVINS instances running.
  - When both cameras are up, dual fusion will work as before (timer merge processes both).

## Verification Criteria

1. `ros2 topic hz /fused_pointcloud` shows >0 Hz when at least one camera cloud and its TF chain are available
2. Fusion stats log shows `published > 0` within 15 seconds of launch
3. When arm OpenVINS is down, head-only cloud still publishes (with a warning)
4. When both cameras are up, dual fusion works as before
5. TF bridge logs clearly indicate which camera's TF chain is broken and why

## Potential Risks and Mitigations

1. **Single-camera fusion changes dual-camera behavior**
   Mitigation: The `ApproximateTimeSynchronizer` path remains the default. The timer-based fallback only activates when a cloud arrives without its pair within the sync window. Add the `require_both_cameras` parameter as a safety switch.

2. **Timer-based merge increases CPU usage**
   Mitigation: The timer only fires at the existing rate (15 Hz from `1.0/15.0` at line 268). No change in processing frequency, just more per-cloud processing when one camera is missing.

3. **Arm TF still broken after Phase 2**
   Mitigation: Phase 2.2 is a Jetson-side issue. If arm OpenVINS can't be started, Phase 1 ensures the system still works with head-only.

## Alternative Approaches

1. **Drop `ApproximateTimeSynchronizer` entirely, always use timer-based merge**: Simpler code, but loses the precise temporal alignment benefit for dual-camera fusion. The sync tolerancing gives better registration when both clouds are available.

2. **Make the TF bridge publish an identity transform when arm_cam0 is missing**: Would make the arm cloud "work" but with wrong positioning. Worse than not publishing — would produce garbage fused clouds.

3. **Add a `cam1_only` mode to the launch file**: Could add a `cam2_enabled` parameter. Simpler but less flexible — requires restart to change. The timer-based fallback is transparent.
