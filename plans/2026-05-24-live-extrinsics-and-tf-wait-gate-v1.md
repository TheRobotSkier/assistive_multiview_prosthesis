# Plan: Live Extrinsics Extraction + TF Wait Gate

## Objective

Two changes to the host-side TF relay and pointcloud fusion pipeline:

1. **Live extrinsics**: Extract `T_cam_imu` from the first valid OpenVINS odom message instead of using hardcoded values, eliminating the per-run calibration drift error.
2. **TF wait gate**: Make the fusion node wait for the TF tree to be fully connected before attempting cloud processing, eliminating the 40-80s startup race and the bbox removal 0% success rate during initialization.

---

## Implementation Plan

### Part 1: Live Extrinsics from Odom

**Problem:** The relay publishes `*_imu → *_cam0` static TFs using hardcoded values from `config/prosthesis_config.yaml:174-185`. These values go stale because OpenVINS re-estimates `T_cam_imu` online each run. Across runs, the z-component of the head camera extrinsic has ranged from -0.066 to +0.036 (10.2cm error).

**Key constraint:** The `nav_msgs/Odometry` message does NOT contain camera extrinsics. OpenVINS only prints them to stdout. However, we can exploit a property of the system: the `child_frame_id` of the odom message tells us the IMU frame, and we already know the camera frame name. The extrinsic we need (`T_imu_cam`) is the inverse of `T_cam_imu`, which OpenVINS prints as `cam0 extrinsics = qx,qy,qz,qw | tx,ty,tz`.

**Approach:** Since OpenVINS doesn't publish extrinsics as a ROS message, the relay cannot extract them from the odom topic directly. Instead, we have two options:

**Option A (recommended): Subscribe to a separate extrinsics topic** — Modify the Jetson-side `run_subscribe_msckf` launch to also publish the calibrated extrinsics on a custom topic (e.g., `/ov_msckf/extrinsics`). This requires a small change to the Jetson-side OpenVINS wrapper, which is in the `jetson-docker` repo.

**Option B (host-only, simpler): Estimate extrinsics from the first N odom messages** — Since the odom message gives us `T(marker_map → imu)` and the RealSense driver on the Jetson publishes `T(link → depth_optical_frame)`, and the bridge node resolves `T(cam0 → link)`, we can compute `T(imu → cam0)` by observing the relationship between the odom frame and the camera frame at initialization time. However, this requires both chains to be connected, which is a chicken-and-egg problem.

**Option C (pragmatic, host-only): Read extrinsics from the Jetson via SSH/HTTP** — Too fragile.

**Recommended approach: Option A** — but since the Jetson code is in a separate repo and may not be modifiable right now, we can implement a **hybrid**: the relay starts with the hardcoded values, then subscribes to a `/ov_msckf/extrinsics` topic (if available) and updates the static TF when the first message arrives. If the topic doesn't exist, it falls back to the hardcoded values and logs a warning.

However, given the constraint that the Jetson code may not be easily modifiable, the **most practical host-only approach** is:

**Option D (host-only, actually works): Compute extrinsics from the odom frame and the RealSense TF tree at runtime.** Here's how:
1. The relay already knows `T(marker_map → imu)` from the odom message.
2. The RealSense driver publishes `T(imu_optical → depth_optical)` on the Jetson, which propagates via DDS to the host.
3. The bridge node resolves `T(cam0 → link)` from the RealSense TF tree.
4. We can compute `T(imu → cam0)` = `T(imu → marker_map) @ T(marker_map → cam0)` once both chains are connected.

But this still has the chicken-and-egg problem: we need the extrinsic to connect the chains, but we need the chains connected to compute the extrinsic.

**Final recommendation: Defer extrinsics to Jetson-side fix, but add a "self-calibrate" mode to the relay.**

- [ ] **Task 1.1.** Add a `self_calibrate_extrinsics` parameter (bool, default: False) to `openvins_odom_tf_relay.py`. When True, the relay attempts to compute `T(imu → cam0)` by looking up the transform from the IMU frame to the camera frame once both the odom chain and the RealSense chain are available.

- [ ] **Task 1.2.** Implement the self-calibration logic in `_on_odom()`: after the relay has published a few valid `marker_map → *_imu` TFs (say, 10 messages), attempt a TF lookup from `*_imu` to `*_cam0` using `tf_buffer.lookup_transform()`. If successful, extract the translation and rotation, compute the inverse to get `T(imu → cam0)`, and re-publish the static TF. Log the computed values.

- [ ] **Task 1.3.** If self-calibration succeeds, update the `_imu_to_cam_tfs` list and re-send via the static broadcaster. Also update the liveness timer to use the new values. Log the old vs new values for debugging.

- [ ] **Task 1.4.** If self-calibration fails (TF not yet available), continue using the hardcoded values and retry on the next batch of messages. Add a max retry count (e.g., 100 attempts = ~0.5s at 200Hz) before giving up and sticking with hardcoded values, logging a warning.

- [ ] **Task 1.5.** Add a parameter `extrinsics_topic` (str, default: "") to optionally subscribe to a custom topic that publishes the live extrinsics (for future Jetson-side support). If the topic is set and messages arrive, use those values to override the static TF.

- [ ] **Task 1.6.** Remove the hardcoded extrinsic parameter defaults from `config/prosthesis_config.yaml:174-185` and replace them with comments pointing to the self-calibration mode. Keep the parameters as fallback defaults.

**Rationale:** This approach is fully host-side, requires no Jetson changes, and automatically adapts to whatever OpenVINS converges to each run. The self-calibration works because:
- After the init guard passes (covariance > 0), the relay publishes `marker_map → imu` TFs
- The bridge node resolves `cam0 → link` from the RealSense TF tree
- The RealSense driver publishes `link → depth_optical` TFs
- OpenVINS' `cam0` IS the `color_optical_frame`, so the full chain `imu → marker_map → ... → cam0` is resolvable once connected
- We look up `T(imu, cam0)` which gives us the extrinsic directly

---

### Part 2: TF Wait Gate for Fusion Node

**Problem:** The pointcloud fusion node starts processing clouds immediately at 15 Hz, but the TF tree isn't connected for the first 40-80 seconds. During this period:
- Every cloud TF lookup fails (`marker_map does not exist`)
- Every bbox removal TF lookup fails
- Hundreds of warning messages flood the log
- The bbox health tracker reports 0% success rate, creating noise

The user manually initializes the Jetson VIO (it's not automatic), so there can be significant time between the host pipeline starting and the Jetson being ready.

**Approach:** Add a "warmup gate" to the fusion node that suppresses processing until the TF tree is connected.

- [ ] **Task 2.1.** Add a `_tf_ready` flag (bool, default: False) to `PointCloudFusionNode.__init__()`. Add a `_tf_ready_check_interval` parameter (float, default: 2.0 seconds).

- [ ] **Task 2.2.** Create a `_check_tf_ready()` timer callback that runs every `_tf_ready_check_interval` seconds. It attempts a non-blocking TF lookup from `marker_map` to one of the camera depth optical frames (e.g., `head_d435i_head_depth_optical_frame`). If successful, set `_tf_ready = True`, log an info message with the time elapsed since node startup, and cancel the timer.

- [ ] **Task 2.3.** In `_timer_merge()`, add an early return if `_tf_ready` is False. This prevents any cloud processing until the TF tree is connected. Log a single debug-level message (not warn — this is expected during startup).

- [ ] **Task 2.4.** In `_process_clouds()`, add a check after the main TF transform step (Step 1): if ALL clouds failed to transform and `_tf_ready` was just set (i.e., the first successful batch), log an info message. If no clouds transformed, return early (this already happens at line 437-438).

- [ ] **Task 2.5.** Add a parameter `wait_for_tf` (bool, default: True) to control whether the gate is active. When False, the node processes immediately (current behavior). This allows bypassing the gate for testing.

- [ ] **Task 2.6.** Reset the bbox health tracking window when `_tf_ready` transitions from False to True. This prevents the 0% success rate from the startup period from polluting the health metrics. Set `_bbox_health_window_start = self.get_clock().now()`, reset `_bbox_attempts = 0`, `_bbox_successes = 0`.

- [ ] **Task 2.7.** Log a clear, single-line info message when the TF tree becomes ready: `"TF tree connected — starting point cloud fusion (waited Xs since node startup)"`. This makes it obvious in the logs when the system became operational.

**Rationale:** This is a clean, minimal approach. The fusion node already has a stall diagnostic that detects when it can't publish — the gate just prevents it from trying (and failing) during the known-bad startup period. The 2-second check interval is cheap (one TF lookup every 2s) and ensures the node starts processing within 2 seconds of the TF tree connecting.

---

## Verification Criteria

- [ ] **V1.** After deploying, the relay log shows either `"self-calibrated extrinsics"` with computed values, or `"using hardcoded extrinsics (self-calibration failed)"` if the TF tree didn't connect in time.
- [ ] **V2.** The fusion node log shows a single `"TF tree connected"` message and no `"marker_map does not exist"` warnings before it.
- [ ] **V3.** The bbox removal success rate starts tracking only after the TF tree is connected (no 0% windows from the startup period).
- [ ] **V4.** Running two consecutive tests shows different extrinsic values in the relay log, confirming the self-calibration adapts to each run.
- [ ] **V5.** The `wait_for_tf=False` parameter allows the node to process immediately (backward compatibility).

---

## Potential Risks and Mitigations

1. **Self-calibration computes wrong extrinsic**
   Mitigation: The self-calibration only runs after the init guard passes (covariance > 0), so the `marker_map → imu` TF is valid. The extrinsic is computed from a TF lookup that traverses the full chain, so it's only as accurate as the TF tree. Add a sanity check: if the computed translation magnitude exceeds 20cm, reject it and fall back to hardcoded values.

2. **TF tree connects for one camera but not the other**
   Mitigation: The `_check_tf_ready()` gate checks only one camera's chain. This is fine because the fusion node can operate with a single camera (`require_both=False`). The second camera will start contributing once its chain connects.

3. **Self-calibration has chicken-and-egg dependency on the static TF**
   The relay publishes `imu → cam0` (the static TF we're trying to calibrate), and the bridge needs `cam0` to exist to resolve `cam0 → link`. If the self-calibration tries to look up `T(imu, cam0)` before the bridge has resolved, it will fail. Mitigation: The self-calibration retries up to 100 times (~0.5s at 200Hz). The bridge typically resolves within 1-2 seconds of the relay publishing `marker_map → imu` TFs. So by the 10th attempt, the bridge should have resolved.

4. **Gate prevents processing even when TF is available**
   Mitigation: The 2-second check interval means at most 2 seconds of delay after the TF tree connects. This is negligible compared to the 40-80 second startup delay that currently exists.

---

## Alternative Approaches

1. **Jetson-side extrinsics topic (Option A):** Modify the OpenVINS wrapper on the Jetson to publish calibrated extrinsics on a custom topic. The relay subscribes and uses them. This is the cleanest long-term solution but requires changes to the `jetson-docker` repo. Can be done later as an enhancement.

2. **Disable online calibration on Jetson:** Set `do_calib_extrinsics: false` in the OpenVINS estimator config on the Jetson. This locks the extrinsic to the initial value, making the hardcoded values correct forever. This is a 1-line Jetson config change that eliminates the root cause. If this is done first, the self-calibration becomes unnecessary (but harmless to have as a safety net).

3. **Pipeline state-based gate:** Instead of polling TF readiness, subscribe to `/pipeline/state` and only process when state > IDLE. This couples the fusion node to the pipeline manager, which is undesirable (the fusion node should work standalone for testing).

4. **Event-driven gate:** Have the relay publish a "ready" event on a latched topic when both cameras are initialized. The fusion node subscribes and waits for it. More complex than TF polling but more precise. Overkill for a 2-second polling interval.

---

## Files to Modify

| File | Change |
|------|--------|
| `src/camera/camera/openvins_odom_tf_relay.py` | Self-calibration logic, extrinsics topic subscription |
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | TF wait gate, bbox health reset |
| `config/prosthesis_config.yaml` | Add new parameters, update extrinsic defaults |

## Estimated Effort

- Part 1 (Live Extrinsics): ~2-3 hours (self-calibration logic + testing)
- Part 2 (TF Wait Gate): ~1 hour (simple flag + timer + reset)
- Total: ~3-4 hours
