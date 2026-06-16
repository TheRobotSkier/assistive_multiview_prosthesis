# Fix Plan: Immediate Fixes from Log Analysis (v6_20260616_134948)

## Issues with Clear Immediate Fixes

Three of the eight issues have straightforward code/config fixes.

---

### Fix 1 — TSDF Fusion: "Executor is already spinning" (CRITICAL)

**Root cause:** `_fetch_all_keyframes()` (line 609) and `_fetch_keyframes()` (line 573) call `rclpy.spin_until_future_complete(self, future, ...)` from within a timer callback. With a `SingleThreadedExecutor`, the executor is already spinning the timer callback on its only thread, so nested spinning is rejected.

**Fix approach:** Replace the default `SingleThreadedExecutor` with a `MultiThreadedExecutor` using a `ReentrantCallbackGroup` for the timer and service clients. This allows the service response to be processed on a different thread while the timer callback blocks on `spin_until_future_complete`.

**Files to change:**
- `src/tsdf_fusion/tsdf_fusion/tsdf_fusion_node.py` — create node with ReentrantCallbackGroup, assign to timer + kf clients, change main() to MultiThreadedExecutor

---

### Fix 2 — Diagnostics Node: TransformListener crash (HIGH)

**Root cause:** `pipeline_diagnostics_node.py:170` passes only the Buffer to `TransformListener()`. In ROS 2 Jazzy, `TransformListener` requires the node as a positional argument: `TransformListener(buffer, node)`.

**Fix approach:** Add `self` as the second argument.

**Files to change:**
- `src/camera/camera/pipeline_diagnostics_node.py:170` — `TransformListener(self._tf_buffer)` → `TransformListener(self._tf_buffer, self)`

---

### Fix 3 — SIFT Depth Radius: Still Too Tight (MEDIUM)

**Root cause:** Despite the previous increase from 0.02 → 0.05, SIFT published only 1 visual factor in the entire run. The unorganized, decimated clouds need a larger perpendicular tolerance for ray-casting depth lookup.

**Fix approach:** Increase to 0.10 (10cm). This is still geometrically reasonable at typical arm/head ranges of 0.5-2m.

**Files to change:**
- `config/prosthesis_config.yaml:543` — `depth_search_radius_m: 0.05` → `0.10`
- `src/cross_camera_features/cross_camera_features/sift_feature_node.py` — update default param and function signature default to match

---

## Issues NOT Addressing Now

| # | Issue | Why not |
|---|-------|---------|
| 3 | Arm VIO jumps 1.1m every 2s | Jetson/OpenVINS — needs live diagnostics first to determine if marker visibility, anchor delivery, or VIO tuning is the cause |
| 4 | Head VIO diverges at end | Jetson/OpenVINS — same |
| 5 | GTSAM poses not visible in RViz | Need live verification — check if RViz shows green/red checkmark on GtsamHeadPose/GtsamArmPose displays. QoS should match now (both RELIABLE). Could be a tf_frame issue. |
| 6 | Clock offset climbs to 379ms | Infrastructure (chrony/NTP) — not a code fix |
| 8 | Raw cloud misalignment | Cascades from arm VIO oscillation (Issue 3) |
