# Pointcloud Fusion Debugging Session — 2026-05-20

## Session Overview

A multi-hour debugging session to fix the pointcloud fusion pipeline. The system has two RealSense D435i cameras (head + arm) on a Jetson, with OpenVINS VIO providing localization. The host machine runs the fusion pipeline inside a Docker container. The session went through 8 major discoveries, each building on the previous.

---

## Discovery 1: Executor Starvation from Blocking TF Lookups

### Symptom
Fusion node published ~100 clouds in the first 10 seconds, then froze permanently. Stats showed `published=100` unchanged for minutes. Node consumed 49% CPU.

### Root Cause
`can_transform()` with `timeout=0.05s` blocks the single-threaded `rclpy` executor for 50ms per call. With 2 clouds at 15Hz timer rate, that's 100ms+ of blocking per 67ms timer period. The executor can't process incoming TF messages during blocking, so the TF buffer goes stale, causing more timeouts — a death spiral.

### Fix
Replaced all `can_transform()` calls with non-blocking `lookup_transform()` (no timeout parameter). This returns immediately and never blocks the executor.

**Files changed:**
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` — removed `can_transform`, use `lookup_transform` with try/except

---

## Discovery 2: Cumulative Stats Masking Stalls

### Symptom
Stats like `published=100` were cumulative and never reset, making it impossible to distinguish a running system from a stalled one.

### Fix
Made stats per-interval (reset every 10 seconds). Added `last_publish_ago` field showing seconds since last successful publish.

**Files changed:**
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` — per-interval stats reset, `last_publish_ago` diagnostic

---

## Discovery 3: TF Bridge Anchor Oscillation

### Symptom
The arm bridge oscillated between correct anchor transform and incorrect fallback:
```
arm: publishing ... from anchor arm_d435i_arm_color_optical_frame_body_display as link
arm: publishing ... from fallback assuming arm_cam0 == arm_d435i_arm_depth_optical_frame
```
Each time the aruco marker was lost, the bridge fell back to a geometrically wrong identity assumption.

### Fix
Added last-good transform caching. When the anchor disappears, the bridge reuses the last successfully resolved transform instead of falling back to the wrong identity.

**Files changed:**
- `src/camera/camera/openvins_realsense_tf_bridge_node.py` — added `_last_good_matrix` caching

---

## Discovery 4: Bridge Has Same Executor Starvation Bug

### Symptom
The bridge's `_lookup_matrix` also used `timeout=self._timeout` (50ms blocking), causing the same executor starvation pattern. The bridge couldn't resolve TF lookups, disconnecting the TF chain.

### Fix
Removed blocking timeout from `_lookup_matrix`, using non-blocking `lookup_transform`.

**Files changed:**
- `src/camera/camera/openvins_realsense_tf_bridge_node.py` — removed timeout from `_lookup_matrix`

---

## Discovery 5: Wrong Transform — `body_display` is Not the Link Frame

### Symptom
Fused clouds from both cameras didn't overlap. Points were systematically offset — "high precision, low accuracy."

### Root Cause (Multiple Layers)

**Layer 1:** The bridge used `head_d435i_head_color_optical_frame_body_display` as the anchor frame, assuming it represented the RealSense link frame. But `body_display` is a visualization-only frame published by the Jetson's aruco marker detection node. It includes an extra rotation (optical-to-body convention: `rpy(-90, 0, -90)`) that corrupts the transform.

**Layer 2:** Investigation of the Jetson's OpenVINS calibration (`kalibr_imucam_chain.yaml`) revealed that `head_cam0` IS the RealSense color optical frame — OpenVINS uses the RealSense color camera as its tracking camera. The `body_display` frame is just the tracking camera's own position rotated to body convention — it doesn't represent the RealSense body at all.

**Layer 3:** Since `cam0 == color_optical_frame`, the bridge edge `cam0 -> link` is simply the known RealSense hardware extrinsic `T(color_optical -> link)`: a 15mm translation + 90-degree rotation. No aruco detection needed.

### Fix
Simplified the bridge to directly look up `T(color_optical -> link)` from the RealSense static chain (which the bridge itself publishes). Removed all anchor/locking/fallback logic. The bridge now publishes the correct static extrinsic immediately on startup.

**Files changed:**
- `src/camera/camera/openvins_realsense_tf_bridge_node.py` — simplified to direct extrinsic lookup
- `config/prosthesis_config.yaml` — changed `anchor_frames` to `color_optical_frame` names

---

## Discovery 6: Executor Starvation from Heavy Processing

### Symptom
After fixing the TF issues, the fusion node still stalled after ~20 seconds. Clouds arrived at ~1 Hz but the node reported them as stale (300+ seconds old).

### Root Cause
The fusion node processes ~116K-point clouds (1.8 MB each) with numpy in the timer callback. Parsing, transforming, filtering, and downsampling takes 100-200ms per cloud. At 15Hz timer rate, the executor spends all its time in processing and can't fire cloud subscription callbacks.

### Fix
Moved `_process_clouds` to a background daemon thread. The timer callback now spawns a thread and returns immediately, keeping the executor free for cloud and TF callbacks. Added `_stats_lock` for thread safety.

**Files changed:**
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` — threading for processing, stats lock

---

## Discovery 7: QoS Mismatch — BEST_EFFORT vs RELIABLE

### Symptom
The fusion node subscribed with `BEST_EFFORT` QoS but the RealSense on the Jetson publishes with `RELIABLE`. While technically compatible in one direction, this caused silent data loss under load.

### Fix
Changed fusion node's cloud subscription QoS from `BEST_EFFORT` to `RELIABLE`.

Also discovered the twist propagation node had the same issue — subscribing `BEST_EFFORT` to `/fused_pointcloud` which the fusion node publishes as `RELIABLE`. This is an incompatible combination (BEST_EFFORT sub cannot receive from RELIABLE pub).

**Files changed:**
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` — QoS changed to RELIABLE
- `src/twist_propagation/twist_propagation/twist_propagation_node.py` — QoS changed to RELIABLE

---

## Discovery 8: Fused Cloud Timestamp from Jetson Clock

### Symptom
Twist propagation node activated but reported `cloud_too_old` with age 3.47s despite fused cloud publishing at ~3 Hz.

### Root Cause
The fusion node preserved the original cloud's header timestamp (from the Jetson's clock). The twist node computes `age = host_now() - cloud_stamp`, which includes network transit time + processing delay + clock offset = 2+ seconds. With `cloud_max_age_s: 2.0`, the cloud is always "too old."

### Fix
Stamp the fused cloud with `self.get_clock().now().to_msg()` (host clock) instead of preserving the original Jetson timestamp.

**Files changed:**
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:475-479` — re-stamp with host clock

---

## Additional Fixes

### RViz Freeze — Missing `ipc: host`
The Docker container had only 64MB of `/dev/shm` (default). RViz's GPU rendering exhausted this when displaying large point clouds, freezing the Qt event loop while rendering continued.

**Fix:** Added `ipc: host` to `docker/docker-compose.yml` for the prosthesis service.

### CycloneDDS XML Syntax Error
Added `<Internal><Watermarks>` tuning to `config/cyclonedds_peer.xml` but with invalid XML syntax (missing `=` signs, stray `"` characters). This crashed every ROS2 node on startup.

**Fix:** Removed the broken `<Internal>` section entirely. The default CycloneDDS settings work fine for this setup.

### Bbox Marker Flickering
The pruning box marker in RViz had a 2-second lifetime but published at variable rates, causing it to disappear and reappear.

**Fix:** Increased marker lifetime from 2s to 5s in `pointcloud_fusion_node.py`.

---

## Summary of All Files Changed

| File | Changes |
|------|---------|
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | Non-blocking TF lookups, per-interval stats, threading for processing, RELIABLE QoS, host clock timestamping, bbox marker lifetime |
| `src/camera/camera/openvins_realsense_tf_bridge_node.py` | Non-blocking TF lookups, direct RealSense extrinsic (no anchor needed), removed fallback/locking logic |
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | RELIABLE QoS for cloud subscriptions |
| `config/prosthesis_config.yaml` | Updated anchor frames, removed obsolete parameters |
| `config/cyclonedds_peer.xml` | Fixed broken XML syntax |
| `docker/docker-compose.yml` | Added `ipc: host` for RViz stability |

---

## Architecture Understanding Gained

### TF Chain (Head Camera)
```
marker_map -> head_imu -> head_cam0 -> head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
             ^^^^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
             OpenVINS VIO (200Hz, Jetson)  RealSense static chain (published by bridge on /tf_static)
                                          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                          THE BRIDGE EDGE (was broken, now fixed)
```

### Key Insight: cam0 == color_optical_frame
OpenVINS uses the RealSense color camera as its tracking camera (confirmed via `kalibr_imucam_chain.yaml` on Jetson: `rostopic: /head/d435i_head/color/image_raw`). Therefore `head_cam0` and `head_d435i_head_color_optical_frame` are the same physical camera. The bridge edge `cam0 -> link` is just the known RealSense hardware extrinsic (15mm + 90-degree rotation), not a dynamic quantity requiring aruco detection.

### Data Flow
```
Jetson (10.42.0.2)                         Host (10.42.0.1)
┌─────────────────────┐                    ┌──────────────────────────────────┐
│ RealSense D435i     │                    │ Docker container (prosthesis)    │
│  - depth/color pts  │ ──DDS/Ethernet──> │  - TF bridge (cam0->link)        │
│ OpenVINS VIO        │                    │  - Pointcloud fusion             │
│  - marker_map->cam0 │ ──DDS/Ethernet──> │  - Segmentation                  │
│ Aruco detection     │                    │  - Twist propagation             │
│  - body_display     │ ──DDS/Ethernet──> │  - RViz                          │
└─────────────────────┘                    └──────────────────────────────────┘
```
