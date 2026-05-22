# Pointcloud Fusion Node — TF Timing Stall Fix

**Date:** 2026-05-20
**Status:** Implemented & Verified
**Files changed:**
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`
- `src/camera/camera/openvins_realsense_tf_bridge_node.py`

---

## Symptoms

The fusion node logged stats that were **frozen** at the same values for minutes:

```
Stats: published=100 (dual=21, cam1_only=79) dist_removed=1289010 bbox_removed=0
       tf_fail={arm_d435i_arm_depth_optical_frame:250, head_d435i_head_depth_optical_frame:254}
```

The `/fused_pointcloud` topic published nothing (0 Hz) despite both input clouds arriving at ~4.5 Hz and TF being available.

---

## Root Cause: Executor Starvation from Blocking `can_transform`

### The death spiral

1. The `_timer_merge` callback fires at 15 Hz.
2. Each tick calls `can_transform(..., timeout=0.05s)` for each cloud frame.
3. `can_transform` with a timeout **blocks the single-threaded rclpy executor** for up to 50 ms.
4. While blocked, the executor **cannot process incoming TF messages** on `/tf`.
5. The TF buffer becomes stale → the next `can_transform` also blocks.
6. After ~100 successful publishes during the initial TF warm-up, the buffer falls permanently behind.
7. The node enters a zombie state: the stats timer fires (10 Hz), the bbox timer fires (1 Hz), but `_process_clouds` never succeeds again.

### Evidence

- **CPU at 49.2%** for a Python node doing no useful work — all CPU spent in blocking `can_transform` timeout polling.
- **Both cameras failing equally** (254 head vs 250 arm) — proved the issue was systemic, not camera-specific.
- **`tf2_echo` worked fine** from a separate process — TF data was available, just not reaching the fusion node's buffer.
- **Standalone reproduction test** confirmed: using non-blocking `lookup_transform` (no timeout) produced 678 successful transforms in 25 seconds (27.1 Hz) with only 1 warm-up failure.

### Why the old code used `can_transform`

The original pattern was:

```python
if self._tf_buffer.can_transform(target, source, rclpy.time.Time(), timeout=Duration(seconds=0.05)):
    t = self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
```

The intent was to wait briefly for TF to become available. But in a single-threaded executor, the wait prevents the TF listener callbacks from running, creating a deadlock.

---

## Fixes Applied

### Fix 1: Remove blocking `can_transform` — use non-blocking `lookup_transform` (fusion node)

**File:** `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:344-364`

Before:
```python
if self._tf_buffer.can_transform(target, source, rclpy.time.Time(), timeout=Duration(seconds=0.05)):
    t = self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
    transformed.append(tf2_sensor_msgs.do_transform_cloud(cloud, t))
```

After:
```python
try:
    t = self._tf_buffer.lookup_transform(
        self._target_frame, cloud.header.frame_id,
        rclpy.time.Time(),
    )
    transformed.append(tf2_sensor_msgs.do_transform_cloud(cloud, t))
except Exception as exc:
    frame = cloud.header.frame_id
    self._stats["tf_fail"][frame] = self._stats["tf_fail"].get(frame, 0) + 1
    self.get_logger().warn(
        f"TF transform failed for {frame}: {exc}",
        throttle_duration_sec=10.0,
    )
```

`lookup_transform` without a timeout parameter returns immediately with the latest available transform or throws. This never blocks the executor.

### Fix 2: Per-interval stats reset (fusion node)

**File:** `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:522-541`

Stats now reset every 10 seconds so stalls are immediately visible. Added `last_publish_ago` field:

```
Stats: published=45 (dual=12, cam1_only=33) dist_removed=52340 bbox_removed=120 last_publish_ago=0.1s
```

If `last_publish_ago` exceeds a few seconds, something is wrong.

### Fix 3: Cache last-good anchor transform (TF bridge)

**File:** `src/camera/camera/openvins_realsense_tf_bridge_node.py:379-401`

The arm's aruco `body_display` anchor frame appears and disappears as the marker becomes visible/hidden. Previously, each disappearance caused the bridge to fall back to the geometrically incorrect identity assumption, making the arm TF chain jump between correct and incorrect transforms.

Now the bridge caches the last-good matrix and reuses it for up to 5 seconds when the anchor disappears:

```python
def _resolve_bridge_transform(self, spec):
    matrix, source = self._resolve_bridge_transform_live(spec)
    if matrix is not None:
        self._last_good_matrix[spec.name] = matrix
        self._last_good_stamp[spec.name] = self.get_clock().now().nanoseconds / 1e9
        return matrix, source

    # Live lookup failed — try the cached matrix if it's fresh enough.
    cached = self._last_good_matrix.get(spec.name)
    if cached is not None:
        age = self.get_clock().now().nanoseconds / 1e9 - self._last_good_stamp[spec.name]
        if age <= self._matrix_staleness_limit_s:
            return cached, f"cached ({age:.1f}s ago)"
    return None, ""
```

---

## Diagnostic Steps Performed

| Check | Result |
|---|---|
| Clock skew (host vs Jetson) | ~71 ms difference — negligible |
| Head cloud rate | 4.6 Hz (healthy) |
| Arm cloud rate | 4.5 Hz (healthy) |
| Fused output rate | 0 Hz (stalled) |
| TF chain `marker_map -> head_d435i_head_depth_optical_frame` | Resolves via `tf2_echo` |
| TF chain `marker_map -> arm_d435i_arm_depth_optical_frame` | Resolves via `tf2_echo` |
| `/tf` topic rate | ~280 Hz (healthy) |
| Fusion node CPU | 49.2% (busy-looping in blocking calls) |
| `can_transform(..., Time(), timeout=0.05s)` from Python | Returns False even when TF is available (buffer starvation) |
| `lookup_transform(..., Time())` from Python | Succeeds once buffer has data |

---

## Verification

Standalone test replicating the fusion node's exact subscription pattern:

```
Running for 25 seconds...
  [1] Transformed arm_d435i_arm_depth_optical_frame OK
  [2] Transformed head_d435i_head_depth_optical_frame OK
  ...
  [660] Transformed arm_d435i_arm_depth_optical_frame OK

Final stats: published=678 tf_fail=1
Rate: 27.1 Hz
```

---

## Remaining Recommendations

1. **Restart the pipeline** to activate the fixes: Ctrl+C the current launch, then re-run.
2. **Monitor stats** — after restart, `published` should increment every 10s and `last_publish_ago` should stay below ~0.5s.
3. **Arm anchor oscillation** — if the aruco marker for the arm camera is frequently lost, consider improving marker visibility or increasing `_matrix_staleness_limit_s` beyond 5s.
4. **QoS mismatch** — the fusion node subscribes BEST_EFFORT but the RealSense publishers use RELIABLE. While compatible, matching RELIABLE would be more robust. This is a minor improvement, not urgent.
