# Fix Plan: Bbox Removal (Fix 2) + Clock Skew TF Errors (Fix 3)

## Objective

Fix two host-side bugs that contribute to point cloud instability:
1. **Fix 2**: Bbox removal 0% success rate — change the TF lookup strategy in `_lookup_bbox_transform()`
2. **Fix 3**: Clock skew TF extrapolation errors — stamp relay TFs with odom message timestamps instead of host clock

---

## Fix 2: Bbox Removal Always Fails

### Problem

`_lookup_bbox_transform()` at `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:577-595` uses:
- `stamp = now - 100ms` (a specific past timestamp)
- `timeout = Duration(seconds=0)` (zero timeout, non-blocking)

The full chain `marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> screw_frame -> palm_frame` involves edges from three independent publishers at different rates (200 Hz, 10 Hz, 10 Hz). With zero timeout, TF2 cannot wait for interpolation, so the lookup **always fails**. The cache is never populated, and the fallback always hits "no cached transform available".

### Solution

Change the bbox lookup to use `rclpy.time.Time()` (latest) with a small non-zero timeout (50ms). This matches the approach used by the main cloud transform (line 422-425, which uses `rclpy.time.Time()` with default infinite timeout) and the diagnostics node (which uses `can_transform()` with `Time()` and 1-second timeout).

### Files to Change

#### `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`

### Implementation Plan

- [ ] **Task 2a.** Remove the `lookup_stamp` computation at lines 506-512. The comment block and the `now - 100ms` computation are no longer needed since we'll use `rclpy.time.Time()` inside `_lookup_bbox_transform()` directly.

  Replace lines 506-512:
  ```python
  # Use a recent-but-not-zero timestamp for bbox TF lookups.
  # rclpy.time.Time() (= zero) means "latest", but can cause
  # extrapolation-into-the-past errors when the TF buffer only has
  # data starting slightly after the requested time.  A 100ms offset
  # gives the buffer a safe margin while still being recent enough.
  lookup_stamp = (self.get_clock().now()
                  - rclpy.duration.Duration(seconds=0.1))
  ```
  With a single comment:
  ```python
  # Use latest-available TF for bbox lookups (rclpy.time.Time()).
  ```

- [ ] **Task 2b.** Update the `_lookup_bbox_transform` call at line 518-519 to no longer pass `lookup_stamp`. Change:
  ```python
  transform_result = self._lookup_bbox_transform(
      frame, lookup_stamp, xyz_all)
  ```
  To:
  ```python
  transform_result = self._lookup_bbox_transform(
      frame, xyz_all)
  ```

- [ ] **Task 2c.** Update the `_bbox_fallback` call at lines 543-545 to remove the `lookup_stamp` argument. Change:
  ```python
  handled = self._bbox_fallback(
      frame, bbox_min, bbox_max,
      xyz_all, rgb_all, lookup_stamp)
  ```
  To:
  ```python
  handled = self._bbox_fallback(
      frame, bbox_min, bbox_max,
      xyz_all, rgb_all)
  ```

- [ ] **Task 2d.** Rewrite `_lookup_bbox_transform()` at lines 577-595. Change the signature to remove `stamp`, use `rclpy.time.Time()` for the timestamp, and use a 50ms timeout. New implementation:

  ```python
  def _lookup_bbox_transform(self, frame: str, xyz_all: np.ndarray):
      """Try to look up the transform for a bbox pruning box.

      Returns (R, t_vec, xyz_box) on success, or None if lookup fails.
      Uses rclpy.time.Time() (= latest available) with a 50ms timeout
      so TF2 has time to interpolate the multi-edge chain while still
      keeping latency low for the fusion pipeline.
      """
      try:
          t = self._tf_buffer.lookup_transform(
              frame, self._target_frame, rclpy.time.Time(),
              timeout=rclpy.duration.Duration(seconds=0.05),
          )
      except Exception:
          return None

      R, t_vec = _extract_rotation_translation(t)
      xyz_box = (xyz_all.astype(np.float64) @ R.T) + t_vec
      return R, t_vec, xyz_box.astype(np.float32)
  ```

- [ ] **Task 2e.** Update `_bbox_fallback()` signature at line 597 to remove the `lookup_stamp` parameter. Change:
  ```python
  def _bbox_fallback(self, frame: str, bbox_min, bbox_max,
                     xyz_all, rgb_all, lookup_stamp):
  ```
  To:
  ```python
  def _bbox_fallback(self, frame: str, bbox_min, bbox_max,
                     xyz_all, rgb_all):
  ```

### Verification

- [ ] `python3 -m py_compile src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` passes
- [ ] In the next camera test log: `bbox_removed > 0` and `bbox_cache_hits` stays low (cache only needed as fallback)
- [ ] Bbox removal success rate > 90% in health check logs
- [ ] No "no cached transform available" warnings for palm_frame or screw_frame

---

## Fix 3: Clock Skew TF Extrapolation Errors

### Problem

The `openvins_odom_tf_relay` at `src/camera/camera/openvins_odom_tf_relay.py:212-223` stamps its TF transforms with `self.get_clock().now().to_msg()` (host clock). But the point clouds arriving from the Jetson are stamped with the Jetson clock. When the Jetson clock is behind the host clock by 125ms-1.16s (as observed in v9), the fusion node's main cloud transform at line 422-425 uses `rclpy.time.Time()` which TF2 resolves to the latest transform. However, the cloud's frame_id timestamps are in the Jetson time domain, and TF2's internal interpolation may fail when the requested time falls outside the available data range.

The key insight from the docstring at lines 11-13:
> "We extract that pose and publish it as a local TF transform stamped with the **host clock** to avoid the 'extrapolation into the past' errors caused by Jetson-host clock skew."

This was an intentional design choice, but it creates a different problem: the TF timestamps are in a different time domain than the cloud timestamps. The fusion node works around this by using `rclpy.time.Time()` (latest) for the main cloud transform, which effectively ignores the cloud's timestamp and uses whatever is newest in the TF buffer. This works for the main transform but causes issues when TF2 needs to interpolate across the chain.

### Solution

Stamp the relay TFs with the **odom message's timestamp** (`msg.header.stamp`) instead of the host clock. This aligns the TF time domain with the Jetson's time domain, which is the same domain used by the point clouds. The fusion node already re-stamps the output cloud with the host clock at line 569, so downstream consumers are unaffected.

The original concern about "extrapolation into the past" errors was valid when the host was consuming Jetson `/tf` directly. But since the relay now creates its own TFs from odom messages, using the odom's timestamp ensures consistency with the point cloud timestamps.

### Files to Change

#### `src/camera/camera/openvins_odom_tf_relay.py`

### Implementation Plan

- [ ] **Task 3a.** Update the docstring at lines 10-13 to reflect the new behavior. Change:
  ```
  Each message carries ``T(marker_map -> imu)`` in its pose field.  We extract
  that pose and publish it as a local TF transform stamped with the **host clock**
  to avoid the "extrapolation into the past" errors caused by Jetson-host clock
  skew.
  ```
  To:
  ```
  Each message carries ``T(marker_map -> imu)`` in its pose field.  We extract
  that pose and publish it as a local TF transform stamped with the **odom
  message timestamp** so that TF timestamps share the same time domain as the
  point clouds arriving from the Jetson.
  ```

- [ ] **Task 3b.** Update `_on_odom()` at lines 212-223 to use the odom message's timestamp. Change:
  ```python
  def _on_odom(self, msg: Odometry, parent: str, child: str, name: str):
      """Extract pose from odom and publish as TF with host clock stamp."""
      x, y, z = _extract_translation_from_odom(msg)
      qx, qy, qz, qw = _extract_quaternion_from_odom(msg)

      # Use host clock stamp — avoids extrapolation-into-the-past errors
      # that occur when the Jetson clock and host clock differ.
      tf_msg = _make_transform(
          parent, child, x, y, z, qx, qy, qz, qw,
          self.get_clock().now().to_msg(),
      )
  ```
  To:
  ```python
  def _on_odom(self, msg: Odometry, parent: str, child: str, name: str):
      """Extract pose from odom and publish as TF with odom's timestamp."""
      x, y, z = _extract_translation_from_odom(msg)
      qx, qy, qz, qw = _extract_quaternion_from_odom(msg)

      # Use the odom message's timestamp so TF stamps share the same
      # time domain as the point clouds from the Jetson.  The fusion
      # node re-stamps the output cloud with the host clock, so
      # downstream consumers are unaffected.
      tf_msg = _make_transform(
          parent, child, x, y, z, qx, qy, qz, qw,
          msg.header.stamp,
      )
  ```

### Verification

- [ ] `python3 -m py_compile src/camera/camera/openvins_odom_tf_relay.py` passes
- [ ] In the next camera test log: no "extrapolation into the past" errors for head or arm optical frames
- [ ] `tf_fail` counts in stats lines should drop significantly (from 10-50 per window to near 0)
- [ ] TF diagnostics continues to report all chains healthy

---

## Potential Risks and Mitigations

1. **Fix 2 Risk: 50ms timeout adds latency to the fusion pipeline**
   - Mitigation: 50ms is well within the 66ms frame budget at 15 Hz. The main cloud transform already uses an infinite timeout and works fine. 50ms is a conservative choice — if needed, it can be reduced to 10ms.

2. **Fix 2 Risk: `rclpy.time.Time()` (latest) may return a stale transform if the chain is disconnected**
   - Mitigation: The bbox removal has a cache fallback and health monitoring. If the chain disconnects, the health checker will report the issue. The current code already handles this case — it just never reaches the fallback because the fresh lookup always fails.

3. **Fix 3 Risk: Using odom timestamp may cause "extrapolation into the future" errors if odom messages arrive before TF buffer processes them**
   - Mitigation: TF2's `lookup_transform` with `rclpy.time.Time()` returns the latest available transform, not a future one. The odom timestamps are always in the past relative to when they're processed. The fusion node uses `rclpy.time.Time()` for the main cloud transform, which always returns the latest — this is unaffected by the timestamp change.

4. **Fix 3 Risk: If odom timestamps are zero or invalid, TF2 may reject them**
   - Mitigation: OpenVINS always stamps its odom messages with the Jetson's ROS clock. These are valid non-zero timestamps. If there's ever a zero timestamp, TF2 will treat it as "latest" (same as `rclpy.time.Time()`), which is safe.

## Alternative Approaches

1. **Fix 2 Alternative: Increase TF buffer duration** — Instead of changing the lookup strategy, increase the TF buffer from the default 10s to 60s. This doesn't fix the zero-timeout issue but gives TF2 more data to work with. Rejected because it doesn't address the root cause (zero timeout).

2. **Fix 3 Alternative: Keep host clock stamping, increase TF buffer** — Keep using `get_clock().now()` but increase the buffer duration so the fusion node can always find data in the Jetson time domain. Rejected because it doesn't solve the fundamental time domain mismatch — the cloud timestamps will still be in a different time domain than the TF timestamps.

3. **Fix 3 Alternative: Add a clock offset parameter** — Measure the clock offset between Jetson and host, and add it to the odom timestamp. Rejected because it adds complexity and the odom timestamp approach is simpler and more correct.
