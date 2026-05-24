# TF Wait Gate Fix — Zero Timeout + Diagnostic Logging

## Objective

Fix the TF wait gate in `pointcloud_fusion_node.py` that never opens despite the TF tree being healthy (v18 regression). The gate's `can_transform` call with a 1-second timeout was blocking the single-threaded executor and preventing `/tf` messages from being processed. Changing to zero timeout eliminates the blocking, and adding warn-level diagnostic logging will reveal what's happening if the issue persists.

## Implementation Plan

- [ ] **Task 1.** In `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`, change `_check_tf_ready()` method (lines 751-827):
  - Change `timeout=rclpy.duration.Duration(seconds=1.0)` to `timeout=rclpy.duration.Duration(seconds=0.0)` at line 778
  - Replace the `debug`-level log at lines 780-784 (exception logging) with `warn`-level logging
  - Replace the `debug`-level log at lines 824-827 (not connected) with `warn`-level logging that includes:
    - Elapsed time since node startup
    - For each camera frame: the exception message from `can_transform`
    - Also try `lookup_transform` with zero timeout to get the specific error (path missing vs timestamp mismatch)

### Exact code for the new `_check_tf_ready`:

Replace lines 751-827 with:

```python
    def _check_tf_ready(self):
        """Check whether the TF tree is fully connected.

        Uses ``can_transform`` with **zero timeout** to avoid blocking
        the single-threaded executor.  A non-zero timeout (even 1 s)
        starves the /tf subscription callback, preventing the TF buffer
        from ever populating — which is why the gate never opened in v18.

        Opens the gate as soon as **either** camera chain is connected —
        the fusion node can operate single-camera (require_both=False).
        """
        if self._tf_ready:
            # Already ready — cancel the timer if it still exists.
            if self._tf_ready_timer is not None:
                self._tf_ready_timer.cancel()
                self._tf_ready_timer = None
            return

        elapsed = 0.0
        if self._node_start_time is not None:
            elapsed = (
                self.get_clock().now() - self._node_start_time
            ).nanoseconds / 1e9

        any_connected = False
        diag_parts = []

        for depth_frame in self._GATE_DEPTH_FRAMES:
            try:
                connected = self._tf_buffer.can_transform(
                    self._target_frame,
                    depth_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0),
                )
            except Exception as exc:
                diag_parts.append(f"{depth_frame}: can_transform threw '{exc}'")
                connected = False

            if connected:
                any_connected = True
                # ── This camera's TF tree is connected! ──────────────────
                self._tf_ready = True
                self._tf_ready_time = self.get_clock().now()

                # Cancel the polling timer.
                if self._tf_ready_timer is not None:
                    self._tf_ready_timer.cancel()
                    self._tf_ready_timer = None

                waited_s = elapsed

                self.get_logger().warn(
                    f"TF tree connected via {depth_frame!r} — "
                    f"starting point cloud fusion "
                    f"(waited {waited_s:.1f}s since node startup)"
                )

                # Reset bbox health tracking to avoid 0% success rate
                # from the startup period polluting the health metrics.
                self._bbox_attempts = 0
                self._bbox_successes = 0
                self._bbox_health_window_start = self.get_clock().now()
                return
            else:
                # Get a more specific diagnostic from lookup_transform
                try:
                    self._tf_buffer.lookup_transform(
                        self._target_frame,
                        depth_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.0),
                    )
                    diag_parts.append(
                        f"{depth_frame}: can_transform=False but "
                        f"lookup_transform succeeded (unexpected)"
                    )
                except Exception as exc2:
                    diag_parts.append(
                        f"{depth_frame}: {exc2}"
                    )

        # Neither camera's chain is ready yet — log at warn level every
        # 10 seconds so we can see what's happening without spamming.
        if elapsed < 5.0 or int(elapsed) % 10 == 0:
            self.get_logger().warn(
                f"TF wait gate: not connected after {elapsed:.0f}s — "
                + "; ".join(diag_parts)
            )
```

## Verification Criteria

- [ ] The fusion node's TF wait gate opens within 30-60 seconds of startup
- [ ] Warn-level logs show the specific reason for each failed attempt (path missing vs timestamp mismatch)
- [ ] `published > 0` in the stats output after the gate opens
- [ ] No executor blocking — cloud callbacks continue to be processed during the wait period

## Potential Risks and Mitigations

1. **Zero timeout might not be enough**
   Mitigation: The diagnostic logs will show exactly what `can_transform` returns and what `lookup_transform` reports. If zero timeout still fails, the logs will tell us why, and we can escalate to removing the gate entirely.

2. **Warn-level logging every 10s might still be too noisy**
   Mitigation: The throttle (`elapsed < 5.0 or int(elapsed) % 10 == 0`) limits it to roughly once every 10 seconds. Can be reduced further if needed.
