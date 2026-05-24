# Fix Mixed Clock Domains in TF Liveness Publishers

## Objective

Eliminate "extrapolation into the past" errors (up to 10.7s gaps in v10) caused by the bridge and camera-mount liveness timers stamping transforms with host-clock time (`self.get_clock().now()`), while the `openvins_odom_tf_relay` stamps transforms with Jetson odom time (`msg.header.stamp`).

Both the bridge edges (`cam0 -> link`) and camera-mount edges (`link -> screw -> palm`) are **fixed hardware extrinsics** — they never change during operation. Publishing them with dynamic timestamps couples them to a specific clock domain unnecessarily. By stamping them with `rclpy.time.Time()` (time zero = "valid at any time"), TF2 can chain them with the relay's Jetson-timed edges without time-domain conflicts.

## Implementation Plan

- [ ] Task 1. **Change `openvins_realsense_tf_bridge_node.py` liveness to use `rclpy.time.Time()`**
  In `_liveness_tick()` (line 430), replace `stamp = self.get_clock().now().to_msg()` with `stamp = rclpy.time.Time().to_msg()`. The bridge edges (`arm_cam0 -> arm_d435i_arm_link`, `head_cam0 -> head_d435i_head_link`) are fixed RealSense camera-to-body extrinsics that never move, so a timeless stamp is semantically correct. This eliminates the host-clock coupling in the liveness path.

- [ ] Task 2. **Change `publish_camera_mounts.py` liveness to use `rclpy.time.Time()`**
  In `_liveness_tick()` (line 216), replace `stamp = self.get_clock().now().to_msg()` with `stamp = rclpy.time.Time().to_msg()`. The camera mount transforms (`arm_d435i_arm_link -> screw_frame`, `screw_frame -> palm_frame`) are fixed geometric relationships from the camera mount CAD — they never move, so a timeless stamp is semantically correct. This eliminates the host-clock coupling in the camera mount liveness path.

- [ ] Task 3. **Verify both files compile correctly**
  Run `python3 -m py_compile` on both modified files.

## Verification Criteria

- Both files pass Python syntax check
- In the next test run, "extrapolation into the past" errors should have substantially smaller gaps (ideally <100ms consistently, matching the relay's Jetson-time TFs)
- The `tf_pipeline_diagnostics` node should continue to report all chains healthy
- Bbox removal success rate should not regress (should remain in the 52-87% range or improve)
- No new warnings or errors from the bridge or camera mount nodes at startup

## Potential Risks and Mitigations

1. **TF2 may not handle time-zero published transforms as expected when chaining with non-zero transforms.**
   Mitigation: TF2's `lookup_transform()` with `Time(0)` already resolves to "latest available," so time-zero edges should integrate into the chain. If this approach fails, an alternative is to subscribe to the relay's odom messages in both the bridge and camera mount, stamping liveness TFs with the odom message's timestamp to unify the time domain.

2. **The static chains published at startup still use `self.get_clock().now()`.**
   Mitigation: The startup tick sends on `/tf_static` (time-independent) plus a brief `/tf` burst before liveness takes over. Once liveness starts, the host-time `/tf` entries age out of the buffer (<10s). The liveness entries with `Time(0)` will replace them. No change needed for the startup path.

## Alternative Approaches

1. **Subscribe to odom in bridge and camera mount, stamp with odom timestamp:** Would fully unify the time domain to Jetson time, but adds complexity (new subscribers, message filtering, handling both head and arm odom topics). Chosen approach is simpler and semantically correct for static transforms.

2. **Revert relay to use `self.get_clock().now()` (undo Fix 3):** Would restore all-host-time homogeneity but would re-introduce the original "extrapolation into the past" errors from v9 (125ms-1.16s gaps) since clouds are still stamped with Jetson time. Worse than the chosen approach.

3. **Only fix the bridge (skip camera mount):** The camera mount edges are further downstream in the chain and are already consistent with each other (both host time). The bridge edge is the critical junction where host and Jetson time domains meet. However, fixing both makes the solution complete and eliminates any residual time-domain issues in the bbox chain.
