# Fix TF Wait Gate: Support Either Camera Initializing First

## Objective

Change `_check_tf_ready()` in the pointcloud fusion node to check **both** camera depth optical frames and open the gate as soon as **either** one connects. Currently it only checks `head_d435i_head_depth_optical_frame`, which means if the head camera never initializes (a known edge case), the gate stays shut permanently even though arm data is valid and `require_both=False`.

## Implementation Plan

- [ ] Task 1. **Add class-level constant `_GATE_DEPTH_FRAMES`** listing both depth optical frames that the gate should poll.
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`
  - Add above `_check_tf_ready()` (around line 741):
    ```python
    _GATE_DEPTH_FRAMES = [
        "head_d435i_head_depth_optical_frame",
        "arm_d435i_arm_depth_optical_frame",
    ]
    ```
  - Rationale: A class-level list makes it easy to add or remove cameras without restructuring the method logic.

- [ ] Task 2. **Rewrite `_check_tf_ready()` to loop over both frames** instead of a single hardcoded try/except.
  - Replace the single frame check at lines 757-770 with a loop that tries each frame in `_GATE_DEPTH_FRAMES`.
  - For each frame, attempt `self._tf_buffer.lookup_transform(self._target_frame, depth_frame, ...)`.
  - On first success: set `_tf_ready = True`, log the specific frame that connected, cancel the timer, reset bbox health, return.
  - If all frames fail: log a debug message noting neither camera is ready yet (keep the existing "not yet connected" message but pluralize to "either camera").
  - Rationale: This is the minimal change — same logic, just iterating over a list instead of a single lookup.

- [ ] Task 3. **Update the log message on success** to indicate which camera triggered the gate.
  - Change `"TF tree connected — starting point cloud fusion"` to include the frame name: `f"TF tree connected via {depth_frame!r} — starting point cloud fusion (waited {waited_s:.1f}s since node startup)"`.
  - Rationale: This provides operational clarity — the operator knows whether head or arm enabled fusion.

- [ ] Task 4. **Update the debug log message on failure** to reflect that both cameras are being checked.
  - Change `"TF tree not yet connected — waiting for OpenVINS bridge"` to `"TF tree not yet connected for either camera — waiting for OpenVINS bridge"`.
  - Rationale: Makes it clear from logs that the gate is polling both, not just head.

- [ ] Task 5. **Verify the `_synced_callback` gate** still works correctly with no changes needed.
  - Confirm lines 403-404 (`if not self._tf_ready: return`) are unchanged — the gate behavior is identical regardless of which camera opened it.
  - Rationale: The early-return gate doesn't need to know which camera triggered readiness.

## Verification Criteria

- [Gate opens when arm initializes first]: With head disconnected, arm VIO converging should open the gate within `tf_ready_check_interval` seconds.
- [Gate opens when head initializes first]: Existing behavior preserved — head connecting first still opens the gate immediately.
- [Gate opens when both connect simultaneously]: Either frame lookup succeeds; gate opens on first success.
- [Gate stays closed when neither connects]: Both lookups fail → gate stays closed → no spurious cloud processing.
- [Log message names the triggering camera]: `"TF tree connected via 'arm_d435i_arm_depth_optical_frame'"` or `"...head_d435i_head_depth_optical_frame'"` appears in logs.
- [Bbox health reset fires regardless of which camera opened gate]: Both `_bbox_attempts` and `_bbox_successes` reset to 0.
- [Timer canceled on first success]: No further `_check_tf_ready()` calls after gate opens.
- [Backward compatibility]: `wait_for_tf=False` still skips the gate entirely (no regression).
- [Existing tests pass]: All 8 fusion node tests continue to pass.

## Potential Risks and Mitigations

1. **Race condition: head connects between checking arm and head**
   Mitigation: Not a problem — the loop checks head after arm, so head connecting late in the same poll cycle is caught. The gate opens on whichever succeeds first; order doesn't matter.

2. **TF lookup timeout accumulates across frames (1s × 2 = 2s)**
   Mitigation: The timer already fires every `tf_ready_check_interval` seconds (default 2s), so a 2s cumulative lookup fits within one cycle. If both time out, the timer fires again. Acceptable.

## Alternative Approaches

1. **Check both frames simultaneously with two `lookup_transform` calls on separate threads**: Overkill — the sequential 1+1=2s is within one gate cycle. No benefit to parallelism.

2. **Make the frame list a ROS parameter**: Flexible but unnecessary complexity. The frame names follow the RealSense naming convention and are stable. Hardcoded class constant is simpler and sufficient.

3. **Use `canTransform` instead of `lookup_transform`**: `canTransform` returns a bool without throwing, which is cleaner. However, `lookup_transform` already works with the try/except pattern used throughout the codebase. Keeping consistency is preferred.
