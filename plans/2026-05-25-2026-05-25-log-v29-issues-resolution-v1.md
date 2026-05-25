# Log v29 Issues Resolution

## Objective

Fix the rapid abort problem in the pipeline manager where REST gestures (gesture=0) immediately cancel any active pipeline state, and address the wrist driver communication issues.

## Issues Found

### Issue 1 (CRITICAL): EMG Abort Fires Instantly on Single REST Frame

**Evidence from log:**

The pipeline manager transitions IDLE->TWISTING and then back to TWISTING->IDLE within **0.3–1.8 seconds** every time. Looking at the timestamps:

| Line | Event | Delta |
|------|-------|-------|
| 370 | IDLE -> TWISTING (gesture=1) | — |
| 375 | TWISTING -> IDLE (abort gesture=0) | **0.35s** |
| 432 | IDLE -> TWISTING (gesture=4) | — |
| 448 | SEGMENTING -> IDLE (abort gesture=0) | **1.97s** |
| 453 | IDLE -> TWISTING (gesture=1) | — |
| 457 | TWISTING -> IDLE (abort gesture=0) | **1.30s** |
| 514 | IDLE -> TWISTING (gesture=2) | — |
| 518 | TWISTING -> IDLE (abort gesture=0) | **0.90s** |
| 530 | IDLE -> TWISTING (gesture=2) | — |
| 535 | TWISTING -> IDLE (abort gesture=0) | **1.25s** |

This happens **20+ times** in the log. The EMG classifier at 10 Hz publishes REST (gesture=0) between intentional gestures. Even a single frame of REST immediately aborts.

**Root cause:** In `pipeline_manager_node.py:370-375`, the `_on_emg_gesture` callback treats REST (gesture=0) as an abort gesture with **zero debounce/grace period**. The EMG classifier runs at 10 Hz, so a single frame of misclassification (REST between two POWER frames) kills the entire pipeline state.

The EMG grasp controller (`emg_grasp_state_machine.py`) already has hold-duration logic for OPEN and POWER, but the pipeline manager subscribes directly to `/emg/gesture_label` (raw gesture), not the filtered output of the state machine. The state machine's intents go to `/emg_grasp/intent`, but the pipeline manager's abort logic bypasses that entirely.

**Fix:** Add a configurable grace period to the abort logic in `_on_emg_gesture`. The REST gesture must be sustained for a minimum duration (default 1.0s) before aborting. This mirrors the OPEN hold-duration logic already in the state machine.

### Issue 2 (MEDIUM): Wrist Driver Communication Errors

**Evidence from log (lines 431, 445-447, 498-500, 505-506, 513, 522, 572, 642, 667):**

```
[wrist_driver] Failed to set position: [TxRxResult] There is no status packet!
[wrist_driver] Dynamixel read failed: pos=[TxRxResult] Communication success!, vel=[TxRxResult] Incorrect status packet!
[wrist_driver] Dynamixel read failed (communication error): list index out of range
```

The wrist driver is surviving these errors (thanks to the v23 fix), but the errors are very frequent — roughly every 5-15 seconds. The "Port is in use" errors from v28 are gone, replaced by these more specific Dynamixel SDK errors. This is a hardware issue (USB contention with Mia Hand on the same bus, or loose USB connection). No code change needed — the error handling is working correctly.

### Issue 3 (LOW): Segmentation Inference Failures

**Evidence from log (lines 527, 581, 681-682):**

```
[segmentation_bridge] Inference request failed: ('Connection aborted.', RemoteDisconnected(...))
```

The segmentation CPU container's HTTP server occasionally drops connections. This happens 3 times in the ~4.5 minute run. Not a code bug — the segmentation server is likely overloaded or restarting.

### Non-issues (per user instructions)

- **OpenVINS head MISSING**: Expected — head not initialized
- **Mia Hand disconnects**: Auto-reconnects, working as designed
- **Pointcloud fusion TF failures**: Consequence of missing head OpenVINS
- **Shutdown noise**: Standard ROS 2 cleanup

## Implementation Plan

- [ ] **Task 1.** Add abort grace period to pipeline manager `_on_emg_gesture` callback
  - Add new parameters: `abort_grace_period_s` (default 1.0) — how long REST must be sustained before aborting
  - Add state tracking: `_abort_start_time` (when REST first appeared), `_abort_pending` (bool)
  - In `_on_emg_gesture`, when an abort gesture arrives:
    - If not already pending: record start time, set pending flag, log debug message
    - If pending and grace period elapsed: execute abort, clear pending state
    - If a non-abort gesture arrives while pending: clear the pending abort (the REST was transient)
  - This mirrors the OPEN hold-duration pattern in `emg_grasp_state_machine.py:130-146`

- [ ] **Task 2.** Add `abort_grace_period_s` to `emg_live.yaml` config
  - Add `abort_grace_period_s: 1.0` to the live config so it can be tuned at runtime
  - Add config reload handling in `_on_emg_config_timer` (alongside existing confidence/grasp parameters)

- [ ] **Task 3.** Verify the fix by re-reading the modified code
  - Ensure the grace period only applies to abort gestures (REST), not to release (OPEN) or emergency stop
  - Ensure the grace period clears properly on state transitions

## Verification Criteria

- [ ] A single frame of REST no longer aborts the pipeline
- [ ] Sustained REST for >1s still correctly aborts
- [ ] A brief REST flicker (<1s) followed by a grasp gesture clears the pending abort
- [ ] OPEN release gesture still works immediately (no grace period)
- [ ] Emergency stop still works immediately (no grace period)
- [ ] The grace period is configurable via `emg_live.yaml` without restart

## Potential Risks and Mitigations

1. **Grace period too long — user can't abort quickly**
   Mitigation: Default 1.0s is short enough to feel responsive. Configurable at runtime via `emg_live.yaml`. OPEN release gesture bypasses grace period entirely.

2. **Grace period interacts with stale EMG timeout**
   Mitigation: The stale timeout in `emg_grasp_controller.py` (1.0s) injects REST when data is lost. The abort grace period (1.0s) means stale data won't abort for 1s, giving time for the connection to recover.

## Alternative Approaches

1. **Use the state machine's filtered output instead of raw gesture**: The `emg_grasp_state_machine` already has hold-duration logic. The pipeline manager could subscribe to intents instead of raw gesture labels. This would be a larger refactor but more architecturally clean. Rejected for now — the grace period is a minimal, targeted fix.

2. **Add debounce at the EMG bridge level**: Filter REST gestures in `ros_bridge_node.py` before publishing. Rejected — this would hide information from other subscribers and make debugging harder.
