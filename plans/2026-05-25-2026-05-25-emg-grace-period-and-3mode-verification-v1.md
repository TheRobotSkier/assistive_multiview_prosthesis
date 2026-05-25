# EMG Grace Period + 3-Mode Integration Verification

## Objective

1. Add a configurable grace period to **all** EMG gesture recognition in the pipeline manager (activate, abort, release, mode toggle)
2. Verify and document how the 3-mode EMG state machine integrates with the pipeline manager

## Architecture Analysis: Two Parallel EMG Paths

The system has **two separate EMG processing paths** that both subscribe to the same raw `/emg/gesture_label` topic:

```
MindRove → /emg/gesture_label ──┬── Pipeline Manager (_on_emg_gesture)
                                  │     Direct raw gesture processing
                                  │     Handles: activate, abort, release, volitional
                                  │
                                  └── EMG Grasp Controller (emg_grasp_state_machine)
                                        Hold-duration filtered processing
                                        Handles: mode toggles, wrist/force control
                                        Publishes intents to /emg_grasp/intent
```

**The pipeline manager subscribes to BOTH:**
- `/emg/gesture_label` (raw, line 211-213) — for state transitions (activate/abort/release)
- `/emg_grasp/intent` (filtered, line 233-234) — for wrist velocity and stop commands

**The problem:** The pipeline manager's raw gesture path has **zero debouncing**, while the EMG grasp controller has proper hold-duration logic (1.0s for OPEN, 1.0s for POWER). A single frame of REST from the classifier immediately aborts the pipeline.

## 3-Mode Integration Status

The 3-mode state machine (NOT_GRASPING / CONTROL_GRASP / CONTROL_WRIST) is **partially integrated**:

| Intent | Emitted By | Consumed By | Status |
|--------|-----------|-------------|--------|
| `RESET_HAND` | SM on OPEN hold | Pipeline manager | **NOT CONSUMED** — pipeline manager handles OPEN directly via raw gesture |
| `ENTER_FORCE_HOLD` | SM on POWER in NOT_GRASPING/WRIST | Pipeline manager | **NOT CONSUMED** — no handler for this intent type |
| `ADJUST_FORCE_TARGET` | SM on FLEXION/EXTENSION in CONTROL_GRASP | Pipeline manager | **NOT CONSUMED** — no handler for this intent type |
| `STOP_ALL` | SM on REST | Pipeline manager | **PARTIALLY** — only stops wrist, doesn't abort pipeline |
| `WRIST_VELOCITY_POSITIVE/NEGATIVE` | SM on FLEXION/EXTENSION | Pipeline manager | **WORKS** — lines 564-576 |

The 3-mode state machine is running but its mode transitions (NOT_GRASPING ↔ CONTROL_GRASP ↔ CONTROL_WRIST) are **not connected** to the pipeline state machine. The pipeline manager has its own duplicate mode tracking (`_emg_volitional_mode`).

**However**, the volitional mode in the pipeline manager (lines 392-430) does handle POWER toggle, FLEXION (PINCH), and EXTENSION (POINT) gestures — just via the raw gesture path, not via the state machine's intents.

**Conclusion:** The 3-mode state machine is architecturally redundant with the pipeline manager's own gesture handling. For now, the grace period fix should be applied to the pipeline manager's raw gesture path (the path that actually controls state transitions). The state machine integration can be improved later as a separate task.

## Implementation Plan

- [ ] **Task 1.** Add grace period infrastructure to `PipelineManagerNode.__init__`
  - Add parameters: `gesture_grace_period_s` (default 1.0) — minimum sustained duration for any gesture to take effect
  - Add state tracking variables:
    - `_pending_gesture: int` — the gesture currently being held
    - `_pending_gesture_start: float` — monotonic time when the gesture first appeared
    - `_pending_gesture_confidence: float` — confidence at the time of first detection
  - Add `gesture_grace_period_s` to `emg_live.yaml` for runtime tuning
  - Add reload handling in `_load_emg_live_config`

- [ ] **Task 2.** Refactor `_on_emg_gesture` to use grace period for all gestures
  - Track the current pending gesture and its start time
  - When a gesture arrives:
    - If same as pending gesture: check if grace period elapsed → execute
    - If different from pending gesture: reset tracking to new gesture, start timer
  - The grace period applies to: grasp activation (POWER/PINCH/POINT), abort (REST), release (OPEN), mode toggle (POWER in VOLITIONAL)
  - Emergency stop remains instant (no grace period)
  - Log at debug level when a gesture is pending ("Gesture X pending, 0.3s/1.0s grace")
  - Log at info level when a pending gesture is cancelled by a different gesture

- [ ] **Task 3.** Add `gesture_grace_period_s` to `emg_live.yaml`
  - Add `gesture_grace_period_s: 1.0` alongside existing parameters
  - Ensure `_load_emg_live_config` reads and applies it

- [ ] **Task 4.** Verify final implementation
  - Re-read modified code to ensure correctness
  - Verify emergency stop bypasses grace period
  - Verify grace period resets properly on gesture changes
  - Verify config reload works

## Verification Criteria

- [ ] A single frame of any gesture does NOT trigger a state transition
- [ ] A gesture sustained for >1.0s triggers the appropriate action
- [ ] Switching from one gesture to another resets the grace period timer
- [ ] Brief gesture flickers (<1.0s) are silently ignored
- [ ] Emergency stop works immediately (no grace period)
- [ ] The grace period is configurable at runtime via `emg_live.yaml`
- [ ] All 3 EMG modes (NOT_GRASPING, CONTROL_GRASP, CONTROL_WRIST) are running
- [ ] Wrist velocity intents from the state machine reach the pipeline manager

## Potential Risks and Mitigations

1. **Grace period makes the system feel sluggish**
   Mitigation: 1.0s default is a reasonable starting point. Can be tuned down to 0.5s or even 0.3s via `emg_live.yaml` without restart. The classifier runs at 10 Hz, so even 0.3s requires 3 consecutive frames.

2. **Grace period delays safety-critical release**
   Mitigation: OPEN release already has a confidence threshold. The grace period ensures sustained intent. If needed, the grace period can be set lower for release gestures specifically.

3. **Interaction with existing EMG state machine hold durations**
   Mitigation: The state machine has its own hold-duration logic (1.0s for OPEN, 1.0s for POWER). The pipeline manager's grace period stacks on top of this for the raw gesture path. This means POWER needs to be held for max(state_machine_hold, grace_period) = ~1.0s, which is acceptable.

## Alternative Approaches

1. **Use state machine intents exclusively**: Route all gesture processing through the 3-mode state machine, have the pipeline manager only subscribe to `/emg_grasp/intent`. This would eliminate the duplicate processing but requires adding pipeline state transition intents (ENTER_TWISTING, ABORT_PIPELINE, etc.) to the state machine. Larger refactor, deferred.

2. **Add debounce at the EMG bridge level**: Filter gestures in `ros_bridge_node.py` before publishing. Rejected — hides information from other subscribers.
