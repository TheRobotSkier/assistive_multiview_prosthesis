# EMG Gesture Grace Period — Final Plan v3

## Objective

Add a hold-duration grace period to **all discrete gesture actions** in the pipeline manager's `_on_emg_gesture` callback. Continuous actions (FLEXION/PINCH and EXTENSION/POINT in VOLITIONAL mode) remain immediate.

## Current State (main worktree, `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`)

The pipeline manager already has:
- `State.VOLITIONAL` with POWER toggle between "force" and "wrist" modes (lines 393-402)
- FLEXION/PINCH → force+/wrist+ and EXTENSION/POINT → force-/wrist- in VOLITIONAL (lines 404-429)
- `State.TWISTING` — POWER from IDLE → TWISTING (line 453)
- OPEN release with confidence check (lines 378-390)
- REST abort — immediate, no debounce (lines 371-375)
- Emergency stop — disabled (gesture = -1) (lines 365-368)

**What's missing:** Hold-duration grace period on all discrete actions.

## Desired Behavior After Fix

| Gesture | Context | Hold Required | Confidence | Action |
|---------|---------|---------------|------------|--------|
| POWER | IDLE | ≥1.0s | ≥0.7 | IDLE → TWISTING (activate pipeline) |
| POWER | VOLITIONAL | ≥1.0s | ≥0.7 | Toggle force ↔ wrist mode |
| OPEN | Any active state | ≥0.5s | ≥0.25 | Any → RELEASING |
| PINCH | VOLITIONAL | Immediate | — | Force+ or Wrist+ (continuous) |
| POINT | VOLITIONAL | Immediate | — | Force- or Wrist- (continuous) |
| REST | Any | — | — | **Ignored** — no action at all |

**Removed:** Emergency stop, REST abort, force emergency override, manual tighten/loosen gestures.

## Implementation Plan

### File 1: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`

- [ ] **Task 1.** Add grace period parameters to `__init__` (after line 133)
  - Declare `gesture_hold_timeout_s` (default 1.0) — for POWER activation and mode toggle
  - Declare `release_hold_timeout_s` (default 0.5) — for OPEN release
  - Declare `gesture_confidence_threshold` (default 0.7) — minimum confidence for POWER actions
  - Read into instance variables after existing parameter reads

- [ ] **Task 2.** Add gesture tracking state variables (after line 186)
  - `_pending_gesture: int = GESTURE_REST` — current gesture being held
  - `_pending_gesture_start: float = time.monotonic()` — when gesture first appeared
  - `_gesture_action_fired: bool = False` — prevents re-firing during one hold

- [ ] **Task 3.** Rewrite `_on_emg_gesture` (lines 361-456) — new logic:
  1. **Gesture change detection**: When gesture changes from `_pending_gesture`, reset `_pending_gesture_start = time.monotonic()`, set `_gesture_action_fired = False`, update `_pending_gesture`
  2. **Compute hold duration**: `held = time.monotonic() - _pending_gesture_start`
  3. **OPEN release** (from any active state except IDLE/RELEASING): require `held >= _release_hold_timeout_s` AND `confidence >= _release_confidence_threshold` AND `_gesture_action_fired == False`. On trigger: transition to RELEASING, deactivate twist, schedule release complete, set `_gesture_action_fired = True`
  4. **POWER from IDLE**: require `held >= _gesture_hold_timeout_s` AND `confidence >= _gesture_confidence_threshold` AND `_gesture_action_fired == False`. On trigger: transition to TWISTING, activate twist propagation, set `_gesture_action_fired = True`
  5. **POWER in VOLITIONAL** (mode toggle): require `held >= _gesture_hold_timeout_s` AND `_gesture_action_fired == False`. On trigger: toggle `_emg_volitional_mode` between "force" and "wrist", set `_gesture_action_fired = True`
  6. **PINCH/POINT in VOLITIONAL**: remain immediate (no hold required) — these are continuous controls
  7. **REST and everything else**: ignored — no action

- [ ] **Task 4.** Remove from `_on_emg_gesture`:
  - Emergency stop block (lines 364-368) — remove entirely
  - Abort gesture block (lines 371-375) — remove entirely (REST is ignored)
  - Manual tighten/loosen blocks (lines 432-444) — remove entirely
  - `grasp_gestures` list check (line 447) — replace with direct POWER check

- [ ] **Task 5.** Clean up unused parameters from `__init__`:
  - Remove declarations: `abort_gestures` (line 105), `emergency_stop_gesture` (line 106), `manual_tighten_gesture` (line 107), `manual_loosen_gesture` (line 108), `release_debounce_frames` (line 115)
  - Remove instance variable reads: lines 150-153, 157
  - Remove: `_abort_gestures`, `_emergency_stop_gesture`, `_manual_tighten_gesture`, `_manual_loosen_gesture`, `_release_debounce_frames`
  - Remove: `_grasp_gestures` — no longer needed (POWER only)

- [ ] **Task 6.** Update `_load_emg_live_config` (lines 578-600) to load grace period params:
  - Add: `gesture_hold_timeout_s`, `release_hold_timeout_s`, `gesture_confidence_threshold`
  - Remove: `grasp_gestures` loading (line 594-595) — no longer configurable

- [ ] **Task 7.** Update docstring (lines 1-34) to reflect simplified gesture contract:
  - POWER only activates (not POWER/PINCH/POINT)
  - No emergency stop, no REST abort
  - OPEN releases with hold duration
  - POWER toggles modes with hold duration

### File 2: `config/emg_live.yaml`

- [ ] **Task 8.** Add grace period parameters, remove unused ones
  - Add: `gesture_hold_timeout_s: 1.0`
  - Add: `release_hold_timeout_s: 0.5`
  - Add: `gesture_confidence_threshold: 0.7`
  - Remove: `grasp_gestures: [1, 2, 4]` — POWER only, hardcoded
  - Keep: `confidence_threshold`, `release_confidence_threshold`, `release_gesture`, `emg_adapter` section

### File 3: `config/prosthesis_config.yaml`

- [ ] **Task 9.** Update `pipeline_manager` ROS parameter section:
  - Add: `gesture_hold_timeout_s: 1.0`
  - Add: `release_hold_timeout_s: 0.5`
  - Add: `gesture_confidence_threshold: 0.7`
  - Remove: `grasp_gestures: [1]` — no longer used
  - Keep: `confidence_threshold`, `release_confidence_threshold`, `release_gesture`, topic names

## What Gets Removed

| Concept | Lines | Why |
|---------|-------|-----|
| Emergency stop | 364-368 | Not needed — OPEN releases |
| REST abort | 371-375 | OPEN handles release. REST is ignored |
| Manual tighten/loosen | 432-444 | Handled by EmgGraspController |
| `grasp_gestures` list | 100, 148, 447, 594 | Only POWER activates — hardcoded |
| Force emergency override | N/A | Not in main worktree |
| `release_debounce_frames` | 115, 157 | Replaced by hold-duration |

## What Stays Unchanged

- VOLITIONAL mode: FLEXION/PINCH and EXTENSION/POINT continuous control (immediate, no grace period)
- `_on_emg_grasp_intent` — intent relay from EmgGraspController
- Force controller status handling → HOLDING → VOLITIONAL transition
- Twist propagation activation/deactivation
- Release monitor (joint position check)
- All state transitions (TWISTING→SEGMENTING→PLANNING→APPROACHING→GRASPING→HOLDING→VOLITIONAL→RELEASING→IDLE)

## Verification Criteria

- [ ] A single frame of REST does nothing — no abort, no state change
- [ ] POWER must be held ≥1.0s with confidence ≥0.7 to activate from IDLE
- [ ] POWER must be held ≥1.0s to toggle between force/wrist mode in VOLITIONAL
- [ ] OPEN must be held ≥0.5s with confidence ≥0.25 to release
- [ ] PINCH and POINT in VOLITIONAL remain immediate (continuous control)
- [ ] All timeouts configurable at runtime via `emg_live.yaml`
- [ ] No emergency stop, no REST abort, no manual tighten/loosen in the code

## Potential Risks and Mitigations

1. **1.0s hold for POWER activation feels slow**
   Mitigation: Configurable at runtime. Can reduce to 0.5s via `emg_live.yaml`.

2. **1.0s hold for POWER mode toggle feels slow in VOLITIONAL**
   Mitigation: Same — configurable. The user must deliberately hold POWER to switch modes.

3. **No way to abort without OPEN**
   Mitigation: OPEN always releases. Ctrl+C stops the pipeline. REST is simply ignored, not an error.

4. **Removing grasp_gestures list means PINCH/POINT can't activate the pipeline**
   This is intentional — only POWER activates. PINCH/POINT are reserved for continuous control in VOLITIONAL.
