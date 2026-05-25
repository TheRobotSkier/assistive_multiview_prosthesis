# EMG Gesture Grace Period — Final Plan v4

## Objective

Add a hold-duration grace period to all discrete gesture actions in the pipeline manager, and make continuous gestures (FLEXION/EXTENSION) use proportional values. Uniform confidence and hold thresholds across all gestures.

## Desired Behavior

| Gesture | Context | Hold Required | Confidence | Action |
|---------|---------|---------------|------------|--------|
| **POWER** | IDLE | ≥1.0s | ≥0.7 | IDLE → TWISTING (activate) |
| **POWER** | VOLITIONAL | ≥1.0s | ≥0.7 | Toggle force ↔ wrist mode |
| **OPEN** | Any active | ≥1.0s | ≥0.7 | Any → RELEASING |
| **FLEXION** (PINCH=2) | VOLITIONAL | Immediate | — | Force+ or Wrist+ (proportional) |
| **EXTENSION** (POINT=4) | VOLITIONAL | Immediate | — | Force- or Wrist- (proportional) |
| **REST** | Any | — | — | **Ignored** |

Note: OPEN now uses the same thresholds as POWER (1.0s hold, 0.7 confidence) — uniform across all gestures.

## Implementation Plan

### File 1: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`

- [ ] **Task 1.** Add grace period parameters to `__init__` (after line 133)
  - Declare `gesture_hold_timeout_s` (default 1.0) — uniform for POWER and OPEN
  - Declare `gesture_confidence_threshold` (default 0.7) — uniform for POWER and OPEN
  - Read into instance variables: `_gesture_hold_timeout_s`, `_gesture_confidence_threshold`

- [ ] **Task 2.** Add gesture tracking state variables (after line 186)
  - `_pending_gesture: int = GESTURE_REST`
  - `_pending_gesture_start: float = time.monotonic()`
  - `_gesture_action_fired: bool = False`

- [ ] **Task 3.** Add proportional subscription and tracking
  - Subscribe to `/emg/proportional` (Float32) with a `_on_emg_proportional` callback
  - Store `_latest_proportional: float = 0.0`
  - This is needed for FLEXION/EXTENSION continuous control in VOLITIONAL

- [ ] **Task 4.** Rewrite `_on_emg_gesture` (lines 361-456) — new logic:
  1. **Gesture change detection**: When gesture changes, reset `_pending_gesture_start`, set `_gesture_action_fired = False`, update `_pending_gesture`
  2. **Compute hold duration**: `held = time.monotonic() - _pending_gesture_start`
  3. **OPEN release** (from any active state except IDLE/RELEASING): require `held >= _gesture_hold_timeout_s` AND `confidence >= _gesture_confidence_threshold` AND `_gesture_action_fired == False`. On trigger: transition to RELEASING, deactivate twist, schedule release complete, set `_gesture_action_fired = True`
  4. **POWER from IDLE**: require `held >= _gesture_hold_timeout_s` AND `confidence >= _gesture_confidence_threshold` AND `_gesture_action_fired == False`. On trigger: transition to TWISTING, activate twist propagation, set `_gesture_action_fired = True`
  5. **POWER in VOLITIONAL** (mode toggle): require `held >= _gesture_hold_timeout_s` AND `_gesture_action_fired == False`. On trigger: toggle mode, set `_gesture_action_fired = True`
  6. **FLEXION (PINCH) in VOLITIONAL**: immediate, proportional — use `_latest_proportional` to scale force adjustment or wrist velocity
  7. **EXTENSION (POINT) in VOLITIONAL**: immediate, proportional — use `_latest_proportional` to scale force adjustment or wrist velocity
  8. **REST and everything else**: ignored

- [ ] **Task 5.** Update FLEXION/EXTENSION handling in VOLITIONAL to use proportional values
  - Currently lines 404-429 use fixed step sizes (`_volitional_force_step`, `_volitional_wrist_scale * 0.1`)
  - Change to scale by `_latest_proportional`:
    - Force mode: `adjustment = _volitional_force_step * _latest_proportional`
    - Wrist mode: `delta = _volitional_wrist_scale * 0.1 * _latest_proportional`
  - If `_latest_proportional == 0.0`, skip publishing (no movement)
  - Rename gesture constants in code comments: PINCH → FLEXION, POINT → EXTENSION (gesture IDs stay the same: 2 and 4)

- [ ] **Task 6.** Remove from `_on_emg_gesture`:
  - Emergency stop block (lines 364-368)
  - Abort gesture block (lines 371-375)
  - Manual tighten/loosen blocks (lines 432-444)
  - `grasp_gestures` list check (line 447) — replace with direct POWER check

- [ ] **Task 7.** Clean up unused parameters from `__init__`:
  - Remove declarations: `abort_gestures` (105), `emergency_stop_gesture` (106), `manual_tighten_gesture` (107), `manual_loosen_gesture` (108), `release_debounce_frames` (115), `release_confidence_threshold` (99 — replaced by uniform threshold), `grasp_gestures` (100)
  - Remove instance variable reads: lines 148-153, 157
  - Remove: `_abort_gestures`, `_emergency_stop_gesture`, `_manual_tighten_gesture`, `_manual_loosen_gesture`, `_release_debounce_frames`, `_release_confidence_threshold`, `_grasp_gestures`

- [ ] **Task 8.** Update `_load_emg_live_config` (lines 578-600):
  - Add: `gesture_hold_timeout_s`, `gesture_confidence_threshold`
  - Remove: `grasp_gestures`, `release_confidence_threshold` loading

- [ ] **Task 9.** Update docstring (lines 1-34) to reflect simplified gesture contract

### File 2: `config/emg_live.yaml`

- [ ] **Task 10.** Add grace period parameters, remove unused ones
  - Add: `gesture_hold_timeout_s: 1.0`
  - Add: `gesture_confidence_threshold: 0.7`
  - Remove: `grasp_gestures: [1, 2, 4]`
  - Remove: `release_confidence_threshold: 0.25` (replaced by uniform threshold)
  - Keep: `confidence_threshold`, `release_gesture`, `emg_adapter` section

### File 3: `config/prosthesis_config.yaml`

- [ ] **Task 11.** Update `pipeline_manager` ROS parameter section:
  - Add: `gesture_hold_timeout_s: 1.0`
  - Add: `gesture_confidence_threshold: 0.7`
  - Remove: `grasp_gestures: [1]`
  - Remove: `release_confidence_threshold: 0.8` (replaced by uniform threshold)
  - Keep: `confidence_threshold`, `release_gesture`, topic names

## What Gets Removed

| Concept | Why |
|---------|-----|
| Emergency stop | OPEN releases the hand |
| REST abort | OPEN handles release. REST is ignored |
| Manual tighten/loosen | Handled by FLEXION/EXTENSION proportional |
| `grasp_gestures` list | Only POWER activates |
| `release_confidence_threshold` | Uniform `gesture_confidence_threshold` for all gestures |
| `release_debounce_frames` | Replaced by hold-duration |

## What Stays Unchanged

- `_on_emg_grasp_intent` — intent relay from EmgGraspController (wrist velocity intents with proportional)
- Force controller status → HOLDING → VOLITIONAL transition
- Twist propagation activation/deactivation
- Release monitor (joint position check)
- All state transitions (TWISTING→SEGMENTING→PLANNING→APPROACHING→GRASPING→HOLDING→VOLITIONAL→RELEASING→IDLE)

## Verification Criteria

- [ ] REST does nothing — no abort, no state change
- [ ] POWER must be held ≥1.0s with confidence ≥0.7 to activate from IDLE
- [ ] POWER must be held ≥1.0s with confidence ≥0.7 to toggle force↔wrist in VOLITIONAL
- [ ] OPEN must be held ≥1.0s with confidence ≥0.7 to release
- [ ] FLEXION in VOLITIONAL scales force/wrist by proportional value
- [ ] EXTENSION in VOLITIONAL scales force/wrist by proportional value
- [ ] Zero proportional → no movement (FLEXION/EXTENSION produce no command)
- [ ] All timeouts configurable at runtime via `emg_live.yaml`
