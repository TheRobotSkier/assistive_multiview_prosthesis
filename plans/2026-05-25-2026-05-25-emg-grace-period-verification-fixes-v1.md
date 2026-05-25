# EMG Grace Period — Verification Findings & Fix Plan

## Objective

Fix all correctness issues discovered during cross-node verification of the EMG grace period implementation.

## Critical Issues Found

### CRITICAL 1: Gesture Label Mismatch (OPEN/FLEXION Swapped)

**Files:** `config.py`, `pipeline_manager_node.py`, `prosthesis_config.yaml`, `emg_grasp_state_machine.py`

The classifier (`config.py:19-25`) defines:
```
GESTURE_NAMES = ["REST", "POWER", "OPEN", "FLEXION", "EXTENSION"]
                  0        1        2       3          4
```

But all consumers expect the OLD ordering:
- `pipeline_manager_node.py:79-80`: `GESTURE_PINCH=2` (FLEXION alias), `GESTURE_OPEN=3`
- `prosthesis_config.yaml:89`: `release_gesture: 3`
- `prosthesis_config.yaml:294-296`: `gesture_flexion: 2`, `gesture_open: 3`
- `emg_grasp_state_machine.py:70-72`: `gesture_flexion=2`, `gesture_open=3`

**Impact:** User does OPEN → classifier outputs 2 → pipeline interprets as FLEXION (no release). User does FLEXION → classifier outputs 3 → pipeline interprets as OPEN (accidental release).

**Fix:** Align all consumers to match `config.py`. Change constants and configs to: OPEN=2, FLEXION=3.

### CRITICAL 2: Double EMG Processing in VOLITIONAL

Both `EmgGraspController` and `pipeline_manager` subscribe to raw `/emg/gesture_label` and process FLEXION/EXTENSION independently. In VOLITIONAL mode, every FLEXION/EXTENSION gesture produces:
1. Direct force/wrist command from pipeline_manager (`_on_emg_gesture`)
2. Intent from EmgGraspController → pipeline_manager (`_on_emg_grasp_intent`) → another force/wrist command

**Impact:** 2x actuation speed in VOLITIONAL mode.

**Fix:** Remove the intent relay processing for wrist/force commands from `_on_emg_grasp_intent`. The pipeline_manager's direct gesture processing is now the single source of truth for VOLITIONAL control. Keep only `STOP_ALL` intent handling.

### HIGH 3: Dead Code — `_confidence_threshold`

`pipeline_manager_node.py:141` loads `_confidence_threshold` from params and live config, but it is never used. The actual threshold is `_gesture_confidence_threshold` (line 144, used at line 375).

**Fix:** Remove `_confidence_threshold` entirely from `__init__` and `_load_emg_live_config`.

### HIGH 4: Wrist Target Position Drift

`_emg_wrist_target_deg` (line 175) is initialized to 0.0 and never reset to the actual hardware position. Between sessions or if the wrist is moved manually, the first wrist command will jump to a stale target.

**Fix:** Read the initial wrist position from `/wrist/state` on startup, and reset `_emg_wrist_target_deg` when entering VOLITIONAL mode.

## Implementation Plan

- [ ] **Task 1.** Fix gesture label constants in `pipeline_manager_node.py:77-81`:
  Change `GESTURE_PINCH = 2` comment to `# FLEXION (OPEN is 2 in config.py, FLEXION is 3)`.
  Actually: rename `GESTURE_PINCH` → `GESTURE_FLEXION = 3`, `GESTURE_OPEN = 2`, `GESTURE_POINT` → `GESTURE_EXTENSION = 4`.
  Update all references in `_on_emg_gesture` to use new names.

- [ ] **Task 2.** Fix `prosthesis_config.yaml` pipeline_manager section:
  Change `release_gesture: 3` → `release_gesture: 2` (OPEN is label 2).

- [ ] **Task 3.** Fix `prosthesis_config.yaml` emg_grasp_controller section:
  Change `gesture_flexion: 2` → `gesture_flexion: 3`, `gesture_open: 3` → `gesture_open: 2`.

- [ ] **Task 4.** Fix `config/emg_live.yaml`:
  Change `release_gesture: 3` → `release_gesture: 2`.

- [ ] **Task 5.** Fix `emg_grasp_state_machine.py:70-72`:
  Change `gesture_flexion: int = 3`, `gesture_open: int = 2` to match config.py.

- [ ] **Task 6.** Fix `prosthesis_config.yaml` reference section line 405:
  Change `gesture_names: ["REST", "POWER", "PINCH", "OPEN", "POINT"]` → `["REST", "POWER", "OPEN", "FLEXION", "EXTENSION"]`.

- [ ] **Task 7.** Fix double processing in `_on_emg_grasp_intent`:
  Remove `WRIST_VELOCITY_POSITIVE` and `WRIST_VELOCITY_NEGATIVE` handling. Keep only `STOP_ALL`.
  Remove `_INTENT_WRIST_POSITIVE` and `_INTENT_WRIST_NEGATIVE` constants.

- [ ] **Task 8.** Remove dead `_confidence_threshold`:
  Remove line 141 (`self._confidence_threshold = ...`).
  Remove the `confidence_threshold` reload block from `_load_emg_live_config` (lines 595-596).

- [ ] **Task 9.** Add wrist position initialization:
  Subscribe to `/wrist/state` (or read initial position from joint_states) to initialize `_emg_wrist_target_deg`.
  Reset `_emg_wrist_target_deg` to current position when entering VOLITIONAL.

## Verification Criteria

- [ ] `GESTURE_OPEN = 2` matches `config.py` GESTURE_NAMES[2] = "OPEN" everywhere
- [ ] `GESTURE_FLEXION = 3` matches `config.py` GESTURE_NAMES[3] = "FLEXION" everywhere
- [ ] No double force/wrist commands in VOLITIONAL mode
- [ ] `_confidence_threshold` is not referenced anywhere in the code
- [ ] Wrist target initialized from hardware, not hardcoded to 0.0
- [ ] `emg_live.yaml` release_gesture matches config.py OPEN label

## Potential Risks and Mitigations

1. **Trained model uses old label ordering**
   Mitigation: Verify the training data labels match config.py before deploying. The `collect_data` script uses config.py's GESTURE_NAMES, so if data was collected with the current config.py, the model is correct.

2. **Removing intent relay breaks future EmgGraspController features**
   Mitigation: Keep the `_on_emg_grasp_intent` callback and `STOP_ALL` handling. Only remove the duplicate wrist/force processing. The EmgGraspController can still publish intents for other purposes.

3. **Wrist state topic format unknown**
   Mitigation: Check `/wrist/state` message type and format before subscribing. Fall back to joint_states if needed.
