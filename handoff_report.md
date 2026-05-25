# Handoff Report — Critical Bug Fixes (2025-05-25)

## Fix 1: EMG Gesture Contract Mismatch (mvp-68x)

### What was wrong

The EMG gesture integer labels were out of sync between the classifier (source of
truth) and all downstream consumers. The classifier in
`src/emg_bridge/emg_bridge/config.py:19` publishes labels on `/emg/gesture_label`:

| Integer | Gesture |
|---------|---------|
| 0 | REST |
| 1 | POWER |
| 2 | OPEN |
| 3 | FLEXION |
| 4 | EXTENSION |

But the pipeline_manager (`pipeline_manager_node.py:78`) used its own private
constants that did **not** match:

| Integer | Pipeline (wrong) | Classifier (correct) |
|---------|-------------------|----------------------|
| 2 | `GESTURE_PINCH` | OPEN |
| 3 | `GESTURE_OPEN` | FLEXION |
| 4 | `GESTURE_POINT` | EXTENSION |

### Why it was bad

The live config (`config/emg_live.yaml`) is loaded at runtime by pipeline_manager
and overrides `grasp_gestures` and `release_gesture` parameters. It specified:

```yaml
grasp_gestures: [1, 2, 4]  # meant to be POWER, PINCH, POINT
release_gesture: 3          # meant to be OPEN
```

Because the integer 2 from the classifier is **OPEN** (not PINCH), and the integer
3 is **FLEXION** (not OPEN), the system had a **full gesture inversion**:

- User performs **OPEN** (classifier → 2) → pipeline treats as PINCH → **triggers grasp**
- User performs **FLEXION** (classifier → 3) → pipeline treats as OPEN → **triggers release**

On the default pipeline path, OPEN would close the hand instead of releasing it.
The emg_grasp_controller had the same broken defaults in
`emg_grasp_state_machine.py:70` and `emg_grasp_controller.py:65`.

### The fix

Aligned all gesture constants and configs to match the classifier's numbering:
`0=REST, 1=POWER, 2=OPEN, 3=FLEXION, 4=EXTENSION`.

**Files changed (5 files, 23 insertions, 23 deletions):**

| File | Change |
|------|--------|
| `src/pipeline_manager/pipeline_manager_node.py:78-82` | `GESTURE_PINCH` → `GESTURE_OPEN=2`, `GESTURE_OPEN` → `GESTURE_FLEXION=3`, `GESTURE_POINT` → `GESTURE_EXTENSION=4`. Updated docstring, default `grasp_gestures` param, volitional mode gesture checks. |
| `config/emg_live.yaml:4-5` | `grasp_gestures: [1, 3, 4]` (POWER, FLEXION, EXTENSION), `release_gesture: 2` (OPEN). |
| `config/prosthesis_config.yaml:90,294,297` | Default `release_gesture: 2`, `gesture_flexion: 3`, `gesture_open: 2`. |
| `src/emg_bridge/emg_bridge/emg_grasp_state_machine.py:70-72` | Defaults `gesture_flexion=3`, `gesture_open=2`. |
| `src/emg_bridge/emg_bridge/emg_grasp_controller.py:16,19,65,67` | Docstring and `declare_parameter` defaults updated. |

**Semantic preservation:** The gesture-to-action mapping remains the same intent:
- **POWER (1)** → grasp trigger and volitional mode toggle
- **FLEXION (3)** → grasp trigger (was PINCH alias), volitional force+ / wrist+
- **EXTENSION (4)** → grasp trigger (was POINT alias), volitional force- / wrist-
- **OPEN (2)** → release from any active state

---

## Fix 2: Pipeline State Enum Drift (mvp-9g4)

### What was wrong

The pipeline_manager expanded its `State` IntEnum from 6 states to 9 states
(adding `VOLITIONAL=7` and `RELEASING=8`), but two dependent nodes still
hard-coded the old numbering:

**force_controller** (`force_controller_node.py:76-82`) had completely wrong numbering:

| Name | force_controller (wrong) | pipeline (correct) |
|------|--------------------------|---------------------|
| `STATE_IDLE` | 0 | 0 ✓ |
| `STATE_SEGMENTING` | 1 | TWISTING=1 ✗ |
| `STATE_PLANNING` | 2 | SEGMENTING=2 ✗ |
| `STATE_APPROACHING` | 3 | PLANNING=3 ✗ |
| `STATE_GRASPING` | 4 | APPROACHING=4 ✗ |
| `STATE_HOLDING` | 5 | GRASPING=5 ✗ |
| `STATE_RELEASING` | 6 | HOLDING=6 ✗ |
| (missing) | — | VOLITIONAL=7 |
| (missing) | — | RELEASING=8 |

**haptic_controller** (`haptic_controller_node.py:24-27`):

| Name | haptic (wrong) | pipeline (correct) |
|------|----------------|---------------------|
| `_GRASPING` | 4 | 5 |
| `_HOLDING` | 5 | 6 |
| `_RELEASING` | 6 | 8 |

### Why it was bad

1. **Force controller never activated** — `STATE_GRASPING=4` compared against
   pipeline's `GRASPING=5` never matched. The PI force regulation never engaged.
   The hand would close but no force feedback control would run.

2. **Force controller never reset on release** — `STATE_RELEASING=6` compared
   against pipeline's `RELEASING=8` never matched. Force buffers weren't cleared.
   The controller would keep stale state across grasp cycles.

3. **Haptic release buzz never fired** — `_RELEASING=6` vs pipeline's 8.

4. **Haptic force feedback never activated** — `pipeline_state in (4, 5)` vs
   actual states 5, 6.

5. **Control loop never entered** — The control tick checked
   `pipeline_state in (STATE_GRASPING=4, STATE_HOLDING=5)` while the pipeline
   was sending 5 and 6. Even if the controller somehow activated, it wouldn't
   run the PI loop.

### The fix

Updated both nodes to use the exact pipeline_manager State enum values (0..8):

**Files changed (3 files, 20 insertions, 11 deletions):**

| File | Change |
|------|--------|
| `src/force_controller/force_controller_node.py:75-95` | 9-state mapping: `IDLE=0` through `RELEASING=8`. Added `STATE_TWISTING`, `STATE_VOLITIONAL`. Updated `STATE_NAMES` dict. |
| `src/haptic_band/haptic_bridge/haptic_controller_node.py:23-32` | 9-state mapping matching pipeline. Added all intermediate states for documentation. |
| `tests/emg_grasp/emg_grasp_test.yaml:67-68` | Comment `6=RELEASING` → `8=RELEASING`, value `[0, 6]` → `[0, 8]`. |

**Activation/deactivation logic preserved:** The transition checks in
`_on_pipeline_state` still compare against the same-named constants — the
numbers are now just correct. Control loop, deactivation, and reset all fire
at the right time.

---

## Verification

- `grep -rn "GESTURE_PINCH\|GESTURE_POINT" src/` — no matches (old constants fully removed)
- `grep -rn "STATE_RELEASING.*=.*6" src/` — no matches (all states now 0..8)
- Both fixes are in commits on `grasp-simplified` branch

## How to test

1. Launch the system with the live EMG config active (the default path)
2. Verify OPEN gesture triggers release (not grasp), FLEXION triggers grasp
3. Verify force controller activates when pipeline enters GRASPING (state 5)
4. Verify haptic buzz fires when pipeline enters RELEASING (state 8)
