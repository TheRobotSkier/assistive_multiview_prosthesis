# EMG-Driven Force Grasp Test

## Quick Start

### Hardware test (with live EMG)
```bash
make up-grasp-test
# In another terminal, start EMG bridge if not in launch:
# ros2 run emg_bridge ros_bridge_node
```

### Mock test (no EMG hardware)
```bash
# Terminal 1: Start grasp test container
make up-grasp-test

# Terminal 2: Publish mock EMG gesture
podman exec -it grasp_test bash
python3 /prosthesis_ws/tests/emg_grasp/mock_emg_publisher.py --gesture 1 --duration 3.0
```

## Safety Checklist
- [ ] Hand serial port accessible (`/dev/ttyUSB0` or env `MIA_PORT`)
- [ ] No objects obstructing finger movement during reset
- [ ] Emergency stop: Ctrl-C in grasp_test container, or `make down-grasp-test`
- [ ] Force thresholds calibrated for current object (see `config/static_grasp_test.yaml`)

## Mode-Based Operation

The EMG grasp node operates in three modes.  Each mode maps gestures to different
functions, and certain gestures trigger mode transitions.

### Gesture IDs
| ID | Gesture |
|----|---------|
| 0  | REST    |
| 1  | POWER   |
| 2  | PINCH   |
| 3  | OPEN    |
| 4  | POINT   |

### Mode: `moving`
Hand is open/relaxed.  Wrist can be positioned.  Grasp can be started.

| Function        | Default Gesture | Action                                    |
|-----------------|-----------------|-------------------------------------------|
| `wrist_pos`     | PINCH (2)       | Wrist positive rotation (+10 deg/s)       |
| `wrist_neg`     | OPEN (3)        | Wrist negative rotation (-10 deg/s)       |
| `grasp_activate`| POWER (1)       | Start grasp → transition to **grasping**  |

### Mode: `grasping`
Fingers are closing with velocity ramp.  Wrist can still be adjusted.
Grasp can be cancelled (returns to **moving**).

| Function        | Default Gesture | Action                                    |
|-----------------|-----------------|-------------------------------------------|
| `grasp_release` | OPEN (3)        | Cancel grasp → return to **moving**       |
| `wrist_pos`     | PINCH (2)       | Wrist positive rotation (+10 deg/s)       |
| `wrist_neg`     | — (disabled)    | No action                                 |

*Transition to **holding** happens automatically on force contact or position limit.*

### Mode: `holding`
Contact has been made.  Grasp force can be adjusted.  Wrist can be repositioned
slowly.  Grasp can be released.

| Function        | Default Gesture | Action                                    |
|-----------------|-----------------|-------------------------------------------|
| `force_inc`     | POWER (1)       | Small additional closure (+0.05 rad/s)    |
| `force_dec`     | OPEN (3)        | Small opening (-0.05 rad/s)               |
| `grasp_release` | REST (0)        | Release grasp → return to **moving**      |
| `wrist_pos`     | PINCH (2)       | Wrist positive rotation (+5 deg/s)        |
| `wrist_neg`     | — (disabled)    | No action                                 |

## Customizing Gesture Mappings

Edit `tests/emg_grasp/emg_grasp_test.yaml` under the `modes:` section.
Each function entry has:

```yaml
function_name:
  gesture_id: 2               # null to disable
  confidence_threshold: 0.7
  hold_time_s: 0.5
  velocity: 10.0              # null when not applicable
```

- `gesture_id`: Which EMG gesture triggers this function (`null` = disabled).
- `confidence_threshold`: Minimum `/emg/confidence` required.
- `hold_time_s`: How long the gesture must be held before triggering.
- `velocity`: Action velocity (rad/s for fingers, deg/s for wrist).

Example — disable wrist control in moving mode:
```yaml
moving:
  wrist_pos:
    gesture_id: null
    confidence_threshold: null
    hold_time_s: null
    velocity: null
```

## Mock EMG Sequences

The mock publisher supports gesture sequences for testing mode transitions:

```bash
# Publish POWER for 2s, then PINCH for 1s, then REST
python3 mock_emg_publisher.py --sequence "1:2.0,2:1.0,0:5.0"
```

Format: `gesture_id:duration_s,gesture_id:duration_s,...`

After the sequence completes, REST (0) is published indefinitely.

### Example: Full grasp cycle
Simulate a complete grasp-and-release cycle:

```bash
# 1. POWER hold  → triggers moving→grasping (closing)
# 2. Wait for contact (publisher sends REST while node auto-holds)
# 3. OPEN hold   → triggers grasping→moving (release)
python3 mock_emg_publisher.py --sequence "1:3.0,0:5.0,3:2.0,0:5.0"
```

The old `--gesture`, `--confidence`, and `--duration` flags are still supported
for simple single-gesture tests.

## Configuration
Edit `tests/emg_grasp/emg_grasp_test.yaml` to adjust:
- Velocity ramp (`closing_velocity_start`, `closing_velocity_end`, `decay_steps`)
- Force thresholds per finger (`force_thresholds`)
- EMG gesture mapping per mode (`modes.<mode>.<function>.gesture_id`)
- Confidence thresholds and hold times per function
