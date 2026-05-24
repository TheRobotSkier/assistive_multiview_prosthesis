# EMG Grasp Test — Troubleshooting Guide

## Container & Build

### `make up-grasp-test` fails with "no container backend found"

**Symptom:** Error about missing docker or podman-compose.

**Fix:**
```bash
# Install podman-compose (recommended)
pip install podman-compose

# Or use Docker explicitly
CONTAINER_BACKEND=docker make up-grasp-test
```

### Container starts but exits immediately

**Symptom:** `docker ps` / `podman ps` shows grasp_test container with "Exited" status.

**Check:**
```bash
make logs-grasp-test
```

**Common causes:**
- Missing launch file at `/prosthesis_ws/src/prosthesis_launch/launch/emg_grasp_test.launch.py`
  → Rebuild: `make build-prosthesis`
- ROS 2 environment not sourced in the command
  → The launch file command should source setup.bash first
- Syntax error in emg_grasp_node.py
  → Run `python3 -c "import py_compile; py_compile.compile('/prosthesis_ws/tests/emg_grasp/emg_grasp_node.py', doraise=True)"` inside the container

### "No module named 'rclpy'" inside container

**Symptom:** Node crashes with import error for rclpy.

**Fix:** Ensure the ROS 2 environment is sourced before running:
```bash
source /opt/ros/jazzy/setup.bash
source /prosthesis_ws/install/setup.bash
python3 /prosthesis_ws/tests/emg_grasp/emg_grasp_node.py ...
```

## Mia Hand Hardware

### Hand serial port not found

**Symptom:** `make test-static-grasp` reports `/dev/ttyUSB0: No such file or directory`.

**Check:**
```bash
# On host
ls -la /dev/ttyUSB*
dmesg | grep -i usb | tail -20

# Check permissions
ls -la /dev/ttyUSB0
groups | grep dialout
```

**Fix:**
- Verify the USB cable is connected and the hand is powered
- Set `MIA_PORT` to the correct device: `MIA_PORT=/dev/ttyUSB1 make test-static-grasp`
- Add user to dialout group: `sudo usermod -a -G dialout $USER` (log out and back in)
- For container: ensure `--device /dev/ttyUSB0:/dev/ttyUSB0` is in the compose file

### Hand does not respond to commands

**Symptom:** Node publishes velocity/position commands but fingers don't move.

**Check:**
```bash
# Inside container — check controller state
ros2 control list_controllers

# Look for group_vel_ff_controller and group_pos_ff_controller
# They should show state: active (one of them)
```

**Fix:**
- If both are inactive or unconfigured, the launch file may not have loaded them
- Verify `mia_hand_system_interface_launch.py` is included in the launch file
- Check that the hand responds to ros2_control at all:
  ```bash
  ros2 topic pub /group_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0]"
  ```
  (should open the hand to relaxed position)

### Force readings are zero or stuck

**Symptom:** `make print-force` shows forces near 0 for all fingers or forces don't change.

**Check:**
```bash
# Echo joint states directly
ros2 topic echo /joint_states --once | grep -A 1 effort
```

**Fix:**
- Force readings require the Mia Hand ros2_control hardware interface to be active
- If only positions appear but no efforts, the hardware interface may not publish effort
- Verify the ros2_control config includes `state_interfaces: [position, effort]` for each joint

### Hand closes too hard / overshoots

**Symptom:** Hand crushes object or overshoots before stopping.

**Fix (in `tests/emg_grasp/emg_grasp_test.yaml`):**
1. Reduce `closing_velocity_start` (e.g., 0.3 → 0.15)
2. Reduce `closing_velocity_end` (e.g., 0.1 → 0.05)
3. Reduce `force_thresholds` to trigger earlier contact detection
4. Increase `decay_steps` for smoother velocity ramp

Check idle forces first with `make print-force` — set force thresholds at
least 50–80 ADC units above idle values.

## EMG Hardware

### MindRove board not connecting

**Symptom:** `ros2 run emg_bridge collect_data` hangs or says "could not connect".

**Fix:**
- Ensure you are connected to the MindRove WiFi network
- Verify the board is powered on (LED indicator)
- Check the board's IP address in `emg_bridge/config.py` matches
- Try pinging the board: `ping 192.168.4.1` (or configured IP)
- Restart the board (power cycle)

### Classifier accuracy is low (< 85%)

**Symptom:** Frequent misclassifications during live inference. Wrong gestures detected.

**Fix:**
- Re-record training data with clean, steady gestures. Avoid:
  - Transitional movements between gestures
  - Partial contractions
  - Fatigue (take breaks between recordings)
- Ensure the EMG band is positioned correctly — electrodes must contact skin
- Clean the electrode sites with alcohol wipes before recording
- If using LDA but accuracy < 85%, the trainer auto-falls-back to SVM.
  Check `models/meta.pkl` for the algorithm used.
- Increase `PREDICTION_SMOOTHING_FRAMES` (default 5) in `emg_bridge/config.py`
  for more temporal smoothing

### False gesture triggers during rest

**Symptom:** Hand closes unexpectedly when operator is at rest.

**Fix:**
- Increase `emg_grasp_confidence_threshold` (0.7 → 0.8 or 0.85)
- Increase `gesture_hold_timeout_s` (1.0 → 1.5 s) to require longer gesture hold
- Check for EMG noise — the band may pick up electrical interference
  - Move away from power supplies, motors, fluorescent lights
  - Ensure proper skin contact
- Retrain the classifier with better REST examples

### OPEN gesture not triggering release

**Symptom:** Hand stays in control-grasp despite performing OPEN gesture.

**Check:**
```bash
# Echo the EMG topics to see what's being published
ros2 topic echo /emg/gesture_label
ros2 topic echo /emg/confidence
```

**Fix:**
- Verify confidence is above threshold (0.7) when performing OPEN
- Check that `emg_release_gesture` is set to 3 (OPEN) in the config
- If confidence is borderline, lower `emg_grasp_confidence_threshold` to 0.6
- The gesture must be held for `gesture_hold_timeout_s` (1.0 s) — hold it steady
- Check if the release is blocked by a different gesture being classified
- As safety fallback: Ctrl-C stops the node and opens the hand

## Wrist Control

### Wrist not moving

**Symptom:** FLEXION/EXTENSION gestures don't rotate the wrist.

**Check:**
- `wrist_control_enabled` must be `true` in `tests/emg_grasp/emg_grasp_test.yaml`
- Current mode matters: in control-grasp mode, FLEXION/EXTENSION control force,
  not wrist. Switch to not-grasping or control-wrist mode.
- Verify wrist driver is running and connected to the Dynamixel motor
- Check the topic: `ros2 topic echo /wrist/set_position`

### Wrist moves unexpectedly

**Symptom:** Wrist rotates without operator intent.

**Fix:**
- Set `wrist_control_enabled: false` to disable wrist entirely
- Reduce wrist velocity in `wrist_gesture_map` (e.g., 15.0 → 5.0)
- If misclassification is the root cause, see "False gesture triggers" above

## Mock Mode Issues

### Mock publisher not working

**Symptom:** Running `mock_emg_publisher.py` says "No module named 'rclpy'".

**Fix:**
```bash
source /opt/ros/jazzy/setup.bash
python3 /prosthesis_ws/tests/emg_grasp/mock_emg_publisher.py --gesture 1
```

### Hand doesn't close in mock test

**Symptom:** Mock publisher sends gesture 1 but hand stays open.

**Check (in order):**
1. Is the EMG grasp node running? Check with `ros2 node list | grep emg_grasp`
2. Is it receiving the gesture? Check `/emg/gesture_label` topic
3. Is confidence above threshold? Mock sends 0.95 by default, threshold is 0.7
4. Is the gesture held long enough? Timeout is 1.0 s, mock publishes for 5.0 s
5. Are joint states being published? The node skips cycles if `_got_js` is false

```bash
# Debug: check all relevant topics
ros2 topic echo /emg/gesture_label &
ros2 topic echo /emg/confidence &
ros2 topic echo /joint_states | head -20 &
```

### Node stuck with "No joint states yet"

**Symptom:** The node logs "No joint states yet — skipping cycle" repeatedly.

**Fix:**
- The node needs `/joint_states` to be published. In mock mode without the hand,
  no joint states are published, so the node cannot operate.
- Use the real hand, or start a mock joint_state publisher:
  ```bash
  ros2 topic pub /joint_states sensor_msgs/msg/JointState \
    "{name: ['j_thumb_fle', 'j_index_fle', 'j_mrl_fle'], position: [0.0, 0.0, 0.0], effort: [200.0, 200.0, 250.0]}" \
    -r 10
  ```

## State Machine

### Node stuck in IDLE / not transitioning

**Symptom:** Node starts but never leaves IDLE despite receiving gestures.

**Check conditions for IDLE → CLOSING transition (all must be true):
1. `_emg_gesture == emg_grasp_trigger_gesture` (gesture ID 1 by default)
2. `_emg_confidence >= emg_grasp_confidence_threshold` (0.7 by default)
3. `_gesture_held_long_enough()` — gesture held for ≥ `gesture_hold_timeout_s` (1.0 s)
4. `_got_js` is true — joint states are being received

**Fix:**
- Verify all four conditions via ROS topic inspection
- The hold timeout only resets when the gesture ID changes. If the same gesture
  has been continuously published, the timer started when it first appeared.

### Hand doesn't release after OPEN gesture

See "OPEN gesture not triggering release" under EMG Hardware above.

### Mode confusion between FLEXION/EXTENSION and PINCH/POINT

**Symptom:** Config uses PINCH/POINT names but docs/code reference FLEXION/EXTENSION.

**Context:** Gesture IDs 2 and 4 are being renamed. ID 2 = FLEXION (was PINCH),
ID 4 = EXTENSION (was POINT). The YAML config currently uses PINCH/POINT.

**Fix:**
- Use ID numbers (2, 4) rather than names until the rename is complete
- Check `GESTURE_NAMES` in `src/emg_bridge/emg_bridge/config.py` for current names
- Training data recorded with old names still works — only the label string changes

## Logging & Debugging

### How to get detailed logs

```bash
# From host — follow all container logs
make logs-grasp-test

# From inside container — ROS-specific
ros2 topic echo /rosout | grep emg_grasp

# Change log level at runtime
ros2 service call /emg_grasp_test/set_logger_level logging_demo/srv/ConfigLogger "{logger_name: '', level: 'debug'}"
```

### Common log messages

| Message | Meaning |
|---------|---------|
| `EMG Grasp Test node started — waiting in IDLE` | Normal startup. Hand is open, waiting. |
| `No joint states yet — skipping cycle` | `/joint_states` not yet received. Check ros2_control. |
| `EMG grasp triggered — entering CLOSING` | POWER gesture held long enough with sufficient confidence. |
| `Contact detected: FORCE CONTACT on j_index_fle` | Force threshold exceeded — hand is holding. |
| `Contact detected: STOP POSITION reached on j_thumb_fle` | Position limit reached (no object detected). |
| `EMG release triggered — entering RELEASING` | OPEN gesture held, hand is opening. |
| `Hand released — returning to IDLE` | Release complete, back to idle. |

## Getting Help

1. Check this troubleshooting guide first
2. Run `make print-force` to verify hardware is responsive
3. Run `make test-static-grasp` to isolate EMG from hand control
4. Check EMG bridge logs: `ros2 topic echo /emg/gesture_label`
5. Check ROS graph: `rqt_graph` or `ros2 node info /emg_grasp_test`
6. Verify all dependencies are satisfied (see bead mvp-egt.11 for full dep list)
7. If all else fails: `make down-grasp-test && make up-grasp-test` (full restart)
