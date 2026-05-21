# EMG Volitional Hand Controller Test

Direct real-time velocity control of the Mia Hand via EMG gestures.

## Quick Start

### Hardware test (with live EMG)
```bash
make test-volitional
```

### Container-based test
```bash
make up-volitional
# In another terminal, start EMG bridge if not in launch:
# ros2 run emg_bridge ros_bridge_node
```

### Mock test (no EMG hardware)
```bash
make up-volitional
# Or with mock EMG cycling through gestures:
ros2 launch prosthesis_launch emg_volitional_test.launch.py mock_emg:=true
```

## Gesture-to-Velocity Mapping

Edit `tests/emg_volitional/emg_volitional_config.yaml` to customize:

| Gesture | ID | Default Velocity | Action |
|---------|----|------------------|--------|
| REST    | 0  | 0.0 rad/s        | Stop   |
| POWER   | 1  | +0.3 rad/s       | Close  |
| PINCH   | 2  | +0.15 rad/s      | Close (slow) |
| OPEN    | 3  | -0.3 rad/s       | Open   |
| POINT   | 4  | 0.0 rad/s        | Stop   |

Positive = closing (finger flexion), negative = opening (finger extension).

## Safety

- **Position limits**: Configured in YAML (`position_limits.min/max`). Velocity is clamped to zero when any finger reaches a limit.
- **Confidence threshold**: Gestures below the configured confidence are ignored (treated as REST).
- **Emergency stop**: The REST gesture always commands zero velocity.
- **Ctrl-C**: Immediately zeros velocity and shuts down the node.
