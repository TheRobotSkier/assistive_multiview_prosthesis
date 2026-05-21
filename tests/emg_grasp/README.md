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

## State Machine
- **IDLE** → Hand open, waiting for POWER gesture hold (1 second)
- **CLOSING** → Velocity ramp closure, stops on contact or position limit
- **HOLDING** → Maintains grasp, waits for OPEN gesture to release
- **RELEASING** → Opens hand, returns to IDLE
- **FAULT** → Safety stop (not yet triggered by specific conditions)

## Configuration
Edit `tests/emg_grasp/emg_grasp_test.yaml` to adjust:
- Velocity ramp (`closing_velocity_start`, `closing_velocity_end`, `decay_steps`)
- Force thresholds per finger (`force_thresholds`)
- EMG gesture mapping (`emg_grasp_trigger_gesture`, `emg_release_gesture`)
