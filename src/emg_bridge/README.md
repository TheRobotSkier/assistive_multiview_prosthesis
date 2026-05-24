## Features Implemented

4 experimental modes, toggled via `config/emg_experiment_config.yaml`:

| # | Feature | Config | New Module |
|---|---------|--------|------------|
| 1 | **NaviFlame** deep-model backend | `classifier_backend: naviflame` | `naviflame_backend.py` |
| 2 | **IMU-augmented sklearn** | `classifier_backend: sklearn_imu` | `features.py` / `sklearn_backends.py` |
| 3 | **Proportional slew limiting** | `proportional_slew.enabled: true` | `control_filters.py` |
| 4 | **Sticky gesture hysteresis** | `gesture_stability.enabled: true` | `gesture_stabilizer.py` |

## Makefile Targets

```bash
# Default mode (standard sklearn, no experiments):
make emg-default

# sklearn + IMU (requires -include-imu collected data):
make emg-sklearn-imu

# Proportional slew limiting:
make emg-slew

# Sticky gesture selection:
make emg-sticky

# NaviFlame backend (needs separate Python 3.10 container!):
make emg-naviflame

# Full pipeline launch with mode:
make emg-pipeline MODE=naviflame

# Hardware validation checklist:
make emg-validate
```

## Config Settings

Edit `config/emg_experiment_config.yaml` (copy from the default):

```yaml
classifier_backend: sklearn          # sklearn | sklearn_imu | naviflame

proportional_slew:
  enabled: true
  max_velocity_per_s: 2.5            # cap rate of change
  max_accel_per_s2: 10.0             # smooth acceleration
  max_fall_velocity_per_s: 10.0      # allow fast release (0 = symmetric)
  reset_on_rest: true
  snap_to_zero_below: 0.0

gesture_stability:
  enabled: true
  min_confidence_to_switch: 0.7
  min_frames: 5
  min_hold_s: 0.2
  release_behavior: allow_rest_immediately   # or require_threshold | hold_previous_when_uncertain
  fallback_behavior: hold_previous          # or rest

imu_features:
  enabled: true
  gyro: true
  accel: false
  movement_gate: 0.0                  # gyro magnitude threshold for REST override

naviflame:
  enabled: true
  config_path: /prosthesis_ws/NaviFlame/config.json
```

## NaviFlame Container Note

NaviFlame requires `tensorflow==2.12.0` + Python 3.10, incompatible with ROS Jazzy's Python 3.12. Run it in a **separate podman container**:

```bash
podman run --rm --network host --userns=keep-id \
  -v $(pwd)/src:/prosthesis_ws/src -v $(pwd)/NaviFlame:/prosthesis_ws/NaviFlame \
  -v $(pwd)/config:/prosthesis_ws/config -v $(pwd)/models:/prosthesis_ws/models \
  python:3.10-slim bash -c \
  'pip install tensorflow==2.12.0 scikit-learn numpy pyyaml && cd /prosthesis_ws && python src/emg_bridge/scripts/run_classifier.py --config config/emg_experiment_config_naviflame.yaml'
```

## Tests

**138 tests** across 8 test files covering config schema, board reader, IMU features, sklearn backends, NaviFlame adapter, control filters, gesture stabilizer, and end-to-end simulation. Run with:
```bash
PYTHONPATH="src/emg_bridge" python -m pytest tests/test_experiment_config.py tests/test_board_reader_imu.py tests/test_imu_features.py tests/test_sklearn_backends.py tests/test_naviflame_backend.py tests/test_control_filters.py tests/test_gesture_stabilizer.py tests/test_simulation.py -v
```
