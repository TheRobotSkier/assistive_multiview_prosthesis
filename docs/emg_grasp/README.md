# EMG-Driven Force and Wrist Grasp Test

Operator and developer documentation for the EMG grasp test feature.

## Overview

The EMG grasp test drives the Mia Hand via MindRove EMG gestures through a
three-mode state machine. The operator wears the EMG band and performs gestures
to control hand grasp force and wrist rotation. The system monitors joint forces
and positions for safety.

```
                  ┌──────────────┐       Int32        ┌────────────────┐
  Operator ──────▶│  MindRove    │──── gesture_label ─▶│  EMG Grasp     │
  (gestures)      │  EMG Band    │       Float32       │  Test Node     │
                  └──────────────┘──── confidence ────▶│                │
                                                       │  ┌──────────┐  │
                                                       │  │ 3-mode   │  │
                  ┌──────────────┐                     │  │ state    │  │
                  │  Mia Hand    │◀── vel/pos cmd ─────│  │ machine  │  │
                  │  ros2_control│──── joint_states ──▶│  └──────────┘  │
                  └──────────────┘                     └────────────────┘
```

## Quick Start

### Hardware test (with live EMG)

```bash
# Terminal 1: Start the grasp test container stack
make up-grasp-test

# (If EMG bridge is not included in the launch, start it separately:)
podman exec -it grasp_test ros2 run emg_bridge ros_bridge_node
```

The container launches `ros2 launch prosthesis_launch emg_grasp_test.launch.py`
which starts Mia Hand ros2_control, the safety node, and the EMG grasp test node.

### Mock test (no EMG hardware)

```bash
# Terminal 1: Start grasp test container
make up-grasp-test

# Terminal 2: Publish mock EMG gesture
podman exec -it grasp_test bash
python3 /prosthesis_ws/tests/emg_grasp/mock_emg_publisher.py --gesture 1 --duration 3.0
```

The mock publisher sends gesture label 1 (POWER) at 0.95 confidence for 3 seconds,
triggering the grasp sequence. After the duration expires it publishes REST (0).

## Hardware Prerequisites

| Component | Device | Notes |
|-----------|--------|-------|
| **EMG band** | MindRove WiFi board | 8-channel, 500 Hz. Connect to MindRove WiFi first. |
| **Hand** | Mia Hand (Prensilia) | Serial via `/dev/ttyUSB0` or `MIA_PORT` env var |
| **Wrist** | Dynamixel motor | Optional; wrist control on `/wrist/set_position` |
| **Container** | Podman or Docker | `make up-grasp-test` starts the stack |

### Hardware Validation Steps

1. **Hand serial port**: Verify the device exists on the host:
   ```bash
   ls -la /dev/ttyUSB*
   ```
   The default is `/dev/ttyUSB0`. Override with `MIA_PORT` environment variable.

2. **Hand idle forces**: Run the force monitor to check baseline forces:
   ```bash
   make print-force
   ```
   Expected idle forces (raw ADC): thumb ~215, index ~208, mrl ~260.
   If any finger exceeds 300 at idle, the hand needs inspection.

3. **EMG board connectivity**: Connect to MindRove WiFi. Run data collection:
   ```bash
   podman exec -it grasp_test ros2 run emg_bridge collect_data
   ```
   The training prompt guides you through recording REST, POWER, FLEXION, OPEN,
   and EXTENSION gestures (3 reps each, 5 seconds per rep).

4. **Static grasp smoke test**: Verify hand closure and force sensing:
   ```bash
   make test-static-grasp
   ```
   This runs a non-EMG velocity ramp closure through `config/static_grasp_test.yaml`
   parameters and prints force/position at each step.

5. **Wrist validation** (if wrist enabled): Send a test position:
   ```bash
   podman exec -it grasp_test ros2 topic pub /wrist/set_position std_msgs/Float64 "data: 0.0"
   ```
   Verify the wrist moves and returns to neutral.

## Configuration Reference

Primary config file: `tests/emg_grasp/emg_grasp_test.yaml`

### Velocity Profile

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `closing_velocity_start` | float | 0.3 | Initial finger closing speed (rad/s) |
| `closing_velocity_end` | float | 0.1 | Final finger closing speed after decay (rad/s) |
| `decay_steps` | int | 5 | Number of discrete steps for velocity ramp |
| `step_interval_s` | float | 0.2 | Time between velocity step changes (s). Total ramp = `decay_steps × step_interval_s` |

### Stop Conditions

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `stop_positions.j_thumb_fle` | float | 1.5 | Position limit for thumb (rad). Range: 0.0 (open) to ~3.0 (closed) |
| `stop_positions.j_index_fle` | float | 1.5 | Position limit for index finger |
| `stop_positions.j_mrl_fle` | float | 1.5 | Position limit for middle/ring/little finger |
| `force_thresholds.j_thumb_fle` | int | 300 | Force threshold for thumb (raw ADC). Idle ~215 |
| `force_thresholds.j_index_fle` | int | 300 | Force threshold for index. Idle ~208 |
| `force_thresholds.j_mrl_fle` | int | 300 | Force threshold for MRL. Idle ~260 |

If **any** finger exceeds its force threshold or stop position, **all** fingers
stop immediately. The stop reason is logged.

### EMG Parameters

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `emg_gesture_topic` | string | `/emg/gesture_label` | ROS topic for gesture label (Int32) |
| `emg_confidence_topic` | string | `/emg/confidence` | ROS topic for confidence (Float32) |
| `emg_proportional_topic` | string | `/emg/proportional` | ROS topic for proportional control (Float32) |
| `emg_grasp_trigger_gesture` | int | 1 | Gesture ID that triggers grasping (POWER) |
| `emg_release_gesture` | int | 3 | Gesture ID that triggers release (OPEN) |
| `emg_grasp_confidence_threshold` | float | 0.7 | Minimum confidence for gesture acceptance |
| `gesture_hold_timeout_s` | float | 1.0 | Seconds the gesture must be held to trigger |

### Wrist Control

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `wrist_control_enabled` | bool | false | Enable wrist velocity control |
| `wrist_cmd_topic` | string | `/wrist/set_position` | Topic for wrist position commands |
| `wrist_gesture_map.0` (REST) | float | 0.0 | Wrist velocity for REST gesture |
| `wrist_gesture_map.1` (POWER) | float | 0.0 | Wrist velocity for POWER gesture |
| `wrist_gesture_map.2` (FLEXION) | float | 15.0 | Wrist velocity for FLEXION (CCW) |
| `wrist_gesture_map.3` (EXTENSION) | float | -15.0 | Wrist velocity for EXTENSION (CW) |
| `wrist_gesture_map.4` (OPEN) | float | 0.0 | Wrist velocity for OPEN gesture |

Positive values = counter-clockwise rotation, negative = clockwise.

### Force-Hold Tuning Guidance

The force-hold behavior is controlled by two velocity parameters:

- **`closing_velocity_start`** (0.3 rad/s): Higher values close the hand faster,
  reducing time-to-contact but increasing overshoot risk. For delicate objects,
  reduce to 0.15–0.2.
- **`closing_velocity_end`** (0.1 rad/s): The steady-state closing velocity after
  the ramp decays. Lower values (0.05–0.08) give better force resolution but
  slower grasp acquisition.
- **`decay_steps`** (5): More steps smooth the transition. Fewer steps (3) make
  the ramp steeper. Each step takes `step_interval_s` (0.2 s).

The velocity ramp formula is linear interpolation:
```
velocity[i] = v_start + (i / (decay_steps - 1)) × (v_end - v_start)
```

After the ramp, the hand continues at `v_end` until contact is detected
(force threshold exceeded) or a position limit is reached.

The force-hold behavior uses velocity-based force control: when contact is
detected, velocity is set to zero and the hand holds position. Force targets
are increased or decreased via FLEXION/EXTENSION gestures in control-grasp mode.

## Three-Mode State Machine

The EMG grasp test operates in three modes with the following state table:

| Mode | OPEN (3) | POWER (1) | FLEXION (2) | EXTENSION (4) |
|------|----------|-----------|-------------|---------------|
| **not-grasping** | Stay in not-grasping + command hand to open position | Transition to **control-grasp** + begin closing | Rotate wrist CCW (or: ignored if wrist disabled) | Rotate wrist CW (or: ignored if wrist disabled) |
| **control-grasp** | Release grasp + command hand to open position + transition to **not-grasping** | Transition to **control-wrist** + hold current force | Increase force target (close fingers slightly) | Decrease force target (open fingers slightly) |
| **control-wrist** | Release grasp + command hand to open position + transition to **not-grasping** | Transition to **control-grasp** + hold current force | Rotate wrist CCW | Rotate wrist CW |

### Mode Descriptions

**not-grasping** — The default mode. Hand is open, waiting for a POWER gesture
(held for `gesture_hold_timeout_s` seconds with confidence ≥ threshold). In this
mode, FLEXION and EXTENSION control the wrist if `wrist_control_enabled` is true.

**control-grasp** — Hand has contacted the object (or is closing). Velocity
commands are active during closing; after contact, force is held. FLEXION
increases the force target (tighter grip). EXTENSION decreases the force target
(looser grip). POWER transitions to control-wrist to adjust wrist angle.

**control-wrist** — Force is held constant while wrist rotation is adjusted.
FLEXION rotates wrist CCW, EXTENSION rotates wrist CW. POWER returns to
control-grasp mode. OPEN always returns to not-grasping.

### Transition Diagram

```
                    ┌──────────────┐
         OPEN (3)   │              │   POWER (1)
     ┌──────────────│ not-grasping │──────────────┐
     │              │              │              │
     ▼              └──────┬───────┘              ▼
 ┌─────────┐               │             ┌──────────────┐
 │  Hand   │    FLEXION(2) │             │ control-grasp │
 │  open   │     /EXT(4)   │             │              │
 │ (safety │   (wrist if   │             │ FLEXION(2):   │
 │  reset) │    enabled)   │             │  +force       │
 └─────────┘               │             │ EXTENSION(4): │
      ▲                    │             │  -force       │
      │                    │             └──────┬────────┘
      │              ┌─────┴──────┐             │
      │     OPEN (3) │            │  POWER (1)  │
      └──────────────│ control-   │◄────────────┘
                     │   wrist    │
                     │            │
                     │ FLEXION(2):│
                     │  wrist CCW │
                     │ EXTEN(4):  │
                     │  wrist CW  │
                     └────────────┘
```

## Gesture Mapping

The MindRove EMG classifier supports 5 gestures:

| ID | Canonical Name | Legacy Alias | Description |
|----|---------------|---------------|-------------|
| 0 | REST | — | Relaxed, no contraction. Default/fallback state. |
| 1 | POWER | — | Power/cylindrical grip. Triggers grasp. |
| 2 | **FLEXION** | PINCH | Wrist flexion / finger curl. Adjusts force target in control-grasp, rotates wrist CCW otherwise. |
| 3 | OPEN | — | Hand open / extension. Triggers release and safety reset. |
| 4 | **EXTENSION** | POINT | Wrist extension / finger point. Adjusts force target in control-grasp, rotates wrist CW otherwise. |

> **Note:** FLEXION and EXTENSION are the canonical names for gesture IDs 2 and 4.
> The legacy names PINCH (ID 2) and POINT (ID 4) are temporary compatibility aliases.
> Future versions will rename these to FLEXION and EXTENSION throughout the codebase.
> Config files, logs, and docs should prefer FLEXION/EXTENSION.

The EMG bridge publishes to:
- `/emg/gesture_label` (Int32) — integer ID 0–4
- `/emg/gesture_name` (String) — human-readable name
- `/emg/confidence` (Float32) — classifier confidence [0, 1]
- `/emg/proportional` (Float32) — proportional control signal

### Data Collection & Training

The EMG classifier requires per-user training. Run these inside the container:

```bash
# Step 1: Collect training data (follow on-screen prompts)
ros2 run emg_bridge collect_data

# Step 2: Train the classifier
ros2 run emg_bridge train

# Step 3: Run live inference
ros2 run emg_bridge run_classifier
```

**Training prompts**: The `collect_data` script guides you through recording
each gesture (REST, POWER, FLEXION, OPEN, EXTENSION) for 5 seconds per rep,
3 repetitions each. A 2-second warm-up precedes each recording. Hold the
gesture steadily during recording — avoid transitional movements.

Training data and model are saved to `models/` (classifier.pkl, scaler.pkl,
meta.pkl). Retrain if you change users or if classifier accuracy drops below 85%.

Live configuration (confidence thresholds, gesture mappings) can be tuned
without restarting via `config/emg_live.yaml` — changes are picked up within
1 second by the pipeline manager.

## Safety Reset Behavior

**OPEN gesture hold (≥0.5 s):** Holding the OPEN gesture (ID 3) for 0.5 seconds
or longer triggers a safety reset from any mode:

1. All velocity commands are set to zero
2. Hand is commanded to the fully open position (0.0 rad on all fingers)
3. State machine returns to **not-grasping** mode

This is the **universal escape** — OPEN always returns to not-grasping and opens
the hand, regardless of current mode.

**Emergency stop:**
- **Ctrl-C** in the grasp_test container stops the ROS node gracefully (zero
  velocity, return to position control, hand opens).
- **`make down-grasp-test`** from the host stops the entire container stack.

**Safety preflight checklist** (before every session):
- [ ] Hand serial port accessible (`/dev/ttyUSB0` or `MIA_PORT` env)
- [ ] No objects obstructing finger movement during open/close cycles
- [ ] Force thresholds calibrated for current object (see config)
- [ ] Wrist range-of-motion clear if wrist control is enabled
- [ ] Emergency stop procedure known: Ctrl-C or `make down-grasp-test`
- [ ] Operator aware that the hand will close on POWER gesture

## Wrist Control Behavior

When `wrist_control_enabled: true` in the config:

- **not-grasping mode**: FLEXION rotates wrist CCW, EXTENSION rotates wrist CW.
  This allows the operator to position the wrist before grasping.
- **control-wrist mode**: Same mapping — FLEXION = CCW, EXTENSION = CW.
- **control-grasp mode**: Wrist commands are suppressed. FLEXION/EXTENSION control
  force target instead.

Wrist velocity is bounded by `wrist_gesture_map` values (rad/s). Default mapping:
- REST: 0.0 (hold position)
- POWER: 0.0 (no wrist motion during grasp)
- FLEXION: +15.0 rad/s (CCW)
- OPEN: 0.0
- EXTENSION: -15.0 rad/s (CW)

The wrist returns to neutral (0.0) on OPEN safety reset.

## Mock Mode

Mock mode allows testing the state machine without MindRove EMG hardware:

```bash
# Option A: Launch with mock EMG publisher built into the launch file
ros2 launch prosthesis_launch emg_grasp_test.launch.py mock_emg:=true

# Option B: Manual mock publisher (from inside the container)
python3 /prosthesis_ws/tests/emg_grasp/mock_emg_publisher.py \
    --gesture 1 \
    --confidence 0.95 \
    --duration 3.0
```

Mock publisher CLI:

| Flag | Default | Description |
|------|---------|-------------|
| `--gesture` | 1 | Gesture ID to publish (0=REST, 1=POWER, 2=FLEXION, 3=OPEN, 4=EXTENSION) |
| `--confidence` | 0.95 | Confidence value to publish |
| `--duration` | 5.0 | How long to publish the gesture before reverting to REST (0) |

After the duration expires, the publisher sends REST (gesture 0), which
effectively releases the hand.

### Expected Mock Test Sequence

1. Start `make up-grasp-test` — node enters not-grasping, hand opens
2. Publish POWER (1) — after `gesture_hold_timeout_s` (1.0 s), hand begins closing
3. Hand closes until force/position stop condition triggers → enters control-grasp
4. Publish OPEN (3) — hand opens, returns to not-grasping

## Running Tests

```bash
# Unit tests (no ROS, no hardware)
cd tests/emg_grasp
python3 -m pytest test_emg_grasp_contract.py -v

# Static grasp test (real hand, no EMG)
make test-static-grasp

# Full EMG grasp test (launch inside container)
make up-grasp-test
make logs-grasp-test  # view logs in follow mode
```

## Host Makefile Targets

| Target | Description |
|--------|-------------|
| `make up-grasp-test` | Start grasp test container stack (grasp_test profile) |
| `make down-grasp-test` | Stop grasp test container stack |
| `make logs-grasp-test` | Follow logs from grasp_test container |
| `make test-static-grasp` | Run static (non-EMG) velocity-ramp grasp test |
| `make print-force` | Print live finger force readings from Mia hand |

## ROS Topics

### Inputs (subscribed by EMG grasp node)

| Topic | Type | Description |
|-------|------|-------------|
| `/emg/gesture_label` | `std_msgs/Int32` | Gesture ID (0–4) |
| `/emg/confidence` | `std_msgs/Float32` | Classifier confidence [0, 1] |
| `/joint_states` | `sensor_msgs/JointState` | Finger positions and efforts |

### Outputs (published by EMG grasp node)

| Topic | Type | Description |
|-------|------|-------------|
| `/group_vel_ff_controller/commands` | `std_msgs/Float64MultiArray` | Velocity commands [thumb, index, mrl] |
| `/group_pos_ff_controller/commands` | `std_msgs/Float64MultiArray` | Position commands [thumb, index, mrl] |
| `/wrist/set_position` | `std_msgs/Float64` | Wrist position command (if enabled) |

## Known Risks

- **Real hand force/wrist motion**: The hand can exert significant force.
  Always start with conservative `force_thresholds` (≤300 ADC units).
  Increase thresholds gradually after observing object-specific contact forces.

- **Wrist motion**: If `wrist_control_enabled` is true and wrist hardware is
  connected, FLEXION/EXTENSION gestures will move the wrist. Ensure the
  workspace is clear before testing.

- **Gesture misclassification**: EMG classifiers can misclassify gestures,
  especially under fatigue. The confidence threshold (0.7) and hold timeout
  (1.0 s) provide a guard. If false triggers occur frequently, increase
  `emg_grasp_confidence_threshold` to 0.8.

- **Container privilege**: The grasp_test container runs with `privileged: true`
  and `network_mode: host` for hardware access. Use only in lab environments.

- **Serial port contention**: Only one process can own `/dev/ttyUSB0`. Stop
  any other Mia Hand processes before starting the grasp test.

## Development

### File Layout

```
tests/emg_grasp/
├── emg_grasp_node.py          # Main EMG grasp test ROS node
├── emg_grasp_test.yaml        # Configuration file
├── mock_emg_publisher.py      # Mock EMG publisher for testing without hardware
├── test_emg_grasp_contract.py # Unit tests (no ROS required)
├── README.md                  # Test-level quick-start guide
└── __init__.py

docs/emg_grasp/
├── README.md                  # This document
└── TROUBLESHOOTING.md         # Troubleshooting guide

config/
├── static_grasp_test.yaml     # Non-EMG static grasp test config
└── emg_live.yaml              # Live-reloadable EMG thresholds

src/prosthesis_launch/launch/
└── emg_grasp_test.launch.py   # Launch file for the test stack
```

### Adding a New Gesture

1. Add the gesture name to `GESTURE_NAMES` in `src/emg_bridge/emg_bridge/config.py`
2. Record training data for the new gesture via `collect_data`
3. Retrain the classifier via `train`
4. Update the config `wrist_gesture_map` if the gesture controls the wrist
5. Update the state machine in `emg_grasp_node.py` for the new transition
