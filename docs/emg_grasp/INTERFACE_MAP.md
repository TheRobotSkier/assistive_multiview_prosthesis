# EMG Grasp Interface Map

> **Scope:** Current code paths for EMG data collection/training, trained classifier
> launch, EMG ROS topics, Mia hand ros2_control startup, static_grasp_test force-hold
> behavior, and wrist driver command/state topics.
>
> **Purpose:** Anchor document for dependent beads (mvp-egt.2–mvp-egt.12).

---

## 1. EMG Data Collection & Training

### 1.1 Hardware
- **Board:** MindRove WiFi armband
- **Sampling rate:** 500 Hz (board-reported, fallback in `config.py`)
- **Channels:** 8 EMG channels (`raw_data[:8]`)

### 1.2 Data Collection Procedure

**Script:** `src/emg_bridge/scripts/collect_data.py`  
**ROS entry point:** `ros2 run emg_bridge collect_data`

```bash
# Default (container)
ros2 run emg_bridge collect_data --output-dir /app/data

# Custom
ros2 run emg_bridge collect_data --reps 5 --duration 6 --gesture-names REST POWER PINCH OPEN POINT
```

**Process:**
1. Connects to MindRove WiFi board
2. Warms up for 2 seconds
3. For each gesture (in order), prompts user and records `duration` seconds
4. Saves to `/app/data/session_YYYYMMDD_HHMMSS.npz`

**Output file format (`*.npz`):**
| Key | Shape | Description |
|-----|-------|-------------|
| `emg` | (total_samples, 8) | Raw EMG samples |
| `labels` | (total_samples,) | Integer label per sample |
| `segment_ends` | (n_segments,) | Cumulative end index of each recording |
| `sampling_rate` | scalar | Actual board rate |
| `gesture_names` | list | Gesture names in label order |

### 1.3 Training Procedure

**Script:** `src/emg_bridge/scripts/train.py`  
**ROS entry point:** `ros2 run emg_bridge train`

```bash
ros2 run emg_bridge train --data-dir /app/data --model-dir /app/models --cv-folds 5
```

**Pipeline:**
1. Loads all `.npz` session files
2. Per-segment: filter → extract windows → compute features
3. Trains classifier (LDA default, SVM fallback if LDA CV < 85%)
4. Computes proportional calibration (per-gesture RMS min/max)
5. Saves to `--model-dir`:
   - `classifier.pkl` — fitted scikit-learn Pipeline
   - `meta.pkl` — algorithm, CV accuracy, class list
   - `prop_calibration.pkl` — per-gesture RMS ranges

### 1.4 Windowing & Filter Parameters

From `src/emg_bridge/emg_bridge/config.py`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `WINDOW_LEN` | 100 samples | 200 ms at 500 Hz |
| `WINDOW_STEP` | 50 samples | 100 ms step (~10 Hz update rate) |
| `HIGHPASS_CUTOFF_HZ` | 20.0 | DC / motion-artifact removal |
| `LOWPASS_CUTOFF_HZ` | 200.0 | Anti-alias (must be < Nyquist=250) |
| `NOTCH_FREQ_HZ` | 50.0 | Mains hum |
| `NOTCH_Q` | 30.0 | Notch quality factor |
| `FILTER_ORDER` | 4 | Butterworth order |

---

## 2. Gesture Names & IDs

### 2.1 Current Enum

From `src/emg_bridge/emg_bridge/config.py`:

| Label | Name | Description |
|-------|------|-------------|
| 0 | `REST` | Relaxed, no contraction |
| 1 | `POWER` | Power / cylindrical grip |
| 2 | `PINCH` | Lateral pinch |
| 3 | `OPEN` | Hand open / extension |
| 4 | `POINT` | Index point / hook |

**`N_GESTURES = 5`**

### 2.2 FLEXION / EXTENSION Status

**Finding:** `FLEXION` and `EXTENSION` do **NOT** currently exist as gesture names or IDs in this repository.

- `OPEN` (3) corresponds to "hand open / extension" conceptually, but the enum name is `OPEN`.
- `POINT` (4) and `PINCH` (2) are the current names.
- **Compatibility aliases needed:** If downstream consumers expect `FLEXION`/`EXTENSION`, the EMG input adapter (mvp-egt.4) must map:
  - `FLEXION` → `POWER` (1) or `POINT` (4) — *TBD by design decision*
  - `EXTENSION` → `OPEN` (3)

**Note:** The `emg_live.yaml` config references grasp gestures as `[1, 2, 4]` (POWER, PINCH, POINT) and release as `3` (OPEN).

### 2.3 Classifier Runtime Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CONFIDENCE_THRESHOLD` | 0.55 | Below this → output REST |
| `PREDICTION_SMOOTHING_FRAMES` | 5 | Majority-vote over last 5 predictions |
| `REST_LABEL` | 0 | Fallback label |

---

## 3. Trained Classifier Launch & ROS Topics

### 3.1 Launch Entry Point

**Script:** `src/emg_bridge/scripts/run_classifier.py`  
**ROS entry point:** `ros2 run emg_bridge run_classifier`

```bash
ros2 run emg_bridge run_classifier --model-dir /app/models --threshold 0.55 --smooth 5
```

**Optional ROS bridge:** If `rclpy` is available, the script auto-starts `EmgRosBridgeNode` in a daemon thread.

### 3.2 Published ROS Topics

All topics use **TRANSIENT_LOCAL** durability (latched, depth 1) at **20 Hz**:

| Topic | Type | Description |
|-------|------|-------------|
| `/emg/gesture_label` | `std_msgs/Int32` | Integer gesture label (0–4) |
| `/emg/gesture_name` | `std_msgs/String` | String name ("REST", "POWER", etc.) |
| `/emg/confidence` | `std_msgs/Float32` | Probability of predicted class |
| `/emg/proportional` | `std_msgs/Float32` | RMS-based proportional control [0.0, 1.0] |

**QoS:** `DurabilityPolicy.TRANSIENT_LOCAL`, depth=1

### 3.3 Proportional Control

- Computed from window RMS normalized by per-gesture calibration min/max
- `REST` always returns 0.0
- Falls back to global RMS normalisation if calibration missing

---

## 4. Mia Hand ros2_control Startup

### 4.1 Launch File

**Primary:** `src/mia_hand_ros2_control/launch/mia_hand_system_interface_launch.py`

```bash
ros2 launch mia_hand_ros2_control mia_hand_system_interface_launch.py serial_port:=/dev/ttyUSB0 controller:="" rviz2_gui:=false
```

**With wrist:** `mia_hand_with_wrist_system_interface_launch.py`

### 4.2 Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Mia Hand serial port |
| `rviz2_gui` | `true` | Start RViz automatically |
| `laterality` | `right` | Right or left hand |
| `prefix` | `''` | Joint/link name prefix |
| `robot_ns` | `mia_hand` | Robot namespace |
| `controller` | `''` | If empty, spawns individual per-finger controllers; else spawns named group controller |
| `use_mock_hardware` | `false` | Mirror commands to states (simulation) |

### 4.3 Controller Architecture

**Controller config:** `src/mia_hand_ros2_control/config/mia_hand_controllers.yaml`

**Manager update rate:** 5 Hz

**Joint names:** `j_thumb_fle`, `j_index_fle`, `j_mrl_fle`

**Available controllers:**

| Controller | Type | Joints | Default State |
|------------|------|--------|---------------|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all 3 | **active** |
| `thumb_pos_ff_controller` | `position_controllers/JointGroupPositionController` | thumb | active |
| `index_pos_ff_controller` | `position_controllers/JointGroupPositionController` | index | active |
| `mrl_pos_ff_controller` | `position_controllers/JointGroupPositionController` | mrl | active |
| `group_pos_ff_controller` | `position_controllers/JointGroupPositionController` | all 3 | active |
| `thumb_vel_ff_controller` | `velocity_controllers/JointGroupVelocityController` | thumb | **inactive** |
| `index_vel_ff_controller` | `velocity_controllers/JointGroupVelocityController` | index | **inactive** |
| `mrl_vel_ff_controller` | `velocity_controllers/JointGroupVelocityController` | mrl | **inactive** |
| `group_vel_ff_controller` | `velocity_controllers/JointGroupVelocityController` | all 3 | **inactive** |
| `thumb_pos_vel_controller` | `pid_controller/PidController` | thumb | — |
| `index_pos_vel_controller` | `pid_controller/PidController` | index | — |
| `mrl_pos_vel_controller` | `pid_controller/PidController` | mrl | — |
| `group_pos_vel_controller` | `pid_controller/PidController` | all 3 | — |
| `thumb_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | thumb | — |
| `index_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | index | — |
| `mrl_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | mrl | — |
| `group_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | all 3 | — |

**Note:** When `controller` arg is empty (default for force-aware path), launch spawns all 6 individual pos+vel controllers (3 pos active, 3 vel inactive). The `static_grasp_test.sh` and `emg_grasp_node.py` switch to `group_vel_ff_controller` at runtime.

### 4.4 Command Topics

| Topic | Type | Controller | Description |
|-------|------|------------|-------------|
| `/group_pos_ff_controller/commands` | `Float64MultiArray` | group_pos_ff | Position commands [thumb, index, mrl] |
| `/group_vel_ff_controller/commands` | `Float64MultiArray` | group_vel_ff | Velocity commands [thumb, index, mrl] |
| `/thumb_pos_ff_controller/commands` | `Float64MultiArray` | thumb_pos_ff | Single-joint position |
| `/thumb_vel_ff_controller/commands` | `Float64MultiArray` | thumb_vel_ff | Single-joint velocity |
| … | … | … | (index and mrl analogous) |

### 4.5 Safety Node

**File:** `src/mia_hand_ros2_control/mia_hand_ros2_control/mia_safety_node.py`

| Service | Type | Command |
|---------|------|---------|
| `/mia_hand/emergency_stop` | `std_srvs/srv/Trigger` | `@AS.............*\r` (serial) |
| `/mia_hand/play` | `std_srvs/srv/Trigger` | `@AR.............*\r` (serial) |

**Parameters:**
- `serial_port` (default: `/dev/mia_hand`)
- `baudrate` (default: 115200)

**Note:** The safety node opens a **separate** serial connection from the ros2_control hardware interface. Both talk to the same device but are independent.

---

## 5. Joint State Force Fields

### 5.1 Topic

**`/joint_states`** — `sensor_msgs/JointState`

Published by `joint_state_broadcaster` at controller manager update rate (5 Hz).

### 5.2 Fields per Finger

| Joint | position | velocity | effort |
|-------|----------|----------|--------|
| `j_thumb_fle` | radians | rad/s | **raw ADC** (force) |
| `j_index_fle` | radians | rad/s | **raw ADC** (force) |
| `j_mrl_fle` | radians | rad/s | **raw ADC** (force) |

**Important:** `effort` contains **raw ADC values**, not Newtons or calibrated force units. Idle forces vary by hardware: thumb~215, index~208, mrl~260. Thresholds in `static_grasp_test.yaml` are set to 300 ADC.

### 5.3 Force Monitoring in static_grasp_test

The `scripts/static_grasp_test.sh` node subscribes to `/joint_states` and checks:
- `effort[i] >= force_thresholds[i]` → **FORCE CONTACT**, stop all fingers
- `position[i] >= stop_positions[i]` → **STOP POSITION**, stop all fingers

Both conditions are checked **per-timestep** and stop **all fingers immediately** (not per-finger).

---

## 6. Wrist Driver Command/State API

### 6.1 Node

**File:** `src/wrist_driver/wrist_driver/wrist_driver_node.py`  
**Launch:** `src/wrist_driver/launch/wrist_driver.launch.py`

```bash
ros2 launch wrist_driver wrist_driver.launch.py use_mock:=false
```

### 6.2 Parameters

From `src/wrist_driver/config/wrist_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | Serial port for Dynamixel |
| `baudrate` | 57600 | Communication baudrate |
| `motor_id` | 1 | Dynamixel motor ID |
| `protocol_version` | 2.0 | X-series protocol |
| `publish_rate_hz` | 20.0 | State publish rate |

### 6.3 Command Topic

**`/wrist/set_position`** — `std_msgs/Float64MultiArray`

| Index | Field | Units | Description |
|-------|-------|-------|-------------|
| 0 | `target_deg` | degrees | Goal position (0–360, wraps modulo 360) |
| 1 | `acceleration` | deg/s² | Profile acceleration (0 = don't set) |

**Implementation notes:**
- Position is converted: `dxl_pos = int((deg % 360) / 360 * 4095)`
- Writes to Dynamixel control table address 116 (GOAL_POSITION)
- Optional write to address 108 (PROFILE_ACCELERATION) if `accel > 0`
- **No velocity command topic exists.** Only position control is implemented.

### 6.4 State Topic

**`/wrist/state`** — `std_msgs/Float64MultiArray` at 20 Hz

| Index | Field | Units | Source |
|-------|-------|-------|--------|
| 0 | `position_deg` | degrees | Present position (addr 132) |
| 1 | `velocity_deg_s` | deg/s | Present velocity (addr 128) × 0.229 |

### 6.5 Sim Node

**`wrist_driver_sim_node`** — publishes fake `/wrist/state` without hardware.

---

## 7. static_grasp_test Force-Hold Behavior

### 7.1 Script

**File:** `scripts/static_grasp_test.sh` (Python, despite `.sh` extension)  
**Config:** `config/static_grasp_test.yaml`

```bash
make test-static-grasp  # runs via podman with MIA_PORT=/dev/ttyUSB0
```

### 7.2 Behavior Sequence

1. **Wait** for `/controller_manager/list_controllers` service
2. **Reset** hand to relaxed (0.0 rad) via `/group_pos_ff_controller/commands`
3. **Wait** `relaxed_wait_s` (default: 2s) for forces to settle
4. **Switch** to velocity controller (`group_vel_ff_controller` active, `group_pos_ff` inactive)
5. **Velocity ramp closure:**
   - Step through `closing_velocity_start` → `closing_velocity_end` over `decay_steps`
   - Each step lasts `step_interval_s`
   - Monitor forces and positions
6. **Post-ramp:** continue at `closing_velocity_end` until contact or position limit
7. **Stop:** zero velocity, switch back to position control, return to 0.0 rad

### 7.3 Config Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `closing_velocity_start` | 0.3 rad/s | Initial closure speed |
| `closing_velocity_end` | 0.1 rad/s | Final closure speed |
| `decay_steps` | 5 | Number of ramp steps |
| `step_interval_s` | 0.2 s | Time per step |
| `stop_positions` | 1.5 rad each | Max position per finger |
| `force_thresholds` | 300 ADC each | Force contact threshold |
| `relaxed_wait_s` | 2 s | Settle time after reset |

---

## 8. EMG Grasp Test Node (tests/)

### 8.1 File

**`tests/emg_grasp/emg_grasp_node.py`**

State machine:
- **IDLE** → waits for EMG grasp trigger (POWER gesture held ≥1s)
- **CLOSING** → velocity ramp closure, monitors forces/positions
- **HOLDING** → contact detected, zero velocity
- **RELEASING** → OPEN gesture or fault, switch to position control, open hand
- **FAULT** → safety stop

### 8.2 Config

**`tests/emg_grasp/emg_grasp_test.yaml`**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `emg_gesture_topic` | `/emg/gesture_label` | EMG label input |
| `emg_confidence_topic` | `/emg/confidence` | EMG confidence input |
| `emg_proportional_topic` | `/emg/proportional` | Proportional input (subscribed but unused) |
| `emg_grasp_trigger_gesture` | 1 (POWER) | Gesture to trigger grasp |
| `emg_release_gesture` | 3 (OPEN) | Gesture to trigger release |
| `emg_grasp_confidence_threshold` | 0.7 | Min confidence for trigger |
| `gesture_hold_timeout_s` | 1.0 | Seconds gesture must be held |

**Wrist control (currently disabled):**
- `wrist_control_enabled: false`
- `wrist_cmd_topic: /wrist/set_position`
- `wrist_gesture_map` maps gesture IDs to target wrist angles (degrees)

---

## 9. Existing Makefile Targets

### 9.1 Host Makefile (repo root)

| Target | Purpose |
|--------|---------|
| `make up-grasp-test` | Start grasp_test container (docker-compose profile) |
| `make down-grasp-test` | Stop grasp_test container |
| `make logs-grasp-test` | Tail grasp_test logs |
| `make test-static-grasp` | Run static_grasp_test.sh in podman with serial device |
| `make print-force` | Run force print script in podman |

### 9.2 In-Container Makefile (Makefile.workspace)

| Target | Purpose |
|--------|---------|
| `make grasp-test` | Launch legacy grasp_test.launch.py |
| `make mock` | Launch mock pipeline (no hardware) |
| `make pipeline` | Launch full hardware pipeline |
| `make camera-test` | Pipeline without hand/wrist/EMG/haptic |

**Pipeline.launch.py hardware flags:**
```bash
ros2 launch prosthesis_launch pipeline.launch.py mia_hand:=false wrist:=false emg:=false haptic:=false
```

---

## 10. Integration Launch Files

### 10.1 pipeline.launch.py

Full pipeline with optional hardware modules. Key flags:
- `mia_hand` (default true) — launches Mia driver + command_bridge
- `wrist` (default true) — launches wrist_driver_node
- `emg` (default true) — launches `run_classifier` with `--model-dir /app/models`
- `rviz` (default true)

EMG node gets `model_dir` from launch arg (default `/app/models`).

### 10.2 emg_grasp_test.launch.py

Dedicated EMG grasp test launch:
- Launches Mia hand system interface with `controller:=""` (individual pos+vel controllers)
- Runs `tests/emg_grasp/emg_grasp_node.py` with config
- Optional mock EMG publisher for testing

### 10.3 grasp_test.launch.py

Legacy end-to-end grasp pipeline (perception-based, not EMG-driven):
- Camera / mock cloud → segmentation → twist propagation → preshaping → proximity controller
- Also includes wrist_driver_node
- Does **not** include EMG bridge by default

---

## 11. Missing Pieces & Gaps

### 11.1 No FLEXION/EXTENSION Gestures
- **Gap:** `FLEXION` and `EXTENSION` are not defined in `config.py`.
- **Impact:** Downstream consumers expecting these names will fail.
- **Needed:** Compatibility alias mapping in EMG input adapter (mvp-egt.4).

### 11.2 Wrist Has No Velocity Control
- **Gap:** `wrist_driver_node.py` only accepts position commands (`/wrist/set_position`).
- **Impact:** Cannot do velocity-based wrist control for EMG grasp.
- **Needed:** Either add velocity command topic or map EMG proportional to position increments (mvp-egt.7).

### 11.3 No Unified EMG Grasp Makefile Target
- **Gap:** No single `make emg-grasp-test` target exists.
- **Current:** Must run `make up-grasp-test` then manually start EMG or mock.
- **Needed:** Orchestration target (mvp-egt.3).

### 11.4 Force Units Uncalibrated
- **Gap:** JointState `effort` is raw ADC, not Newtons.
- **Impact:** Thresholds are hardware-dependent and may drift.
- **Needed:** Calibration routine or at least documented ADC→N mapping.

### 11.5 EMG Grasp Node Lives in `tests/`
- **Gap:** `emg_grasp_node.py` is under `tests/emg_grasp/`, not `src/`.
- **Impact:** Not installed as a proper ROS package node; launched via `ExecuteProcess` with hardcoded path.
- **Needed:** Move to a proper package or at least ensure path stability in deployment.

### 11.6 Wrist and Hand Share Default Serial Port
- **Gap:** Both default to `/dev/ttyUSB0`.
- **Impact:** `pipeline.launch.py` raises `RuntimeError` if both are enabled with same port.
- **Workaround:** Set `MIA_SERIAL_PORT` and `WRIST_SERIAL_PORT` env vars to different devices.

### 11.7 No Explicit Release/Open Behavior for MIA
- **Gap:** The hand has position controllers, but there is no dedicated "fully open" command service.
- **Impact:** Release is done by publishing position 0.0 to `group_pos_ff_controller/commands`.
- **Needed:** Confirm 0.0 rad is the fully open position for all fingers (it is, per config comments).

### 11.8 Controller Switching Is Best-Effort
- **Gap:** `static_grasp_test.sh` falls back from `STRICT` to `BEST_EFFORT` controller switching.
- **Impact:** May leave controllers in unexpected states on failure.
- **Needed:** Retry logic or health check after switch.

---

## 12. Quick Reference: Topic Cheat Sheet

| Topic | Pub/Sub | Type | Owner |
|-------|---------|------|-------|
| `/emg/gesture_label` | PUB | `Int32` | emg_bridge |
| `/emg/gesture_name` | PUB | `String` | emg_bridge |
| `/emg/confidence` | PUB | `Float32` | emg_bridge |
| `/emg/proportional` | PUB | `Float32` | emg_bridge |
| `/joint_states` | PUB | `JointState` | joint_state_broadcaster |
| `/group_pos_ff_controller/commands` | SUB | `Float64MultiArray` | group_pos_ff |
| `/group_vel_ff_controller/commands` | SUB | `Float64MultiArray` | group_vel_ff |
| `/wrist/set_position` | SUB | `Float64MultiArray` | wrist_driver |
| `/wrist/state` | PUB | `Float64MultiArray` | wrist_driver |
| `/mia_hand/emergency_stop` | SRV | `Trigger` | mia_safety |
| `/mia_hand/play` | SRV | `Trigger` | mia_safety |
| `/controller_manager/switch_controller` | SRV | `SwitchController` | controller_manager |

---

*Document generated for bead mvp-egt.1. Last updated: 2026-05-25*
