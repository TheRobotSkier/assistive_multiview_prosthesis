# MIA Hand Grasp Control Architecture

## Overview

The grasp control system has three layers: orchestration, planning, and execution.

```
┌──────────────────────────────────────────────────────────────────────┐
│                        ORCHESTRATION LAYER                           │
│                                                                      │
│  EMG gesture ──→ pipeline_manager (state machine)                    │
│                    │                                                 │
│                    ├─ IDLE → SEGMENTING → PLANNING → APPROACHING     │
│                    │                        → GRASPING → HOLDING     │
│                    └─ Any state → RELEASING (on OPEN gesture)        │
│                                                                      │
│  Segments cloud, triggers grasp computation, monitors pipeline state │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ /grasp_preshaping/compute_grasp
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                         PLANNING LAYER                                │
│                                                                      │
│  preshaping_service_bridge_node (C++ / Rust FFI)                     │
│    • TSDF construction from point cloud                              │
│    • SMC sampling for grasp optimization                             │
│    • Output: target hand pose, finger closures, wrist rotation       │
│                                                                      │
│  grasp_proximity_controller_node (Python)                            │
│    • Tracks 3D hand pose vs target pose (near/far detection)         │
│    • FAR  → publishes /hand_control/preshape (partial closure)       │
│    • NEAR → publishes /hand_control/close (force-aware closure)      │
│    • RELEASING → publishes /hand_control/reset                       │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ /hand_control/preshape
                               │ /hand_control/close
                               │ /hand_control/reset
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        EXECUTION LAYER                                │
│                                                                      │
│  hand_control_interface_node (Python)                                │
│    • Pre-shapes fingers and wrist to approach position               │
│    • Runs velocity-based force-aware closure:                        │
│        velocity A ──(decay N steps)──→ velocity B                    │
│        monitors /joint_states for positions and effort                │
│        stops ALL fingers immediately on force contact or stop_pos     │
│    • Returns hand to relaxed position on reset                       │
│    • Uses NaN-in-Float64MultiArray convention for default values      │
│                                                                      │
│  Config: /prosthesis_ws/config/static_grasp_test.yaml                │
│    • velocity ramp parameters                                        │
│    • per-finger stop positions and force thresholds                   │
│    • default preshape values (fallback when NaN on preshape topic)    │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ /group_pos_ff_controller/commands
                               │ /group_vel_ff_controller/commands
                               │ /wrist/set_position
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        HARDWARE LAYER                                 │
│                                                                      │
│  ros2_control (controller_manager)                                   │
│    • MiaHandSystemInterface (serial USB → MIA hand)                  │
│    • joint_state_broadcaster → /joint_states (pos, vel, effort)       │
│    • group_pos_ff_controller, group_vel_ff_controller                │
│                                                                      │
│  wrist_driver_node (Dynamixel)                                       │
│                                                                      │
│  mia_safety_node (/mia_hand/emergency_stop, /mia_hand/play)          │
└──────────────────────────────────────────────────────────────────────┘
```

## Topic Map

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/hand_control/preshape` | Float64MultiArray | proximity_controller | hand_control_interface | `[wrist_deg, thumb, index, mrl]`. NaN = use YAML default |
| `/hand_control/close` | Empty | proximity_controller | hand_control_interface | Trigger velocity-based force-aware closure |
| `/hand_control/reset` | Empty | proximity_controller | hand_control_interface | Return hand to relaxed (0.0 rad) |
| `/joint_states` | JointState | joint_state_broadcaster | hand_control_interface, proximity_controller | Positions + efforts |
| `/group_pos_ff_controller/commands` | Float64MultiArray | hand_control_interface | controller_manager | Position setpoint |
| `/group_vel_ff_controller/commands` | Float64MultiArray | hand_control_interface | controller_manager | Velocity setpoint |
| `/wrist/set_position` | Float64MultiArray | hand_control_interface | wrist_driver | Wrist Dynamixel target |
| `/grasp_preshaping/target_finger_closures` | Float64MultiArray | preshaping_service | proximity_controller | Planner output |
| `/grasp_preshaping/wrist_pose` | Float64 | preshaping_service | proximity_controller | Wrist target |
| `/grasp_preshaping/target_hand_pose` | PoseStamped | preshaping_service | proximity_controller | 3D hand target for proximity |
| `/hand_pose` | PoseStamped | hand_pose_publisher | proximity_controller | Current hand 3D pose |
| `/pipeline/state` | Int32 | pipeline_manager | proximity_controller | State for release trigger |
| `/mia_hand/emergency_stop` | Trigger | mia_safety | operator/scripts | Hardware emergency stop |
| `/mia_hand/play` | Trigger | mia_safety | operator/scripts | Resume after emergency stop |

## Force-Aware Closure Flow

1. `proximity_controller` detects hand is NEAR target → publishes `Empty` on `/hand_control/close`
2. `hand_control_interface` loads `group_vel_ff_controller`, switches from position to velocity control
3. Computes velocity ramp: `v_start` → `v_end` over N discrete steps (from YAML config)
4. Each step: publish velocity, wait `step_interval_s`, read `/joint_states`
5. Stop conditions (checked at every step):
   - **Force contact**: any finger's `effort` ≥ its `force_threshold` → stop ALL fingers, velocity=0
   - **Stop position**: any finger's `position` ≥ its `stop_position` → stop ALL fingers, velocity=0
6. After ramp: continue at `v_end` until a stop condition triggers
7. On stop: publish velocity=0, switch back to position controller, log final state

## Config File

`config/static_grasp_test.yaml`:

```yaml
# Velocity ramp
closing_velocity_start: 0.3    # rad/s
closing_velocity_end: 0.1      # rad/s
decay_steps: 5                 # number of discrete steps
step_interval_s: 0.2           # time between steps

# Per-finger safety limits
stop_positions:
  j_thumb_fle: 1.5
  j_index_fle: 1.5
  j_mrl_fle:   1.5

# Per-finger force thresholds (raw ADC)
force_thresholds:
  j_thumb_fle: 300
  j_index_fle: 300
  j_mrl_fle:   300

# Default preshape (used when NaN on /hand_control/preshape)
default_preshape:
  wrist_deg: 0.0
  thumb_closure: 0.3
  index_closure: 0.3
  mrl_closure: 0.3

relaxed_wait_s: 2
```

## Running

```bash
# Full hardware pipeline
ros2 launch prosthesis_launch mia_force_grasp_hardware.launch.py

# Force monitor only
make print-force

# Physical grasp test (standalone velocity closure)
make test-static-grasp
```

## Safety

- **Force thresholds**: per-finger, configured in YAML. Exceeding any stops all fingers immediately.
- **Stop positions**: hard position limit per finger, prevents over-closure into empty space.
- **Emergency stop**: `/mia_hand/emergency_stop` service sends `@AS...` serial command to halt all motors. `/mia_hand/play` resumes.
- **Reset**: `/hand_control/reset` returns hand to relaxed (0.0 rad) from any state.`
- **Timeout**: if velocity closure doesn't find contact, fingers stop at `stop_position`.
