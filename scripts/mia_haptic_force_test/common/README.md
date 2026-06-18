# Topic Contract — mia_haptic_force_test (mvp-7jf)

This document defines the canonical ROS 2 topics for the split-node
`mia_haptic_force_test` architecture.  Every node that publishes or
subscribes to an inter-node topic **must** use the name listed here
(overridable via config file where noted).

## Conventions

- **Type** columns use short ROS 2 message names (e.g.
  `std_msgs/Float32MultiArray`).  The ``std_msgs/`` prefix may be omitted
  after the first occurrence within a section.
- **Publisher** and **Subscriber(s)** name the planned node(s); these
  are **not** yet implemented — they document the intended data flow.
- Topics prefixed ``data_streams/`` are raw hardware output from the
  Mia Hand driver (``mia_hand_driver``) and are consumed only by
  input-processing nodes, **not** by the controller directly.

---

## 1. Hand State

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/hand/joint_states` | `sensor_msgs/JointState` | `hand_state_node` | `hand_controller_node`, logger | Filtered/selected finger joint positions, velocities, and efforts for the three flexion joints (`j_thumb_fle`, `j_index_fle`, `j_mrl_fle`). Other joints may be present but are ignored. |
| `/hand/forces` | `std_msgs/Float32MultiArray` | `force_input_node` | `hand_controller_node`, `haptic_node`, logger | Normal contact forces `[thumb, index, mrl]` in hardware-specific units. |
| `/hand/force_source` | `std_msgs/String` | `force_input_node` | `hand_controller_node`, logger | Identifies the active force data source: `"force_data"`, `"joint_state_effort"`, or `"none"`. |

## 2. EMG Input

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/emg/gesture_name` | `std_msgs/String` | `emg_input_node` | `hand_controller_node`, `haptic_node` | Gesture name string, e.g. `"REST"`, `"POWER"`, `"OPEN"`. |
| `/emg/gesture_label` | `std_msgs/Int32` | `emg_input_node` | `hand_controller_node` | Numeric gesture label (0-4+). See `EMG_*_LABEL` constants in `constants.py`. |
| `/emg/confidence` | `std_msgs/Float32` | `emg_input_node` | `hand_controller_node` | Classifier confidence `[0.0, 1.0]`. |
| `/emg/proportional` | `std_msgs/Float32` | `emg_input_node` | `hand_controller_node` | Proportional EMG signal `[0.0, 1.0]` for continuous control. |

## 3. Control Setpoints

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/control/target_force` | `std_msgs/Float64MultiArray` | `supervisor_node` | `hand_controller_node`, `haptic_node`, logger | Per-finger force targets `[thumb, index, mrl]` in hardware units. |
| `/control/target_wrist` | `std_msgs/Float64` | `supervisor_node` | `hand_controller_node`, wrist node | Wrist target angle in degrees. |
| `/control/mode` | `std_msgs/String` | `supervisor_node` | `hand_controller_node`, wrist node, `haptic_node`, logger | Controller mode: `"position"`, `"velocity"`, or `"emergency_backoff"`. |
| `/control/enable` | `std_msgs/Bool` | `supervisor_node` | `hand_controller_node`, wrist node | When `False`, all controller outputs are suppressed (motors stop, hand holds). |
| `/control/hold_mode` | `std_msgs/String` | `supervisor_node` | `hand_controller_node`, `haptic_node`, logger | Hold phase the haptics should render: `"force"` or `"wrist"`. |

## 4. Test Orchestration

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/test/stage` | `std_msgs/String` | `supervisor_node` | `terminal_ui_node`, logger | Current test stage name (values from `Stage` enum in `constants.py`). |
| `/test/event` | `std_msgs/String` | `supervisor_node` | `terminal_ui_node`, logger | JSON-encoded event payload describing noteworthy transitions. |
| `/test/status` | `std_msgs/String` | `supervisor_node` | `terminal_ui_node`, logger | JSON-encoded periodic status snapshot with current measurements. |

## 5. Controller Feedback

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/controller/force_error` | `std_msgs/Float64MultiArray` | `hand_controller_node` | diagnostic / logger | Per-finger force error `[error_thumb, error_index, error_mrl]`. |
| `/controller/active` | `std_msgs/Bool` | `hand_controller_node` | `supervisor_node`, `terminal_ui_node` | ``True`` while the force controller is actively modulating output. |
| `/controller/loop_timing` | `std_msgs/String` | `hand_controller_node` | diagnostic / logger | JSON payload with performance metrics: `hz`, `jitter_ms`, etc. |

## 6. Haptic Band Output

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/haptic_band/motors` | `std_msgs/Float32MultiArray` | `haptic_node` | Haptic band driver | 8 motor intensity values `[0.0, 100.0]`. |

## 7. Hardware Command Topics

These are written by the controller to drive the physical hand.

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/group_vel_ff_controller/commands` | `std_msgs/Float64MultiArray` | `hand_controller_node` | `group_vel_ff_controller` | Velocity commands forwarded to the hand's velocity FF controller. |
| `/group_pos_ff_controller/commands` | `std_msgs/Float64MultiArray` | `hand_controller_node` | `group_pos_ff_controller` | Position commands forwarded to the hand's position FF controller. |

## 8. Wrist Topics

| Topic | Type | Publisher | Subscriber(s) | Semantics |
|---|---|---|---|---|
| `/wrist/set_position` | `std_msgs/Float64MultiArray` | `force_controller_node` / `wrist_node` | `wrist_node` | `[target_deg, acceleration]` — target angle and acceleration for wrist movement. |
| `/wrist/state` | `std_msgs/Float64MultiArray` | `wrist_node` | `force_controller_node` | `[pos_deg, vel_deg_s]` — current wrist angle and velocity. |

## 9. Hardware Input Streams (raw Mia Hand topics)

Consumed by input-processing nodes, **not** directly by the controller.
These topics originate from the Mia Hand driver's data stream publishers.

| Topic | Type | Subscriber (consumer) | Semantics |
|---|---|---|---|
| `data_streams/fingers/forces/data` | `mia_hand_msgs/ForceData` | `force_input_node` | Raw finger normal and tangential forces. |
| `data_streams/motors/positions/data` | `mia_hand_msgs/MotorData` | `force_input_node` | Raw motor position data. |
| `data_streams/motors/speeds/data` | `mia_hand_msgs/MotorData` | `force_input_node` | Raw motor speed data. |
| `data_streams/motors/currents/data` | `mia_hand_msgs/MotorData` | `force_input_node` | Raw motor current draw. |
| `data_streams/joints/positions/data` | `mia_hand_msgs/JointData` | `hand_state_node` | Raw joint angle data from the hand. |
| `data_streams/joints/speeds/data` | `mia_hand_msgs/JointData` | `hand_state_node` | Raw joint speed data from the hand. |
| `/joint_states` | `sensor_msgs/JointState` | `hand_state_node` | Full joint state from `robot_state_publisher` or the hardware driver. |

---

## Topic Name Migration from Legacy

| Legacy topic | New topic | Rationale |
|---|---|---|
| `/emg/gesture_name` | `/emg/gesture` | Shorter, matches the field name. |
| `/mia_haptic_force_test/state` | `/test/stage` | Namespaced under `/test/` for clarity. |
| `/mia_haptic_force_test/status` | `/test/status` | Namespaced under `/test/` for clarity. |
| `/joint_states` (consumed directly) | `/hand/joint_states` (processed) | Input nodes re-publish filtered data on a dedicated topic. |
| *(new)* | `/hand/forces` | Separated from the monolithic node. |
| *(new)* | `/hand/force_source` | New diagnostic topic. |
| *(new)* | `/control/target_force` | Separated force target publication. |
| *(new)* | `/control/target_wrist` | Separated wrist target publication. |
| *(new)* | `/control/mode` | Separated mode publication. |
| *(new)* | `/control/enable` | New supervisor enable signal. |
| *(new)* | `/control/hold_mode` | Separated hold-mode publication. |
| *(new)* | `/test/event` | New structured event topic. |
| *(new)* | `/controller/force_error` | New diagnostic topic. |
| *(new)* | `/controller/active` | New diagnostic topic. |
| *(new)* | `/controller/loop_timing` | New diagnostic topic. |
