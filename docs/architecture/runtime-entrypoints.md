# Runtime Entrypoints

## Primary Launch Files

Located in `src/prosthesis_launch/launch/`.

### `pipeline.launch.py`

Primary hardware pipeline.

Starts:

- `pipeline_manager_node`
- `mia_hand_driver_node`
- `command_bridge_node`
- `wrist_driver_node`
- `run_classifier`
- `emg_grasp_controller`
- `bridge_node` and `haptic_controller_node`
- `openvins_realsense_tf_bridge_node`
- `pointcloud_fusion_node`
- `odom_to_pose_relay`
- `openvins_odom_tf_relay`
- `segmentation_ros2_node`
- `twist_propagation_node`
- `preshaping_service_bridge_node`
- `grasp_proximity_controller_node.py`
- `force_controller_node`
- optional `rviz2`

Key launch arguments include:

- `config_file`
- `camera`
- `camera_tf_bridge`
- `mia_hand`
- `wrist`
- `emg`
- `haptic`
- `mia_serial_port`
- `wrist_serial_port`
- `target_frame`
- `odom_topic`
- `cam1_topic`
- `cam2_topic`
- `camera_mount`
- `mounts_config`
- `inference_url`
- `roi_radius`
- `tf_diagnostics`
- `require_dual_openvins`

### `simple_emg_grasp.launch.py`

Execution-only pipeline with no perception stack.

Starts:

- `simple_pipeline_manager_node`
- `mia_hand_driver_node`
- `command_bridge_node`
- `wrist_driver_node`
- `run_classifier`
- `force_controller_node`
- optional `rviz2`

Use this when changing EMG-triggered closing behavior without needing cameras, segmentation, or grasp planning.

### `mia_haptic_force_test.launch.py` (split-node, multi-process)

Multi-process pipeline for the Mia Hand, wrist, EMG classifier, and
Vibro8 haptics with no perception stack.  Each node is a standalone
Python script under `scripts/mia_haptic_force_test/` and communicates
via the canonical topic contract in
`scripts/mia_haptic_force_test/common/README.md`.

Starts:

- `emg_input_node` (or `keyboard_emg_node` in `USE_MULTI_NODE=true` mode)
- `force_input_node`
- `supervisor_node`
- `hand_controller_node`
- optional `haptic_node`
- optional `terminal_ui_node`
- optional `logger_node`
- optional `hand_simulator_node` (when `mock_hardware:=true`)

Launch arguments:

- `mock_hardware` (default `false`): launch `hand_simulator_node` instead of expecting real hardware.  The simulator publishes the raw Mia hardware streams (`data_streams/fingers/forces/data`, `data_streams/motors/{positions,speeds,currents}/data`, `data_streams/joints/{positions,speeds,efforts}/data`) plus `/hand_sim/joint_states`, `/hand_sim/forces`, and `/wrist/state` (Float64MultiArray `[deg, vel]`).  When true, `force_input_node` is pointed at `/hand_sim/joint_states` and `/hand_sim/forces` via `--ros-args -p ...`.
- `wrist_enable` (default `true`): enable the real wrist driver.  Set to `false` in `mock_hardware:=true` mode so the real `/dev/ttyDynamixel` is not opened.
- `terminal_ui` (default `true`): launch `terminal_ui_node`.  Set to `false` for headless CI runs.
- `logger` (default `true`): launch `logger_node`.  Set to `false` for headless CI runs.

Use this when testing wrist rotation, force closure, force-hold target adjustment, POWER toggling between force and wrist control, OPEN-stop behavior, haptic mappings, and CSV capture without cameras or the production pipeline manager.

### `mia_haptic_force_test.launch.py` (monolith, legacy)

Bench-test pipeline (single Python process) for the Mia Hand, wrist, EMG classifier, and Vibro8 haptics with no perception stack.

Starts:

- Mia Hand ros2_control stack
- `run_classifier`
- optional `wrist_driver_node`
- optional `haptic_bridge_node`
- `scripts/mia_haptic_force_test.py`

Use this as a reference for the split-node behaviour or when running on real hardware with a single process.
### Other Launch Files

- `mock.launch.py`: mock pipeline for non-hardware use
- `digital_twin.launch.py`: digital twin mode
- `grasp_test.launch.py`: legacy grasp testing path
- `emg_grasp_test.launch.py`: EMG-focused test path
- `mia_haptic_force_test.launch.py`: EMG/haptic force test with CSV logging
- `tf_bridge.launch.py`: TF bridge only
- `twist_propagation_test.launch.py`: twist/fusion-related testing

## Central Configuration

Primary file: `config/prosthesis_config.yaml`

This file has two roles:

- ROS parameter source for nodes via namespaced sections like `pipeline_manager.ros__parameters`
- Flat reference store for scripts and external readers

If a node parameter is added or renamed, update both the node implementation and the correct config section here.

## Host and Container Entrypoints

### Host-level

- `Makefile`
- `docker/`
- `scripts/`

Main commands:

- `make dev`
- `make shell`
- `make run`
- `make run-emg-grasp`
- `make test-grasp`
- `make test`

### In-container workspace

- `Makefile.workspace`

Main targets:

- `make build`
- `make test`
- `make pipeline`
- `make run`
- `make run-emg-grasp`
- `make test-grasp`
- `make camera-test`
- `make tonight-*`
