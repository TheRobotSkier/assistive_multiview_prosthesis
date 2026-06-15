# Control And Actuation Architecture

## Scope

This layer covers orchestration, grasp planning, approach control, finger/wrist execution, force-based grasp closure, and haptic feedback.

Relevant packages:

- `pipeline_manager`
- `grasp_preshaping`
- `command_bridge`
- `mia_hand_driver`
- `wrist_driver`
- `force_controller`
- `haptic_band`
- `mia_hand_ros2_control`

## Pipeline Manager

Main file: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`

Role:

- Own the main state machine
- Subscribe to EMG, segmentation, proximity, and force status
- Activate and deactivate twist propagation
- Trigger grasp planning through `/grasp_preshaping/compute_grasp`
- Publish pipeline state topics used by downstream nodes

This node is the runtime orchestrator. When grasp behavior changes, inspect this file before changing other subsystems.

## Grasp Planning

Main file: `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`

Role:

- Gather latest hand pose, hand twist, segmented cloud, and contact override
- Call the Rust preshaping shared library
- Publish target finger closures, wrist pose, target hand pose, and grasp type

Important dependency:

- Rust shared library loaded from the workspace install or source tree via `dlopen`

## Proximity-Based Approach Control

Main file: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

Role:

- Consume planner outputs and predicted contact pose
- Use TF lookup of `marker_map -> grasp_contact_frame`
- Apply partial closure when far away
- Apply fuller closure and wrist approach when near target
- Publish `/proximity/near_zone_entered`

This file is the key place for staged approach behavior before force control takes over.

## Finger Command Bridge

Main file: `src/command_bridge/command_bridge/command_bridge_node.py`

Role:

- Convert position-command topics into raw Mia Hand trajectory service calls
- Republish raw joint data as `/joint_states`

This is the adapter between higher-level controller-style topics and the raw driver API.

## Raw Hand Driver

Main files:

- `src/mia_hand_driver/src/mia_hand_driver/ros_driver.cpp`
- `src/mia_hand_driver/src/mia_hand_driver_node.cpp`

Role:

- Expose serial communication to Mia Hand hardware as ROS services, streams, and actions

## Wrist Driver

Main file: `src/wrist_driver/wrist_driver/wrist_driver_node.py`

Role:

- Accept `/wrist/set_position`
- Drive the Dynamixel wrist
- Publish `/wrist/state`

## Force Controller

Main file: `src/force_controller/force_controller/force_controller_node.py`

Role:

- Activate during `GRASPING`
- Switch controller ownership to velocity-mode control via controller-manager services
- Close until force threshold or stop condition is reached
- Hold force after contact
- Publish `ForceControllerStatus`

Important architectural note:

- The force controller assumes controller-manager-based resources exist.
- The main pipeline also uses the raw driver + command-bridge path.
- Any changes to hand command ownership should be documented carefully because this is one of the easiest integration points to break.

## Haptic Feedback

Main files:

- `src/haptic_band/haptic_bridge/haptic_controller_node.py`
- `src/haptic_band/haptic_bridge/bridge_node.py`

Role:

- Map wrist angle, finger force, and pipeline state into Vibro8 motor commands
- Send those commands over Bluetooth

## Control Change Checklist

If you change control or actuation behavior, likely update:

- `docs/architecture/control-and-actuation.md`
- `docs/architecture/overview.md`
- `docs/architecture/runtime-entrypoints.md`
- `docs/architecture/packages.md`
- `docs/reference/directory-structure.md`
- `config/prosthesis_config.yaml` if any topics, thresholds, modes, or parameters changed
