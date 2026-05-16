# Grasp Integration Report

## Scope

Locked the segmentation-to-grasp-to-digital-hand interface.

## Interface

Inputs to grasp bridge:

- `/hand_pose`
- `/hand_twist`
- `/segmentation/object_cloud`

Service:

- `/grasp_preshaping/compute_grasp`

Outputs:

- `/grasp_preshaping/target_hand_pose`
- `/grasp_preshaping/target_finger_closures`
- `/grasp_preshaping/grasp_type`
- `/thumb_pos_ff_controller/commands`
- `/index_pos_ff_controller/commands`
- `/mrl_pos_ff_controller/commands`

Digital twin consumption:

- command topics above -> `/joint_states`
- `robot_state_publisher` renders Mia hand in RViz

## Digital Twin Behavior

`digital_twin.launch.py` sets `publish_initial_commands:=False` on the
preshaping bridge. That keeps the planner outputs available while leaving the
proximity/controller stage responsible for deciding when to command closure.

## Tests

Passed locally:

```bash
bash scripts/test_grasp_contracts.sh
```
