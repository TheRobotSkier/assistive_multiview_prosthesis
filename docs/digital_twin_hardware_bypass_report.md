# Digital Twin Hardware Bypass Report

## Scope

Confirmed the current integration path renders the Mia hand in RViz and does
not require physical hand hardware.

## Fix

The `digital_twin` compose service now uses the same CycloneDDS peer config as
the Jetson-facing host tools:

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `ROS_DOMAIN_ID=0`
- `CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml`
- mount: `config/cyclonedds_peer.xml`

Without this, the host digital twin container could fail to discover Jetson
camera/OpenVINS topics even after the ethernet link worked.

## Hardware Bypass

`digital_twin.launch.py` starts:

- `robot_state_publisher`
- `digital_twin_joint_state_publisher`
- RViz hand model

It does not start:

- `mia_hand_driver_node`
- `force_controller_node`
- hardware `ros2_control`

Finger command topics still exist, but they drive `/joint_states` for RViz
instead of physical motors.

## Test

Passed locally:

```bash
bash scripts/test_digital_twin_hardware_bypass_contracts.sh
```
