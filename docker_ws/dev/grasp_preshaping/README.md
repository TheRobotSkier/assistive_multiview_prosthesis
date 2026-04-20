# grasp_preshaping

This crate provides the preshaping solver core and a C ABI entrypoint consumed by the C++ ROS2 wrapper.

## Runtime architecture

- Public ROS API: `/grasp_preshaping/compute_grasp` (`std_srvs/srv/Trigger`), implemented by `mia_hand_mujoco` C++ node.
- Rust role: compute library only.
- Entry points exported by this crate:
	- `grasp_preshaping_api_version`
	- `grasp_preshaping_compute`

On each service call, the C++ node:

1. Uses the latest cached simulator messages from:
- `/hand_pose`
- `/hand_twist`
- `/segmented_object_cloud`
2. Calls `grasp_preshaping_compute` through FFI.
3. Publishes controller commands to:
- `/thumb_pos_ff_controller/commands`
- `/index_pos_ff_controller/commands`
- `/mrl_pos_ff_controller/commands`

## Build

Build the Rust library (including `cdylib`):

```bash
cargo build --release --lib
```

## Notes

- This crate no longer hosts a ROS runtime node.
- If required inputs are missing, the wrapper service returns `success=false` with a descriptive message.
