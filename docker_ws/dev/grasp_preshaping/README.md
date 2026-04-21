# grasp_preshaping

This package provides the grasp preshaping solver core (Rust), the FFI type definitions shared between Rust and C++, and the ROS 2 service bridge node.

## Runtime architecture

- **Public ROS API**: `/grasp_preshaping/compute_grasp` (`std_srvs/srv/Trigger`), implemented by the C++ bridge node in this package.
- **Rust role**: compute library only — no ROS runtime node.
- **Entry points exported by the Rust cdylib**:
  - `grasp_preshaping_api_version`
  - `grasp_preshaping_compute`

On each service call, the C++ bridge node:

1. Uses the latest cached simulator messages from:
   - `/hand_pose`
   - `/hand_twist`
   - `/segmented_object_cloud`
2. Calls `grasp_preshaping_compute` through FFI.
3. Publishes controller commands to:
   - `/thumb_pos_ff_controller/commands`
   - `/index_pos_ff_controller/commands`
   - `/mrl_pos_ff_controller/commands`

## Package layout

```
grasp_preshaping/
├── Cargo.toml                  Rust build manifest
├── CMakeLists.txt              ament_cmake: builds Rust cdylib + C++ bridge node
├── package.xml                 ROS 2 package manifest
├── include/grasp_preshaping/
│   └── ffi_types.hpp           Single source of truth for FFI structs
├── nodes/
│   └── preshaping_service_bridge_node.cpp   C++ ROS 2 service bridge
├── src/                        Rust sources
└── data/                       Lookup table data
```

The FFI structs in `include/grasp_preshaping/ffi_types.hpp` must be kept in sync with the `#[repr(C)]` definitions in `src/c_api.rs`.

## Build

Build the full package (Rust library + C++ bridge node):

```bash
colcon build --packages-select grasp_preshaping
```

Build Rust library only (for development):

```bash
cargo build --release --lib
```

## Notes

- If required inputs are missing, the service returns `success=false` with a descriptive message.
- The service callers (trigger clients) live in `mia_hand_mujoco` (interactive and planner-gui system interfaces) and are pure ROS service clients with no dependency on this package's implementation.
