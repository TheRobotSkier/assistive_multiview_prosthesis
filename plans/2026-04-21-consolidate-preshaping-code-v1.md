# Consolidating Preshaping Code into `docker_ws/dev/grasp_preshaping`

## Objective

Move all preshaping-related code — the C++ service bridge node, FFI struct definitions, and service trigger logic — from `docker_ws/mia_hand_mujoco` into `docker_ws/dev/grasp_preshaping`, so the entire preshaping service can be built and run from a single ROS 2 package. This eliminates the current fragmentation where the Rust compute library lives in one package but the C++ wrapper node that loads it lives in another.

---

## Current Fragmentation Map

### Where things live today:

| Component | Current Location | ROS Package |
|---|---|---|
| Rust compute library (core planner) | `dev/grasp_preshaping/src/*.rs` | `grasp_preshaping` |
| Rust FFI struct definitions (`c_api.rs`) | `dev/grasp_preshaping/src/c_api.rs` | `grasp_preshaping` |
| Rust `Cargo.toml`, `Cargo.lock` | `dev/grasp_preshaping/` | `grasp_preshaping` |
| Rust LUT data (`data/finger_contact_lut.npz`) | `dev/grasp_preshaping/data/` | `grasp_preshaping` |
| Rust build CMakeLists + package.xml | `dev/grasp_preshaping/` | `grasp_preshaping` |
| **C++ service bridge node** | `mia_hand_mujoco/src/preshaping_service_bridge_node.cpp` | `mia_hand_mujoco` |
| **C++ FFI struct definitions** (duplicated) | `mia_hand_mujoco/src/preshaping_service_bridge_node.cpp` lines 20-76 | `mia_hand_mujoco` |
| **Launch file** (launches bridge node) | `mia_hand_mujoco/launch/mia_hand_system_interface_launch.py` lines 250-256 | `mia_hand_mujoco` |
| **Preshaping trigger client** (interactive) | `dev/mujoco/interactive_simulator/interactive_system_interface.cpp` lines 715-762 | `mia_hand_mujoco` |
| **Preshaping trigger client** (planner GUI) | `dev/mujoco/gui_simulator/planner_gui_system_interface.cpp` lines 483-537 | `mia_hand_mujoco` |

### The problem:
- The C++ bridge node (`preshaping_service_bridge_node.cpp`) duplicates the FFI struct definitions from `c_api.rs` as anonymous namespace C structs. These must be kept in sync manually.
- The bridge node is built as part of `mia_hand_mujoco`, which depends on MuJoCo, GLFW, and all the simulation infrastructure — just to load a `.so` and call two functions.
- The `grasp_preshaping` package only builds the Rust `.so` and installs it. It has no C++ component and no ROS node.
- Anyone working on the preshaping pipeline has to touch two packages in two different directory trees.

---

## Target Structure

After consolidation, `docker_ws/dev/grasp_preshaping/` would contain:

```
dev/grasp_preshaping/
├── Cargo.toml
├── Cargo.lock
├── CMakeLists.txt              ← extended to also build the C++ bridge node
├── package.xml                 ← updated with C++ dependencies
├── README.md
├── data/
│   └── finger_contact_lut.npz
├── src/                        ← Rust sources (unchanged)
│   ├── c_api.rs
│   ├── planner.rs
│   ├── pointcloud_helper.rs
│   ├── predictor.rs
│   └── lut_helper.rs
├── include/
│   └── grasp_preshaping/
│       └── ffi_types.hpp       ← NEW: single source of truth for FFI structs
├── nodes/
│   └── preshaping_service_bridge_node.cpp   ← MOVED from mia_hand_mujoco/src/
├── benches/
│   └── pipeline.rs
├── scripts/
│   ├── hand_tip_visualizer.py
│   └── check_runtime_drift.sh
└── urdf/
    └── ...
```

---

## Implementation Plan

### Phase 1: Create shared FFI header

- [ ] **1.1. Create `dev/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp`.** Extract the FFI struct definitions currently duplicated in `preshaping_service_bridge_node.cpp:20-76` into a standalone header. This header defines `GraspPoseFFI`, `GraspTwistFFI`, `PointCloudViewFFI`, `GraspComputeRequestFFI`, `GraspComputeResponseFFI`, and the function pointer types `GraspComputeFn` / `GraspApiVersionFn`. These structs must match the Rust `#[repr(C)]` definitions in `c_api.rs` exactly. Having a single header avoids the sync problem.

- [ ] **1.2. Verify struct layout compatibility.** Compare the C++ struct definitions in the new header against the Rust `#[repr(C)]` structs in `c_api.rs:38-90`. Pay special attention to:
  - Field order (C and Rust must match exactly)
  - `std::array<double, 36>` in C++ vs `[f64; 36]` in Rust for covariance
  - `size_t` vs `usize` for PointCloudViewFFI fields
  - `uint8_t` vs `u8` for the success field
  - `int32_t` vs `i32` for grasp_type
  - The function signature: `int (*) (const Request*, Response*, char*, size_t)` in C vs `fn(*const Request, *mut Response, *mut c_char, usize) -> i32` in Rust

### Phase 2: Move the bridge node

- [ ] **2.1. Move `mia_hand_mujoco/src/preshaping_service_bridge_node.cpp` to `dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`.** The physical file move.

- [ ] **2.2. Update the moved file to use the shared FFI header.** Replace the anonymous namespace struct definitions (lines 18-76) with `#include "grasp_preshaping/ffi_types.hpp"`. The rest of the node code (the `PreshapingServiceBridgeNode` class, `main()`) stays the same.

- [ ] **2.3. Update `dev/grasp_preshaping/CMakeLists.txt`** to add the C++ bridge node build target. The extended CMakeLists should:
  - Keep the existing Rust `cdylib` build via `add_custom_command` / `add_custom_target`
  - Add `find_package(rclcpp REQUIRED)`, `find_package(geometry_msgs REQUIRED)`, `find_package(sensor_msgs REQUIRED)`, `find_package(std_msgs REQUIRED)`, `find_package(std_srvs REQUIRED)`
  - Add `add_executable(preshaping_service_bridge_node nodes/preshaping_service_bridge_node.cpp)`
  - Add `ament_target_dependencies` for the new executable
  - Link against `dl` (for `dlopen`/`dlsym`)
  - Add `target_include_directories` for the `include/` directory
  - Add `install(TARGETS preshaping_service_bridge_node RUNTIME DESTINATION lib/${PROJECT_NAME})`
  - Add an explicit dependency so the C++ target builds after the Rust library: `add_dependencies(preshaping_service_bridge_node grasp_preshaping_rustlib)`

- [ ] **2.4. Update `dev/grasp_preshaping/package.xml`** to declare the new dependencies:
  ```xml
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  ```

### Phase 3: Update `mia_hand_mujoco`

- [ ] **3.1. Remove the bridge node from `mia_hand_mujoco/CMakeLists.txt`.** Delete:
  - Lines 139-153: `add_executable(preshaping_service_bridge_node ...)` and its `ament_target_dependencies` and `target_link_libraries`
  - Lines 218-220: `install(TARGETS preshaping_service_bridge_node ...)`

- [ ] **3.2. Delete `mia_hand_mujoco/src/preshaping_service_bridge_node.cpp`.** The file has been moved to `dev/grasp_preshaping/nodes/`.

- [ ] **3.3. Update `mia_hand_mujoco/launch/mia_hand_system_interface_launch.py`.** Change the preshaping bridge node entry at lines 250-256 to reference the new package:
  ```python
  preshaping_service_bridge_node = Node(
      package='grasp_preshaping',           # changed from 'mia_hand_mujoco'
      executable='preshaping_service_bridge_node',
      name='preshaping_service_bridge',
      output='screen',
      condition=IfCondition(enable_preshaping_service),
  )
  ```
  No other changes needed — the node name, service name, and topic names remain identical.

### Phase 4: Update build order

- [ ] **4.1. Verify the colcon build dependency chain.** The `mia_hand_mujoco` launch file now references a node from `grasp_preshaping`, but this is a runtime dependency (launch-time), not a build-time dependency. Colcon will build both packages regardless. However, if `mia_hand_mujoco` is built before `grasp_preshaping`, the launch file will still work because the node is discovered at runtime. No `package.xml` dependency changes needed in `mia_hand_mujoco`.

- [ ] **4.2. Update the Dockerfile build step.** The `Dockerfile` at `docker_ws/docker-deployment/Dockerfile:127` runs `colcon build` which builds all packages. Since both packages are in the workspace, this should work automatically. Verify by checking that `dev/grasp_preshaping/` is under the colcon workspace root (it is — it's at `/miahand_ws/src/dev/grasp_preshaping`).

- [ ] **4.3. Update the `mujoco_interactive` docker-compose command.** Currently at `docker-compose.yml:102-104`:
  ```yaml
  colcon build --packages-select mia_hand_mujoco &&
  ```
  This needs to also build `grasp_preshaping`:
  ```yaml
  colcon build --packages-select grasp_preshaping mia_hand_mujoco &&
  ```
  Similarly update the `mujoco_dynamic` service at line 87.

### Phase 5: Update the preshaping trigger clients (optional, same package)

The preshaping trigger client code in `interactive_system_interface.cpp:715-762` and `planner_gui_system_interface.cpp:483-537` calls the `/grasp_preshaping/compute_grasp` service via a ROS service client. These files remain in `mia_hand_mujoco` because they are part of the hardware interface plugins. They do NOT need to move — they are consumers of the service, not part of the service implementation. No changes needed.

- [ ] **5.1. No changes needed for trigger clients.** The service name `/grasp_preshaping/compute_grasp` is unchanged. The trigger clients in `interactive_system_interface.cpp` and `planner_gui_system_interface.cpp` are pure ROS service callers and have no code-level dependency on the bridge node's implementation.

### Phase 6: Clean up and verify

- [ ] **6.1. Update `dev/grasp_preshaping/README.md`** to reflect that the package now contains both the Rust library and the C++ ROS service bridge node.

- [ ] **6.2. Verify the Rust library search paths.** The bridge node searches for `libgrasp_preshaping.so` at these paths (`preshaping_service_bridge_node.cpp:155-159`):
  - `$GRASP_PRESHAPING_LIB_PATH` (env var)
  - `/miahand_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so`
  - `/miahand_ws/install/lib/libgrasp_preshaping.so`
  - `/miahand_ws/src/dev/grasp_preshaping/target/release/libgrasp_preshaping.so`
  - `/miahand_ws/src/dev/grasp_preshaping/target/debug/libgrasp_preshaping.so`
  
  The colcon build installs to `/miahand_ws/install/grasp_preshaping/lib/`, so the second path should work. Verify after rebuild.

- [ ] **6.3. Test the full build.** Run:
  ```bash
  colcon build --packages-select grasp_preshaping mia_hand_mujoco
  ```
  Verify both packages build without errors.

- [ ] **6.4. Test the interactive container.** Run:
  ```bash
  docker compose run --build --rm mujoco_interactive
  ```
  Verify that the `preshaping_service_bridge` node starts and the "Run Planner" button works.

---

## Verification Criteria

- [ ] `colcon build --packages-select grasp_preshaping` builds both the Rust `.so` and the C++ bridge node
- [ ] `ros2 run grasp_preshaping preshaping_service_bridge_node` starts the node successfully
- [ ] The node loads the Rust backend from the installed path
- [ ] The interactive simulation launch starts all nodes including the bridge
- [ ] The "Run Planner" button triggers the service and fingers move
- [ ] No preshaping-related source code remains in `mia_hand_mujoco/src/`
- [ ] The FFI struct definitions exist in exactly one place (`include/grasp_preshaping/ffi_types.hpp`)

---

## Potential Risks and Mitigations

1. **Build order dependency**
   - Risk: The C++ bridge node doesn't need the Rust `.so` at compile time (it uses `dlopen` at runtime), but the CMakeLists dependency ensures the Rust library is built first. If colcon parallelizes incorrectly, the install step might race.
   - Mitigation: The `add_dependencies(preshaping_service_bridge_node grasp_preshaping_rustlib)` in CMakeLists enforces ordering.

2. **Docker compose `--packages-select` must include both packages**
   - Risk: The current `docker-compose.yml` only selects `mia_hand_mujoco`. If `grasp_preshaping` is not built, the bridge node won't be installed.
   - Mitigation: Update the compose commands to include both packages (Phase 4.3).

3. **FFI struct layout mismatch**
   - Risk: If the C++ header and Rust `#[repr(C)]` structs diverge, the FFI call will corrupt memory.
   - Mitigation: The shared header is now the single source of truth. Add a static_assert or comment in the header pointing to the Rust file for cross-reference. Consider adding a version check using the existing `grasp_preshaping_api_version()` function.

4. **Launch file package reference**
   - Risk: The launch file must reference `package='grasp_preshaping'` for the bridge node. If this is wrong, the node won't be found at launch time.
   - Mitigation: Test the launch explicitly after the change.

5. **Existing `mujoco_dynamic` container also uses the bridge node**
   - Risk: The `mujoco_dynamic` service in `docker-compose.yml` also launches the same launch file, so it also needs the build order update.
   - Mitigation: Update both `mujoco_interactive` and `mujoco_dynamic` compose commands.

---

## Alternative Approaches

1. **Keep the bridge node in `mia_hand_mujoco` but add a symlink or include path.** This avoids the move entirely but doesn't solve the fragmentation problem. The FFI struct duplication remains.

2. **Make `grasp_preshaping` a pure Rust package with a ROS node via `rclrs`.** This would eliminate the C++ bridge entirely, but requires the `rclrs` Rust bindings for ROS 2, which may not be available in the current Docker image. Significant effort for marginal benefit.

3. **Merge `grasp_preshaping` into `mia_hand_mujoco` entirely.** This consolidates everything into one package but makes the preshaping library inseparable from the MuJoCo simulation package. Not recommended — the preshaping library should be usable independently.

4. **Create a third package `grasp_preshaping_bridge`** that contains only the C++ node and depends on `grasp_preshaping`. This adds another package but keeps concerns separated. Overkill for a single node — the recommended approach of putting it in `grasp_preshaping` is simpler.
