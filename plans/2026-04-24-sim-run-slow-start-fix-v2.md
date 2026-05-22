# Diagnosis: sim-run Slow First Start (Not Hanging)

## Objective

Identify and resolve the cause of slow first-run startup for `sim-run`, where the first invocation after a rebuild is very slow but subsequent runs are fast.

---

## Root Cause Analysis

The slow startup has **three contributing factors**, all clearly visible in the compose file and build configuration:

### Factor 1: `colcon build` runs at EVERY container start (PRIMARY CAUSE)

The `mujoco_interactive` service command at `docker-compose.yml:93-101` does:

```bash
colcon build --packages-select grasp_preshaping mia_hand_mujoco &&
```

This rebuilds two packages from scratch **every time** the container starts. This is the biggest contributor to startup latency. Critically:

- **`mia_hand_mujoco`** compiles 8 C++ source files including the entire vendored MuJoCo `simulate.cc` (a large file), plus GLFW adapter code, the interactive simulator, and links against MuJoCo, GLFW, and the custom actuator plugin. See `mia_hand_mujoco/CMakeLists.txt:66-86` for the full source list.
- **`grasp_preshaping`** invokes `cargo build --release` (Rust compilation) via CMake at `dev/grasp_preshaping/CMakeLists.txt:23-24`. Rust release builds are notoriously slow on first compilation.

### Factor 2: The build cache is destroyed between runs

The volume mount at `docker-compose.yml:14` maps `../:/miahand_ws/src` — this only mounts the **source** directory. The `colcon` build output (`build/`, `install/`, `log/` directories) lives inside the container filesystem at `/miahand_ws/`.

On the **first run** after a `sim-build` (image rebuild):
- The container's `/miahand_ws/` contains the image-baked build output from `Dockerfile:130` (`colcon build`)
- The runtime `colcon build --packages-select` must check timestamps and potentially rebuild

On **subsequent `sim-run` calls**:
- If the container wasn't removed, the build cache from the previous run persists
- `colcon` detects that nothing changed and skips compilation (incremental/no-op)

But after a **`sim-build`** (image rebuild), the container is recreated from scratch, and the build cache from the image layer is overwritten by the volume mount of `../:/miahand_ws/src`. The `build/`, `install/`, `log/` directories from the Dockerfile's `colcon build` at `Dockerfile:130` are **hidden** by the bind mount, so the runtime `colcon build` must rebuild everything.

### Factor 3: Rust compilation cold start

The `grasp_preshaping` package uses Cargo (Rust) via CMake (`dev/grasp_preshaping/CMakeLists.txt:23-24`). The first `cargo build --release` after a clean state has to:
1. Compile all Rust dependencies from source
2. Run LLVM optimization passes for release mode
3. This alone can take 30-60+ seconds

On subsequent runs, Cargo's target directory (`CARGO_TARGET_DIR` at `dev/grasp_preshaping/CMakeLists.txt:13`) persists inside the container, so incremental builds are fast or no-ops.

---

## Implementation Plan

- [ ] **Step 1. Move the `colcon build` into the Docker image (eliminate runtime builds).** The `mujoco_interactive` command currently runs `colcon build --packages-select grasp_preshaping mia_hand_mujoco` at every container start. Since these packages are already built during `sim-build` (at `Dockerfile:130`), the runtime rebuild is redundant. Remove the `colcon build` line from the `mujoco_interactive` command in `docker-compose.yml` and change it to just:
  ```
  command: >
    bash -c "
      source /opt/ros/jazzy/setup.bash &&
      source /miahand_ws/install/setup.bash &&
      ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py ...
    "
  ```
  This eliminates the build entirely at runtime. After a `sim-build`, the packages are already compiled in the image.

- [ ] **Step 2. If runtime builds are needed for development iteration, persist the build cache.** If the intent is to allow code changes on the host (via the bind mount) to be picked up without a full `sim-build`, then persist the build output directories by adding volume mounts for the cache:
  ```yaml
  volumes:
    - ../:/miahand_ws/src
    - mujoco_build_cache:/miahand_ws/build
    - mujoco_install_cache:/miahand_ws/install
    - mujoco_log_cache:/miahand_ws/log
    - mujoco_cargo_cache:/miahand_ws/build/grasp_preshaping/cargo-target
  ```
  This way, after the first slow build, subsequent runs reuse the cached artifacts even after container recreation.

- [ ] **Step 3. Apply the same fix to `mujoco_dynamic` service.** The `mujoco_dynamic` service at `docker-compose.yml:74-87` has the same `colcon build` at runtime pattern. Apply the same treatment.

- [ ] **Step 4. Verify the fix.** After changes, run `sim-build && sim-run` and confirm the MuJoCo window opens within a few seconds instead of minutes.

---

## Verification Criteria

- [ ] First `sim-run` after `sim-build` starts the MuJoCo window in under 10 seconds
- [ ] No `colcon build` output appears during `sim-run` (build happens only during `sim-build`)
- [ ] Subsequent `sim-run` calls are equally fast (no regression)

## Potential Risks and Mitigations

1. **Source changes on host not picked up without rebuild**
   Mitigation: This is by design — use `sim-build` to rebuild the image when code changes. The runtime `colcon build` was a convenience that came at a high startup cost. If rapid iteration is needed, use the named volume approach in Step 2.

2. **Rust Cargo cache invalidation**
   Mitigation: If using named volumes for the build cache, the Cargo target directory persists across container recreations, avoiding cold Rust compilation.

3. **Breaking the development workflow**
   Mitigation: The `mujoco_interactive` and `mujoco_dynamic` services are the only ones that do runtime builds. All other services already use `source install/setup.bash` directly, which is the correct pattern for pre-built images.

## Recommended Approach

**Step 1 is the cleanest fix** — remove the runtime `colcon build` from the compose command and rely on the image build (`sim-build`) to compile everything. This is how all the other services (miahand_mujoco, miahand_moveit, etc.) already work. The runtime build was likely added as a development convenience but causes the slow first-start issue.
