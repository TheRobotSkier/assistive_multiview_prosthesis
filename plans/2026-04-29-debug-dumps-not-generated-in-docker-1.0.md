# Debug Dumps Not Being Generated from Preshaping in Docker (mujoco_interactive)

## Objective

Diagnose and fix why debug dump `.npz` files are not being written when running the grasp preshaping pipeline through the `mujoco_interactive` Docker service, despite `DEBUG_VISUALIZATION` being set to `true` in `config.rs`.

---

## Root Cause Analysis

After tracing the full code path, **the most likely root cause is the `CARGO_MANIFEST_DIR` path resolution inside Docker when the Rust shared library is loaded dynamically via `dlopen`**.

### How the Debug Dump Path is Constructed

In `docker_ws/dev/grasp_preshaping/src/debug_export.rs:244-246`:

```rust
pub fn debug_output_path() -> std::path::PathBuf {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let base = std::path::Path::new(manifest_dir).join(crate::config::DEBUG_OUTPUT_DIR);
```

`env!("CARGO_MANIFEST_DIR")` is a **compile-time macro** that embeds the literal string of the working directory where `cargo build` was invoked. `DEBUG_OUTPUT_DIR` is `"data/debug"` (from `config.rs:50`).

### The Problem: CARGO_MANIFEST_DIR Bakes In the Build-Time Path

When `colcon build` invokes Cargo inside the Docker container, the build happens in:

```
/miahand_ws/build/grasp_preshaping/cargo-target/release/
```

But `CARGO_MANIFEST_DIR` resolves to the **source directory**:

```
/miahand_ws/src/dev/grasp_preshaping
```

So the debug output path becomes:

```
/miahand_ws/src/dev/grasp_preshaping/data/debug/grasp_dump_YYYYMMDD_HHMMSS.npz
```

### Why the Dumps Don't Appear on the Host

Looking at the Docker compose file (`docker-compose.linux-podman.yml:16`):

```yaml
volumes:
  - ../:/miahand_ws/src
```

The host directory `docker_ws/` is bind-mounted to `/miahand_ws/src` in the container. This means:

- The debug dumps **should** be written to `/miahand_ws/src/dev/grasp_preshaping/data/debug/`
- Which maps to **host path**: `docker_ws/dev/grasp_preshaping/data/debug/`

### Possible Sub-Issues

There are **three potential failure points**:

1. **Directory creation failure** — `export_npz()` calls `fs::create_dir_all(parent)` but the error may be silently swallowed. The Rust library runs inside a `std::panic::catch_unwind` wrapper in `c_api.rs:514`, and debug export errors only print to stderr via `eprintln!` (line 417-418). If the container user lacks write permissions to the bind-mounted source directory, the write silently fails.

2. **The preshaping pipeline may be erroring out before reaching the debug export** — If `compute_from_request()` returns early with an error (e.g., "No points in ROI"), the debug export block at `c_api.rs:360-420` is never reached because the debug export happens **before** the early return on error at `c_api.rs:333-335`, but the entire function is wrapped in `catch_unwind` which converts panics to error messages.

3. **The preshaping service bridge node may not be receiving data** — The bridge node subscribes to `/hand_pose`, `/hand_twist`, and `/segmented_object_cloud`. If any of these topics are not publishing, the service call will fail with "No X data received yet" before the Rust library is ever invoked.

### Priority Ranking of Root Causes

| Priority | Issue | Reasoning |
|----------|-------|-----------|
| **1 (Highest)** | Filesystem permissions on bind mount | Docker runs as user 1000:1000. The bind-mounted `../` directory from the host may not be writable by that user inside the container. `create_dir_all` would fail, and the error is only logged to stderr. |
| **2** | Debug output going to unexpected path | The `CARGO_MANIFEST_DIR` path is baked in at compile time. If the build environment differs from what's expected, the path may point to a read-only or non-existent location. |
| **3** | Pipeline errors before debug export | If the point cloud is empty or ROI pruning removes all points, `compute_from_request` returns early at `c_api.rs:333` before the debug export block at `c_api.rs:360`. |
| **4** | Service bridge not receiving input data | If the preshaping service isn't getting pose/twist/cloud data, the Rust compute function is never called. |

---

## Verification Steps

Before making changes, verify the actual state:

- [ ] **Step 1**: Check if the `data/debug/` directory exists on the host at `docker_ws/dev/grasp_preshaping/data/debug/`. If it doesn't exist, the dumps were never written.
- [ ] **Step 2**: Check Docker container logs for `[debug_viz]` messages. The Rust code prints `[debug_viz] wrote ...` on success or `[debug_viz] FAILED to write ...` on failure at `c_api.rs:417-418`.
- [ ] **Step 3**: Check if the preshaping service is even being called by looking for `"Planner called with N camera(s)"` log messages from the bridge node (`preshaping_service_bridge_node.cpp:309-316`).
- [ ] **Step 4**: Verify the `CARGO_MANIFEST_DIR` baked into the compiled `.so` by running `strings /miahand_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so | grep grasp_preshaping/data` inside the container.

---

## Implementation Plan

- [ ] **Task 1**: Add a log message at the start of the debug export block in `c_api.rs:360` to confirm the code path is reached. Currently, the only logging is on success/failure of the file write. Add an `eprintln!("[debug_viz] DEBUG_VISUALIZATION is enabled, attempting export...")` before the export logic.
- [ ] **Task 2**: Log the resolved output path before attempting to write. In `c_api.rs:415-416`, the `debug_output_path()` is called but its value is only logged on success. Move the path logging before the write attempt so it appears even if the write fails.
- [ ] **Task 3**: Ensure the `data/debug/` directory exists on the host with correct permissions. Create `docker_ws/dev/grasp_preshaping/data/debug/` and ensure it's writable by the container user (UID 1000).
- [ ] **Task 4**: Consider changing the debug output path to use a configurable environment variable or ROS parameter instead of `CARGO_MANIFEST_DIR`. This would make the output path predictable and independent of build-time paths. Add a `GRASP_DEBUG_OUTPUT_DIR` environment variable check in `debug_output_path()` with a fallback to the current behavior.
- [ ] **Task 5**: Verify that the `mujoco_interactive` compose service actually triggers the preshaping pipeline. The interactive simulator calls `trigger_preshaping_service()` (`interactive_system_interface.cpp:795-843`) when the GUI button is pressed. Confirm that the service call reaches the bridge node and the Rust library.

---

## Verification Criteria

- [ ] Running `mujoco_interactive` and triggering a grasp plan produces a `[debug_viz] wrote ...` log message in the container output.
- [ ] A `.npz` file appears at `docker_ws/dev/grasp_preshaping/data/debug/` on the host (or at the configured output path).
- [ ] The `.npz` file can be loaded and visualized with `scripts/visualize_grasp_debug.py`.
- [ ] Error cases (no points in ROI, permission failures) produce clear log messages explaining why no dump was written.

---

## Potential Risks and Mitigations

1. **Bind mount permission issues**
   Mitigation: The Dockerfile creates user with UID 1000. Ensure the host directory `docker_ws/dev/grasp_preshaping/data/debug/` has write permissions for UID 1000. Run `chmod -R a+w docker_ws/dev/grasp_preshaping/data/` on the host.

2. **CARGO_MANIFEST_DIR path mismatch between build and runtime**
   Mitigation: The path is baked in at compile time but should be consistent since `colcon build` runs inside the same container. However, if the library is cached from a previous build with a different path, it could be wrong. Force a rebuild with `docker compose run --build --rm mujoco_interactive`.

3. **Silent failures in the Rust library**
   Mitigation: The current code only logs via `eprintln!` which goes to stderr. These messages should appear in the Docker container logs but may be mixed with other output. Consider adding ROS logging through the FFI layer for better visibility.

---

## Alternative Approaches

1. **Use a fixed output path instead of CARGO_MANIFEST_DIR**: Change `debug_output_path()` to use `/tmp/grasp_preshaping_debug/` or a path derived from an environment variable. This eliminates the build-time path dependency entirely. The trade-off is that dumps would not automatically appear in the source tree on the host.

2. **Add a ROS 2 topic for debug dump notifications**: Instead of writing to a file, publish the debug data as a ROS 2 topic. This avoids filesystem issues entirely but requires significant refactoring of the debug pipeline and a new subscriber to save the data.

3. **Add a Docker volume mount specifically for debug output**: Add a dedicated volume mount in the compose file for the debug output directory. This ensures the directory is always writable and accessible from the host, regardless of source mount permissions.
