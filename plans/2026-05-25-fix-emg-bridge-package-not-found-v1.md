# Fix: "Package 'emg_bridge' not found" — Persistent Build/Install Volume Desync

## Objective

Fix the recurring issue where `make emg-infer` (and similar host-side targets) fail with `Package 'emg_bridge' not found` even after building inside the container. The root cause is a **volume desync between the named `prosthesis-install` volume and the bind-mounted `src/` directory**, combined with a missing `resource/` marker file in the `emg_bridge` package.

## Root Cause Analysis

### Problem 1: Missing `resource/emg_bridge` marker file (PRIMARY)
The `emg_bridge` package is an `ament_python` package. Its `setup.py:10` references:
```
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
```
But **there is no `resource/emg_bridge` file** in `src/emg_bridge/`. This means:
- `colcon build` may partially fail or produce an incomplete install for `emg_bridge`
- `ros2 pkg list` cannot discover the package because the ament index marker is never installed
- `ros2 run emg_bridge run_classifier` fails with "Package not found"

### Problem 2: Named volumes persist stale builds
The `docker-compose.yml:64-66` uses **named volumes** for build artifacts:
```yaml
- prosthesis-build:/prosthesis_ws/build
- prosthesis-install:/prosthesis_ws/install
- prosthesis-log:/prosthesis_ws/log
```
These persist across container recreations. If a previous build partially failed (due to Problem 1), the stale/incomplete install state persists. Running `make build` inside the shell may succeed for other packages but silently skip or fail for `emg_bridge`, leaving the named volume in a broken state.

### Problem 3: `dev` target recreates the container
`Makefile:164-165` — the `emg-infer` target depends on `dev`, which runs `podman-compose up -d prosthesis`. Podman-compose detects config-hash changes and **recreates the container** (visible in the logs: "recreating: done"). This recreation:
- Stops and removes the old container
- Creates a new container with the same named volumes
- Does NOT rebuild — the stale install state from the named volume carries over

### Problem 4: `run-classifier` does not build first
`Makefile.workspace:129-131` — the `run-classifier` target only does:
```
@$(ROS_SETUP) && ros2 run emg_bridge run_classifier --model-dir /app/models
```
It does NOT call `$(ROS_WORKSPACE_SETUP)` and does NOT trigger a build. Compare with `run` (line 98-103) which does `colcon build` first. So even if you fix the package, the target never ensures the package is built.

### Why it works after a manual rebuild in shell but breaks from host
When you `make shell` and manually run `make build`, the `.bashrc` auto-build (Dockerfile:89-93) or your manual build populates the named volume. But `make emg-infer` from the host calls `dev` first, which recreates the container. If the config-hash changed (e.g., you edited docker-compose.yml or any mounted file), the container is recreated fresh — but the named volumes persist the OLD build state. If that old state was incomplete (missing emg_bridge), it stays broken.

## Implementation Plan

- [ ] **Task 1. Create the missing `resource/emg_bridge` marker file**
  - Create `src/emg_bridge/resource/emg_bridge` as an empty file (standard ament_python package marker)
  - This is required by `setup.py:10` and without it, `ament_python` packages cannot register with the ROS 2 package index
  - Rationale: Every `ament_python` package must have a `resource/<package_name>` file for `ros2 pkg` discovery

- [ ] **Task 2. Add `$(ROS_WORKSPACE_SETUP)` to the `run-classifier` target in `Makefile.workspace`**
  - The `run-classifier` target at line 129-131 currently only sources `/opt/ros/jazzy/setup.bash` but NOT `/prosthesis_ws/install/setup.bash`
  - Without sourcing the workspace install, `ros2 run` cannot find packages built in the workspace
  - Change from: `@$(ROS_SETUP) && ros2 run emg_bridge run_classifier ...`
  - Change to: `@$(ROS_SETUP) && $(ROS_WORKSPACE_SETUP) && ros2 run emg_bridge run_classifier ...`
  - Also apply the same fix to `collect-data` (line 119-123) and `train` (line 125-127) targets which have the same issue

- [ ] **Task 3. Add a build step to `run-classifier` (and related EMG targets)**
  - Add `colcon build --packages-select emg_bridge` before the `ros2 run` call in `run-classifier`
  - This mirrors the pattern used by `run` (line 98-103) and `camera-test` (line 88-93) which build before launching
  - Alternatively, create a helper pattern like `build-emg` that the three EMG targets depend on
  - Rationale: Ensures the package is always built and available, eliminating the "works after manual build but not from host" problem

- [ ] **Task 4. Add `build-emg` convenience target to `Makefile.workspace`**
  - Add a new target: `build-emg: ## Build only the emg_bridge package`
  - Implementation: `@$(ROS_SETUP) && cd /prosthesis_ws && colcon build --packages-select emg_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release`
  - Make `run-classifier`, `collect-data`, and `train` depend on this (or inline the build)
  - Rationale: Provides a quick rebuild target for the EMG package specifically

- [ ] **Task 5. Clean the stale named volumes**
  - After the code fixes, the named volumes must be cleaned to remove the broken build state
  - Run: `make clean-volumes` (or manually: `podman volume rm docker_prosthesis-build docker_prosthesis-install docker_prosthesis-log`)
  - Then rebuild: `make dev` followed by `make shell` → `make build` (or let .bashrc auto-build handle it)
  - Rationale: The existing named volumes contain a broken install that will persist until explicitly removed

- [ ] **Task 6. Verify the fix end-to-end**
  - After cleaning volumes and rebuilding, test from the HOST:
    - `make emg-infer` should now find the `emg_bridge` package
  - Test from inside the container:
    - `make shell` → `make run-classifier` should work
  - Verify `ros2 pkg list | grep emg_bridge` shows the package
  - Rationale: Confirms both the resource marker fix and the workspace setup fix work correctly

## Verification Criteria

- [ ] `ros2 pkg list` inside the container includes `emg_bridge`
- [ ] `make emg-infer` from the host succeeds (finds the package, even if classifier fails due to no model/hardware)
- [ ] `make run-classifier` inside the container succeeds (same criteria)
- [ ] `make shell` → `make build` → exit → `make emg-infer` works without re-building
- [ ] The `resource/emg_bridge` file exists in the source tree

## Potential Risks and Mitigations

1. **Named volumes still stale after code fix**
   Mitigation: Must run `make clean-volumes` after applying the code changes. The plan explicitly includes this step.

2. **`emg_bridge` build fails for other reasons (missing deps)**
   Mitigation: The `setup.py` and `package.xml` look correct. The only missing piece is the resource marker. If build still fails, check `colcon` output for specific errors.

3. **Container recreation clears runtime state**
   Mitigation: Named volumes persist across recreations. The fix ensures the build state in those volumes is correct after a clean rebuild.

4. **Adding build to `run-classifier` slows down repeated invocations**
   Mitigation: `colcon build --packages-select emg_bridge` is fast for an already-built Python package (near-instant). The trade-off for reliability is worth it.

## Alternative Approaches

1. **Replace named volumes with bind mounts for build/install/log**: Mount `./build`, `./install`, `./log` from the host instead of using named volumes. This makes build artifacts visible on the host and easier to inspect/clean. Trade-off: potential UID/GID permission issues on the host, and build artifacts pollute the host checkout.

2. **Add a `make rebuild-emg` host target**: Instead of fixing the in-container Makefile, add a host-side target that does `colcon build --packages-select emg_bridge` before `run-classifier`. Trade-off: duplicates logic, doesn't fix the root cause (missing resource marker).

3. **Use `--symlink-install` for all builds**: This avoids copying files and instead symlinks, making source changes immediately available. Trade-off: doesn't fix the missing resource marker issue, and symlinks can break with container recreation.
