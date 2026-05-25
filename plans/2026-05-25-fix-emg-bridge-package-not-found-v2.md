# Fix: "Package 'emg_bridge' not found" — Root Cause: User Mismatch + Missing Resource Marker

## Objective

Fix the recurring issue where `make emg-infer` (and similar host-side targets) fail with `Package 'emg_bridge' not found`. The root cause is a **user mismatch between `make shell` and `make emg-infer`** that poisons the named volumes with root-owned build artifacts, combined with a missing `resource/` marker file and a missing workspace source in the `run-classifier` target.

## Root Cause Analysis

### Problem 1: `make shell` runs as root, `make emg-infer` runs as prosthesis (PRIMARY)

The critical mismatch:

| Target | Line | Podman exec | Effective user |
|--------|------|-------------|----------------|
| `shell` | 172-173 | `exec prosthesis /bin/bash` (no `--user`) | **root** (inherits outer PID user) |
| `emg-infer` | 164-165 | `exec --user prosthesis prosthesis` | **prosthesis** |
| `train` | 161-162 | `exec --user prosthesis prosthesis` | **prosthesis** |
| `collect-data` | 158-159 | `exec --user prosthesis prosthesis` | **prosthesis** |

Your shell output confirms this: `root@KingBob:/prosthesis_ws#`

**The cycle of pain:**
1. `make shell` → root shell
2. `make build` (or auto-build from .bashrc) → build artifacts owned by **root** in named volumes
3. Exit shell
4. `make emg-infer` → runs as **prosthesis** → can't read root-owned install artifacts → `Package not found`

The `entrypoint.sh` chown (line 16-21) only runs at container **creation**, not on `exec`. After you build as root, new files are root-owned and stay that way.

### Problem 2: Missing `resource/emg_bridge` marker file

`src/emg_bridge/setup.py:10` references `resource/emg_bridge` but the file doesn't exist. This is mandatory for `ament_python` packages — without it, the package can't register with the ROS 2 package index.

### Problem 3: `run-classifier` never sources the workspace install

`Makefile.workspace:129-131` only does `$(ROS_SETUP)` (which is `/opt/ros/jazzy/setup.bash`). It never sources `/prosthesis_ws/install/setup.bash`. So even with a correct build, `ros2 run` can't find workspace packages.

## Implementation Plan

- [ ] **Task 1. Fix `make shell` to run as prosthesis user**
  - File: `Makefile:172-173`
  - Change: `cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash`
  - To: `cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash`
  - Rationale: All other exec targets (`emg-infer`, `train`, `collect-data`) already use `--user prosthesis`. The `shell` target is the outlier. Running as root inside the container poisons the named volumes with root-owned files that `prosthesis` can't read.

- [ ] **Task 2. Create the missing `resource/emg_bridge` marker file**
  - Create `src/emg_bridge/resource/emg_bridge` as an empty file
  - Rationale: Required by `setup.py:10` for ament package index registration. Without it, `ros2 pkg list` cannot discover `emg_bridge`.

- [ ] **Task 3. Fix `run-classifier` to source the workspace and build first**
  - File: `Makefile.workspace:129-131`
  - Change the `run-classifier` target to:
    1. Source both ROS and workspace setup
    2. Build `emg_bridge` before running (mirrors the `run` and `camera-test` target pattern)
  - Also fix `collect-data` (line 119-123) and `train` (line 125-127) which have the same missing workspace source
  - Rationale: Without `$(ROS_WORKSPACE_SETUP)`, `ros2 run` only searches `/opt/ros/jazzy` — it never looks in `/prosthesis_ws/install`.

- [ ] **Task 4. Clean the stale named volumes**
  - Run: `make clean-volumes` (or: `podman volume rm docker_prosthesis-build docker_prosthesis-install docker_prosthesis-log`)
  - Then: `make dev` to recreate the container (the .bashrc auto-build will rebuild everything as prosthesis)
  - Rationale: The existing named volumes contain root-owned artifacts from previous `make shell` sessions. They must be purged.

- [ ] **Task 5. Verify end-to-end**
  - From host: `make emg-infer` — should find `emg_bridge` (may fail on model/hardware, but NOT on "Package not found")
  - From host: `make shell` — prompt should show `prosthesis@KingBob` not `root@KingBob`
  - Inside container: `ros2 pkg list | grep emg_bridge` should show the package
  - Inside container: `make run-classifier` should work

## Verification Criteria

- [ ] `make shell` shows `prosthesis@KingBob` prompt (not root)
- [ ] `ros2 pkg list` inside container includes `emg_bridge`
- [ ] `make emg-infer` from host does NOT say "Package 'emg_bridge' not found"
- [ ] `make run-classifier` inside container does NOT say "Package 'emg_bridge' not found"
- [ ] Named volume contents are owned by `prosthesis:prosthesis` after a build

## Potential Risks and Mitigations

1. **Some operations genuinely need root in the shell** (e.g., `apt install`, device permissions)
   Mitigation: Users can `sudo` inside the container (prosthesis has NOPASSWD sudo per Dockerfile:55). For the rare case root is needed, `sudo bash` or `sudo make build` works.

2. **Named volumes still stale after code fix**
   Mitigation: Task 4 explicitly cleans volumes. This is mandatory, not optional.

3. **Adding build to `run-classifier` slows repeated invocations**
   Mitigation: `colcon build --packages-select emg_bridge` is near-instant for an already-built Python package. The reliability gain outweighs the marginal cost.

## Alternative Approaches

1. **Keep shell as root but chown after build**: Add a post-build chown step. Trade-off: fragile, easy to forget, doesn't fix the root cause.

2. **Use bind mounts instead of named volumes for build/install/log**: Makes artifacts visible on host and easier to clean. Trade-off: UID/GID permission issues on host, pollutes the checkout directory.
