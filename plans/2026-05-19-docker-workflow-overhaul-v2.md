# Docker/Podman Development Workflow Overhaul — v2

## Objective

Eliminate the "rebuild the image for every code change" bottleneck, remove unused/outdated services, merge the hardware service into the main prosthesis service, and introduce an in-container Makefile for easy workspace operations — resulting in a fast inner-loop development experience.

---

## Current State Analysis

### Problem 1: Full image rebuild for every code change

**Root cause:** `docker/Dockerfile:48-62` does `COPY src/ src/` followed by `colcon build` inside the image build step. Every source change triggers `podman build` → `colcon build from scratch`. The container user (`prosthesis`, line 71) can't run `colcon build` inside a running container because `build/`, `install/`, `log/` are owned by root.

### Problem 2: `grasp_test` service is legacy

`docker/docker-compose.yml:92-116` uses `rmw_fastrtps_cpp` (different DDS from everything else) and launches a single-camera pipeline. The `digital_twin.launch.py` supersedes it with dual-camera fusion, ChArUco tracking, and the digital twin hand model.

### Problem 3: `digital_twin` service is redundant

`docker/docker-compose.yml:118-142` is just the prosthesis image running a different launch file. With volume mounts, you run any launch file inside the main container.

### Problem 4: `prosthesis-hw` can merge into `prosthesis`

Since cameras moved to the Jetson, the hardware service only maps 2 USB serial ports. `docker-compose.hw.yml` already targets the `prosthesis` service by name — the separate `prosthesis-hw` definition is redundant.

### Problem 5: `interactive` service is dead code

`docker/docker-compose.yml:44-49` — no Makefile target, no profile restriction, starts with `make up` doing nothing useful.

### Problem 6: Hard-to-remember launch commands

Currently the only way to launch things is memorizing long `ros2 launch prosthesis_launch <file> <args>` commands. There's no quick-reference inside the container.

---

## Proposed Architecture

### Two-Makefile Pattern

```
Host Makefile          →  Container lifecycle (build, up, down, shell)
Container Makefile     →  Workspace operations (build, launch, test, clean)
```

**Separation of concerns:** The host Makefile manages Podman/Docker. The in-container Makefile manages the ROS workspace. A developer's workflow becomes:

```bash
# Host terminal
make dev          # start container (instant, no rebuild)
make shell        # shell into container

# Inside the container
make              # shows all available targets
make build        # colcon build (all packages)
make build-pkg PKG=pipeline_manager   # incremental build
make mock         # launch mock pipeline
make digital-twin # launch digital twin
make test         # run smoke tests
make clean        # wipe build artifacts
```

### Docker Compose Structure (after cleanup)

```
services:
  prosthesis:          # Single development container
    - Source mounted as bind volumes (src/, config/, rviz/, scripts/)
    - Build artifacts in named volumes (build/, install/, log/)
    - In-container Makefile at /prosthesis_ws/Makefile
    - Optional USB device mapping via docker-compose.hw.yml overlay

  segmentation:        # Unchanged — isolated Python 3.8 inference server
    - Add named volume for /weights

  test:                # CI/smoke test runner (builds from image, no volumes)
```

### Removed Services
- `interactive` — unused, no Makefile target
- `grasp_test` — legacy, superseded by digital_twin
- `digital_twin` — just a different launch command, run inside main container
- `prosthesis-hw` — merged into main service via hw overlay file

---

## Implementation Plan

### Phase 1: In-Container Makefile

- [x] **1.1. Create `/prosthesis_ws/Makefile` (the in-container Makefile)**
  This file lives at the workspace root and is mounted into the container via the bind volume. It provides memorable short targets for all common operations. The key design principle: every target auto-sources the ROS setup files so the user doesn't have to.

  Proposed targets (to be placed in the project root as `Makefile.workspace` or directly in the workspace root):

  ```
  make              — print help (list all targets with descriptions)
  make build        — colcon build (all packages, Release)
  make build-pkg PKG=<name>  — colcon build --packages-select <name>
  make build-debug  — colcon build with -DCMAKE_BUILD_TYPE=Debug
  make clean        — rm -rf build/ install/ log/
  make test         — run smoke tests (scripts/run_tests.sh)
  make mock         — ros2 launch prosthesis_launch mock.launch.py
  make pipeline     — ros2 launch prosthesis_launch pipeline.launch.py
  make digital-twin — ros2 launch prosthesis_launch digital_twin.launch.py camera:=true gui:=true
  make digital-twin-mock — ros2 launch prosthesis_launch digital_twin.launch.py camera:=false gui:=true
  make grasp-test   — ros2 launch prosthesis_launch grasp_test.launch.py
  make twist-test   — ros2 launch prosthesis_launch twist_propagation_test.launch.py
  make source       — source setup files (for manual use)
  ```

  Rationale: Gives developers a `make` help menu inside the container. No more memorizing launch file names and arguments. The file is on the host (in the project root) so it's version-controlled and editable without rebuilding.

- [x] **1.2. Ensure the Makefile handles ROS setup correctly**
  Each target that needs ROS should prefix with `source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash &&`. Use a Make variable for this:
  ```makefile
  ROS_SETUP := bash -c 'source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash && eval "$$0"'
  ```
  Alternatively, the `.bashrc` already sources these (via `docker/Dockerfile:74-75`), so interactive shells work. For `make` targets that run commands directly, use the explicit source approach.

  Rationale: `make` targets run in non-interactive subshells that don't source `.bashrc`. Explicit sourcing ensures reliability.

- [x] **1.3. Add a `help` target as the default**
  The default `make` target (`.DEFAULT_GOAL := help`) prints a formatted list of all targets with one-line descriptions. This is the first thing a developer sees when they type `make` inside the container.

  Rationale: Discoverability. Anyone shelling into the container immediately knows what's available.

### Phase 2: Volume-Mount Development Container

- [x] **2.1. Refactor `Dockerfile` into base stage + production stage**
  - **Base stage** (`docker/Dockerfile`): Installs system deps, Python deps, creates user, runs `rosdep install` — does NOT copy source or run `colcon build`. This is the development image.
  - **Production stage** (optional, for deployment/CI): Copies source + builds (current behavior). Used by the `test` service only.
  
  Use Docker multi-stage build:
  - `base` — deps + user setup (the dev image)
  - `production` — `FROM base`, copies source, runs colcon build (for test service)

  Rationale: Base image changes only when system/ROS dependencies change (rare). Source code changes (frequent) never trigger an image rebuild.

- [x] **2.2. Add named volumes for colcon build artifacts in `docker-compose.yml`**
  Add named volumes for `/prosthesis_ws/build`, `/prosthesis_ws/install`, and `/prosthesis_ws/log`. These persist across container restarts and avoid permission issues (named volumes are initialized with the container's filesystem permissions on first create).

  ```yaml
  volumes:
    prosthesis-build:
    prosthesis-install:
    prosthesis-log:
  ```

  Rationale: Solves the "container user can't write to build/" problem. Build artifacts persist across `make down` / `make dev` cycles.

- [x] **2.3. Mount source directories as bind volumes in `docker-compose.yml`**
  Mount `src/`, `config/`, `rviz/`, `scripts/`, and the workspace `Makefile` as bind mounts:
  ```yaml
  volumes:
    - ../src:/prosthesis_ws/src:rw
    - ../config:/prosthesis_ws/config:rw
    - ../rviz:/prosthesis_ws/rviz:rw
    - ../scripts:/prosthesis_ws/scripts:rw
    - ../Makefile.workspace:/prosthesis_ws/Makefile:ro
    - ../tests:/prosthesis_ws/tests:rw
    # Named volumes for build artifacts
    - prosthesis-build:/prosthesis_ws/build
    - prosthesis-install:/prosthesis_ws/install
    - prosthesis-log:/prosthesis_ws/log
  ```

  Rationale: Host edits are immediately visible in the container. No image rebuild needed.

- [x] **2.4. Add auto-build on first startup**
  Add an entrypoint check or `.bashrc` hook: if `/prosthesis_ws/install/setup.bash` doesn't exist (empty named volume on first run), run `colcon build` automatically. This could be a simple check in the `.bashrc`:
  ```bash
  if [ ! -f /prosthesis_ws/install/setup.bash ]; then
      echo "First run — building workspace..."
      cd /prosthesis_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
      source /prosthesis_ws/install/setup.bash
  fi
  ```

  Rationale: First-time startup should "just work" without manual intervention.

### Phase 3: Remove Unused Services

- [x] **3.1. Remove `interactive` service from `docker-compose.yml`**
  Delete `docker/docker-compose.yml:44-49`. No Makefile target, no purpose.

- [x] **3.2. Remove `grasp_test` service from `docker-compose.yml`**
  Delete `docker/docker-compose.yml:92-116`. Remove `grasp_test` from segmentation's `profiles` list. Keep `src/prosthesis_launch/launch/grasp_test.launch.py` — it's still launchable via `make grasp-test` inside the container.

- [x] **3.3. Remove `digital_twin` service from `docker-compose.yml`**
  Delete `docker/docker-compose.yml:118-142`. Remove `digital_twin` from segmentation's `profiles` list. Keep the launch file — run via `make digital-twin` inside the container.

- [x] **3.4. Remove `prosthesis-hw` service from `docker-compose.yml`**
  Delete `docker/docker-compose.yml:53-70`. The hardware overlay file `docker-compose.hw.yml` already applies device mappings to the `prosthesis` service by name — it doesn't need a separate service definition.

- [x] **3.5. Clean up segmentation service profiles**
  Remove `grasp_test` and `digital_twin` from segmentation's `profiles`. Keep only `segmentation`. If you need segmentation alongside the dev container, start it explicitly: `make segmentation` (new host Makefile target).

### Phase 4: Merge Hardware into Main Service

- [x] **4.1. Simplify `docker-compose.hw.yml`**
  Keep the file as a lightweight overlay that only adds device mappings and `group_add` to the `prosthesis` service. No new service definitions. Current file at `docker/docker-compose.hw.yml` is already close to this — just remove the environment variable duplications (they're already in the main compose).

- [x] **4.2. Update `Makefile` `up-hw` target**
  Change to apply the hardware overlay to the main prosthesis service:
  ```makefile
  up-hw:
      cd $(COMPOSE_DIR) && $(COMPOSE) -f docker-compose.yml -f docker-compose.hw.yml up -d prosthesis
  ```
  Remove `--build --force-recreate` flags (no longer needed with volume mounts).

### Phase 5: Host Makefile Cleanup

- [x] **5.1. Add `make dev` target (primary development entry point)**
  ```makefile
  dev:
      cd $(COMPOSE_DIR) && $(COMPOSE) up -d prosthesis
  ```
  Starts the container with volume mounts. No build step. Instant.

- [x] **5.2. Add `make dev-shell` target**
  ```makefile
  dev-shell: dev
      cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash
  ```
  Starts container if not running, then opens a shell.

- [x] **5.3. Add `make segmentation` target**
  ```makefile
  segmentation:
      cd $(COMPOSE_DIR) && $(COMPOSE) --profile segmentation up -d segmentation
  ```
  Starts the inference server alongside the dev container.

- [x] **5.4. Remove dead Makefile targets**
  Remove from host `Makefile`:
  - `up-grasp-test`, `down-grasp-test`, `logs-grasp-test` (lines 50-58)
  - `up-digital-twin`, `down-digital-twin`, `logs-digital-twin`, `test-digital-twin` (lines 61-75)
  - `logs-cameras` (lines 96-97) — cameras run on Jetson now
  
  Update `.PHONY` line accordingly.

- [x] **5.5. Update `make test` target**
  The test service should continue building from the production image stage (no volume mounts) for reproducibility. Update to use the production stage:
  ```makefile
  test:
      cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis && $(COMPOSE) run --rm test
  ```
  The `test` service in compose uses `target: production` to get the full build.

- [x] **5.6. Update `make up-prosthesis` target**
  Simplify to just start the container (no build step):
  ```makefile
  up-prosthesis: dev
  ```
  Or remove entirely if `make dev` replaces it.

### Phase 6: Segmentation Service Cleanup

- [x] **6.1. Add named volume for segmentation weights**
  Add a named volume `segmentation-weights` mounted at `/weights` in the segmentation service. Prevents re-downloading ~200MB model weights.

- [x] **6.2. Clean up segmentation profiles**
  Remove `grasp_test` and `digital_twin` from segmentation's `profiles`. Keep only `segmentation`.

---

## In-Container Makefile — Full Target Reference

This is the Makefile that lives in the project root (e.g., `Makefile.workspace`) and gets mounted into the container at `/prosthesis_ws/Makefile`:

| Target | Command | Description |
|--------|---------|-------------|
| `help` | *(default)* | Print all targets with descriptions |
| `build` | `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` | Build all packages |
| `build-pkg` | `colcon build --packages-select $(PKG)` | Build single package (`PKG=name`) |
| `build-debug` | `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug` | Build with debug symbols |
| `clean` | `rm -rf build/ install/ log/` | Wipe all build artifacts |
| `test` | `bash scripts/run_tests.sh` | Run smoke test suite |
| `mock` | `ros2 launch prosthesis_launch mock.launch.py` | Mock pipeline (no hardware) |
| `pipeline` | `ros2 launch prosthesis_launch pipeline.launch.py` | Full hardware pipeline |
| `digital-twin` | `ros2 launch prosthesis_launch digital_twin.launch.py camera:=true gui:=true` | Digital twin with cameras |
| `digital-twin-mock` | `ros2 launch prosthesis_launch digital_twin.launch.py camera:=false gui:=true` | Digital twin without cameras |
| `grasp-test` | `ros2 launch prosthesis_launch grasp_test.launch.py` | Legacy grasp test mode |
| `twist-test` | `ros2 launch prosthesis_launch twist_propagation_test.launch.py` | Twist propagation with Jetson |

---

## Verification Criteria

- [ ] **VC-1: No image rebuild needed for source changes** — Edit a `.py` file on host, run `make build-pkg PKG=<pkg>` inside container, change reflected without `podman build`
- [ ] **VC-2: `make dev` starts in under 5 seconds** — Container starts instantly with volume mounts
- [ ] **VC-3: `colcon build` works inside the container** — The `prosthesis` user can write to `build/`, `install/`, `log/` via named volumes
- [ ] **VC-4: Build artifacts persist across container restarts** — `make dev` after `make down` doesn't need a full rebuild
- [ ] **VC-5: `make test` (host) still passes** — Smoke tests run in a clean production image
- [ ] **VC-6: Hardware mode still works** — `make up-hw` maps USB devices correctly
- [ ] **VC-7: In-container `make` shows help** — Typing `make` inside the container lists all targets
- [ ] **VC-8: In-container `make digital-twin` launches correctly** — All launch files work through the in-container Makefile
- [ ] **VC-9: No references to removed services remain** — Makefile and compose files are clean

---

## Potential Risks and Mitigations

1. **Bind mount performance on large `src/` directory**
   Mitigation: Linux bind mounts have native performance (no filesystem translation). The `src/` tree is ~17 packages — negligible overhead.

2. **Named volume stale build artifacts after dependency changes**
   Mitigation: `make clean` inside the container wipes build artifacts. Document as recovery step. Host `Makefile` gets a `make clean-volumes` target that removes the named volumes entirely.

3. **File permission issues with bind mounts and `userns_mode: keep-id`**
   Mitigation: Already using `userns_mode: keep-id` which maps host UID to container UID. Test by verifying `touch /prosthesis_ws/src/test_file` works inside the container.

4. **In-container Makefile ROS sourcing in non-interactive shells**
   Mitigation: Each target explicitly sources ROS setup before running. The `.bashrc` sourcing only works for interactive shells — `make` targets use explicit `source` in their recipes.

5. **Removing `grasp_test`/`digital_twin` services may surprise other developers**
   Mitigation: The launch files remain in source. The in-container Makefile provides `make grasp-test` and `make digital-twin` targets. Document the transition.

6. **The `test` service needs a self-contained image**
   Mitigation: Multi-stage Dockerfile — `base` stage for dev, `production` stage (copies source + builds) for test. The test service uses `target: production`.

---

## Alternative Approaches

1. **Dev Containers (VS Code devcontainer.json)**: IDE-integrated container development. Trade-off: Locks into VS Code. Current approach is editor-agnostic.

2. **Docker Compose Watch**: Auto-sync file changes and trigger rebuilds. Trade-off: Not available in `podman-compose`.

3. **Host-native ROS installation**: Skip containers entirely. Trade-off: Loses isolation and reproducibility.

---

## Summary of Files to Modify

| File | Change |
|------|--------|
| `Makefile.workspace` | **NEW** — In-container Makefile with build/launch/test targets |
| `docker/Dockerfile` | Split into `base` (deps only) + `production` (full build) stages |
| `docker/docker-compose.yml` | Remove 4 services, add bind + named volumes to prosthesis, clean profiles |
| `docker/docker-compose.hw.yml` | Simplify to device-only overlay on `prosthesis` service |
| `Makefile` (host) | Add `dev`, `dev-shell`, `segmentation`, `clean-volumes`; remove dead targets |

**Files NOT modified (preserved as-is):**
- `src/prosthesis_launch/launch/grasp_test.launch.py` — stays for manual use
- `src/prosthesis_launch/launch/digital_twin.launch.py` — stays for manual use
- `docker/Dockerfile.segmentation` — unchanged (separate Python 3.8 stack)
- `docker/Dockerfile.jazzy-rviz` — unchanged (host-side RViz)
- `Makefile.rviz` — unchanged (host-side RViz)
