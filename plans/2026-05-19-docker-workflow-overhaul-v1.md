# Docker/Podman Development Workflow Overhaul

## Objective

Eliminate the "rebuild the image for every code change" bottleneck, remove unused/outdated services, and merge the hardware service into the main prosthesis service — resulting in a fast inner-loop development experience with Podman.

---

## Current State Analysis

### Problem 1: Full image rebuild for every code change

**Root cause:** The `Dockerfile` (`docker/Dockerfile:48-62`) does `COPY src/ src/` followed by `colcon build` inside the image build step. This means:
- Any change to any `.py`, `.cpp`, `.launch.py`, config, or rviz file triggers a full `podman build`
- `colcon build` from scratch every time (~minutes depending on packages)
- The container user (`prosthesis`) cannot run `colcon build` inside a running container because the existing `build/`, `install/`, `log/` directories were created by root during image build and are owned by root

**Evidence:**
- `docker/Dockerfile:48-62` — `COPY src/ src/` + `colcon build` at image build time
- `docker/Dockerfile:71` — `USER prosthesis` runs as non-root after build
- `.gitignore:1-3` — `build/`, `install/`, `log/` are gitignored (never on host)
- Command trace shows `make build-prosthesis` + `make up-prosthesis` cycle for every change

### Problem 2: `grasp_test` service is likely outdated

**Finding:** The `grasp_test` service (`docker/docker-compose.yml:92-116`) launches `grasp_test.launch.py` which is a superset of the pipeline with cameras + segmentation + twist propagation + preshaping + wrist driver + RViz. It uses `rmw_fastrtps_cpp` (different from the main service which uses `rmw_cyclonedds_cpp`). The `digital_twin` launch file does everything `grasp_test` does AND adds:
- Pointcloud fusion from dual cameras
- ChArUco board tracking
- Cam2 hand tracker
- Digital twin URDF

The `grasp_test` launch does NOT include pointcloud fusion, ChArUco tracking, or the digital twin hand model. It appears to be an older single-camera grasp test mode that predates the dual-camera digital twin setup.

**Verdict:** `grasp_test` is a legacy profile. The `digital_twin` launch is the current evolution. The `grasp_test` service and profile can be removed. The `grasp_test.launch.py` file itself can stay in the source tree (it may still be useful to launch manually inside a container), but the Docker Compose service wrapping it is redundant.

### Problem 3: `digital_twin` service can be simplified

**Finding:** The `digital_twin` service (`docker/docker-compose.yml:118-142`) is another full prosthesis container that just runs a different launch file. It depends on `segmentation`. With volume mounts (the solution to Problem 1), you don't need a separate container — you can just run the launch file inside the main prosthesis container.

**Verdict:** The `digital_twin` Docker Compose service can be removed. You would instead run the digital twin launch inside the main development container.

### Problem 4: `prosthesis-hw` (hardware service) can merge into `prosthesis`

**Finding:** The hardware service (`docker/docker-compose.hw.yml`) adds USB device mappings and `group_add: keep-groups`. Since cameras now run on the Jetson (accessed via RJ45/Ethernet), the only remaining USB devices are:
- Mia Hand serial (`/dev/ttyUSB0`)
- Wrist Dynamixel (`/dev/ttyUSB1`)

These can be conditionally mapped in the main compose file. The hardware override file adds minimal value — just 2 device mappings that could be optional in the main service.

**Verdict:** Merge hardware into the main service with conditional device mapping. Remove `docker-compose.hw.yml`.

### Problem 5: `interactive` service is unused

**Finding:** The `interactive` service (`docker/docker-compose.yml:44-49`) extends `prosthesis` and runs `bash`. There is no Makefile target for it, and it has no profile restriction so it starts with `make up`. It appears to be a leftover experiment.

**Verdict:** Remove it.

### Problem 6: `segmentation` service is still needed but should use a named volume for weights

**Finding:** The segmentation service downloads ~200MB weights on first run. Currently it re-downloads if the container is removed. A named volume for `/weights` would persist them.

---

## Proposed Architecture

### New Docker Compose Structure

```
services:
  prosthesis:          # Single development container
    - Source mounted as volume (src/, config/, rviz/, scripts/)
    - colcon build artifacts in a named volume (persist across restarts)
    - Optional USB device mapping (env var toggle)
    - Used for ALL development: mock, hardware, digital twin, grasp test

  segmentation:        # Unchanged — isolated Python 3.8 inference server
    - Add named volume for /weights

  test:                # Unchanged — CI/smoke test runner
    - Still builds from image (no volume mount — tests should be reproducible)
```

### Removed Services
- `interactive` — unused, no Makefile target
- `grasp_test` — legacy, superseded by digital_twin
- `digital_twin` — just a different launch command, run inside main container
- `prosthesis-hw` — merged into main service with conditional devices

### New Development Workflow

```
# One-time setup (or after Dockerfile changes):
make build-prosthesis

# Start development container (instant — no rebuild):
make dev

# Inside container, iterate on code:
colcon build --packages-select <changed_package>
# OR
colcon build  # full rebuild (fast — dependencies cached in named volume)

# Run any launch file:
ros2 launch prosthesis_launch digital_twin.launch.py camera:=true gui:=true
ros2 launch prosthesis_launch grasp_test.launch.py
ros2 launch prosthesis_launch mock.launch.py

# Shell into running container:
make shell
```

---

## Implementation Plan

### Phase 1: Volume-Mount Development Container

- [ ] **1.1. Refactor `Dockerfile` into a base image + dev entrypoint**
  Split the current `Dockerfile` so that:
  - **Base stage** (`docker/Dockerfile`): Installs system deps, Python deps, creates user, runs `rosdep install` — but does NOT copy source or run `colcon build`
  - **Production stage** (optional, for deployment): Copies source + builds (current behavior)
  - **Development**: The base image is built once; source and build artifacts live in volumes
  
  Rationale: The base image changes only when system/ROS dependencies change (rare). Source code changes (frequent) never trigger an image rebuild.

- [ ] **1.2. Add named volume for colcon build artifacts in `docker-compose.yml`**
  Add a named volume (e.g., `prosthesis-build`) mounted at `/prosthesis_ws/build`, `/prosthesis_ws/install`, and `/prosthesis_ws/log`. This persists `colcon build` outputs across container restarts and avoids permission issues (the volume is owned by the container user from first use).
  
  Rationale: Solves the "container user can't write to build/" problem. Named volumes are initialized with the container's filesystem permissions on first create.

- [ ] **1.3. Mount source directories as bind volumes in `docker-compose.yml`**
  Mount `src/`, `config/`, `rviz/`, and `scripts/` as bind mounts from the host into `/prosthesis_ws/`. This means:
  - Host edits are immediately visible in the container
  - No image rebuild needed for code changes
  - `colcon build` inside the container picks up changes instantly
  
  Rationale: This is the core of the inner-loop speedup. Edit on host, build in container.

- [ ] **1.4. Add an entrypoint script or `.bashrc` hook that runs `colcon build` on shell entry (optional)**
  Add a flag/env var (e.g., `AUTO_BUILD=1`) that triggers `colcon build` when the container starts or when a shell is opened. This can be a simple check: "if install/ is empty, run colcon build".
  
  Rationale: First-time startup should "just work" — build automatically if the named volume is empty.

- [ ] **1.5. Update the `Makefile` with new `dev` target**
  Add a `make dev` target that starts the prosthesis container with volume mounts (no build step). Update `make up-prosthesis` to use the new flow. Keep `make build-prosthesis` for when the base image needs updating (system dependency changes).

### Phase 2: Remove Unused Services

- [ ] **2.1. Remove `interactive` service from `docker-compose.yml`**
  Delete the `interactive` service definition at `docker/docker-compose.yml:44-49`. It has no Makefile target and serves no purpose.
  
  Rationale: Dead code. Clutters the compose file.

- [ ] **2.2. Remove `grasp_test` service from `docker-compose.yml`**
  Delete the `grasp_test` service definition at `docker/docker-compose.yml:92-116`. Remove the `grasp_test` profile from the segmentation service. Keep `src/prosthesis_launch/launch/grasp_test.launch.py` in source — it can still be launched manually inside the container.
  
  Rationale: Superseded by `digital_twin`. The launch file stays for manual use.

- [ ] **2.3. Remove `digital_twin` service from `docker-compose.yml`**
  Delete the `digital_twin` service definition at `docker/docker-compose.yml:118-142`. Remove the `digital_twin` profile from the segmentation service. The digital twin launch file stays in source and can be run inside the dev container.
  
  Rationale: With volume-mounted development, there's no need for a separate container per launch file.

- [ ] **2.4. Remove corresponding Makefile targets**
  Remove from `Makefile`:
  - `up-grasp-test`, `down-grasp-test`, `logs-grasp-test` (lines 50-58)
  - `up-digital-twin`, `down-digital-twin`, `logs-digital-twin`, `test-digital-twin` (lines 61-75)
  - `up-hw` (line 47) — replaced by new hardware toggle
  - `logs-cameras` (line 96-97) — cameras run on Jetson now
  
  Update `.PHONY` line accordingly.

### Phase 3: Merge Hardware into Main Service

- [ ] **3.1. Add conditional USB device mapping to main `prosthesis` service**
  Use environment variables (e.g., `ENABLE_HARDWARE=1`, `MIA_SERIAL_PORT`, `WRIST_PORT`) to conditionally map USB devices. In Podman/Docker Compose, device mappings can't be truly conditional, so use one of:
  - **Option A**: Keep a minimal `docker-compose.hw.yml` override file (just device mappings, no new service) — simplest, proven pattern
  - **Option B**: Use a wrapper script that adds `--device` flags based on env vars
  
  Recommendation: **Option A** — keep the override file pattern but make it an overlay on the main service only (no separate service). This is already how it works; just remove the separate `prosthesis-hw` service definition from the main compose file.

- [ ] **3.2. Remove `prosthesis-hw` service from `docker-compose.yml`**
  Delete the `prosthesis-hw` service at `docker/docker-compose.yml:53-70`. The hardware override file `docker-compose.hw.yml` already applies device mappings to the `prosthesis` service by name — it doesn't need a separate service.
  
  Rationale: `docker-compose.hw.yml` already targets the `prosthesis` service. The `prosthesis-hw` container_name service is redundant.

- [ ] **3.3. Update `Makefile` `up-hw` target**
  Change `up-hw` to apply the hardware overlay to the main `prosthesis` service:
  ```
  up-hw: build-prosthesis
      cd $(COMPOSE_DIR) && $(COMPOSE) -f docker-compose.yml -f docker-compose.hw.yml up -d prosthesis
  ```
  Remove the `--build --force-recreate` flags (no longer needed with volume mounts).

### Phase 4: Segmentation Service Cleanup

- [ ] **4.1. Add named volume for segmentation weights**
  Add a named volume `segmentation-weights` mounted at `/weights` in the segmentation service. This prevents re-downloading ~200MB model weights on every container recreation.
  
  Rationale: Simple optimization. Weights don't change.

- [ ] **4.2. Clean up segmentation profiles**
  Remove `grasp_test` and `digital_twin` from the segmentation service's `profiles` list. Keep only `segmentation` (and optionally a generic `inference` profile if other launch modes need it).

### Phase 5: Makefile Cleanup and New Targets

- [ ] **5.1. Add `make dev` target**
  ```
  dev:
      cd $(COMPOSE_DIR) && $(COMPOSE) up -d prosthesis
  ```
  Starts the container with volume mounts. No build step. Instant.

- [ ] **5.2. Add `make dev-shell` target**
  ```
  dev-shell: dev
      cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash
  ```
  Starts the container (if not running) and opens a shell.

- [ ] **5.3. Add `make colcon-build` and `make colcon-build-pkg` targets**
  ```
  colcon-build:
      cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis bash -c "source /opt/ros/jazzy/setup.bash && colcon build"
  
  colcon-build-pkg:
      cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-select $(PKG)"
  ```
  Allows building from the host without opening a shell.

- [ ] **5.4. Update `make shell` to depend on container running**
  Currently `make shell` fails if the container isn't running (exit code 2 in command trace). Add a check or dependency:
  ```
  shell:
      cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash
  ```
  (Keep as-is — the error message from compose is clear enough.)

- [ ] **5.5. Remove dead Makefile targets**
  Remove all targets identified in Phase 2.4. Add new targets to `.PHONY`.

- [ ] **5.6. Update `make test` to work with new structure**
  The `test` service should continue building from the image (no volume mounts) for reproducibility. Keep it as-is but ensure it uses the base image that includes rosdep but not the colcon build step. The test service's Dockerfile stage should copy source and build as part of the test run.

---

## Verification Criteria

- [ ] **VC-1: No image rebuild needed for source changes** — Edit a `.py` file on the host, run `make colcon-build-pkg PKG=<pkg>`, and the change is reflected without `podman build`
- [ ] **VC-2: `make dev` starts in under 5 seconds** — Container starts instantly with volume mounts
- [ ] **VC-3: `colcon build` works inside the container** — The `prosthesis` user can write to `build/`, `install/`, `log/` via named volumes
- [ ] **VC-4: Build artifacts persist across container restarts** — `make dev` after `make down` doesn't need a full rebuild
- [ ] **VC-5: `make test` still passes** — Smoke tests run in a clean image (no volume mounts)
- [ ] **VC-6: Hardware mode still works** — `make up-hw` maps USB devices correctly
- [ ] **VC-7: All launch files still work** — `digital_twin.launch.py`, `grasp_test.launch.py`, `mock.launch.py`, `pipeline.launch.py` all launch inside the dev container
- [ ] **VC-8: Removed services don't break anything** — No references to removed services remain in Makefile or compose files

---

## Potential Risks and Mitigations

1. **Bind mount performance on large `src/` directory**
   Mitigation: The `src/` directory is ~17 ROS packages with Python, C++, CUDA, and Rust code. Bind mount performance on Linux is native (no filesystem translation overhead like macOS/Windows). Should be fine.

2. **Named volume stale build artifacts after dependency changes**
   Mitigation: Add a `make clean-build` target that wipes the named volume (`podman volume rm prosthesis-build`) and triggers a fresh build. Document this as the recovery step if builds fail after system dependency changes.

3. **File permission issues with bind mounts and `userns_mode: keep-id`**
   Mitigation: The current setup already uses `userns_mode: keep-id` which maps the host UID to the container UID. Bind mounts should respect this. Test by verifying `touch /prosthesis_ws/src/test_file` works inside the container.

4. **`colcon build` inside container may be slower than Docker layer cache**
   Mitigation: Named volumes persist build artifacts. Incremental `colcon build` is fast (only rebuilds changed packages). The first build after a clean volume will take the same time as the current Dockerfile build, but subsequent builds are incremental.

5. **Removing `grasp_test` service may break someone's workflow**
   Mitigation: The `grasp_test.launch.py` file stays in the source tree. Anyone who needs it can run it inside the dev container with `ros2 launch prosthesis_launch grasp_test.launch.py`. Document this in the Makefile help text.

6. **The `test` service needs a self-contained image**
   Mitigation: Keep a multi-stage Dockerfile where the "test" stage copies source and builds. The "dev" stage only installs dependencies. The test service uses the test stage.

---

## Alternative Approaches

1. **Dev Containers (VS Code / devcontainer.json)**: Instead of manual Makefile targets, use a devcontainer setup that mounts the workspace and provides IDE integration. Trade-off: Locks you into VS Code or compatible editors. Current approach is editor-agnostic.

2. **Docker Compose Watch (Docker 2.22+)**: Use `watch` directives to automatically sync file changes and trigger rebuilds. Trade-off: Not available in `podman-compose`. Would require switching to Docker Compose v2.

3. **Full host-based development**: Install ROS Jazzy natively on the host and skip containers entirely. Trade-off: Loses isolation, reproducibility, and the clean environment that containers provide. Would also need to install all system dependencies natively.

4. **Keep current structure but add a `colcon build` wrapper script**: Instead of volume mounts, keep the current COPY-based approach but add a script that fixes permissions and runs `colcon build` inside the running container. Trade-off: Still requires a full image rebuild when source changes. Doesn't solve the core problem.

---

## Summary of Files to Modify

| File | Change |
|------|--------|
| `docker/Dockerfile` | Split into base (deps only) + production/test stages |
| `docker/docker-compose.yml` | Remove 4 services, add volume mounts to prosthesis, add named volumes |
| `docker/docker-compose.hw.yml` | Simplify to only device mappings on `prosthesis` service |
| `Makefile` | Add `dev`, `dev-shell`, `colcon-build`, `colcon-build-pkg`, `clean-build`; remove dead targets |
| `docker/.env.example` | Document new env vars (`ENABLE_HARDWARE`, etc.) |

**Files NOT modified (preserved as-is):**
- `src/prosthesis_launch/launch/grasp_test.launch.py` — stays for manual use
- `src/prosthesis_launch/launch/digital_twin.launch.py` — stays for manual use
- `docker/Dockerfile.segmentation` — unchanged (separate Python 3.8 stack)
- `docker/Dockerfile.jazzy-rviz` — unchanged (host-side RViz)
- `Makefile.rviz` — unchanged (host-side RViz)
