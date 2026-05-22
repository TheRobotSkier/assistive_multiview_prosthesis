# Fix Podman Build Failures + Missing Scripts + Wrist Driver Polish

## Objective

Fix three categories of issues preventing the project from building and running cleanly:
1. **Podman build failures** — registry resolution, test service permissions, segmentation missing `__init__.py`
2. **Missing grasp_preshaping scripts** — the `scripts/` folder is empty, but `model.py` (LUT generator) and other utility scripts should exist
3. **Empty wrist_driver config/launch** — the package has no config or launch files, making it unusable without manual setup

## Implementation Plan

### Phase 1: Podman Build Fixes

- [ ] **1.1** Fix `Dockerfile` base image reference — change `FROM osrf/ros:jazzy-desktop` to `FROM docker.io/osrf/ros:jazzy-desktop` so podman doesn't try quay.io first
  - File: `docker/Dockerfile` line 1
  - Podman defaults to quay.io registry; fully qualifying with docker.io prevents the `unauthorized: access to the requested resource is not authorized` error

- [ ] **1.2** Fix `Dockerfile.segmentation` base image reference — change `FROM python:3.8-slim-bullseye` to `FROM docker.io/python:3.8-slim-bullseye`
  - File: `docker/Dockerfile.segmentation` line 1
  - Same registry issue as above

- [ ] **1.3** Add explicit `image:` tags to all compose services so podman knows they're local builds
  - File: `docker/docker-compose.yml`
  - Add `image: prosthesis:latest` to `prosthesis` service
  - Add `image: prosthesis-hw:latest` to `prosthesis-hw` service  
  - Add `image: segmentation:latest` to `segmentation` service
  - Add `image: prosthesis-test:latest` to `test` service
  - Without explicit image tags, podman-compose generates names like `docker_prosthesis` and tries to pull them from registries

- [ ] **1.4** Fix test service — add `user: "0:0"` and change `entrypoint` to `command`
  - File: `docker/docker-compose.yml`
  - The test service inherits `USER prosthesis` from the Dockerfile but needs root to do clean builds
  - `podman-compose run` handles `command` overrides better than `entrypoint` overrides
  - Change `entrypoint: ["/bin/bash", "/prosthesis_ws/scripts/run_tests.sh"]` to `command: ["/bin/bash", "/prosthesis_ws/scripts/run_tests.sh"]`

- [ ] **1.5** Simplify `scripts/test_build.sh` — verify build artifacts exist instead of doing a clean rebuild
  - File: `scripts/test_build.sh`
  - The Dockerfile already runs `colcon build` during image build. The test should verify the build succeeded, not redo it from scratch
  - Check that `install/`, `build/`, and `log/` directories exist and are non-empty
  - Optionally run `colcon test` for packages that have tests

- [ ] **1.6** Create `Makefile` at project root for common podman-compose commands
  - Abstracts away `cd docker && podman-compose ...` into simple `make build`, `make up`, `make test`, `make shell`
  - Ensures correct working directory and command ordering (build before up)

### Phase 2: Recover Grasp Preshaping Scripts

- [ ] **2.1** Recover `scripts/model.py` from git history
  - This is the LUT generator script that creates `finger_contact_lut.npz`
  - Command: `git show HEAD~N:docker_ws/dev/grasp_preshaping/scripts/model.py > src/grasp_preshaping/scripts/model.py`
  - The exact commit depth needs to be determined by running `git log --all --full-history -- "**/grasp_preshaping/scripts/model.py"`
  - If the file was never committed, it may need to be recreated from the Rust code that loads it (`src/grasp_preshaping/src/lut_helper.rs`)

- [ ] **2.2** Recover any other useful scripts from git history
  - Check `git log --all --full-history -- "**/grasp_preshaping/scripts/*"` for all scripts that existed
  - Likely candidates: visualization scripts, test scripts, data generation scripts
  - Copy recovered scripts to `src/grasp_preshaping/scripts/`

- [ ] **2.3** Update `CMakeLists.txt` to install scripts
  - File: `src/grasp_preshaping/CMakeLists.txt`
  - Add `install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)` or similar
  - Only if the recovered scripts are useful at runtime (model.py is a dev tool, so may not need install)

### Phase 3: Wrist Driver Config and Launch

- [ ] **3.1** Create `src/wrist_driver/config/wrist_params.yaml`
  - Default parameters for the wrist Dynamixel servo:
    - `port`: `/dev/ttyUSB0`
    - `baudrate`: 57600
    - `motor_id`: 1
    - `protocol_version`: 2.0
    - `publish_rate_hz`: 20.0
  - These match the `declare_parameter()` defaults in `wrist_driver_node.py:64-68` but make them explicit and tunable

- [ ] **3.2** Create `src/wrist_driver/launch/wrist_driver.launch.py`
  - Minimal launch file that:
    - Loads parameters from `config/wrist_params.yaml`
    - Launches the `wrist_driver_node`
    - Includes a `use_mock` parameter that, when true, publishes fake wrist state data instead of connecting to hardware

- [ ] **3.3** Update `src/wrist_driver/setup.py` to install config and launch directories
  - Add `data_files` entries for `config/` and `launch/`
  - Ensure `setup.cfg` has correct `install_scripts` path

### Phase 4: Segmentation `__init__.py` Fix

- [ ] **4.1** Verify all `__init__.py` files exist in `src/segmentation/MinkowskiEngine/`
  - The build error was: `FileNotFoundError: '/MinkowskiEngine-src/MinkowskiEngine/__init__.py'`
  - The file exists on disk at `src/segmentation/MinkowskiEngine/__init__.py` (confirmed: contains `__version__ = "0.5.4"`)
  - Check if `utils/__init__.py`, `modules/__init__.py`, etc. also exist
  - If files exist on disk but the build still fails, the issue is likely that files are not committed to git and podman's build context excludes untracked files
  - Fix: `git add` all `__init__.py` files in the MinkowskiEngine tree

## Verification Criteria

- `podman-compose build` succeeds for all services (prosthesis, segmentation, test) without quay.io errors
- `podman-compose run --rm test` exits with code 0
- `src/grasp_preshaping/scripts/` contains at least `model.py`
- `src/wrist_driver/config/` contains `wrist_params.yaml`
- `src/wrist_driver/launch/` contains `wrist_driver.launch.py`
- `ros2 launch wrist_driver wrist_driver.launch.py` parses without errors (mock mode)
- All MinkowskiEngine `__init__.py` files are tracked by git

## Potential Risks and Mitigations

1. **Scripts lost from git history**
   - The `docker_ws/` was deleted and committed. If `model.py` was never committed before the delete, it's gone from git.
   - Mitigation: Check `git log --all --diff-filter=D -- "**/model.py"` for the deletion event. If found, recover from the parent commit. If not found, the script needs to be recreated from `lut_helper.rs` and the numpy binary format.

2. **Podman build context excludes untracked files**
   - Podman may use `.git`-aware build contexts that only include tracked files.
   - Mitigation: Explicitly `git add` all required files, especially `__init__.py` files and the `finger_contact_lut.npz`.

3. **MinkowskiEngine version mismatch**
   - The `__init__.py` has `__version__ = "0.5.4"` but the source code may be from a different version.
   - Mitigation: This is the version that was working before. If the build fails with version-specific errors, the source tree needs to match 0.5.4.

## Alternative Approaches

1. **For segmentation**: Instead of copying the MinkowskiEngine source tree into the repo, clone it during Docker build (`RUN git clone --depth 1 --branch v0.5.4 ...`). This ensures all files are present and the repo is clean. Trade-off: Docker builds become network-dependent.

2. **For wrist_driver config**: Instead of a separate YAML config file, keep using `declare_parameter()` defaults and override via launch arguments. Trade-off: less discoverable, no single place to see all wrist params.

3. **For Makefile**: Could use a `Justfile` (with `just` command) instead of Makefile for simpler syntax. Trade-off: requires installing `just`.
