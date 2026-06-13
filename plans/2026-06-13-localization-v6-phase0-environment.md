# Localization Rework V6 — Phase 0: Environment Setup

**Date:** 2026-06-13
**Parent plan:** `plans/2026-06-13-localization-rework-plan-v6.md`
**Depends on:** Nothing (this is the root dependency)
**Blocks:** All subsequent phases
**Estimated time:** 30–45 minutes
**Agent:** Single agent (serial, blocking)

---

## Objective

Add `gtsam` and `open3d` Python dependencies to the prosthesis Docker image, rebuild, and verify that the existing workspace still builds and tests pass. This is the hard prerequisite for all V6 implementation work — no new code can run without these libraries available inside the container.

---

## Context

- The project runs ROS2 Jazzy **inside Docker/podman**, not on the host (`/opt/ros` is empty on the host).
- The `prosthesis:latest` image (6.1 GB) exists but the container is not running.
- Current Python deps in `docker/Dockerfile:33-37`: dynamixel-sdk, requests, scikit-learn, mindrove. **Neither `gtsam` nor `open3d` is installed.**
- `cv2` (OpenCV 4.13) IS available on the host but must be verified in-container.
- The build/test workflow: `make dev` → `make shell` → in-container `colcon build` / `make test`.

---

## Tasks

- [x] **0.1** Read `docker/Dockerfile` and locate the Python pip install block (around line 33).
- [x] **0.2** Add `gtsam` and `open3d` to the `pip install --no-cache-dir --break-system-packages` command in `docker/Dockerfile`. Use these exact package names. Add `pip install` cache-busting by placing them on the same line or a new RUN layer. Do NOT remove existing packages.
- [x] **0.3** Rebuild the prosthesis image: `make build-prosthesis` (from host). This rebuilds the base stage. Expect 5–10 minutes for the new pip installs.
- [x] **0.4** Start the dev container: `make dev` then `make shell`.
- [x] **0.5** Inside the container, verify imports succeed:
  ```bash
  python3 -c "import gtsam; print('gtsam', gtsam.__version__)"
  python3 -c "import open3d; print('open3d', open3d.__version__)"
  python3 -c "import cv2; print('cv2', cv2.__version__)"
  python3 -c "import numpy; print('numpy', numpy.__version__)"
  ```
  All four must print without error. If `gtsam` fails to install (it sometimes needs specific wheel availability), fall back to building from source: `pip install gtsam --no-binary gtsam` or pin a known-good version. Document any workarounds in the task notes.
- [x] **0.6** Inside the container, build the existing workspace to confirm nothing broke:
  ```bash
  cd /prosthesis_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
  This must complete with zero errors. Warnings are acceptable.
- [x] **0.7** Run the existing smoke tests:
  ```bash
  make test    # inside container, or: make test from host
  ```
  All existing tests must still pass. If a test fails due to the new dependencies, investigate and fix.
  > **Result:** 4/5 tests pass (build, launch, preshaping, nodes_start). The `twist_prop` test has
  > pre-existing functional failures (twist magnitude = 0.0, collision detection not triggering)
  > that are **unrelated to the new dependencies** — the `twist_propagation` package does not import
  > gtsam/open3d/rosbags. The `nodes_start` test initially failed due to the container's
  > `cyclonedds_peer.xml` referencing a non-existent network interface; it passes when run with
  > `cyclonedds_local.xml`. The `make test` host target also has a pre-existing `gosu`/userns
  > incompatibility with the `test` compose service; tests were run via `docker exec` instead.
- [x] **0.8** Verify `rosbags` Python library is available in-container (needed for bag-based testing in later phases). If not installed, add it to the Dockerfile pip line too:
  ```bash
  python3 -c "import rosbags; print('rosbags', rosbags.__version__)"
  ```
  > **Result:** `rosbags` was proactively added to the Dockerfile pip line (version 0.11.3) and imports successfully in-container.

---

## Verification Gate

Phase 0 is complete when ALL of the following are true:
1. `docker/Dockerfile` contains `gtsam`, `open3d` in the pip install block.
2. `make build-prosthesis` succeeds.
3. Inside the container: `import gtsam`, `import open3d`, `import cv2`, `import rosbags` all succeed.
4. `colcon build` of the existing workspace succeeds with zero errors.
5. Existing smoke tests pass (`make test`).

---

## Risk Notes

- **`gtsam` wheel availability:** The `gtsam` PyPI package provides prebuilt wheels for some Python versions but not all. The container uses Python 3.12 (Jazzy base). If no wheel exists, building from source requires Boost and CMake — add `libboost-all-dev` and `cmake` to the apt-get block if needed. This could extend build time to 20+ minutes.
- **`open3d` size:** Open3D is a large package (~500 MB wheel). The image will grow by ~1 GB. This is expected.
- **Do not commit the rebuilt image** — only commit the Dockerfile change. The image is rebuilt locally by each developer.
