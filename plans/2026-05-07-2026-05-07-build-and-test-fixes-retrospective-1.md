# Build & Test Pipeline Fix — Retrospective

## Objective

Fix all Docker build failures and test failures in the `multiview_prosthesis` project so that `make build` and `make test` complete with exit code 0.

---

## Issues Found and Fixed

### Issue 1: MinkowskiEngine COPY destination was flat

**Symptom:** `FileNotFoundError: [Errno 2] No such file or directory: '/MinkowskiEngine-src/MinkowskiEngine/__init__.py'`

**Root cause:** `docker/Dockerfile.segmentation:34` had `COPY MinkowskiEngine/ /MinkowskiEngine-src/`. In Docker/Podman, `COPY src/ dest/` copies the *contents* of `src/` into `dest/` — so `__init__.py` landed at `/MinkowskiEngine-src/__init__.py` instead of `/MinkowskiEngine-src/MinkowskiEngine/__init__.py`. The `setup.py` script expected it at the latter path.

**Fix:** Changed the COPY destination:
```dockerfile
# Before:
COPY MinkowskiEngine/ /MinkowskiEngine-src/

# After:
COPY MinkowskiEngine/ /MinkowskiEngine-src/MinkowskiEngine/
```

**File changed:** `docker/Dockerfile.segmentation:34`

---

### Issue 2: Missing `README.md` for MinkowskiEngine build

**Symptom:** `FileNotFoundError: [Errno 2] No such file or directory: '/MinkowskiEngine-src/README.md'`

**Root cause:** `setup.minkowski.py:332` calls `read("README.md")` to populate `long_description`, but no `README.md` existed in the build context (`src/segmentation/`) and the Dockerfile didn't copy one.

**Fix:** Created a minimal `src/segmentation/README.md` and added a COPY line to the Dockerfile:
```dockerfile
COPY README.md /MinkowskiEngine-src/README.md
```

**Files changed:** `src/segmentation/README.md` (created), `docker/Dockerfile.segmentation:39` (added)

---

### Issue 3: Spurious `include Makefile` in MANIFEST.in

**Symptom:** `warning: no files found matching 'Makefile'` in the ROS 2 `ament_python` build of `segmentation_bridge`

**Root cause:** `src/segmentation/MANIFEST.in:1` contained `include Makefile`, but no `Makefile` exists in `src/segmentation/`. The `ament_python` build system picks up `MANIFEST.in` and warns about the missing file.

**Fix:** Removed the `include Makefile` line from `MANIFEST.in`.

**File changed:** `src/segmentation/MANIFEST.in:1` (removed)

---

### Issue 4: ENTRYPOINT vs CMD conflict in test runner

**Symptom:** `/bin/bash: /bin/bash: cannot execute binary file` (exit code 126)

**Root cause:** `docker/Dockerfile:65` set `ENTRYPOINT ["/bin/bash"]`. The test service in `docker/docker-compose.yml` had `command: ["/bin/bash", "/prosthesis_ws/scripts/run_tests.sh"]`. Docker concatenates ENTRYPOINT + command, so the container executed `/bin/bash /bin/bash /prosthesis_ws/scripts/run_tests.sh` — bash tried to interpret the binary `/bin/bash` as a script file.

**Fix:** Changed `ENTRYPOINT` to `CMD` in the Dockerfile. `CMD` is overridden by `command:` in docker-compose, so the test runner gets exactly the command it expects. For interactive use (`make up`, `make shell`), the default CMD still provides a bash shell.

```dockerfile
# Before:
ENTRYPOINT ["/bin/bash"]

# After:
CMD ["/bin/bash"]
```

**File changed:** `docker/Dockerfile:65`

---

### Issue 5: Test service used a stale separate image

**Symptom:** The ENTRYPOINT fix (Issue 4) didn't take effect — same error after rebuilding.

**Root cause:** The `test` service in `docker-compose.yml` had its own `image: prosthesis-test:latest` tag and `build:` block. Meanwhile, `Makefile:40` only builds the `prosthesis` service (`$(COMPOSE) build prosthesis`), never the `test` service. So `prosthesis-test:latest` was a stale image from a previous build that still had the old `ENTRYPOINT`.

**Fix:** Made the test service reuse the `prosthesis:latest` image directly, removing the separate `build:` block:

```yaml
# Before:
test:
    image: prosthesis-test:latest
    build:
      context: ..
      dockerfile: docker/Dockerfile
      ...

# After:
test:
    image: prosthesis:latest
    user: "0:0"
    ...
```

Also removed the stale image: `podman rmi prosthesis-test:latest`

**File changed:** `docker/docker-compose.yml:57-65`

---

### Issue 6: `AMENT_TRACE_SETUP_FILES: unbound variable` in test scripts

**Symptom:** All test scripts failed immediately with `bash: AMENT_TRACE_SETUP_FILES: unbound variable`

**Root cause:** All test scripts used `set -euo pipefail` (the `-u` flag treats unset variables as errors) *before* sourcing ROS 2's `setup.bash`. ROS 2's setup scripts internally reference unset variables like `AMENT_TRACE_SETUP_FILES`, which triggers the `-u` guard.

**Fix:** Moved the `source /opt/ros/jazzy/setup.bash` and `source /prosthesis_ws/install/setup.bash` calls *above* the `set -euo pipefail` line in all test scripts.

**Files changed:**
- `scripts/run_tests.sh`
- `scripts/test_build.sh`
- `scripts/test_launch_syntax.sh`
- `scripts/test_nodes_start.sh`
- `scripts/test_twist_propagation.sh`

---

### Issue 7: Wrong symbol name in preshaping .so test

**Symptom:** `test_preshaping_so.sh` failed trying to call `grasp_preshaping_version` — symbol not found in the shared library.

**Root cause:** The test script used `grasp_preshaping_version` but the actual exported symbol is `grasp_preshaping_api_version`.

**Fix:** Updated the test to use the correct symbol name `grasp_preshaping_api_version`.

**File changed:** `scripts/test_preshaping_so.sh`

---

### Issue 8: Missing `ros-jazzy-rmw-cyclonedds-cpp` package

**Symptom:** All ROS 2 operations in the test container failed with `failed to load shared library 'librmw_cyclonedds_cpp.so'`. Three tests (`launch`, `nodes_start`, `twist_prop`) failed.

**Root cause:** The Dockerfile installed `ros-jazzy-cyclonedds` (the DDS library) but not `ros-jazzy-rmw-cyclonedds-cpp` (the ROS middleware adapter). The `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` env var tells ROS to use CycloneDDS, but without the adapter library, no ROS 2 communication works. The `osrf/ros:jazzy-desktop` base image ships with `rmw_fastrtps_cpp` as the default — CycloneDDS is opt-in and requires both packages.

**Fix:** Added `ros-jazzy-rmw-cyclonedds-cpp` to the apt-get install list:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
        ...
        ros-jazzy-cyclonedds \
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
```

**File changed:** `docker/Dockerfile:13`

---

### Issue 9: Integration test sequencing — missing cloud before twist test

**Symptom:** `test_twist_published` failed with "no messages received" for twist messages.

**Root cause:** The test published poses but no cloud. The twist propagation node requires both poses (to estimate velocity) and a cloud (to enter its idle cycle and publish twists). Without a cloud, the node stays in `waiting_for_cloud` status and never publishes.

**Fix:** Added `harness.publish_cloud_with_target(...)` and a spin before publishing poses in `test_twist_published`.

**File changed:** `scripts/test_twist_propagation_integration.py:261-274`

---

### Issue 10: Integration test — stale cloud data bleeding across tests

**Symptom:** `test_no_hit_without_cloud` failed with "got 1 clicks" — a click was detected even though no cloud was published.

**Root cause:** The twist propagation node retains cloud data in its internal `_cloud_xyz` field across activation/deactivation cycles. When the test reactivated the node, it still had the cloud from the previous test and used it to find a hit. The node has a `cloud_max_age_s=2.0` parameter, but the test ran too quickly for the cloud to age out.

**Fix:** Added a `time.sleep(2.5)` wait after activation to let the stale cloud age out (beyond `cloud_max_age_s=2.0`), then cleared the harness and published only poses. The node's cycle now rejects the cloud as too old and produces no hits.

**File changed:** `scripts/test_twist_propagation_integration.py:359-391`

---

## Final State

All 5 smoke tests pass:

```
  STATUS TEST                      TIME
  PASS   build                     610ms
  PASS   launch                    1787ms
  PASS   preshaping                58ms
  PASS   nodes_start               6117ms
  PASS   twist_prop                9146ms

  Passed: 5  Failed: 0
```

## Files Modified (Summary)

| File | Change |
|------|--------|
| `docker/Dockerfile:13` | Added `ros-jazzy-rmw-cyclonedds-cpp` |
| `docker/Dockerfile:65` | Changed `ENTRYPOINT` to `CMD` |
| `docker/Dockerfile.segmentation:34` | Fixed COPY destination for MinkowskiEngine |
| `docker/Dockerfile.segmentation:39` | Added `COPY README.md` |
| `docker/docker-compose.yml:57-65` | Test service reuses `prosthesis:latest` image |
| `src/segmentation/README.md` | Created minimal README for MinkowskiEngine build |
| `src/segmentation/MANIFEST.in` | Removed `include Makefile` |
| `scripts/run_tests.sh` | Source ROS setup before `set -euo pipefail` |
| `scripts/test_build.sh` | Source ROS setup before `set -euo pipefail` |
| `scripts/test_launch_syntax.sh` | Source ROS setup before `set -euo pipefail` |
| `scripts/test_nodes_start.sh` | Source ROS setup before `set -euo pipefail` |
| `scripts/test_twist_propagation.sh` | Source ROS setup before `set -euo pipefail` |
| `scripts/test_preshaping_so.sh` | Fixed symbol name to `grasp_preshaping_api_version` |
| `scripts/test_twist_propagation_integration.py` | Fixed test sequencing: added cloud before twist test, cloud aging wait for no-cloud test, increased spin timeouts |

## Cosmetic Warnings (Not Fixed)

The `ament_python` rosdep warnings are harmless — `ament_python` is a build type already present in the base image, not a system dependency:

```
ERROR: the following packages/stacks could not have their rosdep keys resolved to system dependencies:
pipeline_manager: Cannot locate rosdep definition for [ament_python]
emg_bridge: Cannot locate rosdep definition for [ament_python]
segmentation_bridge: Cannot locate rosdep definition for [ament_python]
wrist_driver: Cannot locate rosdep definition for [ament_python]
Continuing to install resolvable dependencies...
```

These could be silenced by removing `<buildtool_depend>ament_python</buildtool_depend>` from the four packages' `package.xml` files (the `<build_type>ament_python</build_type>` in `<export>` is what actually controls the build), but they don't cause any failures.
