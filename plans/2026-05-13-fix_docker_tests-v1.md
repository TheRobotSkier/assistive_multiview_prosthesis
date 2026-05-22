# Fix Docker Build Failures, Test Copying, and Container Stale Name Issues

## Objective

Resolve three distinct problems preventing the Docker-based test pipeline from working:

1. **Build failure** — `grasp_preshaping` fails to compile due to an undeclared `target_pose` variable.
2. **Tests not copied into container** — The `tests/` directory is never `COPY`-ed into the Docker image, so `/prosthesis_ws/tests/` does not exist at runtime.
3. **Stale container name collisions** — `podman-compose up -d` reuses old container names, causing "already in use" errors. Additionally, the `:latest` tag concern is evaluated.

---

## Issue Analysis

### Issue 1: Compilation Error — `target_pose` undeclared

**Source**: `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:448`

At line 448, the code block assigns fields to `target_pose` and publishes it via `target_hand_pose_pub_`, but `target_pose` is never declared as a local variable. The publisher type is `geometry_msgs::msg::Pose` (line 138, 514), and the code uses `geometry_msgs::msg::PoseStamped` elsewhere (included at line 12), but `geometry_msgs/msg/pose.hpp` is not included and no local `geometry_msgs::msg::Pose target_pose;` declaration exists within the `try_handle_direct_request` method.

**Fix**: Add a local `geometry_msgs::msg::Pose target_pose;` declaration inside the block at line 447 (just before line 448), and ensure the `geometry_msgs/msg/pose.hpp` header is included (or rely on the existing `pose_stamped.hpp` which may or may not transitively include it — safest to add the explicit include).

### Issue 2: Tests Not Copied into Docker Image

**Source**: `docker/Dockerfile:40-43`

The Dockerfile copies `src/`, `config/`, `rviz/`, and `scripts/` into `/prosthesis_ws/`, but **never copies `tests/`**. The test scripts reference paths like `/prosthesis_ws/tests/test1_software_verification/run_tier_b.py` (see `tests/test1_software_verification/run_tier_b.py:36`), which simply do not exist inside the container.

**Fix**: Add `COPY tests/ tests/` to the Dockerfile alongside the other COPY directives.

### Issue 3: Stale Container Name Collisions

**Source**: `docker/docker-compose.yml` — all services use static names like `docker_prosthesis_1`, `docker_segmentation_1`, `docker_test_1`.

When `podman-compose up -d` is run after a previous run without `podman-compose down`, the old containers still exist with those names. `podman-compose` tries to `run` new containers with the same names, fails, then falls back to `start`-ing the old ones. This means:
- Code changes are NOT picked up (old container image is started).
- The `test` container re-runs the old `run_tests.sh` from the stale image.

**The `:latest` tag concern**: The `:latest` tag itself is not the root cause. The issue is that `podman-compose up` does not automatically rebuild images before starting. The `Makefile` `up` target only runs `$(COMPOSE) up -d` without a prior build. The real fix is to either run `make build` before `make up`, or modify the `up` target to build first.

**Fix**: 
- Add `--build` flag to the `up` target in the Makefile, or add `down` before `up`, or use `--force-recreate`.
- Optionally add `--rm` for the test service to auto-remove containers after exit.

---

## Implementation Plan

### Fix 1: Compilation Error — `target_pose` undeclared

- [ ] **1a.** Add `#include "geometry_msgs/msg/pose.hpp"` to the includes section of `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` (near line 12, alongside the existing `pose_stamped.hpp` include). This ensures the `geometry_msgs::msg::Pose` type is fully available even if the transitive include is not guaranteed.

- [ ] **1b.** Add a local variable declaration `geometry_msgs::msg::Pose target_pose;` at the beginning of the block that starts at line 447 (just before `target_pose.position.x = ...` on line 448). This resolves the "not declared in this scope" error.

### Fix 2: Tests Not Copied into Docker Image

- [ ] **2a.** Add `COPY tests/ tests/` to `docker/Dockerfile` after the existing COPY directives (after line 43, before the `RUN chmod +x` line). This ensures the test scripts and data are available at `/prosthesis_ws/tests/` inside the container.

- [ ] **2b.** Ensure the test runner script `scripts/run_tests.sh` references tests at the correct path. Currently it runs smoke tests from `scripts/` (lines 48-52), not from `tests/`. The user's command `python /prosthesis_ws/tests/test1_software_verification/run_tier_b.py --method both` is a direct invocation, not via `run_tests.sh`. No change needed to `run_tests.sh` unless integration is desired.

### Fix 3: Stale Container Collisions and `:latest` Tag

- [ ] **3a.** Modify the `up` target in the `Makefile` to include `--build` so images are always rebuilt before starting: change `cd $(COMPOSE_DIR) && $(COMPOSE) up -d` to `cd $(COMPOSE_DIR) && $(COMPOSE) up -d --build`. This ensures code changes are always picked up.

- [ ] **3b.** Add `--force-recreate` to the `up` target to ensure old containers are replaced even if the image hasn't changed: `cd $(COMPOSE_DIR) && $(COMPOSE) up -d --build --force-recreate`. This eliminates the "name already in use" errors.

- [ ] **3c.** Modify the `test` target in the Makefile to add `--rm` flag for the test container run, ensuring the test container is removed after execution: change `cd $(COMPOSE_DIR) && $(COMPOSE) run --rm test` (it already has `--rm`, so verify this is correct — yes, line 40 already uses `--rm`).

- [ ] **3d.** (Optional) Add a `Makefile` convenience target `reup` that does `down` + `build` + `up` for a clean slate, or document that `make down && make build && make up` is the recommended workflow when experiencing stale container issues.

### Fix 4: The `:latest` Tag

- [ ] **4a.** The `:latest` tag is a red herring — it is the **default** tag in Docker/Podman and behaves correctly. The actual issue is that `podman-compose up` without `--build` reuses cached images. **No change needed** to remove `:latest`. The fix in 3a/3b (adding `--build --force-recreate`) addresses the underlying problem. If desired for clarity, the tags can be kept as-is or changed to a specific version — but this is cosmetic, not functional.

---

## Verification Criteria

- [ ] `colcon build` succeeds for `grasp_preshaping` package with zero errors (the `target_pose` compilation error is resolved)
- [ ] `podman build` of the prosthesis image succeeds (all packages build, including `grasp_preshaping`)
- [ ] `podman-compose up -d` starts without "container name already in use" errors
- [ ] The file `/prosthesis_ws/tests/test1_software_verification/run_tier_b.py` exists inside the running container (verify with `make shell` then `ls /prosthesis_ws/tests/`)
- [ ] `python /prosthesis_ws/tests/test1_software_verification/run_tier_b.py --method both` can be executed inside the container (may still fail if mock launch is not running, but the script itself should be found and importable)

## Potential Risks and Mitigations

1. **Transitive include may already provide `geometry_msgs::msg::Pose`**
   Mitigation: Adding the explicit `#include "geometry_msgs/msg/pose.hpp"` is harmless (header guards prevent double-inclusion) and ensures portability across ROS 2 distributions.

2. **Adding `--build` to `up` makes startup slower**
   Mitigation: Docker layer caching means unchanged layers rebuild instantly. Only when `src/` or `tests/` change will the full rebuild occur. The trade-off of correctness vs. speed favors correctness.

3. **`--force-recreate` may disrupt running workloads**
   Mitigation: This only affects `make up`, which is used to start the development environment. Production deployments would use a different workflow.

4. **Test data files (`.npz`, `.stl`) in `tests/` may bloat the Docker image**
   Mitigation: The test data is needed for the tests to run inside Docker. If image size becomes a concern, a multi-stage build or `.dockerignore` exclusion with runtime mount could be considered later.

## Alternative Approaches

1. **For Fix 3 — Use `podman-compose down` before `up` instead of `--build --force-recreate`**: This would remove all containers first, but is more destructive and slower. The `--build --force-recreate` approach is cleaner.

2. **For Fix 2 — Mount `tests/` as a volume instead of COPY**: Add `-v ./tests:/prosthesis_ws/tests` to docker-compose.yml. This avoids rebuilding on test changes but means tests are not available in standalone images. Less portable but faster iteration during development.

3. **For Fix 1 — Use `auto` type deduction**: Replace the explicit `geometry_msgs::msg::Pose target_pose;` with `auto target_pose = geometry_msgs::msg::Pose();`. Functionally equivalent; explicit type is clearer for readability.
