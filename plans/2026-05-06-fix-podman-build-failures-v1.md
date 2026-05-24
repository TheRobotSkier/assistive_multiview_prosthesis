# Fix Podman Build Failures

## Objective

Fix three issues preventing `podman-compose` from building and running the prosthesis system:
1. Segmentation image build fails (missing `__init__.py` in build context)
2. Podman tries to pull images from `quay.io` instead of building locally
3. Test service fails due to permissions and entrypoint issues

## Root Cause Analysis

### Issue 1: Segmentation build — `FileNotFoundError: MinkowskiEngine/__init__.py`

The file `src/segmentation/MinkowskiEngine/__init__.py` **exists on disk** (confirmed at line 25: `__version__ = "0.5.4"`), but the Docker build fails to find it at `COPY MinkowskiEngine/ /MinkowskiEngine-src/`.

**Root cause**: The file is likely **not tracked by git**. When podman builds with a context directory, it may use git-aware context filtering (especially if using `podman build` with Buildah). Files not committed to git are excluded from the build context.

**Evidence**: No `.gitignore` or `.dockerignore` files exist anywhere in the project, yet the file is missing from the build. The most likely explanation is that `__init__.py` files were never `git add`-ed.

### Issue 2: Podman pulls from `quay.io` instead of building locally

```
Trying to pull quay.io/osrf/ros:jazzy-desktop...
Error: unauthorized: access to the requested resource is not authorized
```

Then:
```
Trying to pull quay.io/docker_prosthesis:latest...
Error: StatusCode: 404
```

**Root cause**: Podman's default registry is `quay.io`. When `podman-compose up` runs, it tries to pull images first before building. The `osrf/ros:jazzy-desktop` base image lives on Docker Hub (`docker.io`), not `quay.io`. And the locally-built images (`docker_prosthesis`, `docker_test`, etc.) don't exist on any registry.

**Fix**: Configure podman's registry, add `image:` tags to the compose file, and use explicit `podman-compose build` before `up`.

### Issue 3: Test service runs as non-root

The Dockerfile switches to `USER prosthesis` at line 52. The test scripts need write access to `/prosthesis_ws/` for `colcon build`. The test service needs to override the user.

## Implementation Plan

### Phase 1: Fix MinkowskiEngine build context

- [ ] **1.1** Ensure all `__init__.py` files in `src/segmentation/MinkowskiEngine/` are tracked by git
  - Run `git add src/segmentation/MinkowskiEngine/__init__.py` and check for any other missing `__init__.py` files in `utils/` and `modules/` subdirectories
  - Rationale: Podman build context may exclude untracked files; these must be committed

- [ ] **1.2** Add a `.dockerignore` to `src/segmentation/` that explicitly includes everything needed
  - Create `src/segmentation/.dockerignore` with explicit allowlist: `MinkowskiEngine/`, `src/`, `pybind/`, `setup.minkowski.py`, `MANIFEST.in`, `nodes/`, `docker/`, `segmentation_bridge/`
  - Rationale: Ensures podman doesn't filter files unexpectedly; also excludes `setup.py` (the ROS one) from the segmentation build context to avoid confusion

### Phase 2: Fix podman registry and image naming

- [ ] **2.1** Add explicit `image:` tags to all services in `docker-compose.yml`
  - Add `image: prosthesis` to the `prosthesis` service
  - Add `image: prosthesis-hw` to the `prosthesis-hw` service  
  - Add `image: segmentation` to the `segmentation` service
  - Add `image: prosthesis-test` to the `test` service
  - Rationale: Podman-compose needs explicit image names to know these are locally-built images, not ones to pull from a registry

- [ ] **2.2** Fully qualify the base image in `docker/Dockerfile`
  - Change `FROM osrf/ros:jazzy-desktop` to `FROM docker.io/osrf/ros:jazzy-desktop`
  - Rationale: Forces podman to pull from Docker Hub instead of defaulting to quay.io

- [ ] **2.3** Add a `Makefile` at the project root with podman-friendly commands
  - Targets: `build`, `up`, `up-hw`, `test`, `down`, `clean`
  - Each target runs the correct `podman-compose` commands in the right order
  - Rationale: Abstracts away docker vs podman-compose differences, ensures correct build-before-run ordering

### Phase 3: Fix test service

- [ ] **3.1** Add `user: "0:0"` to the test service in `docker-compose.yml`
  - The test runs `rm -rf build/ install/ log/` and `colcon build` which needs root
  - Rationale: Test container is ephemeral; running as root is fine

- [ ] **3.2** Change test service `entrypoint` to `command`
  - Replace `entrypoint: ["/bin/bash", "/prosthesis_ws/scripts/run_tests.sh"]` with `command: ["/prosthesis_ws/scripts/run_tests.sh"]`
  - The Dockerfile already has `ENTRYPOINT ["/bin/bash"]`, so `command` will be passed as args
  - Rationale: `podman-compose run` handles `command` overrides more reliably than `entrypoint` overrides

- [ ] **3.3** Simplify `test_build.sh` to not do a clean rebuild
  - Instead of `rm -rf build/ install/ log/ && colcon build`, just verify that the install directory exists and has the expected packages
  - The Dockerfile already does `colcon build` during image build — a clean rebuild in the test is expensive and unnecessary
  - Rationale: Faster tests, avoids permission issues, tests the actual image content

## Verification Criteria

- [ ] `podman-compose build` succeeds for all services (prosthesis, segmentation, test)
- [ ] `podman-compose up` starts the prosthesis container successfully
- [ ] `podman-compose run --rm test` exits with code 0
- [ ] No `quay.io` pull attempts in the build output
- [ ] `make build && make test` works from the project root

## Potential Risks and Mitigations

1. **`__init__.py` still missing after `git add`**
   - Risk: The file might have been in a submodule or generated by a build step
   - Mitigation: If `git add` fails (file doesn't exist), the file can be recreated from the MinkowskiEngine v0.5.4 GitHub release

2. **Podman registry configuration differs between systems**
   - Risk: Some systems may have `quay.io` configured as default, others `docker.io`
   - Mitigation: The `docker.io/` prefix in the FROM line forces the correct registry regardless of system config

3. **`podman-compose` version differences**
   - Risk: Older versions of podman-compose may not support `extends` or `profiles` correctly
   - Mitigation: The Makefile documents the minimum required version and provides fallback commands

## Alternative Approaches

1. **Use Docker instead of Podman**: Install Docker CE and use `docker compose` directly. Avoids all registry and context issues. Trade-off: requires Docker daemon, which you may not want.

2. **Use Buildah directly instead of podman-compose**: Build images with `buildah bud` and run with `podman run`. More control but loses the compose orchestration. Trade-off: more manual, no service dependency management.

3. **Use a `.env` file to switch between docker and podman-compose**: Add a `COMPOSE_CMD` variable and a wrapper script. Trade-off: adds complexity for little gain when you only use podman.
