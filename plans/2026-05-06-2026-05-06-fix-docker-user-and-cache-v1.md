# Fix Docker Build Failures — User Creation + Cache Busting

## Objective

Fix two remaining build errors:
1. **User creation fails** — `osrf/ros:jazzy-desktop` already has a `ubuntu` user with UID 1000, so `useradd` fails
2. **MinkowskiEngine `__init__.py` not found** — Docker is using a cached layer from before the file was tracked; need to bust the cache
3. **Makefile improvement** — run `up` detached, add `make logs` for following output

## Implementation Plan

### Phase 1: Fix Dockerfile User Creation

- [ ] **1.1** Replace the user creation RUN step in `docker/Dockerfile` lines 24-26

  **Current** (broken):
  ```dockerfile
  RUN groupadd -g ${USER_GID} prosthesis || true \
      && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash prosthesis \
      && usermod -aG dialout prosthesis
  ```

  **Fixed**: The `osrf/ros:jazzy-desktop` image already has a `ubuntu` user with UID 1000. The `|| true` only applies to `groupadd`, not to the entire chain. `useradd` exits with code 4 ("UID not unique"). Fix: detect the existing user and rename it instead of creating a new one.

  ```dockerfile
  RUN if id -u ${USER_UID} >/dev/null 2>&1; then \
          EXISTING_USER=$(getent passwd ${USER_UID} | cut -d: -f1) && \
          usermod -l prosthesis -d /home/prosthesis -m "$EXISTING_USER" 2>/dev/null || true; \
      else \
          groupadd -g ${USER_GID} prosthesis 2>/dev/null || true && \
          useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash prosthesis; \
      fi \
      && usermod -aG dialout prosthesis 2>/dev/null || true
  ```

  This handles three cases:
  - UID 1000 already exists (the `ubuntu` user) → rename it to `prosthesis`
  - UID doesn't exist → create the user normally
  - Either way → add to `dialout` group for serial port access

### Phase 2: Fix MinkowskiEngine Cache Issue

- [ ] **2.1** Bust the Docker build cache for the segmentation image

  The `__init__.py` files exist on disk and are tracked by git, but the Docker build is using **cached layers** from before the files were in the build context. Evidence: every step shows `--> Using cache`.

  **Fix**: Run `podman-compose build --no-cache segmentation` (or `make build-segmentation-no-cache`). This forces a full rebuild without cached layers.

  Alternatively, add a build arg that changes to bust the cache:
  ```bash
  cd docker && podman-compose build --build-arg CACHEBUST=$(date +%s) segmentation
  ```

  But the simplest fix is just `--no-cache` on the segmentation build. This only needs to happen once — future builds will cache correctly since the files are now in the build context.

### Phase 3: Improve Makefile

- [ ] **3.1** Update `Makefile` — run `up` detached, add `make logs`

  **Current**: `make up` runs `podman-compose up` which blocks the terminal.

  **Improved**:
  ```makefile
  up:
      cd $(COMPOSE_DIR) && $(COMPOSE) up -d

  up-hw:
      cd $(COMPOSE_DIR) && $(COMPOSE) --profile hardware up -d

  logs:
      cd $(COMPOSE_DIR) && $(COMPOSE) logs -f

  # Add a no-cache build option for when cache is stale
  rebuild:
      cd $(COMPOSE_DIR) && $(COMPOSE) build --no-cache
  ```

  This way:
  - `make up` starts containers in the background
  - `make logs` follows the output (Ctrl+C to stop following, containers keep running)
  - `make rebuild` does a full clean build when cache is stale

## Verification Criteria

- `make build` succeeds for all services without UID/GID errors
- `make build-segmentation` (with `--no-cache` once) finds `__init__.py` and completes
- `make up` starts containers in detached mode
- `make logs` shows container output

## Potential Risks and Mitigations

1. **`usermod -l` may fail if user has running processes**
   Mitigation: The `2>/dev/null || true` handles this gracefully. In the Docker build context there are no running processes.

2. **Segmentation `--no-cache` build is slow (~10-15 min for MinkowskiEngine compile)**
   Mitigation: Only needed once. Future builds use the correct cache.

3. **Home directory rename from `/home/ubuntu` to `/home/prosthesis`**
   Mitigation: The `-m` flag moves the home directory contents. The `.bashrc` setup at the end of the Dockerfile writes to `/home/prosthesis/.bashrc` which will be correct.
