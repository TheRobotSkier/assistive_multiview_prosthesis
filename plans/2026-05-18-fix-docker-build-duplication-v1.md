# Fix: Docker Build Rebuilding Prosthesis Image Multiple Times

## Objective

Diagnose and resolve why `make build` (which runs `podman-compose build` or `docker compose build`) is building the prosthesis image multiple times instead of once, causing the build to be very slow.

## Root Cause Analysis

The problem is in `docker/docker-compose.yml`. The `build` target runs `cd docker && $(COMPOSE) build`, which builds **all services that have a `build:` block**. However, the real issue is that **6 services extend `prosthesis`**, and each one triggers a separate build of the same image because of how `extends` interacts with compose build:

### Services in docker-compose.yml:

| Service | Build Config | Extends | Profile |
|---|---|---|---|
| `prosthesis` | Has `build:` block (Dockerfile) | — | default |
| `interactive` | No `build:` block | `prosthesis` | default |
| `rviz` | No `build:` block | `prosthesis` | `rviz` |
| `prosthesis-hw` | No `build:` block | `prosthesis` | `hardware` |
| `grasp_test` | No `build:` block | `prosthesis` | `grasp_test` |
| `digital_twin` | No `build:` block | `prosthesis` | `digital_twin` |
| `test` | No `build:` block, uses `image: prosthesis:latest` | `prosthesis` | `test` |
| `segmentation` | Has own `build:` block (Dockerfile.segmentation) | — | `segmentation` |

### The Core Problem

When `extends` is used in Docker Compose, the child service **inherits the `build:` configuration from the parent**. This means:

1. **`prosthesis`** — builds the image (correct)
2. **`interactive`** — inherits `build:` from `prosthesis`, triggers another build
3. **`rviz`** — inherits `build:` from `prosthesis`, triggers another build
4. **`prosthesis-hw`** — inherits `build:` from `prosthesis`, triggers another build
5. **`grasp_test`** — inherits `build:` from `prosthesis`, triggers another build
6. **`digital_twin`** — inherits `build:` from `prosthesis`, triggers another build

That's potentially **6 builds of the same Dockerfile** instead of 1. Each one does the full `rosdep install` + `colcon build` which is the expensive part.

The `test` service avoids this because it explicitly sets `image: prosthesis:latest` which tells compose to just use the already-built image tag.

## Implementation Plan

- [ ] **Step 1. Add explicit `image: prosthesis:latest` to the `prosthesis` service** in `docker/docker-compose.yml:15-41`. This gives the built image a stable tag name that child services can reference. Without an explicit `image:`, compose auto-generates a per-service tag (e.g., `docker_prosthesis`), and each extended service gets its own auto-generated tag, triggering separate builds.

- [ ] **Step 2. Add `image: prosthesis:latest` to each service that `extends: prosthesis`** — specifically `interactive`, `rviz`, `prosthesis-hw`, `grasp_test`, and `digital_twin`. This tells compose "use this image" instead of "build this image". The `extends` keyword copies the `build:` block from the parent; setting `image:` explicitly overrides this behavior and tells compose to reuse the already-built image.

- [ ] **Step 3. Verify the `test` service is already correct** — it already has `image: prosthesis:latest` at line 151, so it should already be fine. No change needed.

- [ ] **Step 4. (Optional) Add `image:` to the `segmentation` service** at `docker/docker-compose.yml:79-93` for consistency, e.g. `image: segmentation:latest`. This doesn't affect the multi-build issue but gives a stable tag.

### Why This Works

Docker Compose's `extends` copies all keys from the parent service, including `build:`. When a child service has `build:` set (inherited or explicit), compose will build it. By adding `image: prosthesis:latest` to each child:

- Compose sees `image:` is set → it uses that image tag
- Since the `prosthesis` service already built and tagged that image → no rebuild
- The `build:` key is still inherited but compose prefers the existing image

## Verification Criteria

- [ ] `make build` completes in roughly 1/N of the previous time (where N was the number of services being rebuilt)
- [ ] `podman images` or `docker images` shows only one `prosthesis:latest` image built
- [ ] `make up` still works correctly — all services start and can run
- [ ] `make test` still passes

## Potential Risks and Mitigations

1. **`extends` + `image:` interaction varies by compose version**
   Mitigation: Test with both `podman-compose` and `docker compose` to ensure consistent behavior. The `image:` override is well-documented in Docker Compose spec.

2. **Stale image if `prosthesis` service build is skipped**
   Mitigation: The `prosthesis` base service still has `build:` + `image:`, so it will always build and tag. Children reference the tag.

3. **`prosthesis-hw` in docker-compose.hw.yml may need the same treatment**
   Mitigation: That file only adds device mappings and environment, it doesn't add `build:`, so it should be fine. But worth verifying.

## Alternative Approaches

1. **Use `build.target` or multi-stage builds**: Define the prosthesis image as a build stage and have all services reference it. More complex, no real benefit over the `image:` fix.

2. **Remove `extends` and use YAML anchors**: Replace `extends` with YAML anchors (`&prosthesis_defaults` / `<<: *prosthesis_defaults`). This avoids the `build:` inheritance entirely. However, this is a larger refactor and `extends` is the standard compose pattern.

3. **Use `--no-build` on child services**: Not practical — compose doesn't support per-service build flags.

## Recommended Approach

**Option 2 (YAML anchors) is the cleanest long-term fix** because it avoids the `extends` + `build:` inheritance problem entirely. However, **the `image:` fix (Steps 1-2) is the quickest, lowest-risk change** and should be applied first. If the project grows more services, consider migrating to anchors later.
