# Fix Segmentation Service: Make CUDA Work via `make up`

## Objective

The segmentation service falls back to CPU when started via `make up` because the `up` target in the Makefile uses the **Docker** GPU compose override (`docker-compose.segmentation.cuda.yml`) instead of the **Podman** one (`docker-compose.segmentation.podman-gpu.yml`). On this WSL2 + Podman system, the Docker override is a no-op (no `runtime: nvidia`), so `torch.cuda.is_available()` returns `False` inside the container.

The previous session already fixed the `typing_extensions` crash (baked `>=4.6` into the Dockerfile) and verified CUDA works with proper GPU passthrough. The remaining problem is purely a **Makefile routing bug**.

## Root Cause Analysis

Three issues compound:

1. **`make up` hardcodes the Docker GPU override** — `Makefile:108` uses `$(COMPOSE_SEGMENTATION_CUDA)` which is `-f docker-compose.yml -f docker-compose.segmentation.cuda.yml`. This Docker override uses `runtime: nvidia` and `deploy.resources.reservations.devices`, which Podman ignores.

2. **`ifeq ($(CONTAINER_BACKEND),podman)` checks the wrong variable** — At `Makefile:91`, `Makefile:177`, the conditional checks `CONTAINER_BACKEND`, but in auto-detect mode (no explicit `CONTAINER_BACKEND=`), this variable is empty. The auto-detect logic only sets `DOCKER_CMD=podman`, not `CONTAINER_BACKEND=podman`.

3. **CDI spec may not persist across WSL2 restarts** — The `sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml` from the previous session may have been lost if WSL2 was restarted (which it was — the user's terminal history shows a fresh `make build; make up` cycle).

## Implementation Plan

- [x] **Task 1. Add `COMPOSE_CUDA_RUNTIME` variable to Makefile** — Add after line 55, a variable that selects the correct GPU compose override based on `DOCKER_CMD` (which is always set correctly by auto-detect):
  ```
  ifeq ($(DOCKER_CMD),podman)
    COMPOSE_CUDA_RUNTIME := $(COMPOSE_SEGMENTATION_CUDA_PODMAN)
  else
    COMPOSE_CUDA_RUNTIME := $(COMPOSE_SEGMENTATION_CUDA)
  endif
  ```
  Rationale: `DOCKER_CMD` is always set correctly by both the explicit and auto-detect paths. `CONTAINER_BACKEND` is only set when explicitly provided.

- [x] **Task 2. Fix `up` target to use `COMPOSE_CUDA_RUNTIME`** — Change `Makefile:108` from:
  ```
  cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA) up -d prosthesis segmentation-cuda
  ```
  to:
  ```
  cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_CUDA_RUNTIME) --profile segmentation-cuda up -d prosthesis segmentation-cuda
  ```
  Note: also add `--profile segmentation-cuda` which is required for the service to start. The current `up` target is missing it.

- [x] **Task 3. Fix `segmentation-cuda` target to use `COMPOSE_CUDA_RUNTIME`** — Replace the `ifeq ($(CONTAINER_BACKEND),podman)` block at lines 91-95 with a simple:
  ```
  cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_CUDA_RUNTIME) --profile segmentation-cuda up -d segmentation-cuda
  ```

- [x] **Task 4. Fix `validate-segmentation-config` target** — Replace the `ifeq ($(CONTAINER_BACKEND),podman)` block at lines 177-181 with `ifeq ($(DOCKER_CMD),podman)`.

- [x] **Task 5. Ensure CDI spec is generated before container start** — Add a guard to the `segmentation-cuda` target that checks for the CDI spec and regenerates it if missing. This is needed because `/var/run/cdi/nvidia.yaml` does not persist across WSL2 restarts. Add before the compose up:
  ```makefile
  ifeq ($(DOCKER_CMD),podman)
    @test -f /var/run/cdi/nvidia.yaml || { echo "Regenerating NVIDIA CDI spec..."; sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml; }
  endif
  ```

- [x] **Task 6. Rebuild the segmentation-cuda image** — Skipped: image already has `typing_extensions>=4.6` baked in from the previous session's Dockerfile edit.

- [x] **Task 7. Start segmentation-cuda with `make up` and verify** — Verified: `curl http://127.0.0.1:5678/health` returns `"device": "cuda"`, `"torch_cuda_available": true`, `"model_ready": true`.

## Verification Criteria

- [ ] `make segmentation-status` shows the container running and healthy
- [ ] `curl -sf http://127.0.0.1:5678/health` returns `"device": "cuda"`, `"torch_cuda_available": true`, `"model_ready": true`
- [ ] `make up` uses the Podman GPU override (visible in the compose output)
- [ ] The Makefile no longer depends on `CONTAINER_BACKEND` being set explicitly for GPU passthrough

## Potential Risks and Mitigations

1. **CDI spec regeneration requires `sudo`**
   Mitigation: The Makefile guard prints a clear message. If sudo isn't available, the container will still start but fall back to CPU (graceful degradation, not a crash).

2. **`/var/run/cdi/nvidia.yaml` lost on every WSL2 restart**
   Mitigation: The guard in Task 5 auto-regenerates it. For a more permanent fix, the CDI spec could be generated at boot via a WSL2 startup script, but that's outside this plan's scope.

3. **`podman-compose` may not support the `devices:` key in the override**
   Mitigation: This was verified working in the previous session. The `docker-compose.segmentation.podman-gpu.yml` uses `devices: [nvidia.com/gpu=all]` which podman-compose translates to `--device nvidia.com/gpu=all`.

4. **The `--profile` flag might not work with `podman-compose` in the `up` target**
   Mitigation: The `segmentation-cuda` target already uses `--profile segmentation-cuda` with podman-compose and it works. The `up` target just needs the same flag.

## Alternative Approaches

1. **Roll back to CPU-only**: Skip all GPU work, use `segmentation:cpu` image. Trade-off: inference is ~10x slower, but no GPU/NVIDIA/CDI complexity. This is the fallback if GPU passthrough proves unreliable on WSL2.

2. **Switch from podman-compose to direct `podman run`**: The Makefile could run `podman run` directly instead of via compose, giving full control over `--device` flags. Trade-off: more verbose Makefile, loses compose orchestration, but more reliable on WSL2+Podman.

3. **Use Docker instead of Podman on WSL2**: Docker Desktop for WSL2 has native NVIDIA runtime support. Trade-off: Docker Desktop is heavier, requires license for commercial use, but GPU passthrough "just works".
