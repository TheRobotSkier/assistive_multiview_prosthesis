# Implementation Plan: Step 4 — Docker Service Definition

**Beads Issue:** `docker-service`  
**Type:** task  
**Priority:** 2 (blocked by `launch-digital-twin` and `bridge-nodes`; blocks `integration-test`)  
**Estimated Effort:** Small

---

## Objective

Add a new `digital_twin` service to `docker-compose.linux-podman.yml` and `docker-compose.windows.yml` so the entire digital twin pipeline can be started with a single `docker compose` command.

---

## Acceptance Criteria

- [ ] `digital_twin` service exists in both `docker-compose.linux-podman.yml` and `docker-compose.windows.yml`.
- [ ] Service extends `miahand_ros2` (reuses build, network, volumes, X11, etc.).
- [ ] Service has profile `standalone`.
- [ ] Service command sources the workspace and launches `digital_twin_launch.py`.
- [ ] `docker compose --profile standalone config` validates without errors.
- [ ] Documentation comment above the service explains prerequisites (`rust_build`, `segmentation_inference`, `multiview_full`).

---

## Implementation Details

### 4.1 Linux / Podman Compose

Add to `docker-compose.linux-podman.yml` after the existing `full_system_test` service (or in the services list):

```yaml
  # ── Digital Twin: real cameras + simulated Mia Hand ───────────────────────
  # Visualizes real pointclouds, runs segmentation, computes grasps, and
  # drives a simulated Mia Hand as a digital twin.
  #
  # Prerequisites:
  #   1. rust_build   (one-shot) — ensures libgrasp_preshaping.so exists.
  #   2. segmentation_inference (standalone or full_system) — HTTP server on :5678.
  #   3. multiview_full (full_system) — publishes /fused_pointcloud from real D435s.
  #
  # Usage:
  #   docker compose --profile standalone up digital_twin
  #
  # After RViz opens:
  #   - Click "Publish Point" tool, click on the object in the pointcloud.
  #   - Call the grasp service:
  #       ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger
  #   - Watch the simulated hand preshape (far) and close (near).
  digital_twin:
    extends: miahand_ros2
    container_name: miahand_digital_twin
    profiles: [standalone]
    environment:
      - USE_TRAJECTORY=${USE_TRAJECTORY:-false}
      - INFERENCE_URL=${INFERENCE_URL:-http://127.0.0.1:5678}
      - SEGMENTATION_CUBEEDGE=${SEGMENTATION_CUBEEDGE:-0.05}
    command: >
      bash -c "
        source install/setup.bash &&
        ros2 launch dev/mujoco/launch/digital_twin_launch.py
          use_trajectory:=${USE_TRAJECTORY:-false}
          inference_url:=${INFERENCE_URL:-http://127.0.0.1:5678}
          segmentation_cubeedge:=${SEGMENTATION_CUBEEDGE:-0.05};
        /bin/bash
      "
```

### 4.2 Windows Compose

Add the equivalent block to `docker-compose.windows.yml`. The main differences are:
- Windows uses `${XAUTHORITY}` differently (already handled by base `miahand_ros2` service).
- No `userns_mode: "keep-id"` (Podman-specific).
- Ensure the same environment variables and command are present.

### 4.3 Validation

Run:
```bash
docker compose --profile standalone config | grep -A 30 "digital_twin:"
```
This should print the resolved service configuration without YAML syntax errors.

---

## Sub-tasks (beads tracking)

1. Add `digital_twin` service to `docker-compose.linux-podman.yml`.
2. Add `digital_twin` service to `docker-compose.windows.yml`.
3. Run `docker compose config` validation.
4. Commit.

---

## Notes / Risks

- **Risk:** `ros2 launch dev/mujoco/launch/digital_twin_launch.py` may fail if the workspace is not built or if `dev/mujoco` is not on `ROS_PACKAGE_PATH`.
  - **Mitigation:** The service sources `install/setup.bash`, which adds the workspace to the path. `dev/mujoco` is not a formal ROS package, so the launch file should be referenced by absolute path: `/miahand_ws/src/dev/mujoco/launch/digital_twin_launch.py`. Update the command accordingly.
- **Risk:** The `extends` keyword requires Docker Compose v2.20+ or Podman Compose 1.0+.
  - **Mitigation:** The project already uses `extends` extensively (e.g. `miahand_description`, `miahand_driver`), so this is safe.
- **Risk:** `127.0.0.1:5678` for inference will not work if the inference server is in a separate container and the digital twin container is not on host network.
  - **Mitigation:** Pass `INFERENCE_URL` explicitly (e.g. `http://host.docker.internal:5678` on Docker Desktop, or the host bridge IP on Linux). Document this in the runbook.
