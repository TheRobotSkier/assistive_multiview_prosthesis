# Mount Tests Directory for Bidirectional Result Transfer

## Objective

Ensure test results written inside the Docker container at `/prosthesis_ws/tests/test1_software_verification/results/` are visible on the host at `tests/test1_software_verification/results/`.

## Root Cause

`docker/docker-compose.yml:28-30` only mounts `/tmp/.X11-unix` for X11 forwarding. The `tests/` directory is `COPY`'d into the image at build time (`docker/Dockerfile:44`) — this is a one-way copy. Results written inside the container stay inside the container and are lost on `--force-recreate`.

## Implementation Plan

- [ ] **1. Add volume mount for `tests/` in `docker-compose.yml`.** In the `prosthesis` service's `volumes` section (line 28-30), add:
  ```
  - ../tests:/prosthesis_ws/tests:rw
  ```
  This bind-mounts the host `tests/` directory over the container's `/prosthesis_ws/tests/`, making results written inside the container immediately visible on the host.

- [ ] **2. Ensure the `results/` directory exists on the host.** The `tests/test1_software_verification/results/` directory already exists on the host with existing CSV files, so no action needed. But if new test directories are added, they'll need their own `results/` subdirectory.

- [ ] **3. Also add the same mount to `prosthesis-hw` service.** Since `prosthesis-hw` extends `prosthesis`, it inherits the volumes automatically — no additional change needed.

- [ ] **4. Add the same mount to the `test` service.** The `test` service (line 57-65) runs smoke tests and exits. It also needs the results mount. Add `volumes: - ../tests:/prosthesis_ws/tests:rw` to the test service.

- [ ] **5. The `chown` in Dockerfile is still needed.** Even with the bind mount, the host files need to be writable by the container's `prosthesis` user. The bind mount uses the host's UID/GID. Since the Dockerfile maps `prosthesis` to UID 1000 (same as the host user `daniel`), permissions should match. But the existing `results/` CSV files may be owned by `daniel:daniel` (UID 1000) which matches — so this should work. The `chown` in the Dockerfile is still useful as a safety net for the non-mounted case.

## Verification Criteria

- [ ] Run `run_tier_b.py --method service` inside the container
- [ ] After completion, `ls tests/test1_software_verification/results/tier_b_latency_results.csv` shows the file on the host
- [ ] The CSV file is readable and contains the expected rows

## Potential Risks and Mitigations

1. **UID mismatch**: If the host user UID is not 1000, the `prosthesis` user inside the container (UID 1000) won't have write permission to the bind-mounted host directory.
   Mitigation: The Dockerfile already uses `ARG USER_UID=1000` and the host user `daniel` is UID 1000 (standard first user on Ubuntu). The `make build` passes `USER_UID=${USER_UID:-1000}`.

2. **Stale test data**: The bind mount replaces the `COPY`'d tests with the host's live version. If the host has different test files than what was baked into the image, you get the host version.
   Mitigation: This is actually desirable — you always want the latest test scripts, not the build-time snapshot.

3. **Build cache**: The `COPY tests/ tests/` in the Dockerfile still happens, but the bind mount overlays it at runtime. The COPY is still useful for the `test` service which runs without a shell.
   Mitigation: No issue — the bind mount takes precedence at runtime.
