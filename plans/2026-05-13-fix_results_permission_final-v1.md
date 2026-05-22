# Fix Results Permission Error (Rootless Podman UID Mapping)

## Objective

Fix `PermissionError` when `run_tier_b.py` writes results to the bind-mounted `tests/` directory. Rootless podman maps container UIDs differently from host UIDs, so even though the host files are owned by `daniel:1000`, the container's `prosthesis` user (also UID 1000) may not have write access.

## Root Cause

Rootless podman uses user namespace mapping. The container's UID 1000 does NOT necessarily map to the host's UID 1000. The bind mount preserves host permissions, so the `prosthesis` user inside the container can't write to the host-mounted `results/` directory.

## Implementation Plan

- [ ] **1. Add `RESULTS_DIR` env var support to `run_tier_b.py`.** At line 47, change `RESULTS_DIR` to check for an env var first:
  ```python
  RESULTS_DIR = os.environ.get("RESULTS_DIR", os.path.join(SCRIPT_DIR, "results"))
  ```
  This is a one-line change that preserves backward compatibility.

- [ ] **2. Add `RESULTS_DIR` env var to docker-compose.yml.** In the `prosthesis` service environment section, add `RESULTS_DIR=/tmp/test_results`. This writes results to `/tmp/` inside the container, which is always writable.

- [ ] **3. Add a dedicated volume mount for results.** In docker-compose.yml, add `../tests/test1_software_verification/results:/tmp/test_results:rw` to the volumes. This maps the host results directory to the container's `/tmp/test_results`, bypassing the UID mapping issue because podman handles `/tmp/` mounts differently. Actually — this has the same UID mapping problem.

**Better approach**: Use a named volume or just mount the results directory with `:z` relabeling. But the simplest approach that actually works with rootless podman is:

**Simplest fix**: Change the host's `results/` directory permissions to `777` (world-writable). This is a test output directory — not sensitive.

- [ ] **Alternative (recommended): Use `os.environ.get` for RESULTS_DIR + write to `/tmp/test_results` in Docker + add a separate bind mount.**

Actually, the truly simplest fix that works:

- [ ] **1. Change `run_tier_b.py` line 47** to support env var override:
  ```python
  RESULTS_DIR = os.environ.get("RESULTS_DIR", os.path.join(SCRIPT_DIR, "results"))
  ```

- [ ] **2. Set `RESULTS_DIR=/tmp/test_results`** in the Docker environment (docker-compose.yml and Dockerfile).

- [ ] **3. Add volume mount** `../tests/test1_software_verification/results:/tmp/test_results:rw` in docker-compose.yml.

Wait — this still has the same UID mapping issue with the bind mount.

**The real simplest fix**: Just `chmod 777` the results directory on the host. One command, no code changes.

## Final Recommendation

Run on the host:
```bash
chmod -R 777 tests/test1_software_verification/results/
```

If that's not acceptable, the env var approach with writing to a non-mounted path inside the container is the alternative, but results won't automatically appear on the host.
