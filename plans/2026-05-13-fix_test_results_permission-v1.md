# Fix Test Results Permission Error

## Objective

Fix the `PermissionError: [Errno 13] Permission denied` when `run_tier_b.py` tries to write results to `/prosthesis_ws/tests/test1_software_verification/results/tier_b_latency_results.csv`.

## Root Cause

In `docker/Dockerfile:44`, `COPY tests/ tests/` copies the tests directory as `root:root`. The `USER prosthesis` directive at line 62 then switches to the non-root `prosthesis` user. When `run_tier_b.py` tries to `os.makedirs(RESULTS_DIR, exist_ok=True)` and write a CSV, it fails because the `prosthesis` user doesn't have write permission to the `results/` subdirectory.

## Implementation Plan

- [ ] **1. Add `chown` for tests directory in Dockerfile.** After line 46 (`RUN chmod +x scripts/*.sh scripts/*.py 2>/dev/null || true`), add a `RUN chown -R prosthesis:prosthesis tests/` so the prosthesis user owns the tests directory and can write results.

## Verification Criteria

- [ ] `run_tier_b.py --method service` completes without PermissionError
- [ ] Results CSV is written to `/prosthesis_ws/tests/test1_software_verification/results/tier_b_latency_results.csv`

## Potential Risks and Mitigations

1. **Full rebuild required**: Adding a `RUN` layer after `colcon build` doesn't invalidate the build cache, but adding it *before* would. Placing it after `chmod` but before `colcon build` is safe since it's a simple chown.
   Mitigation: Place the chown after the COPY and chmod, before colcon build — it won't affect the build cache for the expensive colcon step.
