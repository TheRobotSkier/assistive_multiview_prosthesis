# Fix: `emg_bridge.scripts` Module Not Found

## Objective

Fix `ModuleNotFoundError: No module named 'emg_bridge.scripts'` that crashes the `run_classifier`, `collect_data`, and `train` entry points at startup.

## Root Cause

`src/emg_bridge/setup.py:8` declares `packages=['emg_bridge', 'scripts']`, which:
- Installs a top-level package named `scripts` (doesn't exist as one)
- **Does NOT install** `emg_bridge.scripts` (the actual sub-package at `src/emg_bridge/emg_bridge/scripts/`)

The entry points (`run_classifier_entry.py`, `collect_data_entry.py`, `train_entry.py`) all import from `emg_bridge.scripts.xxx`, which isn't installed → `ModuleNotFoundError`.

## Implementation Plan

- [ ] **Fix `setup.py` packages list** — Change line 8 from `packages=['emg_bridge', 'scripts']` to `packages=find_packages()` (from `setuptools`), which auto-discovers `emg_bridge` and `emg_bridge.scripts` (and any future sub-packages). This is more robust than manually listing.
- [ ] **Rebuild inside container** — `colcon build --packages-select emg_bridge` (or `make build` / `make run`)
- [ ] **Verify all three entry points work** — `ros2 run emg_bridge run_classifier --help`, `ros2 run emg_bridge collect_data --help`, `ros2 run emg_bridge train --help`

## Verification Criteria

- `ros2 run emg_bridge run_classifier --help` exits 0 without `ModuleNotFoundError`
- `ros2 run emg_bridge collect_data --help` exits 0
- `ros2 run emg_bridge train --help` exits 0
- `make run` launches the pipeline without the classifier crashing

## Potential Risks and Mitigations

1. **`find_packages()` picks up unintended packages**
   Mitigation: `find_packages()` scoped under `src/emg_bridge/` will only find `emg_bridge` and `emg_bridge.scripts` — there are no other packages there. Alternatively, use the explicit list `['emg_bridge', 'emg_bridge.scripts']`.

## Alternative Approaches

1. **Explicit list**: `packages=['emg_bridge', 'emg_bridge.scripts']` — simpler, no `find_packages()` import needed, but must be updated if sub-packages are added later.
2. **`find_packages(exclude=['*test*'])`** — more robust for future growth, but slightly more complex.
3. **`find_packages()` with `include=['emg_bridge*']`** — most targeted, only discovers `emg_bridge` and its sub-packages.
