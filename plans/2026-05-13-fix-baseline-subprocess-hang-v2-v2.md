# Fix: Baseline Subprocess Hang in run.py

## Objective

Fix the `multiprocessing.Process` hang when computing high-fidelity baselines. The root cause is that `multiprocessing.Process` uses `fork()`, which copies the parent's already-initialized Rust `OnceLock` (with the production config) into the child. The child's `GRASP_CONFIG_PATH` environment variable change has no effect because `OnceLock` is already populated. Additionally, the forked Rust `.so` may deadlock due to internal mutexes or thread state.

## Implementation Plan

- [ ] 1. Replace `multiprocessing.Process` with `subprocess.Popen` in `compute_baseline()` (`run.py:171-197`). Use `subprocess.Popen` to run a fresh Python interpreter with an inline script. This ensures the Rust `.so` is loaded fresh in a new process where `GRASP_CONFIG_PATH` is set before `OnceLock` initializes.
- [ ] 2. Remove `_run_baseline_worker()` function (`run.py:83-168`) — its logic moves into the inline subprocess script.
- [ ] 3. Remove `import multiprocessing` from `run.py:26` since it's no longer needed.
- [ ] 4. The inline script will: (a) set `GRASP_CONFIG_PATH` to baseline config, (b) import `ffi_bridge`, `object_registry`, `hand_approaches` fresh, (c) run baseline computation with full cloud, (d) aggregate results (consensus grasp type, median pose, max score), (e) write result to a temp JSON file.
- [ ] 5. Add per-object timeout (120s) with progress indication so the user can see which object is being processed and how long it takes.
- [ ] 6. Add `--baseline-timeout` CLI argument to allow overriding the default timeout.

## Verification Criteria

- [ ] `python run.py --occlusion-only --repetitions 3` completes without hanging
- [ ] Baseline results show `pipeline_time_ms` values significantly higher than production (~500-2000ms vs ~100ms) confirming the baseline config is actually being used
- [ ] `grasp_type_name` in baseline results may differ from `expected_grasp` in object metadata (this is expected — the baseline represents the algorithmic optimum)
- [ ] No `multiprocessing` import remains in `run.py`

## Root Cause Analysis

1. **`runtime_config.rs:199`**: `static RUNTIME_CONFIG: OnceLock<RuntimeConfig> = OnceLock::new()` — config is loaded once on first access via `get_or_init()`
2. **`ffi_bridge.py:165`**: `ctypes.CDLL(so_path)` loads the Rust `.so` at `GraspLibrary()` construction time, which triggers `OnceLock` initialization
3. **`run.py:605`**: `lib = GraspLibrary()` is called in the main process, initializing `OnceLock` with the production config
4. **`run.py:176`**: `multiprocessing.Process(target=_run_baseline_worker, ...)` forks the parent, copying the already-initialized `OnceLock`
5. **`run.py:91`**: `os.environ["GRASP_CONFIG_PATH"] = config_path` in the child has no effect — `OnceLock` is already set

## Potential Risks and Mitigations

1. **Inline script complexity**: The inline script must be self-contained and handle all edge cases (failed computations, empty results).
   Mitigation: Reuse the exact same logic from `_run_baseline_worker`, just wrapped in a subprocess.

2. **Python interpreter path**: The subprocess needs to use the same Python interpreter (including venv).
   Mitigation: Use `sys.executable` to get the current interpreter path.

3. **Large point cloud serialization**: Passing numpy arrays via command-line is impractical. The inline script needs to load objects from disk.
   Mitigation: The script already loads objects via `object_registry.load_object()` from `.npz` files.

4. **Temp file cleanup**: The subprocess writes results to a temp JSON file that must be cleaned up.
   Mitigation: Use the existing `_baseline_{obj_name}.json` pattern with cleanup in the parent.
