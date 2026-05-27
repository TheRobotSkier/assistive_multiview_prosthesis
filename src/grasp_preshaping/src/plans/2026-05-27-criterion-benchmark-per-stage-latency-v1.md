# Criterion Benchmark for Per-Stage Latency Table

## Objective

Add a Criterion benchmark that measures the four pipeline stages independently using a synthetic 10k-point cylinder point cloud, with 20k SMC samples and 5 iterations, to fill out the LaTeX table:

| Pipeline Phase | Mean Execution Time (ms) |
|---|---|
| ROI Filtering & Morton Sort | TODO |
| Superquadric Backside Estimation | TODO |
| TSDF Volumetric Construction | TODO |
| Evolutionary Optimization Loop (per iteration) | TODO |
| **Total Pipeline Latency** | **TODO** |

## Context

- The crate already has `criterion` as a dev-dependency in `Cargo.toml:21` but no `[[bench]]` targets.
- The four pipeline stages map directly to function calls in `c_api.rs:366-508`:
  1. `predict_roi_with_samples` → `prune` → `morton` (lines 395-418)
  2. `fit_best_superquadric` (lines 421-426)
  3. `get_tsdf` (lines 428-436)
  4. SMC loop: `sample_initial_particles` + `score_all_particles` + resampling (lines 446-508)
- The LUT file (`data/finger_contact_lut.npz`) is needed for the SMC scoring stage but does not exist on disk — it must be generated first via `scripts/model.py`.
- Config defaults already match: `PREDICTION_SAMPLES=20000`, `ITERATIONS=5`.

## Implementation Plan

- [ ] **Step 1. Generate the LUT file** — Run `scripts/model.py` (or equivalent) to produce `data/finger_contact_lut.npz` in the `src/grasp_preshaping/` directory. The benchmark cannot run without this file. Verify it exists and is loadable by `FingerLUT::load`.

- [ ] **Step 2. Add `[[bench]]` target to `Cargo.toml`** — Add a `[[bench]]` section after `[dev-dependencies]`:
  ```
  [[bench]]
  name = "pipeline_stages"
  harness = false
  ```
  This tells Cargo to look for `benches/pipeline_stages.rs` and use Criterion's custom harness.

- [ ] **Step 3. Create `benches/pipeline_stages.rs`** — This is the main benchmark file. Structure:

  **Shared setup (outside benchmark groups):**
  - Generate a synthetic ~10k point cloud: points on the surface of a cylinder (radius ~3cm, height ~10cm, centered at origin). Use the same approach as `test_fit_sphere_points` in `superquadric.rs:854` but for a cylinder shape.
  - Load `FingerLUT` from `data/finger_contact_lut.npz` (same path logic as `c_api.rs:338-342`).
  - Initialize `RuntimeConfig` by calling any config accessor once (triggers `ensure_init`).
  - Create a fixed `GraspComputeRequestFFI`-like setup: hand pose at (0, 0, 0.15) pointing down, zero twist, 4 cameras at realistic positions.

  **Benchmark group 1: "roi_filter_morton"**
  - Calls `predict_roi_with_samples` → `prune` → `morton` in sequence.
  - This is the ROI prediction + filtering + Morton encoding stage.
  - Measures: `c_api.rs:395-418` equivalent.

  **Benchmark group 2: "sq_backside"**
  - Calls `fit_best_superquadric` on the pruned point cloud.
  - Measures: `c_api.rs:421-426` equivalent.

  **Benchmark group 3: "tsdf_construction"**
  - Calls `get_tsdf` with the morton output and SQ params.
  - Measures: `c_api.rs:428-436` equivalent.

  **Benchmark group 4: "smc_per_iteration"**
  - Runs one full SMC iteration: `score_all_particles` on 20k particles.
  - Setup: call `sample_initial_particles` once (not measured), then benchmark a single scoring pass.
  - Measures: the dominant cost per iteration in `c_api.rs:468-508`.

  **Benchmark group 5: "total_pipeline"**
  - Calls the full `compute_from_request`-equivalent logic end-to-end (or as much as possible without the FFI layer).
  - Measures: total latency for comparison with the sum of individual stages.

  **Sample count:** Use `Criterion::default().sample_size(20)` for ~20 iterations per benchmark (adjustable). Criterion does its own warmup and statistical analysis.

- [ ] **Step 4. Handle the `compute_from_request` dependency chain** — The full pipeline in `c_api.rs:366-631` is a private function behind the FFI layer. The benchmark needs to either:
  - (a) Replicate the pipeline stages by calling the public functions directly (`prune`, `morton`, `fit_best_superquadric`, `get_tsdf`, `sample_initial_particles`, `score_all_particles`), OR
  - (b) Make `compute_from_request` public (or `pub(crate)`) and call it from the benchmark via a test-friendly wrapper.

  **Recommendation: Option (a)** — call the individual public functions. This gives per-stage numbers directly and avoids touching the FFI layer. All the functions needed are already `pub`.

- [ ] **Step 5. Run the benchmark** — Execute:
  ```
  cd src/grasp_preshaping
  cargo bench --bench pipeline_stages
  ```
  Criterion will output mean, median, and confidence intervals for each group. The `mean` values fill the table.

- [ ] **Step 6. Fill in the LaTeX table** — Read the Criterion output (stdout or `target/criterion/report/`) and fill each TODO with the mean time in ms. The "Evolutionary Optimization Loop (per iteration)" row gets the per-iteration time multiplied by 5 (or report per-iteration and note "×5 iterations"). The "Total Pipeline Latency" row gets the sum (or the measured total from group 5).

## Verification Criteria

- `cargo bench --bench pipeline_stages` runs without errors
- All 5 benchmark groups produce numerical results
- The total pipeline time is approximately consistent with existing ROS measurements (~70-80ms for 30k points; expect faster for 10k points)
- The sum of individual stages approximately equals the total pipeline measurement

## Potential Risks and Mitigations

1. **LUT file missing** — The `finger_contact_lut.npz` file is not in the repo. Mitigation: generate it first with `scripts/model.py`, or create a minimal synthetic LUT for benchmarking purposes.

2. **Config initialization in benchmark context** — `RuntimeConfig` uses `OnceLock` and `AtomicPtr`, initialized on first access. In a Criterion benchmark, this happens once during setup. No issue, but ensure `runtime_config::get()` is called at least once before any timed code.

3. **Randomness in SMC sampling** — Particle sampling uses `rand::rng()`. Criterion's statistical analysis handles variance, but if the SMC loop has high variance, increase `sample_size`. Mitigation: seed the RNG if deterministic behavior is desired.

4. **Benchmark measures setup + compute** — The `prune` function takes `&PointCloud` and produces a new one. If the point cloud allocation is included in the measurement, it adds noise. Mitigation: pre-allocate the pruned cloud for stages 2-4 benchmarks, only measure the actual computation.

5. **TSDF construction is the bottleneck** — Based on the existing total latency (~77ms) and the nature of BFS-based TSDF, this stage likely dominates. The benchmark will confirm this.

## Alternative Approaches

1. **Instrument `compute_from_request` with timing** — Add `eprintln!` timing inside the existing function and run the ROS integration test. Simpler but requires the full ROS stack running. Advantage: measures real end-to-end latency including ROS overhead.

2. **Use `std::time::Instant` in a `#[test]`** — Write a Rust test that calls each stage with `Instant::now()` checkpoints and prints results. Simpler than Criterion but no statistical rigor (no warmup, no outlier rejection, no confidence intervals). Good enough for a one-off table fill.

3. **Use the existing `tier_b_latency_results.csv` data** — The existing data already has `pipeline_time_ms` totals. Could estimate per-stage breakdowns by profiling ratios from a single instrumented run. Less accurate but zero implementation cost.
