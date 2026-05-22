# Test 1 Software Verification — Restructuring Plan

## Objective

Restructure Test 1 to produce more meaningful intent precision and latency measurements:

1. **Intent Precision (Req 2.7)**: Refocus on **grasp type correctness** as the primary metric, with wrist rotation error as a cumulative subplot. Position error should be logged but de-emphasized as a success criterion. Increase sample count for statistical robustness.
2. **Latency (Req 2.4)**: Add full-pipeline latency numbers (Tier B) alongside the existing grasp-planning-only latency (Tier A). The full pipeline includes EMG processing, segmentation, and ROS overhead.

## Problem Analysis

### Current Intent Precision Issues

Looking at the existing results (`results/intent_precision_summary.csv`), the current test measures "fully correct" as requiring **all three** criteria simultaneously: correct grasp type AND position error < 30mm AND orientation error < 45deg. This is far too strict for a stochastic SMC planner:

- **`fully_correct_pct` is 0% for nearly every object/condition** — only `ellipsoid/multi_view` achieves 10%. This means the metric is not discriminative.
- The baseline comparison is against the high-fidelity baseline's **grasp type**, but the baseline itself often disagrees with the object's `expected_grasp`. For example:
  - `banana` has `expected_grasp=cylindrical` but baseline says `pinch` (type 2)
  - `cylinder_upright` has `expected_grasp=cylindrical` but baseline says `pinch`
  - `small_cube` has `expected_grasp=pinch` but baseline says `cylindrical`
- This means the baseline is the **wrong ground truth** in several cases, and the comparison is noisy.

### What We Actually Want (per user request)

1. **Primary metric**: Grasp type correctness — does the predicted grasp type match the expected/baseline grasp type? This is what Req 2.7 ("correct autonomous grasp type and wrist pose predictions") cares about most.
2. **Secondary visualization**: Cumulative distribution plot of wrist rotation errors — shows how close we get, not just pass/fail.
3. **Logged but not scored**: Position error — nice to know, not a pass/fail criterion.
4. **More samples**: 10 repetitions per object/condition is too few for a stochastic planner. Need 30+ for meaningful statistics.

### Current Latency Gap

- **Tier A** (`run.py`): Measures only the grasp planning computation time via the Rust FFI (~80-130ms). This is the grasp-planning stage only.
- **Tier B** (`run_tier_b.py`): A skeleton that would measure the full ROS 2 pipeline (EMG → segmentation → grasp planning → motor command). Currently unimplemented.
- **Req 2.4** specifies: "Maximum computation time from EMG signal to autonomous motor command output." This is the full pipeline, not just grasp planning.
- The existing plan for Tier B is documented in `run_tier_b.py:66-109` but needs actual implementation.

## Implementation Plan

### Part 1: Intent Precision Restructuring

#### 1.1 Fix the ground truth comparison

- [ ] **Re-evaluate baseline vs. expected_grasp alignment.** Currently the baseline is computed from the full point cloud with high-fidelity config, but it often disagrees with the object metadata's `expected_grasp`. Decide on the ground truth source:
  - **Option A (recommended)**: Use the **baseline consensus** (high-fidelity full-cloud result) as ground truth, since it represents the algorithmic optimum. Update `expected_grasp` in object metadata to match where they disagree.
  - **Option B**: Keep both and report accuracy against each separately.
  
  Rationale: The baseline represents what the algorithm would produce with perfect information. If the baseline says "pinch" for a banana but metadata says "cylindrical", the algorithm is not wrong — the metadata's expected grasp may be subjective. The baseline is the fairer comparison.

#### 1.2 Redefine "fully correct" to be grasp-type-only

- [ ] **Update `compute_intent_precision()` in `run.py`** to make the primary "correct" metric be **grasp type match only** (matching the baseline consensus). Remove the requirement for position and orientation thresholds from the primary metric. Specifically:
  - Rename `fully_correct_pct` → `grasp_correct_pct` (grasp type match only)
  - Keep `orientation_accuracy_pct` and `position_accuracy_pct` as logged diagnostics
  - The Δ metric should be `delta_grasp_correct_pct` (MV minus SV grasp type match rate)

  Rationale: Per user request, "for the fully correct, I think we should just do grasp type, not also wrist rotations."

#### 1.3 Increase sample count

- [ ] **Change default repetitions from 10 to 30** in `run.py:554` (the `--repetitions` default). This gives much better statistical confidence for a stochastic planner. With 10 objects × 2 conditions × 30 reps = 600 data points per metric.
  
  Rationale: Per user request, "we might need more samples from both the multi and single view to really understand the full picture."

#### 1.4 Add per-object detailed logging of correct values

- [ ] **Add a detailed per-repetition comparison table** to the CSV output. For each repetition, log:
  - `baseline_grasp_type_name` (ground truth)
  - `predicted_grasp_type_name` (what the planner output)
  - `grasp_type_correct` (bool)
  - `wrist_rotation_error_deg` (absolute angular difference from baseline)
  - `position_error_mm` (distance from baseline target)
  - `combined_score`
  
  This is already partially in `occlusion_results.csv` but should be more prominently documented and easy to analyze.

#### 1.5 New visualization: Cumulative wrist error plot

- [ ] **Add `plot_wrist_error_cumulative()` to `plot_results.py`**. This should produce:
  - **Main subplot**: Cumulative distribution function (CDF) of wrist rotation errors, with separate curves for single-view and multi-view. X-axis = error threshold (degrees), Y-axis = fraction of samples within that threshold.
  - **Secondary subplot**: Stacked bar or grouped bar showing grasp type correctness rate per object for single-view vs multi-view.
  
  The CDF plot lets readers see things like "90% of multi-view predictions have wrist error < 20deg" vs "only 60% of single-view predictions do."

  Rationale: Per user request, "like cumulative plot of wrist errors and correct grasp or something like that."

#### 1.6 Update the intent precision bar chart

- [ ] **Update `plot_intent_precision()` in `plot_results.py`** to show:
  - Panel 1: Grasp type accuracy (%) — single-view vs multi-view per object
  - Panel 2: Mean wrist rotation error (degrees) — single-view vs multi-view per object
  - Panel 3: Mean position error (mm) — single-view vs multi-view per object (diagnostic)
  
  Replace the old "Fully Correct", "Grasp Type Match", "Position Error" panels.

#### 1.7 Update the delta chart

- [ ] **Update `plot_intent_delta()` in `plot_results.py`** to use the new grasp-type-only Δ metric. The bar chart should show Δ grasp accuracy per object, with the MAR (>0%) and IDE (≥10%) threshold lines.

#### 1.8 Update LaTeX table generation

- [ ] **Update `generate_latex_table()` in `plot_results.py`** to reflect the new metrics: grasp accuracy instead of "fully correct", and include mean wrist error in the table.

### Part 2: Full Pipeline Latency (Tier B)

#### 2.1 Implement Tier B latency test

- [ ] **Implement `run_tier_b.py`** to measure end-to-end ROS 2 pipeline latency. The architecture (already sketched in the file at `run_tier_b.py:66-109`) should:
  1. Create a ROS 2 node
  2. Subscribe to `/prosthesis/grasp_command` (or equivalent output topic)
  3. For each object/repetition:
     a. Load synthetic point cloud
     b. Convert to `PointCloud2` message
     c. Publish to segmentation input topic
     d. Inject EMG trigger signal (publish to `/emg/gesture_label`)
     e. Record wall-clock timestamp T_start
     f. Wait for grasp command callback
     g. Record wall-clock timestamp T_end
     h. Compute `total_latency_ms = (T_end - T_start) * 1000`
  4. Write results to `tier_b_latency_results.csv` with fields: `object`, `repetition`, `total_latency_ms`, `segmentation_ms` (if available), `grasp_planning_ms` (if available), `ros_overhead_ms`, `status`

- [ ] **Add per-stage timing instrumentation** where possible. If the pipeline nodes publish timing info (e.g., on a `/pipeline/timing` topic), subscribe to it. Otherwise, estimate overhead as `total_latency - tier_a_latency`.

- [ ] **Add Tier B latency visualization** to `plot_results.py`:
  - A combined figure showing Tier A (grasp planning only) and Tier B (full pipeline) latency as grouped bars per object
  - Threshold lines for MAR (400ms) and IDE (100ms)
  - Annotate the "overhead" (Tier B - Tier A) to show where time is spent

- [ ] **Tier B must use only actual wall-clock measurements** — no estimated overhead, no simulated delays. Every number must come from a real ROS 2 pipeline invocation.

#### 2.2 Tier B requires a running ROS 2 mock system

- [ ] **Document the prerequisite**: Tier B requires `ros2 launch prosthesis_launch mock.launch.py` to be running with all pipeline nodes active (EMG mock, segmentation, grasp planning, motor command).
- [ ] **Tier B test should fail gracefully** if the mock system is not running (already partially implemented via `check_ros_available()` and `check_mock_running()`).
- [ ] **Do NOT implement any simulated/estimated overhead fallback.** If Tier B cannot run, it reports "not available" — no fake numbers.

### Part 3: Run the updated tests

#### 3.1 Re-run with new parameters

- [ ] **Run the updated Tier A test** with 30 repetitions: `python run.py --repetitions 30`
- [ ] **Run the updated plot script**: `python plot_results.py`
- [ ] **Verify** that the new metrics produce meaningful, discriminative results

## Verification Criteria

- [ ] `grasp_correct_pct` (grasp-type-only) is non-zero for most objects and shows a clear difference between single-view and multi-view
- [ ] The cumulative wrist error CDF plot is generated and shows the multi-view curve dominating the single-view curve
- [ ] Mean Δ grasp accuracy across all objects is > 0% (MAR threshold for Req 2.7)
- [ ] Tier A latency numbers remain consistent with previous runs (~80-130ms)
- [ ] Tier B latency is measured from actual ROS 2 pipeline invocations (no estimates)
- [ ] Per-object detail logging shows which grasp types are predicted vs. expected, enabling root-cause analysis

## Potential Risks and Mitigations

1. **Baseline disagrees with expected_grasp for many objects**
   Mitigation: Use baseline consensus as ground truth; document disagreements. The baseline represents the algorithmic optimum with full information.

2. **30 repetitions may take significantly longer** (~3x the current runtime)
   Mitigation: The current 10-object × 10-rep run takes ~2-3 minutes for Tier A. 30 reps would be ~6-9 minutes, which is acceptable for an offline test.

3. **Full ROS 2 pipeline (Tier B) may not be stable enough for automated testing**
   Mitigation: Tier B simply reports "not available" if the mock system isn't running. No estimated numbers. Tier B can be validated separately when the pipeline is stable.

4. **Grasp type accuracy may not show consistent multi-view improvement**
   Mitigation: The CDF plot and per-object breakdown will reveal where multi-view helps and where it doesn't. Even if the mean Δ is small, the detailed analysis is valuable for the report.

## Alternative Approaches

1. **Use expected_grasp from metadata as ground truth instead of baseline**: Simpler, but many objects have subjective or incorrect expected grasps. The baseline is algorithmically fairer.

2. **Bootstrap confidence intervals instead of more repetitions**: Could achieve similar statistical power with fewer raw samples, but is more complex to implement and explain in the report.

3. **Separate wrist rotation into roll/pitch/yaw components**: More detailed, but the combined angular distance (quaternion distance) is the standard metric and easier to interpret as a CDF.
