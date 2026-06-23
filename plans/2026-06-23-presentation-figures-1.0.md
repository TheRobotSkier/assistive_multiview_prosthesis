# Presentation Figures Generation Plan

## Objective

Generate 4 presentation-quality figures (no titles) in a separate `scripts/presentation_figures/` folder, using a specified 6-color palette plus black/white. Each figure reads from existing data sources in the repository.

## Color Palette

| Hex | Visual | Assigned Name |
|-----|--------|---------------|
| `#3494ba` | Teal | `TEAL` |
| `#58b6c0` | Light cyan | `CYAN` |
| `#75bda7` | Sage green | `SAGE` |
| `#7a8c8e` | Gray | `GRAY` |
| `#84acb6` | Blue-gray | `BLUE_GRAY` |
| `#2683c6` | Blue | `BLUE` |

Plus `#000000` (black) and `#ffffff` (white) as needed.

## Data Sources (verified)

| Plot | Source File(s) | Key Columns |
|------|---------------|-------------|
| 1 | `tests/test1_software_verification/results/segmentation_cpu_trials.csv`, `segmentation_cuda_trials.csv` | `latency_ms`, `status` (100 rows each) |
| 2 | `tests/test1_software_verification/results/score_sweep_results.csv` | `prediction_samples`, `combined_score` (3781 rows; sample counts: 1K, 2K, 5K, 10K, 20K, 50K, 100K) |
| 3 | `tests/test1_funtional_user_trial/data.json` | `data[].Failure` (strings of chars: t,l,v,i,s,d,p) |
| 4 | `data/bags/v6_*/*.mcap` (40 bags) | `/jetson/head/odom`, `/jetson/arm/odom` — `nav_msgs/msg/Odometry`, field `pose.pose.position.{x,y,z}` |

## Implementation Plan

### Shared Infrastructure

- [ ] 1. Create `scripts/presentation_figures/` directory with an `__init__.py` (empty) and a `generate_all.py` entry-point script that imports and calls each plot function. The entry point should accept `--format` (default `png`) and `--dpi` (default `300`) arguments, and save all outputs into `scripts/presentation_figures/output/`.
  - Rationale: Single entry point for regenerating all figures; matches the pattern in `tests/test1_software_verification/plot_results.py:1-50`.

- [ ] 2. Create `scripts/presentation_figures/palette.py` defining the 6-color palette as hex strings plus `BLACK` and `WHITE` constants, and a helper `apply_presentation_style(ax)` that sets white background, removes top/right spines, uses light gray grid, and sets consistent font sizes. All four plot modules import from here.
  - Rationale: Ensures visual consistency across figures and avoids repeating styling code.

### Plot 1 — CUDA vs Non-CUDA Latency (Horizontal Boxplot)

**File:** `scripts/presentation_figures/plot1_latency.py`

**Data:** `segmentation_cpu_trials.csv` and `segmentation_cuda_trials.csv` (filter `status == "ok"`).

**Design:**
- Horizontal boxplot with 2 boxes: "Non-CUDA" (CPU) and "CUDA".
- Non-CUDA box color: `#7a8c8e` (GRAY). CUDA box color: `#2683c6` (BLUE).
- Vertical dashed line at x=228 (the target threshold), colored black, with a small text annotation "228 ms target" above it.
- X-axis: "Latency (ms)". Y-axis: category labels "Non-CUDA" and "CUDA".
- No title. White background. `figsize=(8, 3)`.

- [ ] 3. Implement `plot_latency(fmt, dpi)` function that loads both CSVs via `csv.DictReader` (same pattern as `plot_results.py:1512-1521`), extracts `latency_ms` values filtered by `status == "ok"`, creates a horizontal boxplot with `vert=False`, applies the two box colors via `patch.set_facecolor()`, draws the 228ms vertical line, and saves to `output/plot1_latency.{fmt}`.
  - Rationale: The data is straightforward — 100 trials each. The 228ms threshold is user-specified as the target. Horizontal orientation emphasizes the latency comparison.

### Plot 2 — Samples vs Output Variability (Strip + Boxplot)

**File:** `scripts/presentation_figures/plot2_variability.py`

**Data:** `score_sweep_results.csv` — column `prediction_samples` (discrete: 1000, 2000, 5000, 10000, 20000, 50000, 100000) and `combined_score`.

**Design:**
- X-axis: discrete sample counts (7 categories), displayed as "1K", "2K", "5K", "10K", "20K", "50K", "100K".
- Y-axis: `combined_score` (output variability metric).
- Boxplot layer: one box per sample count, color `#84acb6` (BLUE_GRAY) with alpha ~0.5, shown behind.
- Strip/scatter layer: individual data points overlaid on top, colored `#3494ba` (TEAL) with low alpha (~0.15) and small marker size, jittered horizontally.
- No title. `figsize=(8, 5)`.

- [ ] 4. Implement `plot_variability(fmt, dpi)` function that loads the CSV via pandas, groups by `prediction_samples`, draws a boxplot using `ax.boxplot()` with `positions` set to 0-6, then overlays a jittered scatter of all individual `combined_score` values per group. X-tick labels formatted as human-readable (1K, 2K, etc.). Save to `output/plot2_variability.{fmt}`.
  - Rationale: The existing figure 12 (`plot_results.py:2143-2284`) uses a log-scale x-axis with median+IQR error bars. The user wants a different visualization: discrete x-axis with boxplot + strip overlay, which better shows distribution shape and individual point density.

### Plot 3 — Error Type Sunburst

**File:** `scripts/presentation_figures/plot3_sunburst.py`

**Data:** `tests/test1_funtional_user_trial/data.json` — the `data` array's `Failure` field (strings of concatenated error chars).

**Design:**
- Two-ring sunburst using `matplotlib.pyplot.pie()` with concentric wedges (inner ring radius 0.6, outer ring radius 1.0).
- **Inner ring** (aggregated categories):
  - EMG (sum of t + d counts): color `#2683c6` (BLUE)
  - Tracking (sum of l + v + i + p counts): color `#3494ba` (TEAL)
  - **Gap** where Segmentation would be — no wedge drawn for the inner ring at that angular position, leaving visible empty space.
- **Outer ring** (individual error types):
  - Within EMG segment: Trigger (t) and Deactivation (d) — shades of blue: `#2683c6` and `#58b6c0`
  - Within Tracking segment: Localisation (l), Vision (v), Intent (i), Proximity (p) — shades of teal/sage: `#3494ba`, `#75bda7`, `#84acb6`, `#58b6c0`
  - Segmentation (s): `#7a8c8e` (GRAY) — placed in the outer ring at the angular position where the inner ring has a gap.
- No title. `figsize=(7, 7)`. White background.

- [ ] 5. Implement `plot_sunburst(fmt, dpi)` function that:
  - Loads `data.json` and iterates all `Failure` strings in the `data` array, counting each character occurrence using a `Counter`.
  - Computes inner-ring sizes: EMG = count(t) + count(d), Tracking = count(l) + count(v) + count(i) + count(p). Segmentation inner-ring size = 0 (gap).
  - Computes outer-ring sizes: individual counts for t, d, l, v, i, p, s.
  - Draws inner ring as a pie with 2 wedges (EMG, Tracking) plus a dummy zero-width wedge for the Segmentation gap position. Draws outer ring as a pie with 7 wedges aligned angularly to their parents.
  - Uses `radius` parameter on `ax.pie()` to create concentric rings, with `wedgeprops=dict(width=0.4)` for ring thickness.
  - Save to `output/plot3_sunburst.{fmt}`.
  - Rationale: Plotly is not installed in this project. Matplotlib's `pie()` with concentric rings and `width` parameter achieves the sunburst effect natively. The gap in the inner ring is the key design requirement — Segmentation (s) appears only in the outer ring.

### Plot 4 — VIO Tracking Plot (Dual-Panel)

**File:** `scripts/presentation_figures/plot4_vio_tracking.py`

**Data:** All MCAP bags in `data/bags/` — reading `/jetson/head/odom` and `/jetson/arm/odom` topics from `nav_msgs/msg/Odometry` messages.

**Dependencies:** `mcap` and `mcap_ros2.decoder` (already used by `scripts/analyze_bag.py:651-656`).

**Design:**
- **Main panel** (top, larger): X/Y trajectory scatter plot.
  - X-axis: x position (asinh-scaled). Y-axis: y position (asinh-scaled).
  - Head camera points: `#2683c6` (BLUE), small markers, low alpha.
  - Arm camera points: `#3494ba` (TEAL), small markers, low alpha.
  - Asinh scale compresses the extreme divergence values (arm odom can reach 1000+ m) while preserving near-origin detail.
- **Bottom panel** (smaller, horizontal boxplot): Euclidean distance √(x²+y²) from origin.
  - Two boxes: "Head" and "Arm".
  - Head box color: `#84acb6` (BLUE_GRAY). Arm box color: `#75bda7` (SAGE).
  - Shared x-axis label or independent.
- No titles. `figsize=(10, 8)` with `gridspec` height ratios [3, 1].
- Bags iterated: all `v6_*` directories under `data/bags/` (excluding `golden_replay` which is a replay bag).

- [ ] 6. Implement `load_odom_from_bags(bags_dir)` function that:
  - Globs all `v6_*` subdirectories in `data/bags/`.
  - For each bag, opens the `.mcap` file with `mcap.reader.make_reader` and `DecoderFactory()`.
  - Iterates decoded messages, filtering for topics `/jetson/head/odom` and `/jetson/arm/odom`.
  - Extracts `pose.pose.position.x` and `pose.pose.position.y` from each message.
  - Accumulates into two lists: `head_xy` and `arm_xy` (each a list of (x, y) tuples).
  - Returns `(head_xy, arm_xy)`.
  - Rationale: Reuses the proven MCAP reading pattern from `scripts/analyze_bag.py:690-725`. The 3 legacy bags using `/ov_msckf*/odomimu` topics are excluded since they predate the standardized relay layer and would require different topic names.

- [ ] 7. Implement `plot_vio_tracking(fmt, dpi)` function that:
  - Calls `load_odom_from_bags()` to get all head/arm XY data across all bags.
  - Creates a 2-row figure using `gridspec` with height ratios [3, 1].
  - **Top panel**: Scatter plot of head XY and arm XY on asinh-scaled axes (`ax.set_xscale('asinh')`, `ax.set_yscale('asinh')`). Head colored `#2683c6`, arm colored `#3494ba`, small markers (s=2), alpha=0.1. Axis labels "X position (m, asinh)" and "Y position (m, asinh)".
  - **Bottom panel**: Computes Euclidean distance `sqrt(x² + y²)` for head and arm. Draws horizontal boxplot with 2 boxes. Colors as specified. Axis label "Euclidean distance from origin (m)".
  - Save to `output/plot4_vio_tracking.{fmt}`.
  - Rationale: The asinh scale is critical because arm odom can diverge to 1000+ m while head stays near origin. Linear scaling would compress all head data into a single point. Asinh provides linear behavior near zero and logarithmic compression for large values.

### Entry Point

- [ ] 8. Implement `generate_all.py` with `argparse` for `--format` (default `png`) and `--dpi` (default `300`), importing and calling all four `plot_*` functions in sequence, creating the `output/` directory if it doesn't exist. Print progress messages to stderr for each figure.
  - Rationale: Single command to regenerate all presentation figures.

## Verification Criteria

- [ ] All 4 figures are generated as PNG files in `scripts/presentation_figures/output/`
- [ ] No figure has a title (`ax.set_title()` never called)
- [ ] Only the 6 specified hex colors plus black/white are used
- [ ] Plot 1 shows 2 horizontal boxes with a vertical line at 228ms
- [ ] Plot 2 shows boxplot + strip overlay with 7 discrete x-axis categories
- [ ] Plot 3 shows a 2-ring sunburst with a visible gap in the inner ring at the Segmentation position
- [ ] Plot 4 shows trajectory scatter on asinh axes + distance boxplot, with head/arm in distinct colors
- [ ] Running `python scripts/presentation_figures/generate_all.py` completes without errors

## Potential Risks and Mitigations

1. **MCAP reading performance**: 40 bags × ~1700 messages each = ~68K messages per topic. Reading all bags may take 30-60 seconds.
   - Mitigation: Process bags sequentially with progress output. Consider subsampling if memory is an issue (every Nth point).

2. **Arm odom extreme divergence**: Some bags have arm X values reaching 1099m (`data/bags/v6_20260618_195731/v6_20260618_195731_analysis.txt:62`). Even asinh scale may compress head data.
   - Mitigation: Asinh scale handles this naturally. If needed, add an inset axes zoomed to ±1m range.

3. **Sunburst gap alignment**: The Segmentation wedge in the outer ring must align angularly with the gap in the inner ring. Matplotlib pie charts start at 90° (top) and go counterclockwise by default.
   - Mitigation: Carefully compute cumulative angles for both rings, ensuring the Segmentation outer wedge starts at the same angle where the inner ring gap begins. Use `startangle` parameter consistently.

4. **mcap_ros2 package availability**: The `mcap` and `mcap_ros2.decoder` packages must be installed. They are already used by `scripts/analyze_bag.py`.
   - Mitigation: Check import at top of module with a clear error message pointing to `pip install mcap mcap-ros2-support`.

## Alternative Approaches

1. **Plot 3 (Sunburst) — Plotly vs Matplotlib**: Plotly's `px.sunburst()` would be simpler but Plotly is not installed anywhere in the codebase. Matplotlib's concentric `pie()` approach avoids adding a new dependency and produces a static PNG suitable for presentations. Trade-off: more manual angle computation but zero new dependencies.

2. **Plot 4 (VIO) — Bag selection**: Could limit to a representative subset of bags rather than all 40. Trade-off: fewer bags = faster but less comprehensive. Decision: use all `v6_*` bags for completeness; the scatter density is the point.

3. **Plot 2 — Seaborn vs raw Matplotlib**: Seaborn's `boxplot` + `stripplot` combination would be concise. Seaborn is already imported in `tests/test1_funtional_user_trial/plot_test3.py`. Trade-off: adds a seaborn dependency to the presentation figures but simplifies the overlay code significantly. Decision: use raw matplotlib to minimize dependencies and maintain full control over styling.
