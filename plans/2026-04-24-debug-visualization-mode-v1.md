# Debug/Visualization Mode for Grasp Preshaping Pipeline

## Objective

Implement a debug/visualization mode in the grasp preshaping node that, when enabled via a config flag, persists the computed TSDF volume and all scored grasp candidates to disk. A companion Python script in `/scripts` will then load and render these artifacts interactively, enabling rapid iteration on grasp-planning parameters.

---

## Initial Assessment

### Project Structure Summary

The grasp preshaping pipeline lives in:
```
docker_ws/dev/grasp_preshaping/
├── src/
│   ├── config.rs            — compile-time constants (resolution, truncation, tolerances)
│   ├── pointcloud_helper.rs — PointCloud, Morton sorting, Tsdf struct + construction
│   ├── planner.rs           — grasp scoring (cylindrical/pinch/lateral), GraspScoreResult
│   ├── predictor.rs         — ROI prediction via twist sampling, SampledPose
│   ├── lut_helper.rs        — FingerLUT, DualQuaternion, Contact enum
│   ├── c_api.rs             — FFI entry point: compute_from_request() orchestrates the full pipeline
│   └── lib.rs               — module declarations
├── benches/pipeline.rs      — criterion benchmarks (full pipeline demo)
├── nodes/
│   └── preshaping_service_bridge_node.cpp  — ROS2 service bridge, calls Rust via FFI
├── scripts/
│   ├── hand_tip_visualizer.py  — existing interactive Matplotlib visualizer for contact points
│   └── model.py                — Pinocchio-based hand model + LUT generator
├── Cargo.toml
└── CMakeLists.txt
```

**Key data flow** (traced through `c_api.rs:312-376`):
1. `compute_from_request()` receives pose, twist, point cloud, camera positions
2. `predict_roi_with_samples()` generates `SampledPose` instances and an ROI AABB
3. `prune()` clips the point cloud to the ROI
4. `morton()` + `get_tsdf()` build the signed distance field
5. `score_all_samples()` evaluates every (sample, grasp_type) pair against the TSDF
6. `select_best_grasp()` picks the winner

**What needs to be saved for visualization:**
- The TSDF 3D volume (data array + dimensions + origin + resolution)
- The pruned point cloud
- All scored grasps (not just the best): sample pose, grasp type, scores, collision info
- The ROI AABB
- The camera positions
- The original hand pose/twist inputs

### Prioritized Challenges

1. **TSDF has no serialization path** — The `Tsdf` struct (`pointcloud_helper.rs:116-123`) is purely in-memory with private fields. No `pub` accessors exist for `data`, `width`, `height`, `depth`, `origin`. Adding public accessors or a serialization method is prerequisite to any export.

2. **Grasp scoring results are discarded** — `score_all_samples()` in `c_api.rs:171-198` returns `Vec<ScoredGrasp>` but only the best is kept. The full list must be preserved and exported.

3. **Conditional compilation vs runtime flag** — A compile-time `cfg` flag avoids any runtime overhead when disabled, but requires recompilation to toggle. A runtime flag (env var or config constant read at startup) is more flexible. Given this is a library loaded via FFI, a **compile-time constant in `config.rs`** is the cleanest approach — it guarantees zero overhead when disabled and is consistent with the existing pattern in that file.

4. **File format for cross-language data exchange** — The Python visualizer needs to read TSDF grids, point clouds, and grasp metadata. NPZ (numpy's `.npz`) is ideal: already used for the LUT, natively supported by Python, and the `npyz` crate is already a dependency.

---

## Implementation Plan

### Phase 1: Rust-side debug infrastructure

- [ ] **1.1 Add `DEBUG_VISUALIZATION` constant to `config.rs`**
  Add `pub const DEBUG_VISUALIZATION: bool = false;` at the top of `config.rs`. This is the single toggle for all debug-export functionality. When `false`, all debug code paths are dead-code eliminated by the compiler (zero runtime cost).
  - File: `docker_ws/dev/grasp_preshaping/src/config.rs:1`

- [ ] **1.2 Add public accessors to `Tsdf` struct**
  Add methods to expose the internal state needed for serialization:
  - `pub fn data(&self) -> &[f32]` — raw distance values
  - `pub fn dimensions(&self) -> (usize, usize, usize)` — (width, height, depth)
  - `pub fn resolution_m(&self) -> f32`
  - `pub fn origin(&self) -> Vector3<f32>`
  - File: `docker_ws/dev/grasp_preshaping/src/pointcloud_helper.rs:116-123` (inside `impl Tsdf`)

- [ ] **1.3 Create `src/debug_export.rs` module**
  New module containing all debug-export logic, gated by `config::DEBUG_VISUALIZATION`:
  - Define `DebugDumpData` struct that collects all artifacts from one pipeline run:
    - TSDF volume (data + dims + origin + resolution)
    - Pruned point cloud (Vec of xyz)
    - ROI AABB (min/max)
    - Camera positions
    - Input pose (as SE3 matrix or position+quaternion)
    - Input twist (linear + angular)
    - All scored grasps: for each (sample_index, grasp_type, GraspScoreResult, combined_score, sample_probability, sample_pose_SE3)
  - Implement `pub fn export_debug_dump(data: &DebugDumpData, output_dir: &str)` that writes:
    - `tsdf_volume.npy` — flat f32 array with shape (W, H, D)
    - `tsdf_metadata.npy` — [origin_x, origin_y, origin_z, resolution, width, height, depth]
    - `point_cloud.npy` — Nx3 f32 array
    - `roi_aabb.npy` — [min_x, min_y, min_z, max_x, max_y, max_z]
    - `cameras.npy` — Cx3 f32 array
    - `input_pose.npy` — [px, py, pz, qx, qy, qz, qw]
    - `input_twist.npy` — [lx, ly, lz, ax, ay, az]
    - `scored_grasps.npy` — structured array: each row = [sample_idx, grasp_type_int, closure_amount, alignment_score, force_closure_score, combined_score, sample_probability, pose_4x4_flattened(16 floats)] = 23 columns
  - Use `std::fs::create_dir_all` for the output directory
  - Use `std::io::BufWriter` + manual little-endian binary write for .npy format (to avoid adding another dependency; the npy header is simple: magic string + version + header dict + padding + data)
  - File: `docker_ws/dev/grasp_preshaping/src/debug_export.rs` (new)

- [ ] **1.4 Register `debug_export` module in `lib.rs`**
  Add `pub mod debug_export;` conditionally:
  ```rust
  #[cfg(feature = "debug_viz")]  // or simply always compile but gate calls
  pub mod debug_export;
  ```
  Since `config::DEBUG_VISUALIZATION` is a `bool` constant, the module can always be compiled — the compiler will eliminate unreachable code. This avoids feature-gate complexity.
  - File: `docker_ws/dev/grasp_preshaping/src/lib.rs:1-6`

- [ ] **1.5 Instrument `compute_from_request()` in `c_api.rs`**
  After the TSDF is built and all grasps are scored (around `c_api.rs:343-356`), add a debug export block:
  ```rust
  if config::DEBUG_VISUALIZATION {
      // Collect all scored grasps with their metadata
      // Build DebugDumpData from available local variables
      // Call debug_export::export_debug_dump()
  }
  ```
  The output directory can be derived from an env var `GRASP_DEBUG_DIR` defaulting to `/tmp/grasp_debug`.
  - File: `docker_ws/dev/grasp_preshaping/src/c_api.rs:343-376`

- [ ] **1.6 Instrument `benches/pipeline.rs` for standalone testing**
  Add a small utility in the bench file (or a separate `examples/` binary) that runs the pipeline with synthetic data and triggers the debug export. This allows testing the full export path without ROS2.
  - File: `docker_ws/dev/grasp_preshaping/benches/pipeline.rs` (or new `examples/debug_dump.rs`)

### Phase 2: Python visualization script

- [ ] **2.1 Create `scripts/visualize_grasp_debug.py`**
  New interactive Python script using **PyVista** (VTK-based, excellent for 3D volumetric data) or **Open3D** + **Matplotlib** as fallback. Recommended: **PyVista** for its interactive 3D plotting with built-in volume rendering, mesh support, and minimal boilerplate.
  
  Script responsibilities:
  - Parse command-line args: `--dir <path_to_debug_dump>` (required), `--grasp-index <N>` (optional, highlight specific grasp)
  - Load all `.npy` files from the dump directory
  - Display the following in an interactive 3D window:

- [ ] **2.2 Implement TSDF visualization**
  - Load `tsdf_volume.npy` and `tsdf_metadata.npy`
  - Reconstruct the 3D grid with correct origin and spacing
  - Use PyVista's `ImageData` (uniform grid) to display:
    - **Isosurface at distance=0** (the zero-level set = reconstructed object surface) as a solid mesh
    - **Signed distance field** as a color-mapped volume (optional, toggle-able)
    - **Cross-section planes** (slider-controlled axial/sagittal/coronal slices showing distance values)
  - Clip away `f32::MAX` voxels (unobserved) for clean visualization

- [ ] **2.3 Implement point cloud visualization**
  - Load `point_cloud.npy`
  - Render as colored scatter points (semi-transparent, behind the TSDF surface)
  - Load `roi_aabb.npy` and render as a wireframe bounding box

- [ ] **2.4 Implement grasp candidate visualization**
  - Load `scored_grasps.npy`
  - For each scored grasp, reconstruct the 4x4 hand base pose
  - Visualize hand poses as:
    - **Coordinate frames** (RGB arrows for XYZ axes) at each sample pose, colored/opacity-coded by combined_score
    - **Best grasp highlighted** in a distinct color (e.g., gold)
    - **Failed grasps** (no collision) shown as faded/transparent
  - Optional: use the LUT to show finger contact positions at the closure amount for the best grasp

- [ ] **2.5 Implement camera and input visualization**
  - Load `cameras.npy` — show camera positions as pyramid/frustum icons
  - Load `input_pose.npy` — show current hand pose as a coordinate frame
  - Load `input_twist.npy` — show twist as an arrow (linear velocity vector) from the hand pose

- [ ] **2.6 Add interactive controls**
  - **Score threshold slider** — filter grasps below a combined_score threshold
  - **Grasp type filter** — checkboxes for cylindrical/pinch/lateral
  - **TSDF slice toggles** — show/hide cross-section planes
  - **Point cloud toggle** — show/hide input points
  - **Info panel** — when clicking a grasp, show its full score breakdown
  - Use PyVista's `add_slider_widget()` and `add_checkbox_button_widget()` for interactivity

- [ ] **2.7 Add a summary statistics printout**
  - On load, print:
    - Number of grasp candidates evaluated
    - Best grasp: type, closure, combined score
    - TSDF grid dimensions and memory size
    - Point cloud size (before/after pruning)
    - ROI AABB extents

### Phase 3: Integration and testing

- [ ] **3.1 Add Python dependencies documentation**
  Document required pip packages in the script's docstring:
  ```
  pip install numpy pyvista
  ```
  (PyVista pulls in VTK and meshio automatically)

- [ ] **3.2 Test with synthetic data from benchmark**
  Run the benchmark/example pipeline with `DEBUG_VISUALIZATION = true` to generate a dump, then verify the Python script loads and displays it correctly.

- [ ] **3.3 Test with ROS2 pipeline end-to-end**
  Enable `DEBUG_VISUALIZATION`, trigger a grasp compute via the service bridge, verify dump files are created, then visualize.

- [ ] **3.4 Verify zero overhead when disabled**
  Confirm with `objdump` or `cargo asm` that the debug export code is eliminated when `DEBUG_VISUALIZATION = false`.

---

## Files to Change (Summary)

| File | Action | Description |
|------|--------|-------------|
| `src/config.rs` | Modify | Add `DEBUG_VISUALIZATION: bool` constant |
| `src/pointcloud_helper.rs` | Modify | Add public accessors to `Tsdf` struct |
| `src/debug_export.rs` | **Create** | New module: data collection + NPZ/NPY file export |
| `src/lib.rs` | Modify | Add `pub mod debug_export;` |
| `src/c_api.rs` | Modify | Add debug export call in `compute_from_request()` |
| `benches/pipeline.rs` | Modify | Add debug-dump test utility (or create `examples/debug_dump.rs`) |
| `scripts/visualize_grasp_debug.py` | **Create** | Interactive 3D visualization script |

---

## Verification Criteria

- [ ] **VC-1**: Setting `DEBUG_VISUALIZATION = false` in `config.rs` produces a library binary with no debug symbols or code paths related to file export (verified via `nm` or `strings`).
- [ ] **VC-2**: Setting `DEBUG_VISUALIZATION = true` and running the pipeline (benchmark or ROS2) produces a directory of valid `.npy` files that load without error in Python via `np.load()`.
- [ ] **VC-3**: The Python visualization script loads all artifacts and renders an interactive 3D scene showing the TSDF isosurface, point cloud, ROI box, camera positions, and all grasp candidates with score-based coloring.
- [ ] **VC-4**: The best grasp is visually distinct (highlighted) and its score breakdown matches the Rust log output.
- [ ] **VC-5**: No regression in benchmark performance when `DEBUG_VISUALIZATION = false` (within measurement noise).

---

## Potential Risks and Mitigations

1. **TSDF volume can be large for fine resolutions**
   - At 5mm resolution, a 30cm cube is 60^3 = 216K floats = ~860KB — manageable.
   - Mitigation: Export as raw `.npy` (no compression needed at this size). Document expected sizes.

2. **Npy format complexity from Rust**
   - Writing valid `.npy` files requires a specific header (magic, version, dtype descriptor, shape).
   - Mitigation: The header format is simple and well-documented. Implement a minimal writer (~30 lines). Alternatively, add the `ndarray-npy` crate for robust support (adds ~3 deps).

3. **PyVista may not be available in all environments**
   - Mitigation: Provide a fallback mode using only `matplotlib` (3D scatter + wireframe) with reduced interactivity. Check for pyvista at import and degrade gracefully.

4. **Scored grasps array shape is wide (23 columns)**
   - Mitigation: Document the column layout clearly in both the Rust export code and the Python loader. Use named constants for column indices in Python.

5. **Thread safety of file I/O in FFI context**
   - The export happens inside `compute_from_request()` which is called from the C++ bridge under a mutex.
   - Mitigation: File I/O is synchronous and single-threaded per call. Use a timestamp-based subdirectory name to avoid collisions if multiple calls happen.

---

## Alternative Approaches

1. **Feature flag instead of const bool**: Use `#[cfg(feature = "debug_viz")]` in Cargo.toml. This gives true compile-time elimination but requires passing `--features debug_viz` during build. Trade-off: cleaner elimination but less convenient to toggle. The `const bool` approach is recommended because the compiler already eliminates dead code for `if false { ... }` blocks, and it matches the existing pattern in `config.rs`.

2. **JSON + binary hybrid format**: Export metadata as JSON and large arrays as raw binary. Trade-off: more files to manage, but JSON is human-readable. NPZ/NPY is simpler for numpy-native consumption.

3. **ROS2 topic-based streaming**: Publish debug data on ROS2 topics instead of files. Trade-off: requires the ROS2 bridge to be running, adds latency, more complex. File-based export is simpler and works offline for post-hoc analysis.

4. **Open3D instead of PyVista**: Open3D has excellent TSDF visualization built in. Trade-off: heavier dependency, less interactive widget support. PyVista is recommended for its slider/button widget system.
