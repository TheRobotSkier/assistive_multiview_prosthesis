# Debug/Visualization Mode for Grasp Preshaping Pipeline

## Objective

Add a debug mode to the grasp preshaping node that persists the full pipeline state (TSDF volume, point cloud, all scored grasp candidates, input data) to a single `.npz` file. A companion Python script provides interactive 3D visualization of these artifacts for rapid parameter tuning and pipeline inspection.

---

## Codebase Context

### Pipeline data flow

The full pipeline executes inside `compute_from_request()` at `src/c_api.rs:312-376`:

```
FFI request (pose, twist, point cloud, cameras)
  → predict_roi_with_samples()   → (Aabb roi, Vec<SampledPose> samples)
  → prune(cloud, roi)            → PointCloud pruned
  → morton() + get_tsdf()        → Tsdf { data, width, height, depth, resolution_m, origin }
  → score_all_samples()          → Vec<ScoredGrasp>   ← currently only best is kept
  → select_best_grasp()          → best ScoredGrasp
```

### Key types and their locations

| Type | Location | Fields relevant to export |
|------|----------|--------------------------|
| `Tsdf` | `pointcloud_helper.rs:116-123` | `data: Vec<f32>`, `width/height/depth: usize`, `resolution_m: f32`, `origin: Vector3<f32>` — all **private** |
| `Aabb` | `pointcloud_helper.rs:6-10` | `min: Vector3<f32>`, `max: Vector3<f32>` — **public** |
| `PointCloud` | `pointcloud_helper.rs:89-106` | `points: Vec<Vector3<f32>>` — **public** |
| `Camera` | `pointcloud_helper.rs:84-87` | `position: Vector3<f32>` — **public** |
| `SampledPose` | `predictor.rs:8-12` | `pose: DualQuaternion`, `sample_probability: f64` — **public** |
| `ScoredGrasp` | `c_api.rs:117-121` | `grasp_type: GraspType`, `result: GraspScoreResult`, `combined: f64` — **private** |
| `GraspScoreResult` | `planner.rs:6-12` | `closure_amount`, `alignment_score`, `force_closure_score`, `found_collision` — **public** |
| `GraspType` | `c_api.rs:90-95` | `Cylindrical=1`, `Pinch=2`, `Lateral=3` (via FFI constants) — **private** enum |
| `GraspPoseFFI` | `c_api.rs:26-34` | `px,py,pz,qx,qy,qz,qw: f64` — **public** |
| `GraspTwistFFI` | `c_api.rs:38-45` | `lx,ly,lz,ax,ay,az: f64` — **public** |
| `DualQuaternion` | `lut_helper.rs:6-9` | `real: [f64;4]`, `dual: [f64;4]` — **public**; has `pub fn to_se3(self) -> Matrix4<f64>` |

### Existing NPZ infrastructure

- `npyz = { version = "0.8", features = ["npz"] }` is already a dependency (`Cargo.toml:10`)
- Reading NPZ: `npyz::npz::NpzArchive` is used in `lut_helper.rs:2` to load the finger LUT
- Writing NPZ: `npyz::npz::NpzWriter` is available but not yet used — this is what the export will use
- No new dependencies needed

### Output format: single `.npz` file

An NPZ file is a ZIP archive containing named `.npy` arrays. Python loads it with `np.load("path.npz")` which returns a dict-like object. This matches the existing LUT pattern in `scripts/model.py:147`.

---

## Implementation Plan

### Step 1 — Add debug toggle to `config.rs`

- [ ] **1.1** Add the following constants to `src/config.rs` (after line 23):

```
// Debug visualization
pub const DEBUG_VISUALIZATION: bool = false;
pub const DEBUG_OUTPUT_DIR: &str = "data/debug";
```

**Rationale:** `config.rs` is already the single source of truth for all tunable constants (stated in its module doc comment at line 1). A `bool` constant is the simplest gate — when `false`, the compiler dead-code-eliminates the entire debug block (verified by LLVM for `if false { ... }` patterns on `const bool`). The `DEBUG_OUTPUT_DIR` is relative to the crate manifest dir (resolved at export time with `env!("CARGO_MANIFEST_DIR")`).

**Files changed:** `src/config.rs`

---

### Step 2 — Expose `Tsdf` internals for serialization

- [ ] **2.1** Add public accessor methods to the `impl Tsdf` block in `src/pointcloud_helper.rs` (after the `get_surface_normal` method, around line 210):

Four methods:
- `pub fn data(&self) -> &[f32]` — returns `&self.data`
- `pub fn dimensions(&self) -> (usize, usize, usize)` — returns `(self.width, self.height, self.depth)`
- `pub fn resolution(&self) -> f32` — returns `self.resolution_m`
- `pub fn origin(&self) -> Vector3<f32>` — returns `self.origin`

**Rationale:** The `Tsdf` struct fields are private (lines 117-122) with no accessors. Rather than making fields `pub` (which would break the encapsulation of the interpolation logic), targeted accessors expose only what serialization needs. These are trivially inlined and cost nothing at runtime.

**Files changed:** `src/pointcloud_helper.rs`

---

### Step 3 — Create `debug_export.rs` module

- [ ] **3.1** Create new file `src/debug_export.rs`

This module contains:

**Data collection struct:**
```
pub struct DebugDump<'a> {
    pub tsdf: &'a Tsdf,
    pub point_cloud: &'a PointCloud,
    pub roi: &'a Aabb,
    pub cameras: &'a [Camera],
    pub scored_grasps: &'a [ScoredGraspExport],
    pub samples: &'a [SampledPose],
    pub input_pose: &'a GraspPoseFFI,
    pub input_twist: &'a GraspTwistFFI,
}
```

Where `ScoredGraspExport` is a public struct that `c_api.rs` will construct from its private `ScoredGrasp`:
```
pub struct ScoredGraspExport {
    pub sample_index: usize,
    pub grasp_type_i32: i32,       // 1=cylindrical, 2=pinch, 3=lateral
    pub closure_amount: f64,
    pub alignment_score: f64,
    pub force_closure_score: f64,
    pub found_collision: bool,
    pub combined_score: f64,
    pub sample_probability: f64,
    pub pose_se3: [f64; 16],       // row-major 4x4
}
```

**Export function:**
```
pub fn export_npz(dump: &DebugDump, path: &std::path::Path) -> std::io::Result<()>
```

This function:
1. Creates parent directories with `std::fs::create_dir_all`
2. Opens a `BufWriter<File>` at `path`
3. Creates `npyz::npz::NpzWriter::new(buf_writer)`
4. Writes each array using `npz.array::<f32, _>("name")` / `npz.array::<f64, _>("name")` with appropriate shape via `npyz::WriteOptions`, then pushes data element-by-element
5. Finishes with `npz.finish()?`

**NPZ arrays written:**

| Array name | Shape | dtype | Source |
|---|---|---|---|
| `tsdf_volume` | `(W*H*D,)` flat | `f32` | `tsdf.data()` |
| `tsdf_metadata` | `(7,)` | `f64` | `[origin.x, origin.y, origin.z, resolution, width, height, depth]` |
| `point_cloud` | `(N, 3)` flat as `(N*3,)` | `f32` | `point_cloud.points` interleaved xyz |
| `roi_aabb` | `(6,)` | `f32` | `[min.x, min.y, min.z, max.x, max.y, max.z]` |
| `cameras` | `(C*3,)` flat | `f32` | Camera positions interleaved |
| `input_pose` | `(7,)` | `f64` | `[px, py, pz, qx, qy, qz, qw]` |
| `input_twist` | `(6,)` | `f64` | `[lx, ly, lz, ax, ay, az]` |
| `scored_grasps` | `(M, 22)` flat as `(M*22,)` | `f64` | Each row: `[sample_idx, grasp_type, closure, alignment, force_closure, found_collision_as_f64, combined, probability, pose_4x4_row0..row3(16)]` |

**Rationale for flat arrays:** The `npyz::WriteOptions` API pushes elements sequentially. Writing flat arrays with metadata that records the true shape is simpler than trying to express multi-dimensional shapes through the push API. The Python loader reshapes using the metadata.

**Timestamped filenames:** The caller constructs the path as `data/debug/grasp_dump_YYYYMMDD_HHMMSS.npz` using `std::time::SystemTime` to avoid collisions across runs.

**Files created:** `src/debug_export.rs`

---

### Step 4 — Register module in `lib.rs`

- [ ] **4.1** Add `pub mod debug_export;` to `src/lib.rs` (after line 6).

The module is always compiled. When `config::DEBUG_VISUALIZATION` is `false`, the export function exists but is never called — the compiler will eliminate it (and all its transitive dependencies like the `npyz` write path) from the final binary when LTO is enabled (it is: `Cargo.toml:27`).

**Files changed:** `src/lib.rs`

---

### Step 5 — Instrument `compute_from_request()` in `c_api.rs`

- [ ] **5.1** Add a debug export block in `src/c_api.rs` inside `compute_from_request()`, after the `scored` vector is populated (after line 354, before the `select_best_grasp` call).

The block does the following:
1. Check `if !config::DEBUG_VISUALIZATION { /* skip */ } else { ... }`
2. Map the private `Vec<ScoredGrasp>` into a `Vec<debug_export::ScoredGraspExport>` by iterating `scored` with `.enumerate()` and using `sample_index = i / 3` (3 grasp types per sample), `grasp_type.to_ffi()` for the int code, and `samples[sample_index].pose.to_se3()` flattened to `[f64; 16]`
3. Build a `DebugDump` referencing the local variables: `&tsdf`, `&pruned`, `&roi`, `&cameras`, `&scored_grasps_export`, `&samples`, `&request.pose`, `&request.twist`
4. Construct output path: `env!("CARGO_MANIFEST_DIR")` + `config::DEBUG_OUTPUT_DIR` + `"grasp_dump_<timestamp>.npz"`
5. Call `debug_export::export_npz(&dump, &path)`
6. Log the result with `eprintln!` (the library doesn't have a logger; `eprintln!` is consistent with the panic-based error handling already used)

**Key consideration:** The export happens *before* `select_best_grasp()`, so it captures the full scored list even when the function later returns an error (e.g., "No collision found"). This is valuable for debugging failed runs.

**Files changed:** `src/c_api.rs`

---

### Step 6 — Add standalone test example

- [ ] **6.1** Create `examples/debug_dump.rs` (or extend `benches/pipeline.rs` with a `--dump` flag).

A minimal binary that:
1. Creates a synthetic sphere point cloud (reuse `demo_sphere()` from `benches/pipeline.rs:40-55`)
2. Sets `DEBUG_VISUALIZATION = true` in config (requires a local override or just calls the export function directly)
3. Runs the full pipeline manually (mirroring `compute_from_request` logic)
4. Calls `debug_export::export_npz()` directly
5. Prints the output path

This allows testing the debug export without ROS2, the C++ bridge, or any hardware.

**Rationale:** The bench file already has all the building blocks (`demo_sphere`, LUT loading, config construction). A separate example file is cleaner than coupling it into the benchmark harness.

**Files created:** `examples/debug_dump.rs`

---

### Step 7 — Create Python visualization script

- [ ] **7.1** Create `scripts/visualize_grasp_debug.py`

**Dependencies:** `numpy`, `pyvista` (pip install; pyvista pulls VTK automatically). Matplotlib as a fallback if pyvista is unavailable.

**CLI interface:**
```
python visualize_grasp_debug.py <path_to_dump.npz> [--grasp-index N] [--threshold T]
```

**Loading logic:**
```python
data = np.load(path)
meta = data["tsdf_metadata"]
# [origin_x, origin_y, origin_z, resolution, width, height, depth]
tsdf = data["tsdf_volume"].reshape(meta[4:7].astype(int))
origin = meta[0:3].astype(float)
resolution = float(meta[3])
pc = data["point_cloud"].reshape(-1, 3)
roi = data["roi_aabb"].reshape(2, 3)  # [min, max]
cams = data["cameras"].reshape(-1, 3)
pose = data["input_pose"]    # [px,py,pz,qx,qy,qz,qw]
twist = data["input_twist"]  # [lx,ly,lz,ax,ay,az]
grasps = data["scored_grasps"].reshape(-1, 22)
```

**Visualization components (all in one PyVista plotter window):**

- [ ] **7.2** TSDF isosurface: Create a `pyvista.ImageData` with dimensions/origin/spacing from metadata. Use `.contour([0.0])` to extract the zero-level set. Render as a smooth gray mesh. Clip voxels where value == `f32::MAX` (~3.4e38) by replacing them with `NaN` before contouring.

- [ ] **7.3** Point cloud: Render as `pyvista.PolyData` with XYZ points, semi-transparent white spheres (`plotter.add_mesh(pc_polydata, style="points", point_size=8, opacity=0.4)`).

- [ ] **7.4** ROI bounding box: Render as a wireframe box using `pyvista.Box(bounds=[min_x,max_x,min_y,max_y,min_z,max_z])`.

- [ ] **7.5** Grasp candidates: For each row in `grasps`, reconstruct the 4x4 pose from columns 6-21. Render as coordinate frame arrows using `pyvista.Arrow` for each axis. Color by `combined_score` (column 6) using a diverging colormap. The best grasp (highest combined_score with `found_collision == 1`) is highlighted in gold with thicker arrows.

- [ ] **7.6** Camera positions: Render as small pyramid glyphs at each camera position.

- [ ] **7.7** Input hand pose: Render as a coordinate frame at `[px,py,pz]` with orientation from `[qx,qy,qz,qw]`. Show twist linear component as an arrow.

- [ ] **7.8** Interactive widgets:
  - Score threshold slider: filters grasps below threshold (hides their arrows)
  - Grasp type checkboxes: toggle cylindrical/pinch/lateral visibility
  - TSDF slice plane: draggable orthogonal plane showing distance field values
  - Point cloud toggle: show/hide

- [ ] **7.9** Summary printout on load:
  ```
  === Grasp Debug Dump Summary ===
  TSDF grid:      60 x 52 x 48 = 149,760 voxels (860 KB)
  Point cloud:    4,823 points (pruned)
  ROI AABB:       [0.02, -0.05, 0.00] → [0.12, 0.05, 0.10] m
  Cameras:        2
  Grasp candidates: 3,000 (1,000 samples × 3 types)
  Best grasp:     cylindrical, closure=0.3421, combined=0.7234
  ```

**Files created:** `scripts/visualize_grasp_debug.py`

---

## Files Changed — Complete Summary

| File | Action | Lines affected | Description |
|------|--------|----------------|-------------|
| `src/config.rs` | Modify | After line 23 | Add `DEBUG_VISUALIZATION: bool`, `DEBUG_OUTPUT_DIR: &str` |
| `src/pointcloud_helper.rs` | Modify | After line 210 | Add 4 public accessor methods to `impl Tsdf` |
| `src/debug_export.rs` | **Create** | New file (~150 lines) | `DebugDump` struct, `ScoredGraspExport` struct, `export_npz()` function |
| `src/lib.rs` | Modify | After line 6 | Add `pub mod debug_export;` |
| `src/c_api.rs` | Modify | After line 354 | Add debug export block inside `compute_from_request()` |
| `examples/debug_dump.rs` | **Create** | New file (~80 lines) | Standalone test that generates a dump with synthetic data |
| `scripts/visualize_grasp_debug.py` | **Create** | New file (~300 lines) | Interactive PyVista visualization script |

**No changes to:** `Cargo.toml` (no new deps), `planner.rs`, `predictor.rs`, `lut_helper.rs`, `benches/`, `nodes/`, `CMakeLists.txt`.

---

## Verification Criteria

- [ ] **VC-1 — Zero overhead when disabled:** With `DEBUG_VISUALIZATION = false`, build in release mode (`cargo build -r`) and verify with `nm target/release/libgrasp_preshaping.so | grep export_npz` that the export function is absent from the binary. LTO + dead code elimination will remove it.

- [ ] **VC-2 — Valid NPZ output:** Set `DEBUG_VISUALIZATION = true`, run the example (`cargo run --example debug_dump`), then verify in Python:
  ```python
  import numpy as np
  d = np.load("data/debug/grasp_dump_*.npz")
  print(list(d.keys()))
  # Should show: ['tsdf_volume', 'tsdf_metadata', 'point_cloud', 'roi_aabb',
  #               'cameras', 'input_pose', 'input_twist', 'scored_grasps']
  assert d["tsdf_volume"].dtype == np.float32
  assert d["scored_grasps"].dtype == np.float64
  ```

- [ ] **VC-3 — Python visualization loads without error:** Run `python scripts/visualize_grasp_debug.py <dump.npz>` and confirm the interactive 3D window opens showing the TSDF isosurface, point cloud, ROI box, and grasp candidates.

- [ ] **VC-4 — Data integrity:** The best grasp identified in the Python visualization (highest `combined_score` among grasps with `found_collision > 0.5`) matches the grasp type and closure amount logged by the Rust pipeline.

- [ ] **VC-5 — No regression:** Benchmark scores from `cargo bench` with `DEBUG_VISUALIZATION = false` are within 2% of current baseline (the const-false branch is eliminated, so this should be 0% regression).

---

## Potential Risks and Mitigations

1. **`npyz::npz::NpzWriter` API shape constraints**
   The `NpzWriter::array()` method returns a builder that requires specifying dtype and shape, then pushing elements. Multi-dimensional shapes must be expressed correctly.
   **Mitigation:** Write all arrays as flat 1D with the Python loader reshaping from metadata. This is the simplest path through the `npyz` write API and avoids shape mismatch bugs.

2. **TSDF volume size on disk**
   At 5mm resolution, a 30cm cube = 60^3 = 216K f32 values = ~860KB raw. NPZ uses ZIP compression (deflate), and TSDF data compresses well (many repeated `f32::MAX` values). Expected compressed size: ~100-300KB.
   **Mitigation:** No action needed — sizes are small. Document expected sizes in the config comment.

3. **File I/O in FFI context**
   The export runs inside `compute_from_request()`, called from the C++ ROS2 bridge under a service callback. File I/O is blocking.
   **Mitigation:** The export is fast (one sequential write of ~1MB). The service is already blocking (synchronous FFI call). No concurrency issue. The `DEBUG_VISUALIZATION = false` path adds zero time.

4. **PyVista availability in Docker environments**
   PyVista requires VTK and a display (or offscreen rendering). The Docker environment may not have this.
   **Mitigation:** The script detects `pyvista` import failure and falls back to a matplotlib-based 3D scatter plot with reduced interactivity. Document both options in the script docstring.

5. **Multiple rapid service calls creating overlapping dumps**
   Each call generates a timestamped filename. If calls arrive sub-second, filenames could collide.
   **Mitigation:** Use millisecond-precision timestamps (`SystemTime::now().duration_since(UNIX_EPOCH)`) and append a counter if needed.

---

## Alternative Approaches Considered

1. **Multiple `.npy` files instead of single `.npz`**: Rejected. More files to manage, harder to share, no compression, Python needs multiple `np.load()` calls. NPZ is strictly better and already supported by the existing dependency.

2. **Cargo feature flag (`#[cfg(feature = "debug_viz")]`) instead of `const bool`**: Considered but rejected for this use case. A feature flag requires `--features debug_viz` at build time, which complicates the Docker build. A `const bool` in `config.rs` matches the existing pattern (all tuning is done in that file) and is equally eliminated by LLVM when `false`.

3. **JSON metadata + raw binary arrays**: Rejected. Would require two parsing paths in Python. NPZ/NPY is the native numpy format and already used throughout the project (LUT files, `model.py`).

4. **ROS2 topic-based streaming**: Rejected. Requires the ROS2 bridge running, adds latency, can't do post-hoc analysis. File-based export works offline and is simpler.
