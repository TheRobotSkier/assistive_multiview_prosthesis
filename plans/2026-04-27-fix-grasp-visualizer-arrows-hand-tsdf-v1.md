# Fix Grasp Debug Visualizer: Arrows, Hand Skeleton, TSDF

## Objective

Fix three critical bugs in `visualize_grasp_debug.py`:
1. Grasp arrows extending far beyond the ROI
2. Hand skeleton not correlating to actual hand geometry
3. TSDF not visible at all

## Root Cause Analysis

### Bug 1: Grasp arrows too long (HIGH)

**Location:** `visualize_grasp_debug.py:323`

The arrow length is computed as `scene_extent * 0.15`. The `scene_extent` is the max range of the point cloud, which for a typical scene might be 0.3m (30cm). This makes `arrow_len = 0.045m = 4.5cm`. The best grasp arrow is then `arrow_len * 1.2 = 5.4cm`. The ROI is typically ~10cm, so arrows at 5.4cm are half the ROI width — they extend well beyond the object.

**Fix:** Scale arrows relative to the ROI size, not the full point cloud extent. Use a smaller fraction (e.g., ROI diagonal * 0.15) and cap the maximum length.

### Bug 2: Hand skeleton wrong indexing (CRITICAL)

**Location:** `visualize_grasp_debug.py:754-756`

The LUT tables are stored as 3D arrays of shape `(resolution, n_contacts_per_table, 8)` in the NPZ file. When `load_finger_lut` flattens them to 1D, the data layout is:

```
[sample_0_contact_0(8), sample_0_contact_1(8), ..., sample_0_contact_N(8),
 sample_1_contact_0(8), sample_1_contact_1(8), ..., sample_1_contact_N(8),
 ...]
```

The correct offset for `(sample_idx, contact_idx)` is:
```python
offset = (sample_idx * n_contacts_per_table + contact_idx) * 8
```

where `n_contacts_per_table` is the TOTAL contacts per sample in that table:
- Index: 8 contacts per sample
- Mrl: 10 contacts per sample  
- ThumbAdd: 3 contacts per sample
- ThumbAbd: 3 contacts per sample
- Palm: 4 contacts (static, no sample dimension)

But the current code at line 756 uses:
```python
offset = (sample_idx * len(contact_indices) + ci) * 8
```

`len(contact_indices)` is the number of contacts being drawn for THIS finger chain, NOT the total contacts per sample in the table. For example:
- For "ring" finger: `contact_indices = [4, 5, 6]`, so `len(contact_indices) = 3`
- But the Mrl table has 10 contacts per sample
- So the code uses stride 3 instead of 10, indexing into completely wrong DQ entries

This means every finger except the first one in each table gets wrong positions.

**Fix:** Use the correct per-table contacts-per-sample counts when computing offsets. Define a constant map:
```python
TABLE_CONTACTS_PER_SAMPLE = {
    "Index": 8,
    "Mrl": 10,
    "ThumbAdd": 3,
    "ThumbAbd": 3,
    "Palm": 4,  # static, no sample dimension
}
```

### Bug 3: TSDF data ordering mismatch (CRITICAL)

**Location:** `visualize_grasp_debug.py:368-372`

The TSDF data in the NPZ file is stored in **C order** (row-major, x-major) by the Rust code at `pointcloud_helper.rs:359-360`:
```rust
let stride_y = width;
let stride_z = width * height;
// index = x + y * stride_y + z * stride_z
```

The Python code reshapes it correctly: `tsdf_flat.reshape(shape)` where `shape = (W, H, D)` — this is C-order reshape matching C-order storage.

But then at line 372, it's assigned to PyVista with:
```python
grid.cell_data["distance"] = tsdf_vis.ravel(order="F")
```

The `order="F"` ravel transposes the data! PyVista ImageData cell data expects values in Fortran order (X varies fastest), but the data is in C order where X already varies fastest. Using `order="F"` on a `(W, H, D)` shaped C-order array produces a different ordering than `order="C"`.

Actually, wait — since the Rust code stores data with X varying fastest (stride_y=width, stride_z=width*height), and numpy's default C-order also has the first axis varying fastest, `tsdf_flat.reshape(shape)` with C order is correct. Then `ravel(order="F")` would reorder the data incorrectly.

PyVista ImageData cell data should be provided in Fortran order (i.e., X varies fastest, then Y, then Z). Since the original data IS already in X-fastest order from Rust, using `ravel(order="C")` (which preserves the memory layout) would be correct. The `order="F"` is wrong here.

**Fix:** Change `ravel(order="F")` to `ravel(order="C")` for the TSDF data assignment. Same fix for the TSDF slice at line 425.

## Implementation Plan

### File: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py`

- [ ] **Task 1. Fix hand skeleton LUT indexing (lines 753-756)**
  Replace `len(contact_indices)` with the correct per-table contacts-per-sample count.
  Add a `TABLE_CONTACTS_PER_SAMPLE` constant dict and use it in the offset calculation.
  Rationale: This is the root cause of the hand skeleton being completely wrong.

- [ ] **Task 2. Fix TSDF data ordering (lines 372, 425)**
  Change `ravel(order="F")` to `ravel(order="C")` for both the main TSDF grid and the slice grid.
  Rationale: The TSDF data is stored in C order (x-fastest) by Rust. PyVista ImageData expects Fortran order for cell data, but since the data already has x varying fastest, C-order ravel preserves the correct layout.

- [ ] **Task 3. Reduce grasp arrow length (line 323)**
  Change arrow_len to be based on ROI diagonal instead of full scene extent, and reduce the fraction from 0.15 to 0.08. Also reduce the best-grasp multiplier from 1.2 to 1.0.
  Rationale: Arrows at 15% of scene extent are too large relative to the ~10cm ROI.

## Verification Criteria

- [ ] Grasp arrows should be clearly visible but stay within or near the ROI bounding box
- [ ] Hand skeleton should show a recognizable hand shape with fingers curling inward at the correct positions relative to the grasp pose
- [ ] TSDF isosurface should render as a visible cyan mesh approximating the object surface
- [ ] TSDF slice (z-midplane) should show a coolwarm heatmap with clear positive/negative regions
- [ ] Script runs without errors on both debug dump files

## Potential Risks and Mitigations

1. **PyVista ImageData cell ordering may differ between versions**
   Mitigation: Test with the actual data file and verify the isosurface appears at the correct location relative to the point cloud.

2. **LUT table structure may change if model.py is updated**
   Mitigation: Add a comment documenting the expected table shapes and add a validation check in `load_finger_lut`.

3. **Arrow scaling may still be wrong for very small or very large ROIs**
   Mitigation: Add a minimum and maximum cap on arrow length.

## Alternative Approaches

1. **For TSDF:** Instead of fixing the ravel order, reshape the data as `(D, H, W)` and then use `ravel(order="F")`. This would also work but is less intuitive.
2. **For hand skeleton:** Instead of computing offsets manually, reshape the flat tables back to `(resolution, n_contacts, 8)` 3D arrays and index directly as `table[sample_idx, contact_idx]`. This is cleaner and less error-prone.
3. **For arrows:** Use a fixed world-space length (e.g., 3cm) instead of a scene-relative length. Simpler but less adaptive.
