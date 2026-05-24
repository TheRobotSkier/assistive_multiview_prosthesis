# Fix Voxel Downsample RGB Color Corruption

## Objective

Fix the `_voxel_downsample` function in `pointcloud_fusion_node.py` so that RGB color averaging during voxel downsampling is done per-channel instead of on the packed integer, eliminating color noise and cross-channel contamination.

## Implementation Plan

- [x] **1. Modify `_voxel_downsample` to unpack, average, and repack RGB channels independently.** In `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:127-154`, replace the packed-integer averaging logic (lines 145-153) with per-channel averaging. Specifically:
  - Unpack `rgb_packed` into three separate (N,) uint8 arrays: R = `(rgb_packed >> 16) & 0xFF`, G = `(rgb_packed >> 8) & 0xFF`, B = `rgb_packed & 0xFF`
  - Cast each to uint64 and use `np.add.at` to accumulate per-voxel sums (three separate accumulator arrays, or a single (n_voxels, 3) array)
  - Divide each accumulator by `counts` to get the per-channel mean
  - Round/truncate back to uint8 and repack: `(R_avg.astype(np.uint32) << 16) | (G_avg.astype(np.uint32) << 8) | B_avg.astype(np.uint32)`
  - Return the repacked (n_voxels,) uint32 array as `rgb_out`

- [x] **2. Add a unit test for `_voxel_downsample` RGB correctness.** Create or extend a test file for `pointcloud_fusion_node` that verifies:
  - Two points in the same voxel with colors pure red (`0x00FF0000`) and pure blue (`0x000000FF`) average to `0x007F0080` (R=127, G=0, B=128) — not a carry-corrupted value.
  - Multiple points with high blue channel values (e.g., B=200) don't carry into the green channel.
  - Single-point voxels preserve color exactly.

## Verification Criteria

- [x] `_voxel_downsample` produces correct per-channel averaged colors with no cross-channel contamination
- [x] No carry propagation from blue→green or green→red channels during averaging
- [ ] The published `/fused_pointcloud` colors appear visually correct and noise-free when voxel downsampling is enabled
- [ ] Existing point count and XYZ averaging behavior is unchanged

## Potential Risks and Mitigations

1. **Performance regression from per-channel processing**: The new approach uses three accumulator arrays instead of one. Mitigation: The arrays are small (n_voxels × 3 uint64), and the bit-shift/unpack/shift-or/repack operations are vectorized numpy — performance impact should be negligible compared to the `np.add.at` calls which dominate.
2. **Downstream consumers expecting the old (broken) format**: No risk — the output is still a valid `0x00RRGGBB` packed uint32; it's just now mathematically correct.

## Alternative Approaches

1. **Float32 RGB averaging**: Unpack to float32 [0,1] per channel, average, then repack. This is what the segmentation node does when parsing (see `segmentation_ros2_node.py:70-75`). Slightly more expensive due to float conversion but avoids any integer rounding bias. Trade-off: marginally slower but more precise.
2. **Median instead of mean for color**: Use the color of the first point in each voxel (what `np.unique` with `return_index=True` already provides). Zero-cost (no averaging at all), but discards color information from other points in the voxel. Trade-off: faster but less smooth color transitions.
