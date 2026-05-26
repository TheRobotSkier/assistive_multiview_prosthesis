# Reduce Color Trust in Pointcloud Registration

## Objective

The color-only coarse alignment path (`_coarse_color_only`) and hard color inlier filters in both `_coarse_global_align` and `_icp_refine` are producing false-positive matches — matching similarly-colored points on different surfaces, yielding physically-impossible rotations (145-161 degrees). The fix keeps color as a soft hint for steering correspondences, but removes hard color filters and the pure color-only path.

## Implementation Plan

- [ ] **Task 1: Remove the pure color-only coarse path from `_quality_register`.**
  Status: Not Started.
  Rationale: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:1155-1183` currently branches into `_coarse_color_only` when `initial_offset > 0.15` and color is available. This path matches source points to target neighbors purely on Lab color, ignoring spatial position entirely. It produced the 145-161 degree bogus registrations by matching similarly-colored points on geometrically unrelated surfaces. Replace the entire `if has_color and initial_offset > 0.15: ... if global_fit is None:` block with a single call to `_coarse_global_align(...)`, always passing through Lab data when available. The combined XYZ+weighted-Lab tree in `_coarse_global_align` still uses color as a hint, but spatial position dominates.

- [ ] **Task 2: Reduce `color_weight` from 0.003 to 0.0005.**
  Status: Not Started.
  Rationale: `color_weight` controls how much Lab color contributes to the combined XYZ+Lab KD-tree used in `_coarse_global_align` for finding 3-point correspondences. At 0.003, a delta-E of 50 in Lab space contributes 0.15m to the combined distance — comparable to the 0.08m spatial correspondence threshold. At 0.0005, the same delta-E contributes only 0.025m, making spatial position ~6x more important than color. This keeps color as a weak hint. Change the default in `config/prosthesis_config.yaml:256` and in `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:410`.

- [ ] **Task 3: Remove hard color inlier filter from `_coarse_global_align` scoring loop.**
  Status: Not Started.
  Rationale: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:1511-1518` applies `spatial_mask & color_mask` as a hard inlier test. This means a hypothesis can be rejected if the colors don't match, even if the geometry is correct. Replace `inlier_mask = spatial_mask & color_mask` with `inlier_mask = spatial_mask`. Keep color as a soft tiebreaker: when two hypotheses have equal spatial inliers and equal RMSE, prefer the one with better color consistency. Add a `color_inliers` count to the best-tracking logic at lines 1526-1535.

- [ ] **Task 4: Remove hard color inlier filter from `_coarse_global_align` full validation.**
  Status: Not Started.
  Rationale: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:1551-1557` applies the same hard color filter on the full validation pass. Replace `full_inlier_mask = full_spatial_mask & full_color_mask` with `full_inlier_mask = full_spatial_mask`.

- [ ] **Task 5: Remove hard color filter from `_icp_refine`.**
  Status: Not Started.
  Rationale: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:1590-1597` filters each ICP iteration's correspondences by `spatial & color`. This prevents ICP from using geometrically plausible correspondences that happen to have different colors. Remove the `mask = mask & color_mask` line and the color distance computation.

- [ ] **Task 6: Keep `_coarse_color_only` code in the file but document it as unused.**
  Status: Not Started.
  Rationale: The method at `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:1271-1407` may be useful later if OpenVINS accuracy improves or the color-only logic is redesigned. Add a comment above the method noting it is not currently called.

- [ ] **Task 7: Validate syntax and tests.**
  Status: Not Started.
  Rationale: Run `python3 -m py_compile` and `pytest` to confirm no regressions.

## Verification Criteria

- [ ] No more `Color-only coarse succeeded` log messages
- [ ] Registration no longer produces rotations > 30 degrees
- [ ] `registration_used` counter still increments when geometry-based alignment succeeds
- [ ] `proc_avg_ms` stays in a reasonable range (color filtering was adding computation)
- [ ] All 8 existing tests pass

## Potential Risks and Mitigations

1. **Risk: Without color-only coarse path, registration may fail more often when OpenVINS jumps 1.5-2m.**
   Mitigation: This is expected and honest. If the clouds are 2m apart, spatial nearest-neighbor correspondence is meaningless, and color-only correspondence was producing wrong results. The correct fix is better OpenVINS accuracy or a more sophisticated feature-based registration (Open3D FPFH), not trusting weak color cues to recover from large spatial errors.

2. **Risk: Reducing color_weight may make the combined-tree correspondences no better than pure spatial NN.**
   Mitigation: At 0.0005, color still has a measurable effect — a red point will prefer a red spatial neighbor over a blue one. It just won't override a 0.05m spatial difference. This is the right balance for a hint.

3. **Risk: ICP may converge more slowly without color-filtered correspondences.**
   Mitigation: Spatial correspondences with the existing 0.05m distance threshold are already a strong filter. ICP is local by nature and color filtering was adding unnecessary rejection of valid correspondences.
