# Performance Regression Reversion Plan

## Objective

To analyze and resolve the performance degradation between the older grasp preshaping package (6.5s for 1M samples) and the current version (112s for 1M samples) while retaining the benefits of the newly introduced SMC sampling and superquadric backside estimation features.

## Implementation Plan

- [ ] Task 1. **Restore O(1) Camera Occlusion Checking in TSDF**: Revert the `O(V * N)` nested loop in `get_tsdf` (`pointcloud_helper.rs`) back to using the pre-computed `nearest` surface points. This eliminates evaluating thousands of surface points per voxel per camera, returning the algorithm to `O(V)` complexity while preserving the multi-camera tiebreaker logic.
- [ ] Task 2. **Optimize Superquadric Voxel Blending**: Restrict the evaluation of `sq.taubin_distance(vw)` in `get_tsdf` to only the voxels that actually require sign correction or distance blending (e.g., voxels near the truncation band or experiencing camera disagreement), rather than unconditionally evaluating the heavy math functions for every single voxel in the grid.
- [ ] Task 3. **Optimize Proximity Scoring for Non-Colliding Particles**: Refactor the Tier 4 (No Collision) `proximity_score` calculation in `planner.rs`. Instead of evaluating `pos_at_control` and TSDF distances for all 25 sweep points, use a simplified subset (e.g., 3-5 key points like palm and finger bases) or rely solely on the palm center to provide the gradient direction for the optimizer.
- [ ] Task 4. **Optimize Missing Finger Soft Rejection**: Refactor the Tier 2 `missing_dist` calculation. When a grasp is softly rejected for missing thumb/index constraints, approximate the distance using fewer sample points or avoid performing full TSDF lookups for the entire finger chain.
- [ ] Task 5. **Reintroduce TSDF Query Clamping**: Re-enable grid boundary clamping in `Tsdf::get_distance` (`pointcloud_helper.rs`) instead of immediately returning `f32::MAX`. This allows the collision sweep to detect border collisions and short-circuit earlier rather than running the full 0-to-max sweep length for out-of-bounds poses.

## Verification Criteria

- [Benchmark `bench_tsdf_with_superquadric` executes with minimal overhead compared to `bench_tsdf_construction`]
- [Benchmark `bench_full_pipeline` (without SMC) returns to ~6-7 seconds for 1M evaluations]
- [The SMC optimizer still successfully navigates toward valid grasps using the optimized Tier 2 and Tier 4 gradient signals]

## Potential Risks and Mitigations

1. **SMC Optimization Collapse**
   Mitigation: Carefully monitor the SMC convergence rate when simplifying the `proximity_score` and `missing_dist` signals to ensure the optimizer still receives enough directional gradient to find the object.
2. **Superquadric Tiebreaker Inaccuracy**
   Mitigation: If limiting SQ evaluations creates visual artifacts or sign errors on the backside, fall back to evaluating the SQ only when the voxel is explicitly marked as "unseen" or in conflict, rather than evaluating the entire grid.

## Alternative Approaches

1. **Voxel Hierarchies**: Implement an Octree or BVH for the TSDF grid to allow the camera ray checks and superquadric distance math to skip large empty regions entirely.
2. **GPU Acceleration**: Port the TSDF construction and particle scoring loops to CUDA/OpenCL, taking advantage of parallel hardware to handle the heavy ray/SQ computations.

User:
I have a few inputs:
Task 1: Seems highly relevant, i wonder what the feature degradations of this refactor will be?
Task 2: Very good idea. I wonder if we could just use the fact that we can easily calculate the sign from the inital point cloud, and have all the negative values be the only ones we check the superquadric distance to. I know this will mean kinda chaning the whole authority domain idea, but it might be a simple way to do smarter superquadric checks without needing to do a full grid evaluation. Try to figure out if this is a good idea performance wise, or if there is some issue with removing the autority domains. I thin kwe could still do a blend of sorts.
Task 3: Very good idea. The reason we do this is to have a more informative and dense cost for the SMC. I think we can approximate very well, by just doing a distance to maybe center of PC or something like that. I belive we already have the pc center, but i am also nervous that this will give the score less information, so we have less of aa accurate score to follow. Though right now i think i would rather speed this up so i can add more samples, than try to be too smart about things.
Task 4: I am not totally sure what this would mean, or what it would do. Be carefull about not doing degradations in other areas, and that they are evaluted for the right tradeoff.
Task 5: I also donmt really get this. I tnik we can be out of tsdf bounds and still enter onse we sweep the finger, but maybe you are talking about a different sweep?

I ran the benchmarks for both versions with 10k samples and got the following (don't pay attention to the change, or some of the naming of the becnes):

daniel:grasp_preshaping_old% cargo bench -q                                                         󱙺 FORGE  gpt-5.4-mini MEDIUM

running 29 tests
iiiiiiiiiiiiiiiiiiiiiiiiiiiii
test result: ok. 0 passed; 0 failed; 29 ignored; 0 measured; 0 filtered out; finished in 0.00s

Gnuplot not found, using plotters backend
roi_prediction_1000_samples
                        time:   [2.7981 ms 2.9037 ms 3.0387 ms]
                        change: [-99.172% -99.133% -99.084%] (p = 0.00 < 0.05)
                        Performance has improved.
Found 12 outliers among 100 measurements (12.00%)
  5 (5.00%) high mild
  7 (7.00%) high severe

pointcloud_prune_to_roi_5000pts
                        time:   [1.9795 ms 2.0292 ms 2.0748 ms]
                        change: [-7.1490% -3.4254% +0.3422%] (p = 0.07 > 0.05)
                        No change in performance detected.
Found 7 outliers among 100 measurements (7.00%)
  3 (3.00%) low severe
  4 (4.00%) low mild

tsdf_construction_5000pts
                        time:   [2.5548 ms 2.6321 ms 2.7118 ms]
Found 8 outliers among 100 measurements (8.00%)
  5 (5.00%) low mild
  2 (2.00%) high mild
  1 (1.00%) high severe

Benchmarking full_pipeline_predict_to_score: Warming up for 3.0000 s
Warning: Unable to complete 100 samples in 5.0s. You may wish to increase target time to 13.9s, or reduce sample count to 30.
full_pipeline_predict_to_score
                        time:   [112.14 ms 119.80 ms 126.95 ms]
Found 14 outliers among 100 measurements (14.00%)
  7 (7.00%) low severe
  6 (6.00%) high mild
  1 (1.00%) high severe

score_cylindrical       time:   [14.838 µs 15.629 µs 16.539 µs]
Found 7 outliers among 100 measurements (7.00%)
  4 (4.00%) high mild
  3 (3.00%) high severe

score_pinch             time:   [9.2410 µs 9.8937 µs 10.647 µs]
Found 9 outliers among 100 measurements (9.00%)
  5 (5.00%) high mild
  4 (4.00%) high severe

score_lateral           time:   [9.9384 µs 10.984 µs 12.222 µs]
Found 9 outliers among 100 measurements (9.00%)
  3 (3.00%) high mild
  6 (6.00%) high severe

daniel:grasp_preshaping_old% cd ../grasp_preshaping/                                                󱙺 FORGE  gpt-5.4-mini MEDIUM
daniel:grasp_preshaping% cargo bench -q                                                             󱙺 FORGE  gpt-5.4-mini MEDIUM

running 51 tests
iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
test result: ok. 0 passed; 0 failed; 51 ignored; 0 measured; 0 filtered out; finished in 0.00s

Gnuplot not found, using plotters backend
roi_prediction          time:   [3.0634 ms 3.2149 ms 3.3945 ms]
                        change: [+36.891% +49.197% +61.534%] (p = 0.00 < 0.05)
                        Performance has regressed.
Found 8 outliers among 100 measurements (8.00%)
  2 (2.00%) high mild
  6 (6.00%) high severe

pointcloud_prune_to_roi_5000pts
                        time:   [2.0307 ms 2.1177 ms 2.2185 ms]
                        change: [-12.729% -8.4766% -3.5571%] (p = 0.00 < 0.05)
                        Performance has improved.
Found 1 outliers among 100 measurements (1.00%)
  1 (1.00%) high severe

tsdf_construction_5000pts
                        time:   [2.5417 ms 2.5913 ms 2.6372 ms]
                        change: [-13.291% -10.859% -8.6542%] (p = 0.00 < 0.05)
                        Performance has improved.
Found 6 outliers among 100 measurements (6.00%)
  1 (1.00%) low severe
  4 (4.00%) low mild
  1 (1.00%) high mild

Benchmarking full_pipeline_predict_to_score: Warming up for 3.0000 s
Warning: Unable to complete 100 samples in 5.0s. You may wish to increase target time to 11.8s, or reduce sample count to 40.
full_pipeline_predict_to_score
                        time:   [120.54 ms 126.39 ms 132.86 ms]
                        change: [+43.409% +53.412% +64.065%] (p = 0.00 < 0.05)
                        Performance has regressed.
Found 6 outliers among 100 measurements (6.00%)
  3 (3.00%) high mild
  3 (3.00%) high severe

Benchmarking smc_pipeline: Warming up for 3.0000 s
Warning: Unable to complete 100 samples in 5.0s. You may wish to increase target time to 75.1s, or reduce sample count to 10.
smc_pipeline            time:   [675.37 ms 689.19 ms 703.50 ms]
                        change: [+51.518% +56.507% +61.100%] (p = 0.00 < 0.05)
                        Performance has regressed.
Found 2 outliers among 100 measurements (2.00%)
  2 (2.00%) high mild

resample_around_elites_1000_from_100
                        time:   [265.82 µs 276.46 µs 289.33 µs]
                        change: [-47.502% -41.221% -34.483%] (p = 0.00 < 0.05)
                        Performance has improved.
Found 8 outliers among 100 measurements (8.00%)
  6 (6.00%) high mild
  2 (2.00%) high severe

score_cylindrical       time:   [11.762 µs 12.290 µs 12.959 µs]
                        change: [-17.615% -10.652% -3.5958%] (p = 0.01 < 0.05)
                        Performance has improved.
Found 12 outliers among 100 measurements (12.00%)
  5 (5.00%) high mild
  7 (7.00%) high severe

score_pinch             time:   [8.2419 µs 8.9752 µs 9.9435 µs]
                        change: [-39.913% -32.556% -24.804%] (p = 0.00 < 0.05)
                        Performance has improved.
Found 12 outliers among 100 measurements (12.00%)
  5 (5.00%) high mild
  7 (7.00%) high severe

score_lateral           time:   [7.6719 µs 8.1972 µs 8.8480 µs]
                        change: [-33.153% -26.138% -18.021%] (p = 0.00 < 0.05)
                        Performance has improved.
Found 11 outliers among 100 measurements (11.00%)
  4 (4.00%) high mild
  7 (7.00%) high severe

superquadric_fitting_5000pts
                        time:   [3.2868 ms 3.4024 ms 3.5416 ms]
                        change: [-34.087% -31.355% -28.325%] (p = 0.00 < 0.05)
                        Performance has improved.
Found 3 outliers among 100 measurements (3.00%)
  2 (2.00%) high mild
  1 (1.00%) high severe

tsdf_construction_with_superquadric_5000pts
                        time:   [7.7145 ms 7.9288 ms 8.1461 ms]
                        change: [-22.403% -20.037% -17.472%] (p = 0.00 < 0.05)
                        Performance has improved.

So try to have a deeper look into the versions and performance regressions. And make a detailed plan on how you sugegst we fix the degradations, and what the tradeoffs are. I think we can get a lot of the performance back without losing much of the benefits of the new features, but we should be careful about how we do it, and make sure we are not just doing a blind reversion. We should also consider if there are any alternative approaches that could be taken to improve performance without reverting changes.