# Marker Covariance Bag Analysis

Run this after recording the 100 mm marker covariance bags. The analysis uses
only Phase 1 external marker outputs and does not modify OpenVINS internals.

## Command

Run inside the ROS 2 Jazzy `realsense_camera` Docker container after sourcing
the normal workspace overlays. After rebuilding the overlay, use:

```bash
ros2 run sensor_fusion_bringup analyze_marker_covariance_bags.py \
  --bag-root /miahand_ws/src/bags/openvins_tests/head_marker_covariance \
  --config /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
```

For a source-tree run before rebuilding the overlay, use:

```bash
python3 /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/scripts/analyze_marker_covariance_bags.py \
  --bag-root /miahand_ws/src/bags/openvins_tests/head_marker_covariance \
  --config /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
```

By default, outputs are written under:

```text
/miahand_ws/src/bags/openvins_tests/head_marker_covariance/analysis_phase1_marker_covariance
```

## Outputs

- `metadata_summary.csv`: bag durations and topic message counts.
- `all_marker_detections.csv`: flattened `/head/marker_pose/all_marker_quality`
  detections with stationary residuals and offline current-model predictions.
- `stationary_repeatability.csv`: per `bag + marker_id` empirical pose jitter.
- `binned_empirical_covariance.csv`: empirical jitter binned by distance, side
  pixel size, view angle, reprojection error, and marker ID.
- `prediction_coverage.csv`: measured residual coverage against predicted
  current-model standard deviations.
- `published_marker_quality_summary.csv`: active-marker published covariance
  summary from `/head/marker_pose/marker_quality`.
- `validation_summary.csv`: corrected odom, marker-valid, VIO-valid, and
  reanchor behavior summaries for the final validation bags.
- `validation_event_timeline.csv`: ordered reanchor/correction events.
- `recommended_covariance_config.csv` and `analysis_report.md`: conservative
  config-only tuning recommendation and compact review report.

## Interpretation

Use the stationary estimation bags as the calibration target for marker-pose
repeatability. The script treats the first two recordings per near/medium/far
setup as calibration and the third as holdout.

The generated recommendation only increases gains when calibration data shows
the current model underpredicts jitter. It does not lower covariance floors
automatically; lowering floors should be a manual decision after checking
holdout coverage and final validation behavior.

For the 2026-05-07 recorded 100 mm marker bags, the generated report showed the
current Phase 1 covariance model is conservative on stationary repeatability.
No covariance config tuning was applied from that analysis. One final live
Phase 1 validation pass should still be recorded and reviewed before starting
OpenVINS-internal Phase 2 work.
