# Phase 2 OpenVINS EKF Handoff Prompt

Use this prompt in a new chat after the Phase 1 baseline is committed and
pushed.

```text
We have completed Phase 1 external marker covariance validation for the 100 mm
head marker setup in the assistive_multiview_prosthesis repo.

Please read:
- .agents/AGENTS.md
- .agents/marker_pose_covariance_plan.md
- .agents/future_tasks.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_bag_analysis.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_empirical_validation.md
- docker_ws/bags/openvins_tests/head_marker_covariance/analysis_phase1_marker_covariance/analysis_report.md

Important Phase 1 result:
- The external marker covariance model is conservative on stationary 100 mm
  marker repeatability.
- No covariance config tuning was applied.
- Live sanity checking confirmed the expected Phase 1 limitation: when VIO
  drifts and marker ID 0 is reacquired, `/head/marker_pose/ov_corrected_odom`
  snaps back to the marker-consistent pose, but OpenVINS internal covariance
  keeps growing and the velocity estimate can remain/increase inconsistent.
- Phase 1 must remain as the committed baseline.
- Do not modify OpenVINS internals until we agree on a Phase 2 implementation
  plan.

Task:
Create a detailed implementation plan for Phase 2: integrating marker
observations into OpenVINS as a principled EKF update or reanchor/reset path.
The plan should inspect the OpenVINS codebase first, identify the correct
estimator/state/update interfaces, propose how to construct marker measurement
residuals and R_marker from the Phase 1 covariance model, define
gating/rejection behavior, and explain how pose, velocity, and covariance
should be handled without covariance hacking.

Important Phase 2 requirement:
Do not assume a marker pose update alone is sufficient. Explicitly investigate
whether velocity is corrected through EKF cross-correlations, needs a
principled reset/reinitialize path, or needs a separate supported velocity
handling strategy when VIO drift has made the internal velocity inconsistent.

Do not code until the Phase 2 implementation plan is clear.
```
