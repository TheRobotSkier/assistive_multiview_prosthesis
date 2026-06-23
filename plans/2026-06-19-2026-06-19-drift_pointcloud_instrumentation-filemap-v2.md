# Drift Root-Cause + Pointcloud Recovery — File-by-File Execution Plan (Host + Jetson)

## Objective
Create an implementation-ready execution plan that maps each diagnostic and stabilization step to exact files in the host repository and Jetson repository, so implementation can proceed with minimal ambiguity.

## Implementation Plan

- [ ] **Task 1 (Status: Not Started) — Establish baseline artifacts and comparison targets.**  
  Rationale: all instrumentation changes must be compared against a fixed failing run to confirm diagnostic value.
  - Files to use:
    - Host: `logs/host-log-20260619_164708.txt`
    - Jetson: `logs/jetson-run-jetson-debug-20260619_184710.txt`
    - Host analyzer outputs: `logs/host-log-20260619_164708_analysis.txt`, `logs/jetson-run-jetson-debug-20260619_184710_analysis.txt`

- [ ] **Task 2 (Status: Not Started) — Add startup-phase boundaries so expected pre-init warnings are downgraded.**  
  Rationale: startup “no odom yet” is expected in your workflow and should not be ranked as steady-state fault.
  - Jetson implementation files:
    - `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
  - Host analyzer files:
    - `scripts/analyze_log.py`
  - Output sections to update:
    - startup vs steady-state fault categorization in log report

- [ ] **Task 3 (Status: Not Started) — Add structured VIO transition telemetry (head/arm).**  
  Rationale: valid→invalid and invalid→valid transitions plus dwell-time expose instability onset better than end snapshots.
  - Jetson implementation files:
    - `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
    - `../multiview_prosthesis-jetson_docker/docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp`
  - Host analyzer files:
    - `scripts/analyze_log.py`

- [ ] **Task 4 (Status: Not Started) — Add innovation-trend and reject-burst telemetry.**  
  Rationale: correlate pre-failure innovation growth with reject storms to identify first-cause drift trigger.
  - Jetson implementation files:
    - `../multiview_prosthesis-jetson_docker/docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.cpp`
    - `../multiview_prosthesis-jetson_docker/docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.cpp`
    - `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
  - Host analyzer files:
    - `scripts/analyze_log.py`

- [ ] **Task 5 (Status: Not Started) — Add runaway detector events (velocity plausibility + pos_norm slope).**  
  Rationale: encode the observed failure mode explicitly so reports flag onset, not only aftermath.
  - Jetson implementation files:
    - `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
  - Host analyzer files:
    - `scripts/analyze_log.py`
    - `scripts/analyze_bag.py`

- [ ] **Task 6 (Status: Not Started) — Add reanchor-effectiveness metrics.**  
  Rationale: distinguish healthy recovery from immediate re-divergence loops.
  - Jetson implementation files:
    - `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
  - Host analyzer files:
    - `scripts/analyze_log.py`

- [ ] **Task 7 (Status: Not Started) — Harden DIAG block parsing and timeline integrity.**  
  Rationale: avoid mixed-source contamination in chain/timeline sections and preserve trustworthy diagnostics.
  - Host implementation files:
    - `scripts/analyze_log.py`
  - Producer-side file to align with parser expectations:
    - `src/camera/camera/pipeline_diagnostics_node.py`

- [ ] **Task 8 (Status: Not Started) — Instrument host pointcloud pipeline stage counters and reason codes.**  
  Rationale: isolate exactly where `/jetson/*/points` drops to zero when depth/rgb inputs are present.
  - Host launch/wiring files:
    - `src/prosthesis_launch/launch/pipeline.launch.py`
  - Host runtime bridge files:
    - `src/camera/camera/decompress_bridge.py`
  - Host diagnostics producer:
    - `src/camera/camera/pipeline_diagnostics_node.py`
  - Host analyzer files:
    - `scripts/analyze_bag.py`
    - `scripts/analyze_log.py`

- [ ] **Task 9 (Status: Not Started) — Add POINTCLOUD CHAIN HEALTH report section.**  
  Rationale: summarize per-side `depth_in → rgb_in → sync_match → points_out` with dominant failure reason in one place.
  - Host analyzer files:
    - `scripts/analyze_bag.py`
    - `scripts/analyze_log.py`

- [ ] **Task 10 (Status: Not Started) — Add run-level pass/fail gates tied to project goals.**  
  Rationale: enforce deterministic verdicts aligned to your priorities (realistic velocity, bounded drift, non-zero points).
  - Host analyzer files:
    - `scripts/analyze_bag.py`
    - `scripts/analyze_log.py`

- [ ] **Task 11 (Status: Not Started) — Validate with A/B replay and compare report quality.**  
  Rationale: prove instrumentation adds causal clarity and does not only add volume.
  - Inputs:
    - baseline failing run artifacts from Task 1
  - Outputs:
    - before/after report comparison with first-trigger attribution and pointcloud-stage localization

## Verification Criteria

- [ ] Startup-phase warnings are clearly separated from steady-state faults.
- [ ] Drift timeline shows first trigger chain: reject burst → VIO transition → runaway event → reanchor outcome.
- [ ] Reports include per-side valid/invalid transition counts and dwell durations.
- [ ] Pointcloud chain report identifies the failing stage per side when output is absent.
- [ ] New run verdict includes pass/fail gates for velocity plausibility, drift bounds, and pointcloud availability.

## Potential Risks and Mitigations

1. **Risk: Instrumentation adds excessive log noise.**  
   Mitigation: use structured summary events and windowed aggregation instead of per-message spam.

2. **Risk: Cross-repo changes drift out of sync (Jetson emits fields host parser does not consume).**  
   Mitigation: define field schema first, then update parser and emitter in paired commits.

3. **Risk: Pointcloud absence has multiple overlapping causes (sync + QoS + lifecycle).**  
   Mitigation: enforce stage counters and reason codes so each drop is attributed to one dominant stage.

## Alternative Approaches

1. **Drift-first sequencing:** complete Tasks 2–7 before pointcloud tasks.  
   Trade-off: fastest estimator root-cause visibility, slower pointcloud restoration.

2. **Pointcloud-first sequencing:** complete Tasks 8–9 first.  
   Trade-off: fast functional cloud output recovery, weaker immediate drift attribution.

3. **Balanced parallel sequencing (recommended):** run drift and pointcloud instrumentation in parallel with shared report updates.  
   Trade-off: moderate scope, highest end-to-end diagnostic gain.
