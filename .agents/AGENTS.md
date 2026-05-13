# Agent Notes

Repo working directory for ROS work:
`~/Documents/assistive_multiview_prosthesis/docker_ws`

Docker compose directory:
`~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment`

ROS 2 Jazzy is inside the `realsense_camera` Docker container, not on the host.
The host may have ROS 2 Humble, so do not use host ROS commands for Jazzy
validation. Validate builds, launches, and ROS package discovery through
Docker/Jazzy.

Active source package:
`multi_cam_localization/sensor_fusion_bringup`

Generated overlay artifacts must not be committed:
`build_overlay/`, `install_overlay/`, `log_overlay/`

If Docker creates root-owned overlay files, fix ownership only for generated
overlay directories from `docker_ws`:

```bash
sudo chown -R "$(id -u):$(id -g)" build_overlay install_overlay log_overlay
```

When a task requires the user to run many validation commands, create or update
a command recipe markdown file. Use user-facing docs for repeatable project
procedures, for example
`multi_cam_localization/sensor_fusion_bringup/docs/head_openvins_marker_bag_replay_validation.md`.
Use `.agents/` notes for agent-only handoff context.

Head D435i:
- Serial: `336222071386`
- OpenVINS config: `multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/`

Arm D435i:
- Serial: `310622071850`
- Calibration output: `calibration/arm/d435i_310622071850/`

Dynamic ID2 arm workflow:
- User-facing config:
  `multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml`
- One-command launch:
  `multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`
- Current default is `mode: active`, which enables fixed ID0 updates/reanchor
  plus dynamic ID2 normal updates and guarded dynamic ID2 initial-lock/reanchor.
- Roll back with `mode:=observe`, `mode:=update`, `mode:=would_reanchor`, or
  full dynamic disable via `use_dynamic_arm_pose_updates:=false`.
- Keep ID2 separate from fixed marker maps. Do not add ID2 to `marker_fixed_ids`.
- Before treating live dynamic ID2 reanchor as validated, require a bag where
  `/ov_msckf_arm/odomimu`, `/arm/marker_pose/dynamic_arm_pose_observation`, and
  `/ov_msckf_arm/dynamic_arm_update/status` overlap.

Phase 1 marker correction is external only. Do not modify the OpenVINS internal
EKF until the external marker-corrected odom path is validated and committed.

Marker pose frame conventions:
- `marker_map` should be z-up/navigation-compatible for RViz and later
  point-cloud fusion.
- `camera_pose_raw` is the raw ROS/OpenCV optical frame:
  `+X` right, `+Y` down, `+Z` forward through the lens.
- `camera_body_pose` is visualization-only:
  `+X` forward, `+Y` left, `+Z` up.
- `imu_pose` is the true calibrated IMU pose and should not be hacked for
  display.


When you think performance would improve by starting a fresh Codex chat/agent, tell me explicitly and provide a ready-to-copy handoff prompt for the next agent.

Tell the user when it is a good point to commit and push to GitHub. Include: which files should be committed, which files should not be committed, cleanup commands for generated artifacts, and a suggested commit message. Never stage, commit, push, stash, or clean broad file sets without user approval.
