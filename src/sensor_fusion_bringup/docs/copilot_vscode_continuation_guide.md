# Copilot VS Code Continuation Guide

Use this when continuing the project in GitHub Copilot Chat or Copilot coding
agent from VS Code.

## Goal

Get useful results from Copilot without spending time rediscovering the current
state of the Jetson/OpenVINS/D435i work.

## Start With The Right Context

Open VS Code at the repository root:

```text
/home/robotlab/Documents/assistive_multiview_prosthesis
```

Pin or open these files before asking Copilot to change anything:

```text
.agents/AGENTS.md
.agents/phase2_openvins_handoff_status.md
.agents/x86_segmentation_pointcloud_plan.md
.agents/next_chat_prompt_d435i_pointcloud_followup_2026-05-14.md
docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md
docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md
```

Tell Copilot to read those files first and to summarize the current baseline
before proposing edits.

## Model Choice

Model availability changes by Copilot plan, organization policy, and VS Code
version. Use the model picker in the Copilot Chat input box and choose from the
best models actually available to you.

Practical defaults:

- Use `Auto` or a fast mini model for small questions, command cleanup, and
  documentation edits.
- Use a strong reasoning/coding model such as `GPT-5.x`, `GPT-5.x Codex`,
  `Claude Sonnet`, or `Claude Opus` for multi-file code changes, ROS launch
  debugging, or anything that may affect OpenVINS timing.
- Avoid letting a weaker/fast model make broad refactors across OpenVINS.
- For low-credit situations, ask for a plan and exact patch targets first, then
  approve only the smallest useful change.

Official references:

- GitHub Copilot supported models:
  https://docs.github.com/en/copilot/reference/ai-models/supported-models
- GitHub Copilot model picker:
  https://docs.github.com/en/copilot/how-tos/use-ai-models/change-the-chat-model
- VS Code language model picker:
  https://code.visualstudio.com/docs/copilot/customization/language-models

## Prompt Template For Copilot

```text
Read these files first:
- .agents/AGENTS.md
- .agents/phase2_openvins_handoff_status.md
- .agents/x86_segmentation_pointcloud_plan.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md

Context:
- ROS 2 Jazzy validation must run inside Docker service `realsense_camera`.
- Do not use host ROS commands.
- Do not commit build_overlay, install_overlay, or log_overlay.
- OpenVINS and RViz2 color pointclouds now work.
- Do not re-enable RealSense infra streams.
- Do not add marker ID2 to marker_fixed_ids.

Task:
Summarize the current baseline in 10 bullets, then propose the smallest safe
next step. Do not edit files until you have identified exact files and commands.
```

## Commands Copilot Should Prefer

Run from:

```bash
cd /home/robotlab/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
```

Use Docker/Jazzy:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=false start_preview:=false start_rviz:=false'
```

Pointcloud-enabled validation:

```bash
docker compose run --rm --name openvins_pc realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true start_preview:=false start_rviz:=false pointcloud_require_marker_map_locked:=false pointcloud_max_rate_hz:=8.0 pointcloud_voxel_leaf_m:=0.02 pointcloud_max_range_m:=2.0 pointcloud_decimation_magnitude:=3 enable_pointcloud_neon_fix:=false'
```

Status check while that container is running:

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && timeout 8 ros2 topic echo /head/d435i_head/points_marker_map/status'
```

Expected healthy status includes:

```text
accepted:true
reason:"published"
reason:"rate_limited"
```

`rate_limited` is expected and healthy when the output rate cap is working.

## Build Safety

OpenVINS builds on the Jetson must be sequential and done while the Jetson is
otherwise idle:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log_overlay build --symlink-install --build-base build_overlay --install-base install_overlay --executor sequential --parallel-workers 1 --packages-select sensor_fusion_bringup ov_msckf'
```

If only launch/docs/config in `sensor_fusion_bringup` changed:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log_overlay build --symlink-install --build-base build_overlay --install-base install_overlay --executor sequential --parallel-workers 1 --packages-select sensor_fusion_bringup'
```

## How To Ask For Code Changes

Good prompt:

```text
Make the smallest change needed to solve this. Before editing, list the exact
files you will touch and why. After editing, run only syntax/build checks that
are safe on the Jetson. Do not touch generated overlay files.
```

For reviews:

```text
Review this diff for bugs, timing regressions, ROS QoS/TF issues, and missing
validation. Give findings first with file and line references.
```

For drift work:

```text
Do not change pointcloud code. Focus only on OpenVINS drift/reanchor behavior.
Use the already validated pointcloud branch as the baseline and propose a bagged
test plan before editing estimator code.
```

## Commit Checklist

Before committing:

```bash
git status --short
git diff --check
```

Do not commit:

```text
docker_ws/build_overlay/
docker_ws/install_overlay/
docker_ws/log_overlay/
```

Commit source/config/docs/agent notes only. Suggested commit message:

```text
Add Jetson-safe D435i marker-map pointcloud support
```

## Segmentation Direction

Use `sensor_msgs/msg/PointCloud2` as the segmentation input contract, not a
custom voxel-grid contract. Voxel filtering/downsampling should be a preprocessing
choice before publishing:

```text
/segmentation/input_cloud
```

First x86 integration should subscribe to the validated Jetson topics:

```text
/head/d435i_head/points_marker_map
/arm/d435i_arm/points_marker_map
```

If Jetson load becomes too high, move the transform/merge/downsample pipeline to
the x86 PC later by subscribing to raw RealSense pointclouds and TF.
