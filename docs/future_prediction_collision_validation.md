# Future Pose Prediction & Collision — Validation Guide

## Prerequisites
- Jetson running `jetson_docker` branch, publishing:
  - `/ov_msckf_arm/odomimu` (~200 Hz)
  - `/arm/d435i_arm/points_marker_map` (~8 Hz)
  - `/tf`, `/tf_static`
- x86 PC running `work/cleaned_full_test_20260515` branch
- Docker container `prosthesis` (host network, CycloneDDS)

## 1. Build

```bash
cd ~/Documents/GitHub/assistive_multiview_prosthesis/docker
docker compose build prosthesis
docker compose up -d prosthesis
docker exec -it prosthesis bash -c \
  'source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && colcon build --symlink-install --packages-select sensor_fusion_msgs sensor_fusion_bringup'
```

Expected: `Summary: 2 packages finished [time]`

## 2. Launch — disabled (safe default)

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py'
```

Expected output:
```
[future_pose_prediction_collision]: Future pose prediction node started — prediction=False, collision=False, ...
```

Status check:
```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 topic echo --once /prediction/future_trajectory/status'
```

Expected: `enable_prediction: false`

No messages should appear on `/prediction/future_trajectory` or `/segmentation/click_positive`.

## 3. Launch — prediction enabled

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py enable_prediction:=true'
```

### 3a. Check message type

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 interface show sensor_fusion_msgs/msg/FuturePoseTrajectory'
```

### 3b. Check cadence

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 topic hz /prediction/future_trajectory'
```

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 topic hz /arm/d435i_arm/points_marker_map'
```

Expected: Both ~8 Hz.

### 3c. Check frame

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 topic echo --once /prediction/future_trajectory --field header.frame_id'
```

Expected: `marker_map`

### 3d. Check status

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 topic echo /prediction/future_trajectory/status'
```

Expected: JSON with `accepted`, `reason`, `odom_age_s`, `num_poses`, `num_cloud_points`.

## 4. Launch — prediction + collision enabled

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py enable_prediction:=true enable_collision_check:=true'
```

### 4a. Watch for clicks

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 topic echo /segmentation/click_positive'
```

When the predicted trajectory intersects a known pointcloud cluster, a PointStamped click is published with `frame_id: marker_map`.

### 4b. Check click status

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 topic echo /segmentation/click_positive/status'
```

## 5. RViz

```bash
docker exec -it prosthesis rviz2
```

- Fixed frame: `marker_map`
- Add display: `By topic /prediction/future_trajectory` → Path
- Add display: `By topic /segmentation/click_positive` → PointStamped

## 6. Launch args

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py --show-args'
```

Expected:
```
Arguments (pass arguments as '<name>:=<value>'):
    'enable_prediction': Master enable for future trajectory prediction.
    'enable_collision_check': Master enable for proximity/collision checks.
    'use_sim_time': Use simulation (bag) time.
```

## 7. Run tests

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && python3 -m pytest /prosthesis_ws/src/sensor_fusion_bringup/test/test_future_prediction_collision.py -v'
```

Expected: 16 passed

## 8. Rollback

```bash
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py enable_prediction:=false enable_collision_check:=false'
```

Both prediction and click output stop immediately.
