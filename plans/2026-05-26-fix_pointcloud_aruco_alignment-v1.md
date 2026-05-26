# Fix Point Cloud Alignment With World ArUco Marker

## Objective
Ensure transformed point clouds use the ArUco-corrected camera pose instead of the raw OpenVINS TF branch, so clouds align with the world ArUco marker in `marker_map`.

## Root Cause
The `pointcloud_to_frame_node` looks up `marker_map → head_cam0` (or `arm_cam0`), which traverses through OpenVINS' **raw** VIO TF branch. The ArUco-corrected pose is published to a disconnected branch (`marker_map → {imu}_openvins_corrected`) that `head_cam0` is not attached to.

## Fix Strategy
Publish a corrected camera frame TF (`{imu}_openvins_corrected → {cam}0_corrected`) alongside the existing corrected IMU TF, then configure the pointcloud nodes to use the corrected camera frame.

## Files to Modify

### 1. `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`

#### Change A: Add corrected_camera_frame parsing after line 656

After:
```python
        self.imu_frame = self.config["frames"].get("imu_frame", "imu")
```

Add:
```python
        # Corrected camera frame for pointcloud alignment. If set, the node
        # publishes a TF from {imu_frame}_openvins_corrected to this frame so
        # that pointcloud_to_frame_node can use the ArUco-corrected camera pose
        # instead of the raw OpenVINS branch.
        self.corrected_camera_frame = self.config["frames"].get("corrected_camera_frame", "")
        self.publish_corrected_camera_tf = bool(self.corrected_camera_frame)
        if self.publish_corrected_camera_tf:
            # T_cam_imu maps IMU frame → camera optical frame.
            # The TF {imu}_corrected → {cam}_corrected needs the inverse:
            # T_imu_cam maps camera optical frame → IMU frame.
            self.T_imu_cam = T_inv(self.T_cam_imu)
            self.corrected_imu_frame = f"{self.imu_frame}_openvins_corrected"
```

#### Change B: Add startup log after line 873 (after existing log lines)

After:
```python
            self.get_logger().info("Marker detection rate limit: disabled")
```

Add:
```python
        if self.publish_corrected_camera_tf:
            self.get_logger().info(
                f"Corrected camera TF: {self.corrected_imu_frame} -> {self.corrected_camera_frame}"
            )
        else:
            self.get_logger().info("Corrected camera TF: disabled (no corrected_camera_frame in config)")
```

#### Change C: Add corrected camera TF publication in publish_corrected_odom (after line 1962)

After:
```python
        self.publish_tf(msg.header.stamp, self.map_frame, out.child_frame_id, T_map_imu_corrected)
```

Add:
```python
        # Publish corrected camera frame so pointcloud_to_frame_node can look up
        # marker_map -> {cam}0_corrected through the ArUco-corrected branch.
        if self.publish_corrected_camera_tf:
            self.publish_tf(
                msg.header.stamp,
                self.corrected_imu_frame,
                self.corrected_camera_frame,
                self.T_imu_cam,
            )
```

### 2. `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml`

#### Change: Add corrected_camera_frame to frames section (after line 13)

After:
```yaml
  imu_frame: head_imu
```

Add:
```yaml
  # Corrected camera frame published under the ArUco-corrected IMU TF branch.
  # Used by pointcloud_to_frame_node to transform clouds through the corrected pose.
  corrected_camera_frame: head_cam0_corrected
```

### 3. `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`

#### Change: Add corrected_camera_frame to frames section (after line 13)

After:
```yaml
  imu_frame: arm_imu
```

Add:
```yaml
  # Corrected camera frame published under the ArUco-corrected IMU TF branch.
  # Used by pointcloud_to_frame_node to transform clouds through the corrected pose.
  corrected_camera_frame: arm_cam0_corrected
```

### 4. `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`

#### Change: Update camera_pose_frame default (line 126)

Change:
```python
            {"camera_pose_frame": "head_cam0"},
```

To:
```python
            {"camera_pose_frame": "head_cam0_corrected"},
```

### 5. `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`

#### Change: Update camera_pose_frame default (line 126)

Change:
```python
            {"camera_pose_frame": "arm_cam0"},
```

To:
```python
            {"camera_pose_frame": "arm_cam0_corrected"},
```

### 6. `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/x86_raw_pointcloud_marker_map.launch.py`

#### Change A: Update head camera_pose_frame default (line 63)

Change:
```python
            DeclareLaunchArgument("head_camera_pose_frame", default_value="head_cam0"),
```

To:
```python
            DeclareLaunchArgument("head_camera_pose_frame", default_value="head_cam0_corrected"),
```

#### Change B: Update arm camera_pose_frame default (line 64)

Change:
```python
            DeclareLaunchArgument("arm_camera_pose_frame", default_value="arm_cam0"),
```

To:
```python
            DeclareLaunchArgument("arm_camera_pose_frame", default_value="arm_cam0_corrected"),
```

## Verification Criteria
- Pointcloud node status reports `camera_pose_frame` as `head_cam0_corrected` or `arm_cam0_corrected`
- TF tree contains `marker_map → {imu}_openvins_corrected → {cam}0_corrected`
- No duplicate TF authority warnings
- In RViz with fixed frame `marker_map`, transformed point clouds align with the physical ArUco marker
- Before marker lock, corrected camera TF is not published (gated on T_map_global being set)
