# Arm Point Cloud Alignment Bug — TF Chain Analysis

## Objective

Identify why the arm camera point cloud never aligns with the head camera point cloud in the fused output, even though both cameras are tracked by OpenVINS and use the same fusion pipeline.

## Root Cause Identified

**The `openvins_odom_tf_relay` publishes an incorrect `imu → cam0` extrinsic.** The relay applies `rpy(-π/2, 0, -π/2)` (the optical convention rotation) to transform from `*_imu` to `*_cam0`, but the Kalibr calibration files show `T_cam_imu` is **near-identity** for both cameras — meaning the IMU frame and cam0 frame already share the same axis convention.

### Evidence

1. **Kalibr calibration** (`kalibr_imucam_chain.yaml` for both cameras): `T_cam_imu` rotation is within ~1° of identity. Translation is ~15-22mm.

2. **OpenVINS internal**: `T_cam_imu` ≈ identity means the IMU and camera use the **same convention** (optical: Z-forward, Y-down).

3. **RealSense IMU output**: frame_id = `*_imu_optical_frame` — confirms optical convention.

4. **Host relay config** (`config/prosthesis_config.yaml:160-174`): applies `rpy(-π/2, 0, -π/2)` to `imu→cam0` — this is a **90° rotation error**.

### Why the head cloud "works" despite the error

The error in `imu→cam0` is compensated by OpenVINS's self-consistent tracking. OpenVINS publishes `marker_map → imu` in optical convention. The relay's incorrect `imu→cam0` rotation creates a wrong intermediate frame, but the bridge independently resolves `cam0→link` from the RealSense static chain. The overall chain `marker_map→depth_optical_frame` is correct because OpenVINS adapts.

### Why the arm cloud doesn't align

Both head and arm have the **same** incorrect `imu→cam0` rotation. However, the arm camera is physically oriented very differently from the head camera. The 90° error interacts differently with each camera's physical orientation, causing the arm cloud to be systematically offset relative to the head cloud. When the fusion node merges both clouds in `marker_map`, the arm cloud appears rotated/shifted.

**More precisely**: The TF chain for each camera is:
```
marker_map → *_imu (optical, from OpenVINS odom)
             → *_cam0 (WRONG: extra 90° rotation applied)
                → *_link (RealSense extrinsic, independently correct)
                   → *_depth_frame → *_depth_optical_frame
```

The `*_cam0` frame is wrong (extra 90°), but the `*_cam0→*_link` edge is independently resolved from the RealSense static chain. So the composed transform `marker_map→*_depth_optical_frame` should still be correct IF the bridge's `cam0→link` lookup succeeds.

**But**: The bridge resolves `T(cam0→link)` by looking up `T(color_optical→link)` from the TF buffer. If the `cam0` frame is wrong (extra 90° from the relay's error), then `T(color_optical→link)` would need to compensate. However, the bridge uses the RealSense static chain frames (which are in the correct RealSense frame tree), not the relay's `cam0` frame. The bridge looks up `T(head_d435i_head_color_optical_frame → head_d435i_head_link)` — these frames are in the RealSense static tree, NOT the OpenVINS tree. So the bridge's lookup is independent of the relay's error.

Wait — but the bridge publishes `head_cam0 → head_d435i_head_link`. And the relay publishes `head_imu → head_cam0`. If `head_cam0` is in the wrong convention (due to the relay's error), then the bridge's transform `head_cam0 → head_d435i_head_link` would be applied to the wrong frame, causing the overall chain to be wrong.

Actually, the bridge looks up `T(color_optical→link)` and publishes it as `T(cam0→link)`. Since `cam0 == color_optical_frame`, this is correct. But the relay's `T(imu→cam0)` is wrong. The composed chain is:
```
T(marker_map→imu) @ T(imu→cam0)_wrong @ T(cam0→link)_correct
```

For this to produce the correct result, we'd need:
```
T(marker_map→imu) @ T(imu→cam0)_wrong = T(marker_map→cam0)_true
```

Which means:
```
T(marker_map→imu) = T(marker_map→cam0)_true @ T(imu→cam0)_wrong^{-1}
```

OpenVINS publishes `T(marker_map→imu)` from its odometry. If OpenVINS' internal `imu` frame is in optical convention, then `T(marker_map→imu)` is in optical convention. The relay's `T(imu→cam0)_wrong` applies an extra 90° rotation. So the resulting `T(marker_map→cam0)` has an extra 90° rotation compared to the true `T(marker_map→cam0)`.

But then `T(cam0→link)` is the RealSense extrinsic, which is correct. So the final `T(marker_map→link)` has an extra 90° rotation.

**This is the bug**: the extra 90° rotation in `imu→cam0` causes the entire camera-to-world transform to be wrong by 90°. Both head and arm have the same error, but since they're physically oriented differently, the 90° error affects them differently in world coordinates, causing the misalignment.

## Implementation Plan

- [ ] 1. **Fix the `imu→cam0` extrinsic in `config/prosthesis_config.yaml`**: Replace the incorrect `rpy(-π/2, 0, -π/2)` with near-identity values from the Kalibr calibration files. The correct values for head are approximately `(x=0.023, y=-0.002, z=-0.004, roll=0, pitch=0, yaw=0)` and for arm `(x=0.018, y=-0.001, z=-0.001, roll=0, pitch=0, yaw=0)`.

- [ ] 2. **Verify the fix by checking the TF chain**: After the fix, verify that `marker_map → head_imu → head_cam0 → head_d435i_head_link → head_d435i_head_depth_optical_frame` produces a sensible transform (camera pointing forward in marker_map, not sideways or upward).

- [ ] 3. **Run a camera test**: Use `make jetson-cameras` + the host pipeline to verify that both clouds overlap correctly in RViz.

## Verification Criteria

- Fused point clouds from head and arm cameras overlap in RViz when viewing the same scene
- The `dual` count in fusion stats is consistently high
- No systematic rotation offset between the two camera views
- The `cam0` frame in RViz is oriented the same as the `color_optical_frame` (Z-forward)

## Potential Risks and Mitigations

1. **Risk**: The Kalibr values might not exactly match the current OpenVINS calibration if it was recalibrated after the Kalibr files were generated.
   **Mitigation**: Use the actual Kalibr file values from the Jetson config directory. These are the same files OpenVINS loads at runtime.

2. **Risk**: Changing the extrinsic might break other parts of the pipeline that depend on the current (wrong) frame conventions.
   **Mitigation**: The bridge and fusion node use the RealSense static chain independently, so they should not be affected. The twist propagation uses the `marker_map` frame directly and doesn't depend on intermediate frames.

## Alternative Approaches

1. **Remove the `imu→cam0` static TF entirely**: If OpenVINS already publishes `marker_map→cam0` directly (via its own TF publishing), the relay's `imu→cam0` edge is redundant. Check if OpenVINS on the Jetson publishes `marker_map→head_imu→head_cam0` on `/tf`. If it does, the relay only needs to publish `marker_map→head_imu` (which it does from the odom topic), and the `imu→cam0` edge can be skipped.
   **Trade-off**: This avoids the calibration mismatch entirely but requires the Jetson's `/tf` to be reliable for the `imu→cam0` edge.

2. **Use the exact Kalibr values from the Jetson config**: Instead of the nominal D435i values, load the actual calibrated `T_cam_imu` from the OpenVINS config files on the Jetson.
   **Trade-off**: More accurate but requires access to the Jetson config files at parameter-load time.
