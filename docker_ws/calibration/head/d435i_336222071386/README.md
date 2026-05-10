# Head RealSense D435i Calibration

Calibration artifacts and reproduction notes for the head/forehead-mounted Intel RealSense D435i used by the `assistive_multiview_prosthesis` project.

- Camera: Intel RealSense D435i
- Serial number: `336222071386`
- Mount location: head / forehead
- Intended estimator: OpenVINS
- Intended visual input: RGB monocular + IMU
- Not used for this workflow: IR stereo, depth, RGB-D

## Current Recommendation

Use the **dynamic-bag camera intrinsics** and the **10x inflated camera-IMU calibration** as the current best calibration for OpenVINS integration/testing.

Recommended final files:

```text
docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml
docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
```

The original camera-only intrinsics and nominal IMU-noise results are kept for traceability and comparison.

## Folder Layout

Tracked location in the repository:

```text
docker_ws/calibration/head/d435i_336222071386/
├── README.md
├── aprilgrid_6x6_80mm_0p3.yaml
├── camera_imu_extrinsics_inflated10x
│   ├── head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
│   ├── head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml
│   ├── head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-imu.yaml
│   ├── head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-report-imucam.pdf
│   └── head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-results-imucam.txt
├── camera_intrinsics
│   ├── head_camera_intrinsics_ros1-camchain.yaml
│   ├── head_camera_intrinsics_ros1-report-cam.pdf
│   └── head_camera_intrinsics_ros1-results-cam.txt
├── camera_intrinsics_from_dynamic
│   ├── head_rgb_imu_dynamic_20260429_131443_ros1-camchain.yaml
│   ├── head_rgb_imu_dynamic_20260429_131443_ros1-report-cam.pdf
│   └── head_rgb_imu_dynamic_20260429_131443_ros1-results-cam.txt
└── imu_noise
    ├── acceleration.png
    ├── gyro.png
    ├── head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
    ├── head_d435i_imu_200hz_trim_first30min.yaml
    └── head_d435i_imu_allan_config_200hz_trim_first30min.yaml
```

Large ROS bags should stay in the gitignored `docker_ws/bags/` folder and should not be committed.

## ROS Topics

Camera RGB image topic:

```text
/head/d435i_head/color/image_raw
```

IMU topic:

```text
/head/d435i_head/imu
```

## Calibration Target

Target type: Aprilgrid

```yaml
target_type: aprilgrid
tagCols: 6
tagRows: 6
tagSize: 0.08
tagSpacing: 0.3
```

Target file:

```text
aprilgrid_6x6_80mm_0p3.yaml
```

Kalibr reports the tag spacing as `0.024 m`, because:

```text
0.08 * 0.3 = 0.024 m
```

## Machine and Docker Setup

Calibration was run on an x86 Ubuntu machine using Docker because building/running Kalibr on Jetson ARM64 was too heavy and failed due to architecture and memory constraints.

Recording setup:

- Source robot: Jetson Orin Nano
- Jetson host OS: Ubuntu 22.04
- Jetson host ROS version: ROS 2 Humble
- Project runtime container: Ubuntu 24.04 + ROS 2 Jazzy
- Original camera/IMU data: recorded inside the ROS 2 Jazzy project container
- Kalibr input: ROS 1 bags converted from ROS 2 Jazzy MCAP bags

Kalibr requires ROS 1 bags for this workflow. Do not run Kalibr directly on the ROS 2 MCAP bags unless using a separate workflow that explicitly supports conversion.

## Build Kalibr Docker Image

```bash
mkdir -p "$HOME/src"
cd "$HOME/src"

git clone https://github.com/ethz-asl/kalibr.git
cd kalibr

sudo docker build \
  --platform linux/amd64 \
  -t kalibr:ros1_20_04 \
  -f Dockerfile_ros1_20_04 .
```

## Build Allan Variance Docker Image

This image extends the Kalibr image with `allan_variance_ros` for IMU noise estimation.

```bash
cat > /tmp/Dockerfile.allan_variance_ros <<'DOCKER'
FROM kalibr:ros1_20_04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git \
    libyaml-cpp-dev \
    python3-yaml \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN cd /catkin_ws/src && \
    git clone https://github.com/ori-drs/allan_variance_ros.git

RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
                   source /catkin_ws/devel/setup.bash && \
                   cd /catkin_ws && \
                   catkin build allan_variance_ros"
DOCKER

sudo docker build \
  --platform linux/amd64 \
  -t kalibr_allan:ros1_20_04 \
  -f /tmp/Dockerfile.allan_variance_ros \
  /tmp
```

Check the image:

```bash
sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  kalibr_allan:ros1_20_04 \
  -lc '
    source /catkin_ws/devel/setup.bash
    rospack find allan_variance_ros
  '
```

Expected result:

```text
/catkin_ws/src/allan_variance_ros
```

## Input Bags

The bags are not tracked in Git. They were stored under:

```text
docker_ws/bags/calibration/head/
```

Camera intrinsics ROS 1 bag:

```text
docker_ws/bags/calibration/head/camera_intrinsics/head_camera_intrinsics_ros1.bag
```

Stationary IMU ROS 1 bags:

```text
docker_ws/bags/calibration/head/imu_noise/head_imu_stationary_15h_unified_ros1.bag
docker_ws/bags/calibration/head/imu_noise/head_imu_stationary_15h_trim_first30min_ros1.bag
```

Dynamic camera-IMU ROS 1 bag:

```text
docker_ws/bags/calibration/head/cam_imu_dynamic/head_rgb_imu_dynamic_20260429_131443_ros1.bag
```

### Original Stationary IMU Bag

```text
duration:    14hr 46:28s (53188s)
messages:    10613442
topic:       /head/d435i_head/imu
type:        sensor_msgs/Imu
```

The first 30 minutes were removed because the device was powered on immediately before recording and was still warming up.

### Trimmed Stationary IMU Bag

```text
duration:    14hr 16:28s (51388s)
messages:    10254256
topic:       /head/d435i_head/imu
type:        sensor_msgs/Imu
```

### Dynamic Camera-IMU Bag

The dynamic bag contained both:

```text
/head/d435i_head/color/image_raw    sensor_msgs/Image
/head/d435i_head/imu                sensor_msgs/Imu
```

Kalibr read:

```text
Images:       8571
IMU readings: 57795
Duration:     about 289.9 s
```

## Verify Bags

Set paths:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export TRACKED_CALIB="$AMP_WS/calibration/head/d435i_336222071386"
export CAM_CALIB="$AMP_WS/bags/calibration/head/camera_intrinsics"
export IMU_CALIB="$AMP_WS/bags/calibration/head/imu_noise"
export DYN_CALIB="$AMP_WS/bags/calibration/head/cam_imu_dynamic"
export DYN_BAG="head_rgb_imu_dynamic_20260429_131443_ros1.bag"
```

Verify camera intrinsics bag:

```bash
sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -v "$CAM_CALIB:/data:ro" \
  kalibr:ros1_20_04 \
  -lc '
    set -e
    source /catkin_ws/devel/setup.bash
    rosbag info /data/head_camera_intrinsics_ros1.bag
  '
```

Verify dynamic camera-IMU bag:

```bash
sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -v "$DYN_CALIB:/data:ro" \
  kalibr:ros1_20_04 \
  -lc "
    source /catkin_ws/devel/setup.bash
    rosbag info /data/$DYN_BAG | grep -E 'duration:|messages:|/head/d435i_head/color/image_raw|/head/d435i_head/imu|sensor_msgs'
  "
```

Expected ROS 1 message types:

```text
sensor_msgs/Image
sensor_msgs/Imu
```

## Camera Intrinsics Calibration: Original Camera-Only Bag

Command used:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export CAM_CALIB="$AMP_WS/bags/calibration/head/camera_intrinsics"

sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -e MPLBACKEND=Agg \
  -v "$CAM_CALIB:/data:rw" \
  -w /data \
  kalibr:ros1_20_04 \
  -lc '
    set -e
    source /catkin_ws/devel/setup.bash

    rosrun kalibr kalibr_calibrate_cameras \
      --models pinhole-radtan \
      --topics /head/d435i_head/color/image_raw \
      --target /data/aprilgrid_6x6_80mm_0p3.yaml \
      --bag /data/head_camera_intrinsics_ros1.bag \
      --bag-freq 10.0
  '
```

Kalibr output files:

```text
head_camera_intrinsics_ros1-camchain.yaml
head_camera_intrinsics_ros1-results-cam.txt
head_camera_intrinsics_ros1-report-cam.pdf
```

Result:

```text
Model: pinhole-radtan
Topic: /head/d435i_head/color/image_raw
Projection: [602.64447786, 603.19240904, 333.86433006, 243.29999879]
Distortion: [0.10145786, -0.18046494, -0.00554649, 0.00627361]
Reprojection error: [0.000001, -0.000000] +- [0.400945, 0.366891] px
Processed images: 903
Images used: 32
Removed outlier corners: 181
```

Assessment:

```text
Usable, but not the preferred final intrinsics.
The board had small wrinkles during this recording, which may have contributed to outliers and residual error.
```

## Camera Intrinsics Calibration: Dynamic Camera-IMU Bag

The dynamic camera-IMU bag was also used to compute a second camera intrinsics calibration. This one is preferred because it produced lower reprojection error and fewer removed outliers.

Command used:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export DYN_CALIB="$AMP_WS/bags/calibration/head/cam_imu_dynamic"
export DYN_BAG="head_rgb_imu_dynamic_20260429_131443_ros1.bag"
export TARGET="$AMP_WS/calibration/head/d435i_336222071386/aprilgrid_6x6_80mm_0p3.yaml"

sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -e MPLBACKEND=Agg \
  -v "$DYN_CALIB:/data:rw" \
  -v "$TARGET:/target.yaml:ro" \
  -w /data \
  kalibr:ros1_20_04 \
  -lc "
    set -e
    source /catkin_ws/devel/setup.bash

    rosrun kalibr kalibr_calibrate_cameras \
      --models pinhole-radtan \
      --topics /head/d435i_head/color/image_raw \
      --target /target.yaml \
      --bag /data/$DYN_BAG \
      --bag-freq 10.0
  "
```

Kalibr output files:

```text
head_rgb_imu_dynamic_20260429_131443_ros1-camchain.yaml
head_rgb_imu_dynamic_20260429_131443_ros1-results-cam.txt
head_rgb_imu_dynamic_20260429_131443_ros1-report-cam.pdf
```

Result:

```text
Model: pinhole-radtan
Topic: /head/d435i_head/color/image_raw
Projection: [599.17399284, 599.268246, 324.97194633, 250.51904788]
Distortion: [0.11030607, -0.20736098, -0.00227622, 0.00184355]
Reprojection error: [0.000001, -0.000002] +- [0.304388, 0.309307] px
Processed images: 2859
Images used: 34
Removed outlier corners: 162
```

Assessment:

```text
Preferred camera intrinsics for the camera-IMU calibration.
Compared with the original camera-only bag, this had lower reprojection error and fewer outlier corners.
```

## IMU Noise Calibration

The IMU noise calibration was done using the stationary IMU bag after trimming the first 30 minutes for warm-up.

### Trim First 30 Minutes

Original bag start time:

```text
1777390591.40
```

Trimmed start time, 30 minutes later:

```text
1777392391.40
```

Command:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export IMU_CALIB="$AMP_WS/bags/calibration/head/imu_noise"

sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -v "$IMU_CALIB:/data:rw" \
  kalibr:ros1_20_04 \
  -lc '
    set -e
    source /catkin_ws/devel/setup.bash

    rosbag filter \
      /data/head_imu_stationary_15h_unified_ros1.bag \
      /data/head_imu_stationary_15h_trim_first30min_ros1.bag \
      "t.to_sec() > 1777392391.40"
  '
```

### Allan Config

File:

```text
head_d435i_imu_allan_config_200hz_trim_first30min.yaml
```

Contents:

```yaml
imu_topic: "/head/d435i_head/imu"
imu_rate: 200
measure_rate: 200
sequence_time: 51388
```

### Cook Trimmed Bag

```bash
mkdir -p "$IMU_CALIB/allan_input_200hz_trim_first30min"
mkdir -p "$IMU_CALIB/allan_output_200hz_trim_first30min"

sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -v "$IMU_CALIB:/data:rw" \
  -w /data \
  kalibr_allan:ros1_20_04 \
  -lc '
    set -e
    source /catkin_ws/devel/setup.bash

    roscore >/tmp/roscore.log 2>&1 &
    sleep 3

    rosrun allan_variance_ros cookbag.py \
      --input /data/head_imu_stationary_15h_trim_first30min_ros1.bag \
      --output /data/allan_input_200hz_trim_first30min/head_imu_stationary_15h_trim_first30min_cooked.bag
  '
```

### Run Allan Variance at 200 Hz

```bash
sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -v "$IMU_CALIB:/data:rw" \
  -w /data \
  kalibr_allan:ros1_20_04 \
  -lc '
    set -e
    source /catkin_ws/devel/setup.bash

    roscore >/tmp/roscore.log 2>&1 &
    sleep 3

    rosrun allan_variance_ros allan_variance \
      /data/allan_input_200hz_trim_first30min \
      /data/head_d435i_imu_allan_config_200hz_trim_first30min.yaml
  '
```

Expected Allan CSV:

```text
allan_input_200hz_trim_first30min/allan_variance.csv
```

### Generate IMU YAML and Allan Plots

```bash
sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -e MPLBACKEND=Agg \
  -v "$IMU_CALIB:/data:rw" \
  -w /data/allan_output_200hz_trim_first30min \
  kalibr_allan:ros1_20_04 \
  -lc '
    set -e
    source /catkin_ws/devel/setup.bash

    rosrun allan_variance_ros analysis.py \
      --data /data/allan_input_200hz_trim_first30min/allan_variance.csv \
      --config /data/head_d435i_imu_allan_config_200hz_trim_first30min.yaml \
      --output /data/allan_output_200hz_trim_first30min/head_d435i_imu_200hz_trim_first30min.yaml
  '
```

Fix ownership if needed:

```bash
sudo chown -R "$USER:$USER" \
  "$IMU_CALIB/allan_input_200hz_trim_first30min" \
  "$IMU_CALIB/allan_output_200hz_trim_first30min" \
  "$IMU_CALIB/head_imu_stationary_15h_trim_first30min_ros1.bag"
```

### IMU Noise Result: Allan Estimate

Final trimmed 200 Hz result:

```yaml
#Accelerometer
accelerometer_noise_density: 0.0009278837334054291
accelerometer_random_walk: 2.8048259057595945e-05

#Gyroscope
gyroscope_noise_density: 0.00019841370058405538
gyroscope_random_walk: 1.7544399415266354e-06

rostopic: '/head/d435i_head/imu'
update_rate: 200
```

For comparison, the original untrimmed 200 Hz result was:

```yaml
#Accelerometer
accelerometer_noise_density: 0.000929487404651461
accelerometer_random_walk: 2.8029514366435016e-05

#Gyroscope
gyroscope_noise_density: 0.0001981778924303882
gyroscope_random_walk: 1.8683255201963384e-06

rostopic: '/head/d435i_head/imu'
update_rate: 200
```

The trimmed and untrimmed values are very similar. This is a good sign. The trimmed 200 Hz result was used as the base IMU noise estimate.

### IMU Noise Result: 10x Inflated Runtime/Calibration Version

The Allan-estimated IMU values were inflated by 10x to account for unmodeled errors. This produced better normalized residuals during camera-IMU calibration.

Final 10x inflated IMU noise file:

```text
imu_noise/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
```

Contents:

```yaml
#Accelerometer
accelerometer_noise_density: 0.009278837334054291
accelerometer_random_walk: 0.0002804825905759594

#Gyroscope
gyroscope_noise_density: 0.0019841370058405537
gyroscope_random_walk: 1.7544399415266354e-05

rostopic: '/head/d435i_head/imu'
update_rate: 200
```

## Camera-IMU Extrinsics and Time Offset Calibration

Camera-IMU calibration used:

```text
Camera intrinsics: camera_intrinsics_from_dynamic/head_rgb_imu_dynamic_20260429_131443_ros1-camchain.yaml
IMU noise:         imu_noise/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
Target:            aprilgrid_6x6_80mm_0p3.yaml
Dynamic bag:       bags/calibration/head/cam_imu_dynamic/head_rgb_imu_dynamic_20260429_131443_ros1.bag
```

A symlinked bag name was used for the 10x run so Kalibr would not overwrite the nominal output:

```bash
cd "$DYN_CALIB"
ln -sf head_rgb_imu_dynamic_20260429_131443_ros1.bag \
       head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x.bag
```

Command used for the 10x inflated calibration:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export DYN_CALIB="$AMP_WS/bags/calibration/head/cam_imu_dynamic"
export DYN_BAG_10X="head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x.bag"
export TRACKED_CALIB="$AMP_WS/calibration/head/d435i_336222071386"
export TARGET="$TRACKED_CALIB/aprilgrid_6x6_80mm_0p3.yaml"
export CAM_FOR_IMUCAM="$TRACKED_CALIB/camera_intrinsics_from_dynamic/head_rgb_imu_dynamic_20260429_131443_ros1-camchain.yaml"
export IMU_YAML_10X="$TRACKED_CALIB/imu_noise/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml"

sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -e MPLBACKEND=Agg \
  -v "$DYN_CALIB:/data:rw" \
  -v "$TARGET:/target.yaml:ro" \
  -v "$CAM_FOR_IMUCAM:/camchain.yaml:ro" \
  -v "$IMU_YAML_10X:/imu.yaml:ro" \
  -w /data \
  kalibr:ros1_20_04 \
  -lc "
    set -e
    source /catkin_ws/devel/setup.bash

    rosrun kalibr kalibr_calibrate_imu_camera \
      --target /target.yaml \
      --cam /camchain.yaml \
      --imu /imu.yaml \
      --bag /data/$DYN_BAG_10X \
      --imu-models calibrated
  "
```

Kalibr output files:

```text
head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml
head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-imu.yaml
head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-results-imucam.txt
head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-report-imucam.pdf
```

### Camera-IMU Calibration Result: 10x Inflated

Final residuals:

```text
Reprojection error (cam0) [px]:     mean 0.6085045236342096, median 0.464548510997314, std 0.5160939857871177
Gyroscope error (imu0) [rad/s]:     mean 0.017541621509591522, median 0.015475129742388965, std 0.010153973501470995
Accelerometer error (imu0) [m/s^2]: mean 0.09125694440044545, median 0.0832767268788441, std 0.052133769492380445
```

Normalized residuals:

```text
Reprojection error (cam0): mean 0.6085045236342096, median 0.464548510997314, std 0.5160939857871177
Gyroscope error (imu0):    mean 0.6251483383419513, median 0.5515027010924173, std 0.3618673255800177
Accelerometer error (imu0): mean 0.6954363126842715, median 0.6346219485380425, std 0.3972926845218351
```

Estimated transform:

```text
T_ci: imu0 to cam0
[[ 0.99987512  0.01520848  0.00429518  0.0225262 ]
 [-0.01520471  0.99988399 -0.00090895 -0.00213086]
 [-0.0043085   0.00084353  0.99999036 -0.00386642]
 [ 0.          0.          0.          1.        ]]
```

Inverse transform:

```text
T_ic: cam0 to imu0
[[ 0.99987512 -0.01520471 -0.0043085  -0.02257245]
 [ 0.01520848  0.99988399  0.00084353  0.00179128]
 [ 0.00429518 -0.00090895  0.99999036  0.00376769]
 [ 0.          0.          0.          1.        ]]
```

Time shift:

```text
t_imu = t_cam + 0.01334857234167497 s
```

Interpretation:

```text
The 10x inflated calibration is the recommended one to use for OpenVINS testing.
The residuals are more balanced than the nominal Allan-noise calibration, and the transform/time shift remain physically plausible.
```

### Comparison: Nominal vs 10x Inflated Camera-IMU Result

Nominal result:

```text
Reprojection mean:       0.7555389538514138 px
Gyro normalized mean:    2.165666677975137
Accel normalized mean:   3.2107940608899974
Time shift:              0.013723439570657896 s
T_ci translation:        [0.02353388, -0.00842372, -0.0045148] m
```

10x inflated result:

```text
Reprojection mean:       0.6085045236342096 px
Gyro normalized mean:    0.6251483383419513
Accel normalized mean:   0.6954363126842715
Time shift:              0.01334857234167497 s
T_ci translation:        [0.0225262, -0.00213086, -0.00386642] m
```

Assessment:

```text
The 10x inflated result is preferred. The time shift changed by less than 0.4 ms compared with nominal, and rotation changed only slightly. Translation changed by a few millimeters, but both transforms are physically plausible.
```

## Notes on Report Quality

The camera-IMU report's reprojection-error plot still has some points outside the dashed ellipse. This is not an automatic failure. The dataset contains thousands of detections, and residual tails are expected with a real D435i, dynamic motion, target corner noise, possible motion blur, rolling-shutter effects, and non-ideal target flatness.

Do not keep increasing IMU noise inflation only to make this plot look prettier. Inflating IMU noise changes the weighting between the IMU and camera residuals; it does not directly fix camera reprojection outliers.

Current practical recommendation:

```text
Use the 10x inflated result for OpenVINS integration/testing.
Redo the recording only if OpenVINS diverges, has unstable scale/orientation, or online calibration makes large corrections.
```

## Files to Use

For OpenVINS integration/testing, use:

```text
camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml
camera_imu_extrinsics_inflated10x/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
```

For future calibration/reproduction, keep:

```text
aprilgrid_6x6_80mm_0p3.yaml
camera_intrinsics/head_camera_intrinsics_ros1-*.{yaml,txt,pdf}
camera_intrinsics_from_dynamic/head_rgb_imu_dynamic_20260429_131443_ros1-*.{yaml,txt,pdf}
imu_noise/head_d435i_imu_200hz_trim_first30min.yaml
imu_noise/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
imu_noise/acceleration.png
imu_noise/gyro.png
camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-*.{yaml,txt,pdf}
```

## Copy Outputs Into This Tracked Folder

Example commands used to create/update this tracked calibration folder:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export TRACKED_CALIB="$AMP_WS/calibration/head/d435i_336222071386"
export CAM_CALIB="$AMP_WS/bags/calibration/head/camera_intrinsics"
export DYN_CALIB="$AMP_WS/bags/calibration/head/cam_imu_dynamic"
export IMU_CALIB="$AMP_WS/bags/calibration/head/imu_noise"

mkdir -p "$TRACKED_CALIB/camera_intrinsics"
mkdir -p "$TRACKED_CALIB/camera_intrinsics_from_dynamic"
mkdir -p "$TRACKED_CALIB/imu_noise"
mkdir -p "$TRACKED_CALIB/camera_imu_extrinsics_inflated10x"

cp "$CAM_CALIB/aprilgrid_6x6_80mm_0p3.yaml" \
   "$TRACKED_CALIB/aprilgrid_6x6_80mm_0p3.yaml"

cp "$CAM_CALIB/head_camera_intrinsics_ros1-camchain.yaml" \
   "$CAM_CALIB/head_camera_intrinsics_ros1-results-cam.txt" \
   "$CAM_CALIB/head_camera_intrinsics_ros1-report-cam.pdf" \
   "$TRACKED_CALIB/camera_intrinsics/"

cp "$DYN_CALIB/head_rgb_imu_dynamic_20260429_131443_ros1-camchain.yaml" \
   "$DYN_CALIB/head_rgb_imu_dynamic_20260429_131443_ros1-results-cam.txt" \
   "$DYN_CALIB/head_rgb_imu_dynamic_20260429_131443_ros1-report-cam.pdf" \
   "$TRACKED_CALIB/camera_intrinsics_from_dynamic/"

cp "$IMU_CALIB/head_d435i_imu_allan_config_200hz_trim_first30min.yaml" \
   "$IMU_CALIB/allan_output_200hz_trim_first30min/head_d435i_imu_200hz_trim_first30min.yaml" \
   "$IMU_CALIB/allan_output_200hz_trim_first30min/acceleration.png" \
   "$IMU_CALIB/allan_output_200hz_trim_first30min/gyro.png" \
   "$TRACKED_CALIB/imu_noise/"

cp "$TRACKED_CALIB/imu_noise/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml" \
   "$TRACKED_CALIB/camera_imu_extrinsics_inflated10x/"

cp "$DYN_CALIB/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml" \
   "$DYN_CALIB/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-imu.yaml" \
   "$DYN_CALIB/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-results-imucam.txt" \
   "$DYN_CALIB/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-report-imucam.pdf" \
   "$TRACKED_CALIB/camera_imu_extrinsics_inflated10x/"
```

## Git Commit

Check Git ignore status:

```bash
cd "$AMP_WS"
git check-ignore -v calibration/head/d435i_336222071386/README.md || echo "Not ignored"
```

Commit:

```bash
cd "$AMP_WS"
git status --short calibration/head/d435i_336222071386
git add calibration/head/d435i_336222071386
git commit -m "Add head D435i calibration outputs and README"
```

## Transfer to Jetson

Example:

```bash
rsync -av \
  "$AMP_WS/calibration/head/d435i_336222071386/" \
  jetson:/path/to/assistive_multiview_prosthesis/docker_ws/calibration/head/d435i_336222071386/
```

## OpenVINS Integration Notes

The workspace already contains OpenVINS example configurations under:

```text
docker_ws/src/src/open_vins/config/
```

Examples found in the current workspace include:

```text
euroc_mav/
kaist/
kaist_vio/
rpng_aruco/
rpng_ironsides/
rpng_plane/
rpng_sim/
rs_d455/
rs_t265/
tum_vi/
uzhfpv_indoor/
uzhfpv_indoor_45/
uzhfpv_outdoor/
uzhfpv_outdoor_45/
```

For the head-mounted D435i, create a project-local OpenVINS config folder instead of editing the upstream examples directly, for example:

```text
multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/
```

Recommended input files for the first OpenVINS RGB monocular + IMU test:

```text
docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml
docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
```

Suggested setup commands from the Jetson host:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws

mkdir -p multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386

cp calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml \
   multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/

cp calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml \
   multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/
```

Transform convention warning:

```text
Kalibr reports T_ci as imu0-to-cam0 and T_ic as cam0-to-imu0.
Use the final Kalibr camchain-imucam YAML as the source of truth.
Before copying values into an OpenVINS config field, confirm whether that field expects T_cam_imu, T_imu_cam, T_CtoI, or T_ItoC.
Do not invert or transpose the matrix unless the target config field explicitly requires that convention.
```

Runtime topics expected from the head D435i:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/imu
```

Keep the older `camera_intrinsics/` result for traceability, but prefer the dynamic-bag intrinsics already embedded in the final `camera_imu_extrinsics_inflated10x/*camchain-imucam.yaml` for OpenVINS testing.

## Future Redo Criteria

Redo the calibration only if OpenVINS testing shows clear problems, such as:

- VIO diverges quickly.
- Scale or orientation drift is severe.
- Camera-IMU online calibration changes the transform/time offset substantially.
- Feature tracks are poor because the dynamic calibration images were too blurred.
- The D435i mounting changed mechanically.
- The RGB stream resolution, cropping, or image pipeline changes.

If re-recording, warm up the D435i for 20-30 minutes, keep the Aprilgrid rigid and flat, move the camera/IMU rig slowly enough to avoid blur, and excite roll, pitch, yaw, and translation while keeping the Aprilgrid visible across the full image.