# Head RealSense D435i Calibration

Calibration artifacts and reproduction notes for the head/forehead-mounted Intel RealSense D435i used by the `assistive_multiview_prosthesis` project.

- Camera: Intel RealSense D435i
- Serial number: `336222071386`
- Mount location: head / forehead
- Intended estimator: OpenVINS
- Intended visual input: RGB monocular + IMU
- Not used for this workflow: IR stereo, depth, RGB-D

## Folder Layout

Recommended tracked location in the repository:

```text
docker_ws/calibration/head/d435i_336222071386/
├── README.md
├── aprilgrid_6x6_80mm_0p3.yaml
├── camera_intrinsics
│   ├── head_camera_intrinsics_ros1-camchain.yaml
│   ├── head_camera_intrinsics_ros1-report-cam.pdf
│   └── head_camera_intrinsics_ros1-results-cam.txt
└── imu_noise
    ├── acceleration.png
    ├── gyro.png
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

Calibration was run on an x86 Ubuntu machine using Docker. This was done because building/running Kalibr on Jetson ARM64 was too heavy and failed due to architecture and memory constraints.

The robot/recording setup was:

- Jetson Orin Nano host OS: Ubuntu 22.04
- Jetson host ROS version: ROS 2 Humble
- Project runtime container: Ubuntu 24.04 + ROS 2 Jazzy
- Original camera/IMU data: recorded inside the ROS 2 Jazzy project container
- Kalibr input: converted ROS 1 bags

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

Original stationary IMU ROS 1 bag:

```text
docker_ws/bags/calibration/head/imu_noise/head_imu_stationary_15h_unified_ros1.bag
```

Trimmed stationary IMU ROS 1 bag, first 30 minutes removed:

```text
docker_ws/bags/calibration/head/imu_noise/head_imu_stationary_15h_trim_first30min_ros1.bag
```

The original stationary IMU bag had:

```text
duration:    14hr 46:28s (53188s)
messages:    10613442
topic:       /head/d435i_head/imu
type:        sensor_msgs/Imu
```

The first 30 minutes were removed because the device was powered on immediately before recording and was still warming up.

The trimmed IMU bag had:

```text
duration:    14hr 16:28s (51388s)
messages:    10254256
topic:       /head/d435i_head/imu
type:        sensor_msgs/Imu
```

## Verify Camera Intrinsics Bag

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export CAM_CALIB="$AMP_WS/bags/calibration/head/camera_intrinsics"

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

Expected camera topic:

```text
/head/d435i_head/color/image_raw
```

Expected ROS 1 message type:

```text
sensor_msgs/Image
```

## Camera Intrinsics Calibration

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

The PDF generation may print an X display warning in some Docker/headless setups. The calibration itself is still valid if the camchain and text result files were written.

### Camera Intrinsics Result

Model:

```text
pinhole-radtan
```

Camera topic:

```text
/head/d435i_head/color/image_raw
```

Estimated projection parameters:

```text
[602.64447786, 603.19240904, 333.86433006, 243.29999879]
```

Estimated distortion parameters:

```text
[0.10145786, -0.18046494, -0.00554649, 0.00627361]
```

Reprojection error:

```text
[0.000001, -0.000000] +- [0.400945, 0.366891] px
```

Other notes:

```text
Processed images: 903
Images used: 32
Removed outlier corners: 181
```

Assessment:

```text
Usable for OpenVINS initial testing.
The result is acceptable but not perfect. It could be improved later with a flatter Aprilgrid and more accepted views.
```

Important caveat: the calibration board had small wrinkles during recording. This may have contributed to outlier corners and residual error, especially near the image edges. If final VIO quality is poor, redo the camera intrinsics calibration with the Aprilgrid mounted on a rigid, flat backing.

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

Verify trimmed bag:

```bash
sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -v "$IMU_CALIB:/data:ro" \
  kalibr:ros1_20_04 \
  -lc '
    source /catkin_ws/devel/setup.bash
    rosbag info /data/head_imu_stationary_15h_trim_first30min_ros1.bag
  '
```

Expected trimmed duration:

```text
14hr 16:28s (51388s)
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

### IMU Noise Result

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

The trimmed and untrimmed values are very similar. This is a good sign. The trimmed 200 Hz result is the recommended one to use because it excludes the camera/IMU warm-up period.

## Files to Use for OpenVINS / Future Kalibr Calibration

Camera intrinsics:

```text
camera_intrinsics/head_camera_intrinsics_ros1-camchain.yaml
```

IMU noise:

```text
imu_noise/head_d435i_imu_200hz_trim_first30min.yaml
```

Aprilgrid target:

```text
aprilgrid_6x6_80mm_0p3.yaml
```

Reports for inspection/documentation:

```text
camera_intrinsics/head_camera_intrinsics_ros1-report-cam.pdf
camera_intrinsics/head_camera_intrinsics_ros1-results-cam.txt
imu_noise/acceleration.png
imu_noise/gyro.png
```

## Remaining Calibration Step: Camera-IMU Extrinsics and Time Offset

For OpenVINS, camera intrinsics and IMU noise are not enough. A camera-IMU extrinsics and time-offset calibration is still required.

Record a new dynamic ROS 2 bag on the Jetson, then convert it to a ROS 1 bag. The bag must contain both:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/imu
```

During recording:

- Keep the Aprilgrid stationary.
- Move the head-mounted RealSense D435i in front of the Aprilgrid.
- Excite all six degrees of freedom: roll, pitch, yaw, x, y, z.
- Move slowly enough to avoid image blur.
- Keep the Aprilgrid visible as much as possible.
- Use many different distances and angles.
- Include motion that creates meaningful IMU excitation, not just slow translation.
- Do not use the 15-hour stationary IMU bag for camera-IMU extrinsics; it is only for IMU noise.

The eventual Kalibr command will have this form:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"

sudo docker run --rm -it \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  -e MPLBACKEND=Agg \
  -v "$AMP_WS/bags/calibration/head:/data:rw" \
  -w /data \
  kalibr:ros1_20_04 \
  -lc '
    set -e
    source /catkin_ws/devel/setup.bash

    rosrun kalibr kalibr_calibrate_imu_camera \
      --target /data/camera_intrinsics/aprilgrid_6x6_80mm_0p3.yaml \
      --cam /data/camera_intrinsics/head_camera_intrinsics_ros1-camchain.yaml \
      --imu /data/imu_noise/allan_output_200hz_trim_first30min/head_d435i_imu_200hz_trim_first30min.yaml \
      --bag /data/camera_imu_dynamic/head_camera_imu_dynamic_ros1.bag \
      --imu-models calibrated
  '
```

Adjust paths once the dynamic camera-IMU bag exists.

## Copy Outputs Into This Tracked Folder

Example commands used to create the tracked calibration folder:

```bash
export AMP_WS="$HOME/Documents/GitHub/assistive_multiview_prosthesis/docker_ws"
export CAM_CALIB="$AMP_WS/bags/calibration/head/camera_intrinsics"
export IMU_CALIB="$AMP_WS/bags/calibration/head/imu_noise"
export CALIB_TRACKED="$AMP_WS/calibration/head/d435i_336222071386"

mkdir -p "$CALIB_TRACKED/camera_intrinsics"
mkdir -p "$CALIB_TRACKED/imu_noise"

cp "$CAM_CALIB/aprilgrid_6x6_80mm_0p3.yaml" \
   "$CALIB_TRACKED/aprilgrid_6x6_80mm_0p3.yaml"

cp "$CAM_CALIB/head_camera_intrinsics_ros1-camchain.yaml" \
   "$CALIB_TRACKED/camera_intrinsics/"

cp "$CAM_CALIB/head_camera_intrinsics_ros1-results-cam.txt" \
   "$CALIB_TRACKED/camera_intrinsics/"

cp "$CAM_CALIB/head_camera_intrinsics_ros1-report-cam.pdf" \
   "$CALIB_TRACKED/camera_intrinsics/"

cp "$IMU_CALIB/head_d435i_imu_allan_config_200hz_trim_first30min.yaml" \
   "$CALIB_TRACKED/imu_noise/"

cp "$IMU_CALIB/allan_output_200hz_trim_first30min/head_d435i_imu_200hz_trim_first30min.yaml" \
   "$CALIB_TRACKED/imu_noise/"

cp "$IMU_CALIB/allan_output_200hz_trim_first30min/acceleration.png" \
   "$CALIB_TRACKED/imu_noise/"

cp "$IMU_CALIB/allan_output_200hz_trim_first30min/gyro.png" \
   "$CALIB_TRACKED/imu_noise/"
```

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

