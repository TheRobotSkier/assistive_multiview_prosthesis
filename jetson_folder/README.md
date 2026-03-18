# Start Realsens D435

A reliable startup command for camera in ROS2:
´´´
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=true
´´´

In another terminal:
´´´
source /opt/ros/humble/setup.bash
ros2 param set /camera/camera pointcloud__neon_.enable true
rviz2
´´´


# Intel RealSense D435 with ROS 2 Humble on Jetson

This guide explains how to:

- run the Intel RealSense D435 with ROS 2 Humble
- visualize RGB, depth, and point cloud data in RViz2
- record a ROS 2 bag
- replay the ROS 2 bag later

This setup was tested on a Jetson running **Ubuntu 22.04** with **ROS 2 Humble**. ROS 2 Humble is the supported Ubuntu 22.04 ROS 2 release, and the current `realsense-ros` wrapper supports Ubuntu 22.04 with Humble. 

---

## 1. Install ROS 2 Humble

Install ROS 2 Humble Desktop, which includes RViz2:

```bash
sudo apt update
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool git -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

ROS 2 Humble provides Ubuntu 22.04 packages, including `arm64`, and `ros-humble-desktop` includes RViz2.

Test RViz2:
```bash
rviz2
```
If RViz2 opens, the ROS 2 desktop install is working.


## 2. Install RealSense ROS packages
Install the RealSense ROS wrapper packages:
```bash
sudo apt update
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-*
```

The `realsense-ros` wrapper documents these Humble package names and supports Ubuntu 22.04 with ROS 2 Humble.

## 3. Launch the D435 camera
Start the RealSense camera node:
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=true
```

Then verify the camera topics exist:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep camera
```

Typical topics include:
* `/camera/camera/color/image_raw`
* `/camera/camera/color/camera_info`
* `/camera/camera/depth/image_rect_raw`
* `/camera/camera/depth/camera_info`

The RealSense launch file exposes parameters including `align_depth.enable`, and the wrapper publishes color/depth streams and camera calibration topics.

## 4. Enable the point cloud
On this Jetson setup, the point cloud parameter is exposed as:
* `pointcloud__neon_.enable`
instead of the more common:
* `pointcloud.enable`

Check available point cloud parameters:
```bash
source /opt/ros/humble/setup.bash
ros2 param list /camera/camera | grep pointcloud
```

Enable point cloud output:
```bash
source /opt/ros/humble/setup.bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```

Then verify the point cloud topic exists:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep points
```

Expected topic:
* `/camera/camera/depth/color/points` 

The RealSense ROS wrapper supports point-cloud output, and current Jetson reports show some installs exposing `pointcloud__neon_.*` parameters rather than `pointcloud.enable`.


## 5. RViz2 setup
Start RViz2:
```bash
rviz2
```

Recommended displays

Add these displays in RViz2:

**RGB image**
* Display type: `Image`
* Topic: `/camera/camera/color/image_raw`

**Depth image**
* Display type: `Image`
* Topic: `/camera/camera/depth/image_rect_raw`

**Point cloud**
* Display type: `PointCloud2`
* Topic: `/camera/camera/depth/color/points`

**TF**
* Display type: `TF`

**Fixed frame**

Set Global Options → Fixed Frame to a valid camera frame, such as:
* `camera_link`
* `camera_depth_optical_frame`
* `camera_color_optical_frame`

Pick whichever frame exists in your TF tree.

RViz2 supports `Image`, `PointCloud2`, and `TF` displays, and requires a valid fixed frame for 3D visualization.

**Notes**
* The RViz message `Stereo is NOT SUPPORTED` is informational and does not indicate a failure.
* Depth images may look dark depending on the RViz display scaling and scene conditions.


## 6. Record a ROS 2 bag
Create the recordings folder:
```bash
mkdir -p /home/robotlab/Documents/multiview_prosthesis/jetson_folder/ros_bag_recordings
```

Record RGB, depth, point cloud, camera info, and TF:
```bash
source /opt/ros/humble/setup.bash
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/depth/color/points \
  /tf \
  /tf_static \
  -o /home/robotlab/Documents/multiview_prosthesis/jetson_folder/ros_bag_recordings/d435_rgb_depth_pointcloud_01
```
This creates a ROS 2 bag folder containing the recording and metadata. The ROS 2 bag CLI records named topics and stores them for playback later.

**Why record `camera_info` and `tf`?**

These topics are useful for downstream processing:
* `camera_info` contains camera calibration/intrinsics
* `tf` and `tf_static` contain frame transforms
They are often needed for image projection, frame alignment, and point-cloud processing.


## 7. Inspect the bag

Check the bag contents:
```bash
source /opt/ros/humble/setup.bash
ros2 bag info /home/robotlab/Documents/multiview_prosthesis/jetson_folder/ros_bag_recordings/d435_rgb_depth_pointcloud_01
```

This shows:
* storage format
* duration
* message count
* recorded topics
ROS 2 provides `ros2 bag info` to inspect the metadata for a recorded bag.


## 8. Replay the bag
Replay the recording:
```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/robotlab/Documents/multiview_prosthesis/jetson_folder/ros_bag_recordings/d435_rgb_depth_pointcloud_01
```

Then open RViz2 and subscribe to the same topics again:
* `/camera/camera/color/image_raw`
* `/camera/camera/depth/image_rect_raw`
* `/camera/camera/depth/color/points`
ROS 2 bag playback republishes the recorded topics so they can be consumed by RViz2 or other ROS 2 nodes.


## 9. Typical startup workflow
**Terminal 1: launch camera**
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=true
```

**Terminal 2: enable point cloud**
```bash
source /opt/ros/humble/setup.bash
ros2 param set /camera/camera pointcloud__neon_.enable true
```

**Terminal 3: open RViz2**
```bash
rviz2
```

**Terminal 4: record bag**
```bash
source /opt/ros/humble/setup.bash
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/depth/color/points \
  /tf \
  /tf_static \
  -o /home/robotlab/Documents/multiview_prosthesis/jetson_folder/ros_bag_recordings/d435_rgb_depth_pointcloud_01
```