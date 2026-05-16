# Jetson deploy Makefile
# Run from ~/multiview_prosthesis/jetson on the Jetson.
# Manages Docker containers for cameras and OpenVINS.
#
# Assumes the Docker image 'docker-deployment-realsense_camera' is already loaded.

SUDO      := echo robotlab | sudo -S
IMAGE     := docker-deployment-realsense_camera:latest
REPO_ROOT := $(abspath $(CURDIR)/..)

# Source files (single source of truth — live in src/ in the repo)
SFBP_SRC    := $(REPO_ROOT)/src/sensor_fusion_bringup
CAM_CFG_DIR := $(SFBP_SRC)/config
LAUNCH_DIR  := $(SFBP_SRC)/launch
SCRIPT_DIR  := $(SFBP_SRC)/scripts
CYCLONE_CFG := $(CURDIR)/config/cyclonedds_robotlab.xml

CAMERA_CONFIG ?= d435i_cameras.yaml
RIG_MODE      ?= dual_d435i

# Container lifetime limits (seconds). Adjust here to change all Jetson containers.
# 7200 = 2 hours
JETSON_CONTAINER_LIFETIME := 7200

# IMU dead reckoning test paths
IMU_LAUNCH     := $(REPO_ROOT)/src/sensor_fusion_bringup/launch/imu_test.launch.py
IMU_SCRIPT     := $(REPO_ROOT)/src/sensor_fusion_bringup/scripts/imu_dead_reckoning_node.py
IMU_CFG_DIR    := $(REPO_ROOT)/src/sensor_fusion_bringup/config

IMU_LAUNCH_INSTALL := /miahand_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/launch/imu_test.launch.py
IMU_SCRIPT_INSTALL := /miahand_ws/install/sensor_fusion_bringup/lib/sensor_fusion_bringup/imu_dead_reckoning_node.py
IMU_CFG_INSTALL    := /miahand_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/config

# Install paths inside the cameras container
PKG_SHARE_INSTALL := /miahand_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup
PKG_LIB_INSTALL   := /miahand_ws/install/sensor_fusion_bringup/lib/sensor_fusion_bringup

# OpenVINS overlay and source paths
OVINS_OVERLAY   := /home/robotlab/openvins_overlay/install_overlay

# Container paths for the openVINS overlay share (volume-mounted at runtime)
OVINS_PKG_SHARE := /ovins_overlay/install/sensor_fusion_bringup/share/sensor_fusion_bringup
OVINS_PKG_LIB   := /ovins_overlay/install/sensor_fusion_bringup/lib/sensor_fusion_bringup

.PHONY: cameras cameras-final cameras-mixed cameras-stop cameras-logs openvins openvins-final openvins-mixed openvins-stop openvins-logs list-cameras imu-test imu-test-stop imu-test-logs

# ── Cameras container (depth + pointcloud for both cameras) ──────────────────
cameras:
	$(SUDO) docker rm -f cameras_test 2>/dev/null || true
	$(SUDO) docker run -d \
		--name cameras_test \
		--network host \
		--ipc host \
		--runtime nvidia \
		--privileged \
		--user root \
		--group-add video \
		--group-add plugdev \
		--group-add dialout \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CYCLONE_CFG):/tmp/cyclonedds_peer.xml:ro \
		-v $(CAM_CFG_DIR):$(PKG_SHARE_INSTALL)/config:ro \
		-v $(LAUNCH_DIR):$(PKG_SHARE_INSTALL)/launch:ro \
		-v $(SCRIPT_DIR):$(PKG_LIB_INSTALL):ro \
		-v /dev:/dev \
		-v /run/udev:/run/udev:ro \
		$(IMAGE) \
		bash -lc 'source /opt/ros/jazzy/setup.bash; source /miahand_ws/install/setup.bash 2>/dev/null || true; timeout $(JETSON_CONTAINER_LIFETIME) ros2 launch sensor_fusion_bringup dual_d435i.launch.py camera_config:=$(CAMERA_CONFIG)'

cameras-mixed:
	$(MAKE) cameras CAMERA_CONFIG=mixed_d435_d435i_cameras.yaml

cameras-final:
	$(MAKE) cameras CAMERA_CONFIG=d435i_cameras.yaml

cameras-stop:
	$(SUDO) docker rm -f cameras_test 2>/dev/null || true

cameras-logs:
	$(SUDO) docker logs -f cameras_test

# ── OpenVINS container (RGB + IMU → VIO + ArUco correction) ─────────────────
# Volume-mounts the overlay and repo config/launch/scripts so changes
# to these files take effect on next container start without rebuilding.
openvins:
	$(SUDO) docker rm -f openvins 2>/dev/null || true
	$(SUDO) docker run -d \
		--name openvins \
		--network host \
		--ipc host \
		--runtime nvidia \
		--privileged \
		--user root \
		--group-add video \
		--group-add plugdev \
		--group-add dialout \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CYCLONE_CFG):/tmp/cyclonedds_peer.xml:ro \
		-v $(OVINS_OVERLAY):/ovins_overlay/install \
		-v $(SFBP_SRC)/config:$(OVINS_PKG_SHARE)/config:ro \
		-v $(SFBP_SRC)/launch:$(OVINS_PKG_SHARE)/launch:ro \
		-v $(SFBP_SRC)/scripts:$(OVINS_PKG_LIB):ro \
		$(IMAGE) \
		bash -lc 'set -e; source /opt/ros/jazzy/setup.bash; source /miahand_ws/install/setup.bash 2>/dev/null || true; for p in /ovins_overlay/install/*; do if [ -d "$$p/share" ]; then export AMENT_PREFIX_PATH="$$p:$${AMENT_PREFIX_PATH:-}"; export CMAKE_PREFIX_PATH="$$p:$${CMAKE_PREFIX_PATH:-}"; [ -d "$$p/lib" ] && export LD_LIBRARY_PATH="$$p/lib:$${LD_LIBRARY_PATH:-}"; fi; done; timeout $(JETSON_CONTAINER_LIFETIME) ros2 launch sensor_fusion_bringup dual_openvins_phase2.launch.py start_camera:=false rig_mode:=$(RIG_MODE)'

openvins-mixed:
	$(MAKE) openvins RIG_MODE=mixed_d435_d435i

openvins-final:
	$(MAKE) openvins RIG_MODE=dual_d435i

openvins-stop:
	$(SUDO) docker rm -f openvins 2>/dev/null || true

openvins-logs:
	$(SUDO) docker logs -f openvins

list-cameras:
	$(SUDO) bash $(CURDIR)/scripts/list_cameras.sh

# ── IMU dead reckoning test container ────────────────────────────────────────
imu-test:
	$(SUDO) docker rm -f imu_test 2>/dev/null || true
	$(SUDO) docker run -d \
		--name imu_test \
		--network host \
		--ipc host \
		--runtime nvidia \
		--privileged \
		--user root \
		--group-add video \
		--group-add plugdev \
		--group-add dialout \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CYCLONE_CFG):/tmp/cyclonedds_peer.xml:ro \
		-v $(IMU_LAUNCH):$(IMU_LAUNCH_INSTALL):ro \
		-v $(IMU_SCRIPT):$(IMU_SCRIPT_INSTALL):ro \
		-v $(IMU_CFG_DIR):$(IMU_CFG_INSTALL):ro \
		-v /dev:/dev \
		-v /run/udev:/run/udev:ro \
		$(IMAGE) \
		bash -lc 'source /opt/ros/jazzy/setup.bash; source /miahand_ws/install/setup.bash 2>/dev/null || true; timeout $(JETSON_CONTAINER_LIFETIME) ros2 launch sensor_fusion_bringup imu_test.launch.py'

imu-test-stop:
	$(SUDO) docker rm -f imu_test 2>/dev/null || true

imu-test-logs:
	$(SUDO) docker logs -f imu_test
