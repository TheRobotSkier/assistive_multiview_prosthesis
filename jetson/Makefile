# Jetson deploy Makefile
# Run from ~/multiview_prosthesis/jetson on the Jetson.
# Manages Docker containers for cameras and OpenVINS.
#
# Assumes the Docker image 'docker-deployment-realsense_camera' is already loaded.

SUDO      := echo robotlab | sudo -S
IMAGE     := docker-deployment-realsense_camera:latest
REPO_ROOT := $(abspath $(CURDIR)/..)

# Source files (single source of truth — live in src/ in the repo)
CAM_CFG     := $(REPO_ROOT)/src/sensor_fusion_bringup/config/d435i_cameras.yaml
LAUNCH_FILE := $(REPO_ROOT)/src/sensor_fusion_bringup/launch/dual_d435i.launch.py
CYCLONE_CFG := $(CURDIR)/config/cyclonedds_robotlab.xml

# Install paths inside the cameras container
CAM_CFG_INSTALL := /miahand_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/d435i_cameras.yaml
LAUNCH_INSTALL  := /miahand_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/launch/dual_d435i.launch.py

# OpenVINS overlay and source paths
OVINS_OVERLAY   := /home/robotlab/openvins_overlay/install_overlay
SFBP_SRC        := $(REPO_ROOT)/src/sensor_fusion_bringup

# Container paths for the openVINS overlay share (volume-mounted at runtime)
OVINS_PKG_SHARE := /ovins_overlay/install/sensor_fusion_bringup/share/sensor_fusion_bringup
OVINS_PKG_LIB   := /ovins_overlay/install/sensor_fusion_bringup/lib/sensor_fusion_bringup

.PHONY: cameras cameras-stop cameras-logs openvins openvins-stop openvins-logs list-cameras

# ── Cameras container (depth + pointcloud for both cameras) ──────────────────
cameras:
	$(SUDO) docker rm -f cameras_test 2>/dev/null || true
	$(SUDO) docker run -d \
		--name cameras_test \
		--network host \
		--ipc host \
		--runtime nvidia \
		--privileged \
		--group-add video \
		--group-add plugdev \
		--group-add dialout \
		--group-add i2c \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CYCLONE_CFG):/tmp/cyclonedds_peer.xml:ro \
		-v $(CAM_CFG):$(CAM_CFG_INSTALL):ro \
		-v $(LAUNCH_FILE):$(LAUNCH_INSTALL):ro \
		-v /dev:/dev \
		-v /run/udev:/run/udev:ro \
		$(IMAGE) \
		bash -c 'source /miahand_ws/install/setup.bash 2>/dev/null; source /opt/ros/jazzy/setup.bash; ros2 launch sensor_fusion_bringup dual_d435i.launch.py'

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
		--group-add video \
		--group-add plugdev \
		--group-add dialout \
		--group-add i2c \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CYCLONE_CFG):/tmp/cyclonedds_peer.xml:ro \
		-v $(OVINS_OVERLAY):/ovins_overlay/install \
		-v $(SFBP_SRC)/config:$(OVINS_PKG_SHARE)/config:ro \
		-v $(SFBP_SRC)/launch:$(OVINS_PKG_SHARE)/launch:ro \
		-v $(SFBP_SRC)/scripts:$(OVINS_PKG_LIB):ro \
		$(IMAGE) \
		bash -c 'source /miahand_ws/install/setup.bash && source /ovins_overlay/install/local_setup.bash && ros2 launch sensor_fusion_bringup dual_openvins_phase2.launch.py start_camera:=false'

openvins-stop:
	$(SUDO) docker rm -f openvins 2>/dev/null || true

openvins-logs:
	$(SUDO) docker logs -f openvins

list-cameras:
	$(SUDO) bash $(CURDIR)/scripts/list_cameras.sh
