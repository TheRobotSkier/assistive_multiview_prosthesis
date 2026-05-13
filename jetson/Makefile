# Jetson deploy Makefile
# Run from ~/multiview_prosthesis/jetson on the Jetson.
# Manages the D435i cameras Docker container.
#
# Assumes the Docker image 'docker-deployment-miahand_ros2' is already built.

SUDO      := echo robotlab | sudo -S
IMAGE     := docker-deployment-miahand_ros2
REPO_ROOT := $(abspath $(CURDIR)/..)

# Source files (single source of truth — live in src/ in the repo)
CAM_CFG     := $(REPO_ROOT)/src/sensor_fusion_bringup/config/d435i_cameras.yaml
LAUNCH_FILE := $(REPO_ROOT)/src/sensor_fusion_bringup/launch/dual_d435i.launch.py
CYCLONE_CFG := $(CURDIR)/config/cyclonedds_robotlab.xml

# Install paths inside the container
CAM_CFG_INSTALL := /miahand_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/d435i_cameras.yaml
LAUNCH_INSTALL  := /miahand_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/launch/dual_d435i.launch.py

.PHONY: cameras cameras-stop cameras-logs

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
